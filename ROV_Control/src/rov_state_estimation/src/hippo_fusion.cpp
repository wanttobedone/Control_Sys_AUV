#include "rov_state_estimation/hippo_fusion.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rov_state_estimation
{

HippoFusion::HippoFusion(const FusionConfig &config)
    : config_(config), ekf_(std::make_unique<Ekf>())
{
  const double positive_values[] = {config_.water_density_kg_m3,
                                    config_.pressure_height_std_m,
                                    config_.vision_position_std_m,
                                    config_.pair_tolerance_s,
                                    config_.imu_max_gap_s,
                                    config_.vision_timeout_s,
                                    config_.pressure_timeout_s,
                                    config_.startup_settle_s,
                                    config_.startup_position_tolerance_m,
                                    config_.startup_angle_tolerance_rad};
  for (const double value : positive_values)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("融合标准差、时间、容差和水密度必须为有限正数");
    }
  }
  if (!std::isfinite(config_.history_s) || config_.history_s < 0.1 || config_.history_s > 0.8)
  {
    // 上游长度为 1 时会在 interval 计算中除零；Pop 的有效搜索跨度为 1s。
    throw std::invalid_argument("history_s 必须在 [0.1, 0.8] 秒之间");
  }
  ekf_->settings_.vision_delay_us = static_cast<uint64_t>(config_.history_s * 1e6);
  ekf_->settings_.baro_delay_us = 0;
  ekf_->settings_.height_mode = HeightMode::kBaro;
  ekf_->settings_.baro_noise = config_.pressure_height_std_m;
  ekf_->settings_.baro_height_offset = 0.0;
}

bool HippoFusion::reject(const std::string &reason)
{
  ++rejected_inputs_;
  input_error_ = reason;
  return false;
}

bool HippoFusion::setVision(int64_t stamp_ns, const RigidPose &body_WB)
{
  // 故障时内部 IMU 时间已经停止。新观测不再与冻结时钟比较，也不覆盖首次故障。
  if (faulted_)
  {
    return false;
  }
  if (stamp_ns <= 0 || !FrameAdapter::isValid(body_WB))
  {
    return reject("vision_invalid_pose_or_stamp");
  }
  if (vision_ && stamp_ns <= vision_->stamp_ns)
  {
    return reject("vision_stamp_not_increasing");
  }
  if (last_imu_ns_ > 0)
  {
    const double age_s = static_cast<double>(last_imu_ns_ - stamp_ns) * 1e-9;
    if (age_s > config_.history_s || age_s < -config_.imu_max_gap_s)
    {
      return reject("vision_outside_imu_history_check_clock_or_latency");
    }
  }
  vision_ = VisionInput{stamp_ns, body_WB};
  vision_->body_WB.q.normalize();
  tryAlignPressure();
  return true;
}

bool HippoFusion::setPressure(int64_t stamp_ns, double pressure_pa)
{
  // 与 setVision 相同：锁定故障后停止接纳，不制造 pressure_clock 等连锁报错。
  if (faulted_)
  {
    return false;
  }
  if (stamp_ns <= last_pressure_ns_ || !std::isfinite(pressure_pa) || pressure_pa <= 0.0)
  {
    return reject("pressure_invalid_or_stamp_not_increasing");
  }
  if (last_imu_ns_ > 0 &&
      std::abs(static_cast<double>(stamp_ns - last_imu_ns_) * 1e-9) > config_.history_s)
  {
    return reject("pressure_clock_not_aligned_with_imu");
  }
  last_pressure_ns_ = stamp_ns;
  pressures_.push_back({stamp_ns, pressure_pa});
  // 只保存最近约 2s；数量上限防止异常高频来源无限占内存。
  while (pressures_.size() > 200 || pressures_.front().stamp_ns < stamp_ns - 2000000000LL)
  {
    pressures_.pop_front();
  }
  tryAlignPressure();
  return true;
}

void HippoFusion::tryAlignPressure()
{
  if (pressure_aligned_ || !vision_ || pressures_.empty() || last_imu_ns_ == 0)
  {
    return;
  }
  // 相机和压力都来自 Pi，寻找源时间最接近的一对。允许直接在水中启动。
  const auto nearest = std::min_element(pressures_.begin(), pressures_.end(),
                                        [this](const PressureInput &a, const PressureInput &b) {
                                          return std::abs(a.stamp_ns - vision_->stamp_ns) <
                                                 std::abs(b.stamp_ns - vision_->stamp_ns);
                                        });
  const double pair_dt_s =
      std::abs(static_cast<double>(nearest->stamp_ns - vision_->stamp_ns)) * 1e-9;
  const double vision_age_s = static_cast<double>(last_imu_ns_ - vision_->stamp_ns) * 1e-9;
  if (pair_dt_s > config_.pair_tolerance_s || vision_age_s < 0.0 ||
      vision_age_s > config_.history_s)
  {
    return;
  }
  reference_pressure_pa_ = nearest->pressure_pa;
  reference_body_height_m_ = vision_->body_WB.p_m.z();
  pressure_aligned_ = true;
}

void HippoFusion::feedObservations(int64_t imu_stamp_ns)
{
  if (!pressure_aligned_)
  {
    return;
  }
  for (const auto &pressure : pressures_)
  {
    if (pressure.stamp_ns <= fed_pressure_ns_ || pressure.stamp_ns > imu_stamp_ns)
    {
      continue;
    }
    if (static_cast<double>(imu_stamp_ns - pressure.stamp_ns) * 1e-9 > config_.history_s)
    {
      fed_pressure_ns_ = pressure.stamp_ns;
      continue;
    }
    BaroSample sample;
    sample.time_us = static_cast<uint64_t>(pressure.stamp_ns / 1000);
    // 静水压力增加代表下沉；高度 z 向上，所以用减号。
    // 这里是相对起始压力的位移，用视觉将其锚定到标签平面高度，不是测得水面深度。
    sample.height = reference_body_height_m_ - (pressure.pressure_pa - reference_pressure_pa_) /
                                                   (config_.water_density_kg_m3 * kGravity);
    ekf_->SetBaroData(sample);
    fed_pressure_ns_ = pressure.stamp_ns;
  }
  if (!vision_ || vision_->stamp_ns <= fed_vision_ns_ || vision_->stamp_ns > imu_stamp_ns)
  {
    return;
  }
  if (static_cast<double>(imu_stamp_ns - vision_->stamp_ns) * 1e-9 > config_.history_s)
  {
    fed_vision_ns_ = vision_->stamp_ns;
    reject("vision_expired_before_fusion");
    return;
  }
  VisionSample sample;
  // SetVisionData 内部必减 vision_delay + 半个滤波周期。我们的输入已是源采样时间，
  // 在库边界补回相同数值，库缓冲中的最终时间才仍是原始采样时间。
  // 此兼容补偿不修改 ROS 消息时间，也不声称实测视觉延迟恰好是 history_s。
  sample.time_us = static_cast<uint64_t>(vision_->stamp_ns / 1000) +
                   ekf_->settings_.vision_delay_us + kFilterUpdatePeriodUs / 2;
  sample.position = vision_->body_WB.p_m;
  sample.orientation = vision_->body_WB.q;
  // q 与 -q 表示同一旋转。上游直接作四元数分量残差，先选取同半球表示。
  const auto state = ekf_->StateAtFusionTime();
  const Eigen::Quaterniond predicted(state(0), state(1), state(2), state(3));
  if (sample.orientation.coeffs().dot(predicted.coeffs()) < 0.0)
  {
    sample.orientation.coeffs() *= -1.0;
  }
  sample.position_variance.setConstant(config_.vision_position_std_m *
                                       config_.vision_position_std_m);
  // 原库完整姿态融合使用固定四元数分量噪声 0.01；不伪装此字段能调整它。
  sample.angular_variance = 0.01;
  ekf_->SetVisionData(sample);
  fed_vision_ns_ = vision_->stamp_ns;
}

std::optional<FusionOutput> HippoFusion::pushImu(int64_t stamp_ns,
                                                 const Eigen::Vector3d &omega_B_radps,
                                                 const Eigen::Vector3d &specific_force_B_mps2)
{
  if (faulted_)
  {
    return std::nullopt;
  }
  if (stamp_ns <= last_imu_ns_ || !omega_B_radps.allFinite() || !specific_force_B_mps2.allFinite())
  {
    reject("imu_invalid_or_stamp_not_increasing");
    return std::nullopt;
  }
  if (last_imu_ns_ == 0)
  {
    // 首帧只建立时基，不能用电脑 now 与 PX4 源时间相减来伪造首个 dt。
    last_imu_ns_ = stamp_ns;
    tryAlignPressure();
    return std::nullopt;
  }
  int64_t cursor_ns = last_imu_ns_;
  const double dt_s = static_cast<double>(stamp_ns - cursor_ns) * 1e-9;
  diagnostics_.last_imu_dt_s = dt_s;
  last_imu_ns_ = stamp_ns;
  if (dt_s > config_.imu_max_gap_s)
  {
    faulted_ = true;
    status_ = "FAULT_IMU_GAP_RESTART_NODE";
    diagnostics_.fault_stamp_ns = stamp_ns;
    diagnostics_.fault_imu_gap_s = dt_s;
    diagnostics_.fault_reason = "imu_gap_no_invented_integration";
    reject("imu_gap_no_invented_integration");
    return std::nullopt;
  }
  std::optional<FusionOutput> newest_output;
  // 用当前采样值近似它之前一个原始采样间隔内的平均比力/角速度（矩形积分）。
  // 例如 250Hz 的 4+4+4ms：前两个全积累，第三个切成2ms+2ms，前2ms补齐10ms；
  // 剩余2ms留给下一个窗口。这样不把12ms增量错喂给上游固定10ms协方差模型。
  // 纳秒整数累计避免浮点舍入导致周期偶尔跳过；这不是给缺失IMU虚构新测量。
  while (cursor_ns < stamp_ns)
  {
    const int64_t remaining_ns = 10000000LL - accumulated_interval_ns_;
    const int64_t part_ns = std::min(stamp_ns - cursor_ns, remaining_ns);
    const double part_dt_s = static_cast<double>(part_ns) * 1e-9;
    delta_angle_rad_ += omega_B_radps * part_dt_s;
    delta_velocity_mps_ += specific_force_B_mps2 * part_dt_s;
    accumulated_interval_ns_ += part_ns;
    cursor_ns += part_ns;
    if (accumulated_interval_ns_ == 10000000LL)
    {
      const auto output = updateFilter(cursor_ns, omega_B_radps);
      if (faulted_)
      {
        return std::nullopt;
      }
      if (output)
      {
        newest_output = output;
      }
    }
  }
  return newest_output;
}

std::optional<FusionOutput> HippoFusion::updateFilter(int64_t stamp_ns,
                                                      const Eigen::Vector3d &omega_B_radps)
{
  // IMU 已由 USTROV 桥转 FLU；比力静止水平为 +g Z，重力由上游预测阶段扣除。
  ImuSample sample;
  sample.time_us = static_cast<uint64_t>(stamp_ns / 1000);
  sample.delta_angle = delta_angle_rad_;
  sample.delta_velocity = delta_velocity_mps_;
  sample.delta_angle_dt = 0.010;
  sample.delta_velocity_dt = 0.010;
  accumulated_interval_ns_ = 0;
  delta_angle_rad_.setZero();
  delta_velocity_mps_.setZero();
  ekf_->SetImuData(sample);
  tryAlignPressure();
  feedObservations(stamp_ns);
  if (!pressure_aligned_)
  {
    status_ = "WAIT_TIME_MATCHED_VISION_PRESSURE";
    diagnostics_.readiness_blocker = status_;
    return std::nullopt;
  }
  if (!ekf_->Update())
  {
    status_ = "WAIT_EKF_UPDATE";
    diagnostics_.readiness_blocker = status_;
    return std::nullopt;
  }
  if (first_update_ns_ == 0)
  {
    first_update_ns_ = stamp_ns;
  }
  FusionOutput output;
  output.stamp_ns = stamp_ns;
  output.body_WB = {ekf_->Position(), ekf_->Orientation()};
  output.velocity_W_mps = ekf_->Velocity();
  output.angular_velocity_B_radps = omega_B_radps - ekf_->GyroBias();
  if (!FrameAdapter::isValid(output.body_WB) || !output.velocity_W_mps.allFinite() ||
      !output.angular_velocity_B_radps.allFinite())
  {
    faulted_ = true;
    status_ = "FAULT_NONFINITE_EKF_RESTART_NODE";
    diagnostics_.fault_stamp_ns = stamp_ns;
    diagnostics_.fault_reason = "nonfinite_ekf_output";
    reject(diagnostics_.fault_reason);
    return std::nullopt;
  }
  output.ready = updateReadiness(output);
  return output;
}

bool HippoFusion::updateReadiness(const FusionOutput &output)
{
  tilt_variance_rad2_ = ekf_->CalcRotationVectorVariances().head<2>().sum();
  // 所有 gate_* 数据取自同一次检查。外层原始回调可能随后收到更新的消息，
  // 因此同时保存参与检查的视觉/压力时间，避免把两个不同时刻的数值混在一起解释。
  diagnostics_.check_stamp_ns = output.stamp_ns;
  diagnostics_.vision_stamp_ns = vision_ ? vision_->stamp_ns : 0;
  diagnostics_.pressure_stamp_ns = last_pressure_ns_;
  ++diagnostics_.filter_update_count;
  diagnostics_.vision_age_s = vision_ ? static_cast<double>(output.stamp_ns - vision_->stamp_ns) * 1e-9
                                    : std::numeric_limits<double>::quiet_NaN();
  diagnostics_.pressure_age_s = static_cast<double>(output.stamp_ns - last_pressure_ns_) * 1e-9;
  diagnostics_.vision_fresh = diagnostics_.vision_age_s >= 0.0 &&
                              diagnostics_.vision_age_s < config_.vision_timeout_s;
  diagnostics_.pressure_fresh = diagnostics_.pressure_age_s >= 0.0 &&
                                diagnostics_.pressure_age_s < config_.pressure_timeout_s;
  diagnostics_.attitude_valid = ekf_->AttitudeValid();
  diagnostics_.position_error_m = std::numeric_limits<double>::quiet_NaN();
  diagnostics_.angle_error_rad = std::numeric_limits<double>::quiet_NaN();
  if (vision_)
  {
    // 保留原比较：当前输出 vs 最新视觉，不是同一时刻的创新量。
    // 即使视觉已过期也展示差值；是否允许 READY 仍由 fresh 单独判断。
    diagnostics_.position_error_m = (output.body_WB.p_m - vision_->body_WB.p_m).norm();
    diagnostics_.angle_error_rad = output.body_WB.q.angularDistance(vision_->body_WB.q);
  }
  diagnostics_.position_consistent =
      diagnostics_.position_error_m < config_.startup_position_tolerance_m;
  diagnostics_.angle_consistent =
      diagnostics_.angle_error_rad < config_.startup_angle_tolerance_rad;
  diagnostics_.startup_elapsed_s = static_cast<double>(output.stamp_ns - first_update_ns_) * 1e-9;
  const bool fresh = diagnostics_.vision_fresh && diagnostics_.pressure_fresh;
  bool consistent = fresh && diagnostics_.attitude_valid && diagnostics_.position_consistent &&
                    diagnostics_.angle_consistent;
  if (!initialized_)
  {
    if (diagnostics_.startup_elapsed_s < config_.startup_settle_s)
    {
      consistent = false;
    }
    if (!consistent)
    {
      consistent_since_ns_ = 0;
    }
    else if (consistent_since_ns_ == 0)
    {
      consistent_since_ns_ = output.stamp_ns;
    }
    else if (output.stamp_ns - consistent_since_ns_ >= 500000000LL)
    {
      initialized_ = true;
    }
    diagnostics_.startup_consistent_s =
        consistent_since_ns_ == 0 ? 0.0
                                 : static_cast<double>(output.stamp_ns - consistent_since_ns_) * 1e-9;
  }

  // 多项同时不满足时，blocker 给出按以下顺序遇到的第一项；布尔量仍展示所有检查。
  // 不再把视觉时效、位置差或角度差都归为“保持静止”。不改变上面的原就绪判定。
  if (!diagnostics_.vision_fresh)
  {
    diagnostics_.readiness_blocker = "WAIT_FRESH_VISION";
  }
  else if (!diagnostics_.pressure_fresh)
  {
    diagnostics_.readiness_blocker = "WAIT_FRESH_PRESSURE";
  }
  else if (!diagnostics_.attitude_valid)
  {
    diagnostics_.readiness_blocker = "WAIT_TILT_ALIGNMENT";
  }
  else if (!diagnostics_.position_consistent)
  {
    diagnostics_.readiness_blocker = "WAIT_POSITION_AGREEMENT";
  }
  else if (!diagnostics_.angle_consistent)
  {
    diagnostics_.readiness_blocker = "WAIT_ANGLE_AGREEMENT";
  }
  else if (diagnostics_.startup_elapsed_s < config_.startup_settle_s)
  {
    diagnostics_.readiness_blocker = "WAIT_STARTUP_SETTLE";
  }
  else if (!initialized_)
  {
    diagnostics_.readiness_blocker = "WAIT_CONSISTENCY_HOLD";
  }
  else
  {
    diagnostics_.readiness_blocker = "NONE";
  }
  if (!initialized_)
  {
    status_ = diagnostics_.readiness_blocker;
    return false;
  }
  // 丢标后重新收到观测，不等于滞后的EKF已经纠正漂移；比较通过才恢复 READY。
  status_ = !fresh ? "DEGRADED_OBSERVATION_STALE"
                   : (consistent ? "READY" : "RECOVERING_VISUAL_AGREEMENT");
  return fresh && consistent;
}

const std::string &HippoFusion::status() const
{
  return status_;
}
const std::string &HippoFusion::inputError() const
{
  return input_error_;
}
bool HippoFusion::pressureAligned() const
{
  return pressure_aligned_;
}
bool HippoFusion::initialized() const
{
  return initialized_;
}
int64_t HippoFusion::visionStamp() const
{
  return vision_ ? vision_->stamp_ns : 0;
}
int64_t HippoFusion::pressureStamp() const
{
  return last_pressure_ns_;
}
uint64_t HippoFusion::rejectedInputs() const
{
  return rejected_inputs_;
}
double HippoFusion::tiltVarianceRad2() const
{
  return tilt_variance_rad2_;
}
const FusionDiagnostics &HippoFusion::diagnostics() const
{
  return diagnostics_;
}

} // namespace rov_state_estimation

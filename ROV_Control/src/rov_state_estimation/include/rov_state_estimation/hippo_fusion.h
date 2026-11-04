#pragma once

// 上游在 Eigen 前定义插件；必须先包含其头，避免同一进程不同编译单元的 Eigen 定义不一致。
#include <ustrov_state_estimation/ekf.h>

#include "rov_state_estimation/frame_adapter.h"

#include <deque>
#include <memory>
#include <optional>
#include <string>

namespace rov_state_estimation
{

struct FusionConfig
{
  double water_density_kg_m3{1000.0};
  double pressure_height_std_m{0.03};
  double vision_position_std_m{0.03};
  // 上游按 10ms 建模；本层按源时间把原始增量分段积累到严格 10ms 再送入。
  // 0.5s 是保留历史的长度，不是从已经有采样时间戳的视觉上再扣 0.5s。
  double history_s{0.5};
  double pair_tolerance_s{0.10};
  double imu_max_gap_s{0.10};
  double vision_timeout_s{0.8};
  double pressure_timeout_s{0.4};
  double startup_settle_s{3.0};
  // 初始化阶段要求融合结果接近新鲜视觉，并维持 0.5s，避免零状态抢先建立 home。
  double startup_position_tolerance_m{0.08};
  double startup_angle_tolerance_rad{0.20};
};

struct FusionOutput
{
  int64_t stamp_ns{0};
  RigidPose body_WB;
  Eigen::Vector3d velocity_W_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_B_radps{Eigen::Vector3d::Zero()};
  // ready 表示当前输入新鲜且初始化通过；false 时仍可能是惯性预测输出。
  bool ready{false};
};

// 复用 Ekf 计算库的适配器，不继承/复制上游 EKF，不修改其状态方程。
// 调用者串行调用 setVision/setPressure/pushImu；只有 pushImu 推进滤波。
// 时间全部来自同一 Pi 时钟下的消息 header，主机接收时刻不参与积分 dt。
class HippoFusion
{
public:
  explicit HippoFusion(const FusionConfig &config = FusionConfig{});
  bool setVision(int64_t stamp_ns, const RigidPose &body_WB);
  bool setPressure(int64_t stamp_ns, double pressure_pa);
  std::optional<FusionOutput> pushImu(int64_t stamp_ns, const Eigen::Vector3d &omega_B_radps,
                                      const Eigen::Vector3d &specific_force_B_mps2);
  const std::string &status() const;
  const std::string &inputError() const;
  bool pressureAligned() const;
  bool initialized() const;
  int64_t visionStamp() const;
  int64_t pressureStamp() const;
  uint64_t rejectedInputs() const;
  double tiltVarianceRad2() const;

private:
  struct VisionInput
  {
    int64_t stamp_ns;
    RigidPose body_WB;
  };
  struct PressureInput
  {
    int64_t stamp_ns;
    double pressure_pa;
  };

  bool reject(const std::string &reason);
  void tryAlignPressure();
  void feedObservations(int64_t imu_stamp_ns);
  std::optional<FusionOutput> updateFilter(int64_t stamp_ns, const Eigen::Vector3d &omega_B_radps);
  bool updateReadiness(const FusionOutput &output);

  FusionConfig config_;
  std::unique_ptr<Ekf> ekf_;
  std::deque<PressureInput> pressures_;
  std::optional<VisionInput> vision_;
  int64_t last_imu_ns_{0};
  int64_t last_pressure_ns_{0};
  int64_t fed_pressure_ns_{0};
  int64_t fed_vision_ns_{0};
  int64_t first_update_ns_{0};
  int64_t consistent_since_ns_{0};
  double reference_pressure_pa_{0.0};
  double reference_body_height_m_{0.0};
  int64_t accumulated_interval_ns_{0};
  Eigen::Vector3d delta_angle_rad_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_velocity_mps_{Eigen::Vector3d::Zero()};
  bool pressure_aligned_{false};
  bool initialized_{false};
  bool faulted_{false};
  uint64_t rejected_inputs_{0};
  double tilt_variance_rad2_{0.0};
  std::string status_{"WAIT_IMU_VISION_PRESSURE"};
  std::string input_error_;
};

} // namespace rov_state_estimation

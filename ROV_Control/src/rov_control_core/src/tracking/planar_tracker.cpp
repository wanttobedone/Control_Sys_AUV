#include "rov_control_core/tracking/planar_tracker.h"
#include "rov_control_core/planning/workspace_bounds.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rov_control_core
{
namespace
{
bool validState(const RobotState &state)
{
  // core可脱离ROS使用，不能仅依赖StateAdapter保证输入合法。
  return state.p_W.allFinite() && state.v_W.allFinite() && state.omega_B.allFinite() &&
         state.q_WB.coeffs().allFinite() && std::abs(state.q_WB.norm() - 1.0) < 1e-6;
}
double yawOf(const Eigen::Quaterniond &q)
{
  // 将机头轴投影到世界水平面，得到航向；大倾斜在节点的checkState中拒绝。
  const auto heading = q * Eigen::Vector3d::UnitX();
  return std::atan2(heading.y(), heading.x());
}
} // namespace

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
const char *trackingPhaseName(TrackingPhase phase)
{
  switch (phase)
  {
  case TrackingPhase::IDLE:
    return "IDLE";
  case TrackingPhase::BRAKING:
    return "BRAKING";
  case TrackingPhase::TURNING:
    return "TURNING";
  case TrackingPhase::TRACKING:
    return "TRACKING";
  case TrackingPhase::HOLDING:
    return "HOLDING";
  }
  return "UNKNOWN";
}

Result<HomeFrame> HomeFrame::fromState(const RobotState &state)
{
  if (!validState(state))
  {
    return Result<HomeFrame>::failure(ErrorCode::INVALID_ARGUMENT, "无法从非法状态建立 home");
  }
  const Eigen::Vector3d heading = state.q_WB * Eigen::Vector3d::UnitX();
  if (heading.head<2>().norm() < 0.1)
  {
    return Result<HomeFrame>::failure(ErrorCode::UNSUPPORTED_DIRECTION, "初始机头接近竖直");
  }
  HomeFrame frame;
  frame.origin_W = state.p_W;
  frame.q_WH = Eigen::AngleAxisd(yawOf(state.q_WB), Eigen::Vector3d::UnitZ());
  return Result<HomeFrame>::success(frame);
}
Eigen::Vector3d HomeFrame::toWorld(const Eigen::Vector3d &point_H) const
{
  return origin_W + q_WH * point_H;
}
Eigen::Vector3d HomeFrame::toHome(const Eigen::Vector3d &point_W) const
{
  return q_WH.conjugate() * (point_W - origin_W);
}

Status PlanarTracker::validateConfig(const PlanarConfig &c)
{
  const auto bounds = validateBounds(c.bounds);
  if (!bounds.ok())
  {
    return bounds;
  }
  // 名值对只用于报错时指出具体参数；每次构造/参数变更检查，不在控制周期重复检查。
  for (const auto &[name, value] : {std::pair<const char *, double>{"mass_kg", c.mass_kg},
                                    {"max_speed_mps", c.motion.v_max_mps},
                                    {"max_acceleration_mps2", c.motion.a_max_mps2},
                                    {"min_duration_s", c.motion.min_duration_s},
                                    {"lookahead_m", c.lookahead_m},
                                    {"max_force_N", c.max_force_N},
                                    {"arrival_radius_m", c.arrival_radius_m},
                                    {"arrival_speed_mps", c.arrival_speed_mps},
                                    {"arrival_dwell_s", c.arrival_dwell_s},
                                    {"reacquire_radius_m", c.reacquire_radius_m}})
  {
    if (!std::isfinite(value) || value <= 0)
    {
      return {ErrorCode::INVALID_LIMITS,
              std::string(name) + "=" + std::to_string(value) + " 必须为有限正数"};
    }
  }
  // 零增益用于关闭相应反馈/前馈；mission_timeout_s=0 表示不设任务时限。
  for (const auto &[name, value] :
       {std::pair<const char *, double>{"surge_drag_Nspm", c.surge_drag_Nspm},
        {"position_kp_Npm", c.position_kp_Npm},
        {"velocity_kd_Nspm", c.velocity_kd_Nspm},
        {"mission_timeout_s", c.mission_timeout_s}})
  {
    if (!std::isfinite(value) || value < 0)
    {
      return {ErrorCode::INVALID_LIMITS,
              std::string(name) + "=" + std::to_string(value) + " 必须为有限非负数"};
    }
  }
  for (Eigen::Index i = 0; i < 3; ++i)
  {
    for (const auto &[name, value] :
         {std::pair<const char *, double>{"attitude_kp", c.attitude_kp[i]},
          {"attitude_kd", c.attitude_kd[i]},
          {"max_torque_Nm", c.max_torque_Nm[i]}})
    {
      if (!std::isfinite(value) || value < 0)
      {
        return {ErrorCode::INVALID_LIMITS, std::string(name) + "[" + std::to_string(i) +
                                               "]=" + std::to_string(value) + " 必须为有限非负数"};
      }
    }
  }
  if (c.reacquire_radius_m <= c.arrival_radius_m)
  {
    return {ErrorCode::INVALID_LIMITS,
            "reacquire_radius_m=" + std::to_string(c.reacquire_radius_m) +
                " 必须大于 arrival_radius_m=" + std::to_string(c.arrival_radius_m)};
  }
  if (c.motion.j_max_mps3 && (!std::isfinite(*c.motion.j_max_mps3) || *c.motion.j_max_mps3 <= 0))
  {
    return {ErrorCode::INVALID_LIMITS, "j_max_mps3 必须为有限正数"};
  }
  return {};
}
PlanarTracker::PlanarTracker(const PlanarConfig &config) : config_(config)
{
  const auto status = validateConfig(config);
  if (!status.ok())
  {
    throw std::invalid_argument(status.message);
  }
}
void PlanarTracker::setConfig(const PlanarConfig &config)
{
  // ROS 参数事务在提交前调用 validateConfig；纯 C++ 调用者遵守同一约定。
  config_ = config;
}
void PlanarTracker::reset()
{
  phase_ = TrackingPhase::IDLE;
  trajectory_.reset();
  elapsed_s_ = mission_s_ = stable_s_ = 0.0;
}
Status PlanarTracker::setGoal(const Eigen::Vector3d &goal_W, const RobotState &state)
{
  if (!validState(state) || !goal_W.allFinite())
  {
    return {ErrorCode::INVALID_ARGUMENT, "目标或状态非法"};
  }
  // 调用者先将 home 的 x/y 转到世界系；深度目标统一取当前深度，仅用于画参考线。
  Eigen::Vector3d horizontalGoal(goal_W.x(), goal_W.y(), state.p_W.z());
  if (!contains(config_.bounds, horizontalGoal) || !contains(config_.bounds, state.p_W))
  {
    return {ErrorCode::OUTSIDE_WORKSPACE, "目标或起点超出配置的活动盒"};
  }
  if (phase_ != TrackingPhase::IDLE && (horizontalGoal - goal_W_).head<2>().norm() < 1e-6)
  {
    return {}; // ros2 topic pub 不带 --once 时也不会每条都重置轨迹
  }
  reset();
  goal_W_ = horizontalGoal;
  heading_rad_ = yawOf(state.q_WB);
  phase_ = TrackingPhase::BRAKING;
  return {};
}
Status PlanarTracker::planFrom(const RobotState &state)
{
  // 转向已经完成，再用真实当前位置作为线段起点，避免转向漂移留在旧起点误差中。
  PlanRequest request;
  request.p_start_W = state.p_W;
  goal_W_.z() = state.p_W.z();
  request.p_goal_W = goal_W_;
  request.bounds = config_.bounds;
  const auto path = LinePathPlanner::plan(request);
  if (!path.ok())
  {
    return path.status();
  }
  const Eigen::Quaterniond attitude(Eigen::AngleAxisd(heading_rad_, Eigen::Vector3d::UnitZ()));
  const auto trajectory = QuinticTrajectory::create(path.value(), config_.motion, attitude);
  if (!trajectory.ok())
  {
    return trajectory.status();
  }
  trajectory_ = trajectory.value();
  elapsed_s_ = stable_s_ = 0.0;
  phase_ = TrackingPhase::TRACKING;
  return {};
}

Result<TrackingOutput> PlanarTracker::update(const RobotState &state, double dt_s)
{
  if (!validState(state) || !std::isfinite(dt_s) || dt_s <= 0)
  {
    return Result<TrackingOutput>::failure(ErrorCode::INVALID_ARGUMENT, "跟踪状态或时间步非法");
  }
  TrackingOutput output;
  output.reference.p_W = state.p_W;
  if (phase_ == TrackingPhase::IDLE)
  {
    return Result<TrackingOutput>::success(output);
  }
  const double yaw = yawOf(state.q_WB);
  const Eigen::Vector3d velocity_B = state.q_WB.conjugate() * state.v_W;
  const Eigen::Vector2d delta = (goal_W_ - state.p_W).head<2>();
  const double distance = delta.norm();
  const double speed = state.v_W.head<2>().norm();
  mission_s_ += dt_s;
  if (phase_ != TrackingPhase::HOLDING && config_.mission_timeout_s > 0 &&
      mission_s_ > config_.mission_timeout_s)
  {
    return Result<TrackingOutput>::failure(ErrorCode::INVALID_LIMITS, "到点任务超时");
  }

  // 换目标先制动，不把旧轨迹的非零速度硬接到新的静止起点。
  if (phase_ == TrackingPhase::BRAKING)
  {
    output.force_x_N = -config_.velocity_kd_Nspm * velocity_B.x();
    stable_s_ = speed < 0.015 && std::abs(state.omega_B.z()) < 0.08 ? stable_s_ + dt_s : 0;
    if (stable_s_ >= 0.25)
    {
      stable_s_ = 0;
      if (distance <= config_.arrival_radius_m)
      {
        phase_ = TrackingPhase::HOLDING;
      }
      else
      {
        heading_rad_ = std::atan2(delta.y(), delta.x());
        phase_ = TrackingPhase::TURNING;
      }
    }
  }
  if (phase_ == TrackingPhase::TURNING)
  {
    output.force_x_N = -config_.velocity_kd_Nspm * velocity_B.x();
    stable_s_ = std::abs(wrapAngle(heading_rad_ - yaw)) < 0.04 &&
                        std::abs(state.omega_B.z()) < 0.08 && speed < 0.02
                    ? stable_s_ + dt_s
                    : 0;
    if (stable_s_ >= 0.3)
    {
      const auto planned = planFrom(state);
      if (!planned.ok())
      {
        return Result<TrackingOutput>::failure(planned.code, planned.message);
      }
    }
  }
  double headingTarget = heading_rad_;
  if (phase_ == TrackingPhase::TRACKING)
  {
    const auto sampled = trajectory_->sample(elapsed_s_);
    const auto progress = projectProgress(trajectory_->path(), state);
    if (!sampled.ok() || !progress.ok())
    {
      return Result<TrackingOutput>::failure(ErrorCode::INVALID_ARGUMENT, "轨迹采样或投影失败");
    }
    output.reference = sampled.value();
    const Eigen::Vector2d direction(std::cos(heading_rad_), std::sin(heading_rad_));
    const Eigen::Vector2d left(-direction.y(), direction.x());
    output.cross_track_m = left.dot(progress.value().cross_track_W.head<2>());
    // LOS 航向：位于路径左侧时向右修正。最大约34°，避免侧误差造成瞬时反向。
    const double correction =
        std::clamp(-std::atan2(output.cross_track_m, config_.lookahead_m), -0.6, 0.6);
    headingTarget += correction;
    const double positionError = sampled.value().s_m - progress.value().s_m;
    const double velocityError = sampled.value().v_s_mps - progress.value().v_s_mps;
    // 名义沿线动力学 m*dv/dt = Fx - d*v：前馈支付加速与阻力，PD纠正状态误差。
    // 路径与机头近似对齐才使用此近似；不把不可控侧向误差直接变成侧推力。
    const double inertiaFeedforward = config_.mass_kg * sampled.value().a_s_mps2;
    const double dragFeedforward = config_.surge_drag_Nspm * sampled.value().v_s_mps;
    const double positionFeedback = config_.position_kp_Npm * positionError;
    const double velocityFeedback = config_.velocity_kd_Nspm * velocityError;
    output.force_x_N = inertiaFeedforward + dragFeedforward + positionFeedback + velocityFeedback;
    // 大航向偏差时暂缓参考推进并制动；不要求机器人在未对准时侧向追赶轨迹。
    if (std::abs(wrapAngle(headingTarget - yaw)) < 0.3)
    {
      elapsed_s_ += dt_s;
    }
    else
    {
      output.force_x_N = -config_.velocity_kd_Nspm * velocity_B.x();
    }
    stable_s_ = elapsed_s_ >= trajectory_->duration() && distance < config_.arrival_radius_m &&
                        speed < config_.arrival_speed_mps
                    ? stable_s_ + dt_s
                    : 0;
    if (stable_s_ >= config_.arrival_dwell_s)
    {
      phase_ = TrackingPhase::HOLDING;
    }
  }
  if (phase_ == TrackingPhase::HOLDING)
  {
    // 欠驱动保持：沿最终朝向抑制纵向漂移；偏离容差圈时重新制动/转向接近。
    // 这是水平容差内保持，不声称侧向瞬时可控或三维悬停。
    const Eigen::Vector2d direction(std::cos(heading_rad_), std::sin(heading_rad_));
    output.force_x_N = config_.position_kp_Npm * direction.dot(delta) -
                       config_.velocity_kd_Nspm * direction.dot(state.v_W.head<2>());
    output.reference.p_W = goal_W_;
    output.reference.v_W.setZero();
    output.reference.a_W.setZero();
    if (distance > config_.reacquire_radius_m)
    {
      phase_ = TrackingPhase::BRAKING;
      stable_s_ = mission_s_ = 0;
      heading_rad_ = yaw;
    }
  }

  // Hippo 保持小倾斜，所以 roll/pitch 欧拉角有效；节点对大倾斜另行终止。
  const Eigen::Matrix3d rotation = state.q_WB.toRotationMatrix();
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  output.heading_error_rad = wrapAngle(headingTarget - yaw);
  const Eigen::Vector3d angleError(-roll, -pitch, output.heading_error_rad);
  const Eigen::Vector3d requested = config_.attitude_kp.cwiseProduct(angleError) -
                                    config_.attitude_kd.cwiseProduct(state.omega_B);
  output.torque_B_Nm = requested.cwiseMax(-config_.max_torque_Nm).cwiseMin(config_.max_torque_Nm);
  const double force = std::clamp(output.force_x_N, -config_.max_force_N, config_.max_force_N);
  output.limited = force != output.force_x_N || (requested - output.torque_B_Nm).norm() > 1e-12;
  output.force_x_N = force;
  output.distance_xy_m = distance;
  output.phase = phase_;
  output.reference.q_WB_des = Eigen::AngleAxisd(headingTarget, Eigen::Vector3d::UnitZ());
  return Result<TrackingOutput>::success(output);
}
} // namespace rov_control_core

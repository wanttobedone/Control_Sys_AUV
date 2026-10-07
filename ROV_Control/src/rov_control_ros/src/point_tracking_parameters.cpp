/**
 * 到点节点的参数入口。loadConfig读取启动覆盖值；onParameters校验候选副本；
 * applyParameters只在ROS参数事务提交后更新算法，因此失败事务不影响正在运行的任务。
 * 增益/输出限幅下周期生效；已经生成的五次轨迹不被热改，运动参数供下一段规划使用。
 */
#include "rov_control_ros/point_tracking_node.h"
#include <cmath>
#include <stdexcept>

namespace rov_control_ros
{
using namespace rov_control_core;

PlanarConfig PointTrackingNode::loadConfig()
{
  if (!get_parameter("use_sim_time").as_bool())
  {
    throw std::invalid_argument("Hippo执行适配要求 use_sim_time=true");
  }
  rcl_interfaces::msg::ParameterDescriptor fixed;
  fixed.read_only = true;
  fixed.description = "坐标接口在节点启动时固定，修改需重启";
  worldFrame_ = declare_parameter<std::string>("world_frame", "map", fixed);
  bodyFrame_ = declare_parameter<std::string>("body_frame", "uuv00/base_link", fixed);
  homeFrame_ = declare_parameter<std::string>("home_frame", "uuv00/home", fixed);
  if (worldFrame_.empty() || bodyFrame_.empty() || homeFrame_.empty())
  {
    throw std::invalid_argument("world_frame/body_frame/home_frame 不得为空");
  }

  PlanarConfig config;
  // 这里列出全部可调项；默认值集中在结构体，YAML/launch覆盖值由ROS返回。
  const auto vectorParam = [this](const char *name, const Eigen::Vector3d &value)
  {
    const auto list =
        declare_parameter<std::vector<double>>(name, {value.x(), value.y(), value.z()});
    if (list.size() != 3)
    {
      throw std::invalid_argument(std::string(name) + " 必须包含三个数");
    }
    return Eigen::Vector3d(list[0], list[1], list[2]);
  };
  config.bounds.min_W = vectorParam("workspace_min_W", config.bounds.min_W);
  config.bounds.max_W = vectorParam("workspace_max_W", config.bounds.max_W);
  config.attitude_kp = vectorParam("attitude_kp", config.attitude_kp);
  config.attitude_kd = vectorParam("attitude_kd", config.attitude_kd);
  config.max_torque_Nm = vectorParam("max_torque_Nm", config.max_torque_Nm);
  config.motion.v_max_mps = declare_parameter<double>("max_speed_mps", config.motion.v_max_mps);
  config.motion.a_max_mps2 =
      declare_parameter<double>("max_acceleration_mps2", config.motion.a_max_mps2);
  config.mass_kg = declare_parameter<double>("mass_kg", config.mass_kg);
  config.surge_drag_Nspm = declare_parameter<double>("surge_drag_Nspm", config.surge_drag_Nspm);
  config.position_kp_Npm = declare_parameter<double>("position_kp_Npm", config.position_kp_Npm);
  config.velocity_kd_Nspm = declare_parameter<double>("velocity_kd_Nspm", config.velocity_kd_Nspm);
  config.lookahead_m = declare_parameter<double>("lookahead_m", config.lookahead_m);
  config.max_force_N = declare_parameter<double>("max_force_N", config.max_force_N);
  config.arrival_radius_m = declare_parameter<double>("arrival_radius_m", config.arrival_radius_m);
  config.arrival_speed_mps =
      declare_parameter<double>("arrival_speed_mps", config.arrival_speed_mps);
  config.arrival_dwell_s = declare_parameter<double>("arrival_dwell_s", config.arrival_dwell_s);
  config.reacquire_radius_m =
      declare_parameter<double>("reacquire_radius_m", config.reacquire_radius_m);
  config.mission_timeout_s =
      declare_parameter<double>("mission_timeout_s", config.mission_timeout_s);
  limits_.max_actual_speed_mps =
      declare_parameter<double>("max_actual_speed_mps", limits_.max_actual_speed_mps);
  limits_.max_tilt_rad = declare_parameter<double>("max_tilt_rad", limits_.max_tilt_rad);
  limits_.max_angular_speed_radps =
      declare_parameter<double>("max_angular_speed_radps", limits_.max_angular_speed_radps);
  limits_.max_depth_drift_m =
      declare_parameter<double>("max_depth_drift_m", limits_.max_depth_drift_m);
  limits_.max_state_age_s = declare_parameter<double>("max_state_age_s", limits_.max_state_age_s);
  limits_.max_receive_gap_s =
      declare_parameter<double>("max_receive_gap_s", limits_.max_receive_gap_s);
  limits_.max_ros_step_s = declare_parameter<double>("max_ros_step_s", limits_.max_ros_step_s);
  limits_.max_thruster_command =
      declare_parameter<double>("max_thruster_command", limits_.max_thruster_command);
  const auto status = validateParameters(config, limits_);
  if (!status.ok())
  {
    throw std::invalid_argument(status.message);
  }
  return config;
}

void PointTrackingNode::readParameterValues(const std::vector<rclcpp::Parameter> &parameters,
                                            PlanarConfig &config,
                                            TrackingRuntimeLimits &limits) const
{
  // 候选值逐项赋给副本，未出现在事务中的参数保持原值；不改任何ROS或控制状态。
  for (const auto &parameter : parameters)
  {
    const std::string &name = parameter.get_name();
    if (name == "workspace_min_W" || name == "workspace_max_W" || name == "attitude_kp" ||
        name == "attitude_kd" || name == "max_torque_Nm")
    {
      const auto &values = parameter.as_double_array();
      if (values.size() != 3)
      {
        throw std::invalid_argument(name + " 必须包含三个数");
      }
      const Eigen::Vector3d value(values[0], values[1], values[2]);
      if (name == "workspace_min_W")
      {
        config.bounds.min_W = value;
      }
      else if (name == "workspace_max_W")
      {
        config.bounds.max_W = value;
      }
      else if (name == "attitude_kp")
      {
        config.attitude_kp = value;
      }
      else if (name == "attitude_kd")
      {
        config.attitude_kd = value;
      }
      else
      {
        config.max_torque_Nm = value;
      }
      continue;
    }
    if (name == "use_sim_time")
    {
      if (!parameter.as_bool())
      {
        throw std::invalid_argument("仿真执行期间 use_sim_time 必须为 true");
      }
      continue;
    }
    if (name == "world_frame" || name == "body_frame" || name == "home_frame")
    {
      continue; // read_only参数由ROS参数服务拒绝修改
    }
    // 显式名称映射便于从YAML追到物理字段；不使用宏或成员指针注册框架。
    if (name == "max_speed_mps")
    {
      config.motion.v_max_mps = parameter.as_double();
    }
    else if (name == "max_acceleration_mps2")
    {
      config.motion.a_max_mps2 = parameter.as_double();
    }
    else if (name == "mass_kg")
    {
      config.mass_kg = parameter.as_double();
    }
    else if (name == "surge_drag_Nspm")
    {
      config.surge_drag_Nspm = parameter.as_double();
    }
    else if (name == "position_kp_Npm")
    {
      config.position_kp_Npm = parameter.as_double();
    }
    else if (name == "velocity_kd_Nspm")
    {
      config.velocity_kd_Nspm = parameter.as_double();
    }
    else if (name == "lookahead_m")
    {
      config.lookahead_m = parameter.as_double();
    }
    else if (name == "max_force_N")
    {
      config.max_force_N = parameter.as_double();
    }
    else if (name == "arrival_radius_m")
    {
      config.arrival_radius_m = parameter.as_double();
    }
    else if (name == "arrival_speed_mps")
    {
      config.arrival_speed_mps = parameter.as_double();
    }
    else if (name == "arrival_dwell_s")
    {
      config.arrival_dwell_s = parameter.as_double();
    }
    else if (name == "reacquire_radius_m")
    {
      config.reacquire_radius_m = parameter.as_double();
    }
    else if (name == "mission_timeout_s")
    {
      config.mission_timeout_s = parameter.as_double();
    }
    else if (name == "max_actual_speed_mps")
    {
      limits.max_actual_speed_mps = parameter.as_double();
    }
    else if (name == "max_tilt_rad")
    {
      limits.max_tilt_rad = parameter.as_double();
    }
    else if (name == "max_angular_speed_radps")
    {
      limits.max_angular_speed_radps = parameter.as_double();
    }
    else if (name == "max_depth_drift_m")
    {
      limits.max_depth_drift_m = parameter.as_double();
    }
    else if (name == "max_state_age_s")
    {
      limits.max_state_age_s = parameter.as_double();
    }
    else if (name == "max_receive_gap_s")
    {
      limits.max_receive_gap_s = parameter.as_double();
    }
    else if (name == "max_ros_step_s")
    {
      limits.max_ros_step_s = parameter.as_double();
    }
    else if (name == "max_thruster_command")
    {
      limits.max_thruster_command = parameter.as_double();
    }
  }
}

PointTrackingNode::Status
PointTrackingNode::validateParameters(const PlanarConfig &config,
                                      const TrackingRuntimeLimits &limits) const
{
  const auto control = PlanarTracker::validateConfig(config);
  if (!control.ok())
  {
    return control;
  }
  const auto actuator = HippoAllocator::validateMaxCommand(limits.max_thruster_command);
  if (!actuator.ok())
  {
    return actuator;
  }
  for (const auto &[name, value] :
       {std::pair<const char *, double>{"max_actual_speed_mps", limits.max_actual_speed_mps},
        {"max_tilt_rad", limits.max_tilt_rad},
        {"max_angular_speed_radps", limits.max_angular_speed_radps},
        {"max_depth_drift_m", limits.max_depth_drift_m}})
  {
    if (!std::isfinite(value) || value < 0)
    {
      return {ErrorCode::INVALID_LIMITS, std::string(name) + "=" + std::to_string(value) +
                                             " 必须为有限非负数，0表示关闭该项监测"};
    }
  }
  for (const auto &[name, value] :
       {std::pair<const char *, double>{"max_state_age_s", limits.max_state_age_s},
        {"max_receive_gap_s", limits.max_receive_gap_s},
        {"max_ros_step_s", limits.max_ros_step_s}})
  {
    if (!std::isfinite(value) || value <= 0)
    {
      return {ErrorCode::INVALID_LIMITS,
              std::string(name) + "=" + std::to_string(value) + " 必须为有限正秒数"};
    }
  }
  if (limits.max_actual_speed_mps > 0 && limits.max_actual_speed_mps <= config.motion.v_max_mps)
  {
    return {ErrorCode::INVALID_LIMITS,
            "max_actual_speed_mps=" + std::to_string(limits.max_actual_speed_mps) +
                " 必须大于 max_speed_mps=" + std::to_string(config.motion.v_max_mps) +
                "，或设为0关闭实际速度监测"};
  }
  if (limits.max_tilt_rad > 3.141592653589793)
  {
    return {ErrorCode::INVALID_LIMITS, "max_tilt_rad 是机体z轴与世界z轴夹角，必须位于[0,pi]"};
  }
  return {};
}

rcl_interfaces::msg::SetParametersResult
PointTrackingNode::onParameters(const std::vector<rclcpp::Parameter> &parameters)
{
  auto candidate = config_;
  auto runtime = limits_;
  rcl_interfaces::msg::SetParametersResult result;
  try
  {
    readParameterValues(parameters, candidate, runtime);
    const auto status = validateParameters(candidate, runtime);
    result.successful = status.ok();
    result.reason = status.message;
  }
  catch (const std::exception &error)
  {
    result.successful = false;
    result.reason = error.what();
  }
  return result;
}

void PointTrackingNode::applyParameters(const std::vector<rclcpp::Parameter> &parameters)
{
  // 单线程executor内执行，下一次控制回调只能看到提交完的整份参数。
  readParameterValues(parameters, config_, limits_);
  tracker_->setConfig(config_);
  allocator_.setMaxCommand(limits_.max_thruster_command);
  logConfig();
}

void PointTrackingNode::logConfig() const
{
  RCLCPP_INFO(get_logger(),
              "生效参数：参考v<=%.3f m/s, a<=%.3f m/s²; 实际速度监测=%.3f m/s(0关闭); "
              "Fx<=%.3f N, M=[%.3f,%.3f,%.3f] N·m; 电机指令<=%.3f。运动参数用于下一段轨迹",
              config_.motion.v_max_mps, config_.motion.a_max_mps2, limits_.max_actual_speed_mps,
              config_.max_force_N, config_.max_torque_Nm.x(), config_.max_torque_Nm.y(),
              config_.max_torque_Nm.z(), limits_.max_thruster_command);
}
} // namespace rov_control_ros

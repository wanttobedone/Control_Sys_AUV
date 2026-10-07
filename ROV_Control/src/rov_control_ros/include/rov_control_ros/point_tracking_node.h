/**
 * ROS 接线与运行管理：goal(home)→世界目标→PlanarTracker→HippoAllocator→四路电机。
 * 仅用于无上游mixer的Hippo仿真启动入口，拒绝多个控制发布源。
 * 数学计算在core；此处负责时间、状态、TF、显示、服务和诊断。
 */
#ifndef ROV_CONTROL_ROS_POINT_TRACKING_NODE_H_
#define ROV_CONTROL_ROS_POINT_TRACKING_NODE_H_
#include "rov_control_core/tracking/hippo_allocator.h"
#include "rov_control_core/tracking/planar_tracker.h"
#include "rov_control_ros/state_adapter.h"
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ustrov_control_msgs/msg/actuator_controls.hpp>

namespace rov_control_ros
{
// 运行策略与数学控制参数分开。速度/角速度/倾斜/深度漂移为0时关闭该项监测；
// 时间有效性和执行器接口范围始终检查。所有值由参数入口统一校验。
struct TrackingRuntimeLimits
{
  double max_actual_speed_mps = 1.0;
  double max_tilt_rad = 0.4;
  double max_angular_speed_radps = 2.0;
  double max_depth_drift_m = 0.65;
  double max_state_age_s = 0.25;
  double max_receive_gap_s = 0.35;
  double max_ros_step_s = 0.15;
  double max_thruster_command = 0.4;
};

class PointTrackingNode : public rclcpp::Node
{
public:
  explicit PointTrackingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void prepareShutdown(); // 在context有效时发零，main继续处理0.6秒以完成补发
private:
  using Steady = std::chrono::steady_clock;
  using Trigger = std_srvs::srv::Trigger;
  using RobotState = rov_control_core::RobotState;
  using Status = rov_control_core::Status;
  // 以下回调同一executor串行执行；最新状态、轨迹和发送命令不会被另一线程改写。
  // 构造时读取 YAML/launch 参数并校验；返回纯数学配置，同时保存 frame 和监测阈值。
  rov_control_core::PlanarConfig loadConfig();
  // 参数事务：先在副本中读值和校验；ROS正式提交成功后才更新实际控制对象。
  // pre回调不能改变运行状态，否则后续拒绝事务时会留下半套新参数。
  void readParameterValues(const std::vector<rclcpp::Parameter> &parameters,
                           rov_control_core::PlanarConfig &config,
                           TrackingRuntimeLimits &limits) const;
  Status validateParameters(const rov_control_core::PlanarConfig &config,
                            const TrackingRuntimeLimits &limits) const;
  rcl_interfaces::msg::SetParametersResult
  onParameters(const std::vector<rclcpp::Parameter> &parameters);
  void applyParameters(const std::vector<rclcpp::Parameter> &parameters);
  void logConfig() const; // 启动/更新后打印实际生效值，不把修改文件误报成已更新节点
  // 输入上游 Odometry：检查 frame/时间/有限值，转换机体系速度为世界系速度。
  // 首次成功时固定 home；此后只更新 state_/received_，另发 home 表达的显示里程计。
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
  // 输入 home 坐标的水平目标；检查 z=0和状态后转换到世界坐标。任务目标不设消息过期。
  // setGoal 成功才取得输出权限、递增任务号；拒绝的目标和重复目标不重启原任务。
  void onGoal(geometry_msgs::msg::PointStamped::ConstSharedPtr message);
  // wall 20 ms 调用：检查时钟/状态/多源 → update → allocate → sendMotors。
  // 参考推进使用 ROS dt，暂停时仍能按 wall 时间检测故障并补发有限零命令。
  void onTimer();
  // Trigger stop：立即取消任务并归零。它不能清除已经锁存的 FAULT。
  void onStop(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  // Trigger reset_fault：等归零结束，清状态/时间历史但保留 home；不恢复旧任务。
  void onReset(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response);
  Status checkGraph() const; // 只检测实际电机话题多源；订阅者发现/其他话题不构成控制许可
  // 读取快照，不修改状态；返回时效、活动盒、速度/倾斜和深度漂移是否允许控制。
  Status checkState(const rclcpp::Time &rosNow, Steady::time_point wallNow) const;
  // 消息接口边界检查：有限值、归一化[-1,1]；电机业务限幅只在allocator执行。
  void sendMotors(const std::array<double, 8> &command);
  // 所有停止原因共用：reset算法、保留故障、立即零输出并设置0.5 s wall补发截止时刻。
  void stopOutput(const std::string &reason, bool fault);
  void publishHome(); // home一旦建立便固定；reset_fault不会重设原点
  // 降至10 Hz显示；轨迹101点，实际轨迹有限缓存。显示消息不回流参与控制计算。
  void publishDisplay(const rov_control_core::TrackingOutput &output);
  // 发布阶段、拒绝原因、误差/时序/控制量；只在阶段变化时额外写终端日志。
  void publishDiagnostics();

  rov_control_core::PlanarConfig config_;
  TrackingRuntimeLimits limits_;
  std::unique_ptr<rov_control_core::PlanarTracker> tracker_;
  rov_control_core::HippoAllocator allocator_;
  std::unique_ptr<StateAdapter> adapter_;
  std::optional<RobotState> state_;                 // 最近一次通过适配器检查的完整快照
  std::optional<rov_control_core::HomeFrame> home_; // 固定原点，不随新目标或故障复位移动
  std::optional<rclcpp::Time> lastRos_;
  std::optional<Steady::time_point> received_;
  Steady::time_point clockAdvanced_ = Steady::now();
  Steady::time_point nextGraph_ = Steady::now();
  Steady::time_point nextDisplay_ = Steady::now();
  std::optional<Steady::time_point> zeroUntil_;
  Status graphStatus_{rov_control_core::ErrorCode::INVALID_ARGUMENT, "等待ROS图发现"};
  bool clockSeen_ = false;
  bool fault_ = false;
  bool owned_ = false; // 接受目标后才拥有输出权限；归零补发完成后释放
  bool shuttingDown_ = false;
  bool pathPublished_ = false;
  std::string worldFrame_, bodyFrame_, homeFrame_;
  std::string reason_ = "等待有效状态";
  std::string lastPhase_;
  std::uint64_t goalId_ = 0, rejectedGoals_ = 0;
  double lastStep_s_ = 0, maxStep_s_ = 0, compute_ms_ = 0;
  double maxError_m_ = 0, errorSquares_ = 0; // 只累计TRACKING阶段的水平参考误差
  std::uint64_t errorSamples_ = 0;
  rov_control_core::TrackingOutput output_;
  rov_control_core::AllocationOutput allocation_;
  nav_msgs::msg::Path actualPath_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalSub_;
  rclcpp::Publisher<ustrov_control_msgs::msg::ActuatorControls>::SharedPtr motorsPub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_, actualPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localOdomPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr referencePub_, homePub_, goalPub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> homeTf_;
  rclcpp::Service<Trigger>::SharedPtr stopService_, resetService_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterValidation_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameterApplication_;
};
} // namespace rov_control_ros
#endif

// 正式ROS接口回归：C++假bridge只接收电机命令；不把它当Gazebo动力学验证。
#include "rov_control_ros/point_tracking_node.h"
#include <gtest/gtest.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <thread>

class PointInterface : public ::testing::Test
{
protected:
  // executor是成员，在SetUp前构造；因此ROS context必须在创建fixture前初始化。
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    static int number = 0;
    name_ = "point_test_" + std::to_string(++number);
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + name_});
    options.parameter_overrides({{"use_sim_time", true},
                                 {"max_speed_mps", 0.8},
                                 {"max_acceleration_mps2", 0.6},
                                 {"max_actual_speed_mps", 1.0},
                                 {"body_frame", name_ + "/base_link"},
                                 {"home_frame", name_ + "/home"}});
    node_ = std::make_shared<rov_control_ros::PointTrackingNode>(options);
    bridge_ = std::make_shared<rclcpp::Node>("bridge", "/" + name_);
    clock_ = bridge_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    odom_ = bridge_->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS());
    goal_ = bridge_->create_publisher<geometry_msgs::msg::PointStamped>("goal", 1);
    motors_ = bridge_->create_subscription<ustrov_control_msgs::msg::ActuatorControls>(
        "thruster_command", 10,
        [this](ustrov_control_msgs::msg::ActuatorControls::ConstSharedPtr msg)
        {
          last_ = msg->control;
          ++messages_;
        });
    diag_ = bridge_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "point_tracking/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
        {
          for (const auto &s : msg->status)
          {
            for (const auto &v : s.values)
            {
              if (v.key == "state")
              {
                phase_ = v.value;
              }
            }
          }
        });
    executor_.add_node(node_);
    executor_.add_node(bridge_);
    for (int i = 0; i < 100 && phase_ != "READY"; ++i)
    {
      advance(0.02);
    }
    ASSERT_EQ(phase_, "READY");
  }
  void TearDown() override
  {
    node_->prepareShutdown();
    advance(0.6);
    executor_.remove_node(node_);
    executor_.remove_node(bridge_);
    node_.reset();
    bridge_.reset();
  }
  void advance(double duration)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
    while (std::chrono::steady_clock::now() < end)
    {
      const auto wall = std::chrono::steady_clock::now();
      if (feed_ && wall >= next_)
      {
        next_ = wall + std::chrono::milliseconds(20);
        stamp_ += 20000000;
        rosgraph_msgs::msg::Clock clock;
        clock.clock = rclcpp::Time(stamp_);
        clock_->publish(clock);
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = clock.clock;
        odom.header.frame_id = badFrame_ ? "wrong_map" : "map";
        odom.child_frame_id = name_ + "/base_link";
        odom.pose.pose.position.x = 1.3;
        odom.pose.pose.position.y = 1;
        odom.pose.pose.position.z = -0.2;
        odom.pose.pose.orientation.w = 1;
        odom.twist.twist.linear.x = speed_;
        odom_->publish(odom);
      }
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  void goal(double x, double y, double z = 0, const std::string &frame = "")
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = frame.empty() ? name_ + "/home" : frame;
    msg.header.stamp.sec = 1; // 明显早于当前仿真时钟：静态任务目标不应按2秒过期。
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = z;
    goal_->publish(msg);
    advance(0.2);
  }
  bool call(const std::string &service)
  {
    auto client = bridge_->create_client<std_srvs::srv::Trigger>("point_tracking/" + service);
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    for (int i = 0; i < 100; ++i)
    {
      advance(0.01);
      if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        return future.get()->success;
      }
    }
    return false;
  }
  void expectZero()
  {
    for (double v : last_)
    {
      EXPECT_EQ(v, 0);
    }
  }
  std::string name_, phase_;
  std::shared_ptr<rov_control_ros::PointTrackingNode> node_;
  rclcpp::Node::SharedPtr bridge_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_;
  rclcpp::Subscription<ustrov_control_msgs::msg::ActuatorControls>::SharedPtr motors_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  std::array<double, 8> last_{};
  int messages_ = 0;
  bool feed_ = true;
  bool badFrame_ = false;
  double speed_ = 0.0;
  std::int64_t stamp_ = 10000000000LL;
  std::chrono::steady_clock::time_point next_{};
};
TEST_F(PointInterface, OnlyValidHorizontalHomeGoalStartsControl)
{
  EXPECT_EQ(messages_, 0);
  goal(0.2, 0, 0.1);
  EXPECT_EQ(messages_, 0);
  goal(0.2, 0, 0, "map");
  EXPECT_EQ(messages_, 0);
  goal(100, 0);
  EXPECT_EQ(messages_, 0);
  goal(0.2, 0);
  EXPECT_GT(messages_, 0);
  EXPECT_TRUE(call("stop"));
  advance(0.65);
  expectZero();
  EXPECT_EQ(phase_, "READY");
  const int count = messages_;
  advance(0.15);
  EXPECT_EQ(messages_, count);
}
TEST_F(PointInterface, PausedClockLatchesFaultAndResetRequiresNewGoal)
{
  goal(0.2, 0);
  advance(0.8);
  feed_ = false;
  advance(0.95);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
  feed_ = true;
  advance(0.2);
  EXPECT_EQ(phase_, "FAULT");
  ASSERT_TRUE(call("reset_fault"));
  advance(0.2);
  EXPECT_EQ(phase_, "READY");
  const int count = messages_;
  advance(0.2);
  EXPECT_EQ(messages_, count);
}
TEST_F(PointInterface, CompetingMotorPublisherAborts)
{
  goal(0.2, 0);
  auto other =
      bridge_->create_publisher<ustrov_control_msgs::msg::ActuatorControls>("thruster_command", 1);
  advance(1.2); // 多源检查1Hz，不用100ms轮询整个ROS图。
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(PointInterface, ParameterTransactionsAreAtomicAndRuntimeLimitIsConfigurable)
{
  // 参考0.8已经在SetUp成功启动。事务中的第二个值冲突时，第一个值也不得偷偷生效。
  const auto rejected = node_->set_parameters_atomically(
      {rclcpp::Parameter("max_force_N", 8.0), rclcpp::Parameter("max_actual_speed_mps", 0.5)});
  EXPECT_FALSE(rejected.successful);
  EXPECT_NE(rejected.reason.find("max_actual_speed_mps"), std::string::npos);
  EXPECT_NE(rejected.reason.find("max_speed_mps"), std::string::npos);
  EXPECT_DOUBLE_EQ(node_->get_parameter("max_force_N").as_double(), 1.5);
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("position_kp_Npm", 0.0)).successful);
  // 用接口假状态产生确定饱和，验证热更新确实传到分配器且发送端不再卡在旧0.4。
  // 这些是回归试验值，不是Gazebo/实机调参建议。
  ASSERT_TRUE(node_
                  ->set_parameters_atomically({rclcpp::Parameter("max_thruster_command", 0.6),
                                               rclcpp::Parameter("max_force_N", 100.0),
                                               rclcpp::Parameter("velocity_kd_Nspm", 100.0)})
                  .successful);
  // 实际0.7高于旧硬编码0.5，但低于配置1.0，应继续运行。
  speed_ = 0.7;
  advance(0.1);
  goal(0.2, 0);
  EXPECT_NE(phase_, "FAULT");
  double peak = 0;
  for (double command : last_)
  {
    peak = std::max(peak, std::abs(command));
  }
  EXPECT_NEAR(peak, 0.6, 1e-9);
  // 同时降低参考上限和监测阈值，提交后checkState必须使用新值0.65。
  ASSERT_TRUE(node_
                  ->set_parameters_atomically({rclcpp::Parameter("max_speed_mps", 0.4),
                                               rclcpp::Parameter("max_actual_speed_mps", 0.65)})
                  .successful);
  advance(0.15);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(PointInterface, BadFrameIsDroppedAndOnlyPersistentLossFaults)
{
  goal(0.2, 0);
  badFrame_ = true;
  advance(0.04);
  badFrame_ = false;
  advance(0.12);
  EXPECT_NE(phase_, "FAULT");
  badFrame_ = true;
  advance(0.5);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(PointInterface, GraphDiscoveryAndUnconnectedSetpointDoNotGateController)
{
  motors_.reset(); // 没有执行器订阅者不等于配置错误；topic pub也不要求发现订阅者。
  auto unrelated =
      bridge_->create_publisher<ustrov_control_msgs::msg::ActuatorControls>("thrust_setpoint", 1);
  goal(0.2, 0);
  advance(0.8);
  EXPECT_EQ(phase_, "TRACKING");
  EXPECT_TRUE(call("stop"));
}

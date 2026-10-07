/**
 * 使用真实 ROS 发布/订阅/服务和一个 C++ 假 mixer 验证节点行为。
 * 假 mixer 只检查接线与时序，不模拟推力；运动方向另外在 Gazebo 验证。
 * 每个测试使用不同 namespace，避免 DDS 尚未清理的端点互相干扰。
 */
#include "rov_control_ros/actuation_probe_node.h"
#include <gtest/gtest.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <thread>

using namespace rov_control_ros;
using Setpoint = ustrov_control_msgs::msg::ActuatorSetpoint;
using Motors = ustrov_control_msgs::msg::ActuatorControls;
using Trigger = std_srvs::srv::Trigger;

class ProbeIntegration : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
  void SetUp() override
  {
    static int number = 0;
    vehicle_ = "probe_test_" + std::to_string(++number);
    ns_ = "/" + vehicle_;
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true),
                                 rclcpp::Parameter("body_frame", vehicle_ + "/base_link"),
                                 rclcpp::Parameter("amplitude", 0.6)});
    probe_ = std::make_shared<ActuationProbeNode>(options);
    rclcpp::NodeOptions peerOptions;
    peerOptions.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    mixer_ = std::make_shared<rclcpp::Node>("actuator_command_mixer", ns_, peerOptions);
    clockPub_ = mixer_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    odomPub_ =
        mixer_->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS());
    motorPub_ = mixer_->create_publisher<Motors>("thruster_command", rclcpp::SensorDataQoS());
    thrustSub_ = mixer_->create_subscription<Setpoint>("thrust_setpoint", rclcpp::SensorDataQoS(),
                                                       [this](Setpoint::ConstSharedPtr msg)
                                                       {
                                                         thrust_ = *msg;
                                                         ++pairs_;
                                                         if (std::abs(msg->x) > 1e-9)
                                                         {
                                                           ++nonzero_;
                                                         }
                                                         publishMotors();
                                                       });
    torqueSub_ = mixer_->create_subscription<Setpoint>("torque_setpoint", rclcpp::SensorDataQoS(),
                                                       [this](Setpoint::ConstSharedPtr msg)
                                                       {
                                                         torque_ = *msg;
                                                         publishMotors();
                                                       });
    diagnosticSub_ = mixer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "actuation_probe/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
        {
          for (const auto &entry : msg->status)
          {
            reason_ = entry.message;
            for (const auto &field : entry.values)
            {
              if (field.key == "state")
              {
                phase_ = field.value;
              }
            }
          }
        });
    executor_.add_node(probe_);
    executor_.add_node(mixer_);
    for (int i = 0; i < 100 && phase_ != "READY"; ++i)
    {
      advance(0.02);
    }
    ASSERT_EQ(phase_, "READY") << reason_;
  }
  void TearDown() override
  {
    probe_->prepareShutdown();
    advance(0.55);
    executor_.remove_node(probe_);
    executor_.remove_node(mixer_);
    probe_.reset();
    mixer_.reset();
  }
  void publishMotors()
  {
    // 明确展开当前 Hippo 矩阵，验证节点两类 setpoint 能进入同一缓存组合。
    Motors out;
    out.header.stamp = rclcpp::Time(rosNs_, RCL_ROS_TIME);
    out.control[0] = 0.25 * (torque_.x - torque_.y + torque_.z - thrust_.x);
    out.control[1] = 0.25 * (torque_.x + torque_.y + torque_.z + thrust_.x);
    out.control[2] = 0.25 * (torque_.x + torque_.y - torque_.z - thrust_.x);
    out.control[3] = 0.25 * (torque_.x - torque_.y - torque_.z + thrust_.x);
    motorPub_->publish(out);
  }
  void advance(double duration)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
    while (std::chrono::steady_clock::now() < until)
    {
      const auto current = std::chrono::steady_clock::now();
      if (current >= nextFeed_)
      {
        nextFeed_ = current + std::chrono::milliseconds(20);
        if (feedClock_)
        {
          rosNs_ += 20000000;
          rosgraph_msgs::msg::Clock clock;
          clock.clock = rclcpp::Time(rosNs_, RCL_ROS_TIME);
          clockPub_->publish(clock);
        }
        if (feedState_)
        {
          nav_msgs::msg::Odometry state;
          state.header.stamp = rclcpp::Time(rosNs_, RCL_ROS_TIME);
          state.header.frame_id = badFrame_ ? "wrong" : "map";
          state.child_frame_id = vehicle_ + "/base_link";
          state.pose.pose.position.x = 1.3;
          state.pose.pose.position.y = 1.0;
          state.pose.pose.position.z = -0.2;
          state.pose.pose.orientation.w = 1.0;
          odomPub_->publish(state);
        }
      }
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  Trigger::Response::SharedPtr call(const std::string &service)
  {
    auto client = mixer_->create_client<Trigger>("actuation_probe/" + service);
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return nullptr;
    }
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    for (int i = 0; i < 100; ++i)
    {
      advance(0.01);
      if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        return future.get();
      }
    }
    return nullptr;
  }
  void start()
  {
    const auto response = call("start");
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
  }
  void expectZero()
  {
    EXPECT_DOUBLE_EQ(thrust_.x, 0.0);
    EXPECT_DOUBLE_EQ(thrust_.y, 0.0);
    EXPECT_DOUBLE_EQ(thrust_.z, 0.0);
    EXPECT_DOUBLE_EQ(torque_.x, 0.0);
    EXPECT_DOUBLE_EQ(torque_.y, 0.0);
    EXPECT_DOUBLE_EQ(torque_.z, 0.0);
  }

  std::string vehicle_, ns_, phase_, reason_;
  std::shared_ptr<ActuationProbeNode> probe_;
  rclcpp::Node::SharedPtr mixer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::Publisher<Motors>::SharedPtr motorPub_;
  rclcpp::Subscription<Setpoint>::SharedPtr thrustSub_, torqueSub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticSub_;
  Setpoint thrust_, torque_;
  std::int64_t rosNs_ = 10000000000LL;
  int nonzero_ = 0;
  int pairs_ = 0;
  bool feedClock_ = true;
  bool feedState_ = true;
  bool badFrame_ = false;
  std::chrono::steady_clock::time_point nextFeed_{};
};

TEST_F(ProbeIntegration, ExplicitStartAndFinitePulse)
{
  advance(0.2);
  EXPECT_EQ(pairs_, 0); // 节点创建发布器，不代表已经发送任何命令。
  start();
  advance(3.5);
  EXPECT_GT(nonzero_, 0);
  EXPECT_EQ(phase_, "FINISHED") << reason_;
  expectZero();
  const int stoppedCount = pairs_;
  advance(0.3);
  EXPECT_EQ(pairs_, stoppedCount); // 有限零补发结束后不永久刷新下游看门狗。
}

TEST_F(ProbeIntegration, StopInterruptsNonzeroPulse)
{
  start();
  advance(1.25);
  ASSERT_GT(nonzero_, 0);
  const auto response = call("stop");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  advance(0.65);
  expectZero();
  EXPECT_EQ(phase_, "READY");
}

TEST_F(ProbeIntegration, PauseLatchesFaultAndResetDoesNotRestart)
{
  start();
  advance(1.25);
  feedClock_ = feedState_ = false;
  advance(0.65);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
  feedClock_ = feedState_ = true;
  advance(0.2);
  EXPECT_EQ(phase_, "FAULT");
  const auto response = call("reset_fault");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  const int previousNonzero = nonzero_;
  advance(0.4);
  EXPECT_EQ(phase_, "READY");
  EXPECT_EQ(nonzero_, previousNonzero);
}

TEST_F(ProbeIntegration, MissingStateStopsWhileClockContinues)
{
  start();
  advance(1.2);
  feedState_ = false;
  advance(0.4);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(ProbeIntegration, PersistentBadFramesTimeout)
{
  start();
  badFrame_ = true;
  advance(0.3); // 坏帧不刷新有效接收时间，持续超过配置的0.15秒才故障。
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(ProbeIntegration, TimeRollbackStopsTrial)
{
  start();
  advance(1.2);
  rosNs_ -= 1000000000LL;
  advance(0.15);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(ProbeIntegration, SecondCommandPublisherAbortsTrial)
{
  start();
  auto competitor = mixer_->create_publisher<Setpoint>("thrust_setpoint", 1);
  advance(0.3);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

TEST_F(ProbeIntegration, InvalidMixerTimestampDoesNotEscapeCallback)
{
  start();
  Motors invalid;
  invalid.header.stamp.sec = -1;
  motorPub_->publish(invalid);
  advance(0.1);
  EXPECT_EQ(phase_, "FAULT");
  expectZero();
}

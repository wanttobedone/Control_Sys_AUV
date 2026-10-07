/**
 * 验证真实 ROS 2 接线：MotorProbeNode → 未修改的 USTROV bridge → C++ 假 PX4。
 * 假 PX4 只应答模式/解锁命令并广播 VehicleStatus，不模拟电机、ESC 或水动力。
 * CTest 将全部节点限制在 localhost、ROS_DOMAIN_ID=124；没有 Agent/串口。
 * 不把下面的“电机输出数组正确”表述为推进器已经实际转动。
 */
#include "rov_control_ros/motor_probe_node.h"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <future>
#include <gtest/gtest.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char **environ;
using namespace std::chrono_literals;
using rov_control_ros::MotorProbeConfig;
using rov_control_ros::MotorProbeNode;
using Trigger = std_srvs::srv::Trigger;
using Status = px4_msgs::msg::VehicleStatus;
using Motors = px4_msgs::msg::ActuatorMotors;
using Input = ustrov_control_msgs::msg::ActuatorControls;
using Command = px4_msgs::msg::VehicleCommand;

TEST(MotorPulse, ExactBoundariesAndOnlySelectedChannel)
{
  MotorProbeConfig config;
  config.channel = 3;
  config.amplitude = -0.4;
  config.ramp_s = 0.5;
  config.hold_s = 0.25;
  // 总时间 1.25s；0.25s 在上升段中点，1.0s 在下降段中点。
  for (const auto &[time, expected] : std::array<std::pair<double, double>, 7>{{{-1.0, 0.0},
                                                                                {0.0, 0.0},
                                                                                {0.25, -0.2},
                                                                                {0.5, -0.4},
                                                                                {0.75, -0.4},
                                                                                {1.0, -0.2},
                                                                                {1.25, 0.0}}})
  {
    const auto command = MotorProbeNode::makeCommand(config, time);
    for (std::size_t i = 0; i < command.size(); ++i)
    {
      EXPECT_NEAR(command[i], i == 3 ? expected : 0.0, 1e-12);
    }
  }
  config.amplitude = 0.0;
  EXPECT_EQ(MotorProbeNode::makeCommand(config, 0.5), (std::array<double, 8>{}));
  config.amplitude = 0.3;
  config.hold_s = 0.0; // 无平台时退化为三角形，峰值仍然等于设定幅值。
  EXPECT_DOUBLE_EQ(MotorProbeNode::makeCommand(config, 0.5)[3], 0.3);
}

class MotorIntegration : public ::testing::Test
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
    // 即使维护者直接运行本二进制，也必须显式选择隔离域后才启动假 PX4。
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    const char *range = std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
    ASSERT_TRUE(domain && std::string(domain) == "124");
    ASSERT_TRUE(range && std::string(range) == "LOCALHOST");
    static int count = 0;
    ns_ = "/motor_test_" + std::to_string(getpid()) + "_" + std::to_string(++count);
    peer_ = std::make_shared<rclcpp::Node>("fake_px4", ns_);
    state_.arming_state = Status::ARMING_STATE_DISARMED;
    state_.pre_flight_checks_pass = true;
    statusPub_ = peer_->create_publisher<Status>(ns_ + "/fmu/out/vehicle_status_v1", 10);
    commandSub_ = peer_->create_subscription<Command>(
        ns_ + "/fmu/in/vehicle_command", rclcpp::SensorDataQoS(),
        [this](Command::ConstSharedPtr message)
        {
          if (message->command == Command::VEHICLE_CMD_DO_SET_MODE)
          {
            state_.nav_state = Status::NAVIGATION_STATE_OFFBOARD;
          }
          if (message->command == Command::VEHICLE_CMD_COMPONENT_ARM_DISARM)
          {
            if (message->param1 == 1.0F)
            {
              ++armRequests_;
              if (allowArm_)
              {
                state_.arming_state = Status::ARMING_STATE_ARMED;
              }
            }
            else
            {
              ++disarmRequests_;
              state_.arming_state = Status::ARMING_STATE_DISARMED;
              disarmTime_ = std::chrono::steady_clock::now();
            }
          }
        });
    motorsSub_ = peer_->create_subscription<Motors>(
        ns_ + "/fmu/in/actuator_motors", rclcpp::SensorDataQoS(),
        [this](Motors::ConstSharedPtr message)
        {
          lastMotors_ = *message;
          for (std::size_t i = 0; i < message->control.size(); ++i)
          {
            if (std::isfinite(message->control[i]))
            {
              peaks_[i] = std::max(peaks_[i], std::abs(message->control[i]));
              if (std::abs(message->control[i]) > 1e-6F)
              {
                ++nonzeroFrames_;
                EXPECT_EQ(state_.arming_state, Status::ARMING_STATE_ARMED);
              }
            }
          }
        });
    inputSub_ = peer_->create_subscription<Input>("thruster_command", rclcpp::SensorDataQoS(),
                                                  [this](Input::ConstSharedPtr message)
                                                  {
                                                    lastInput_ = *message;
                                                    lastInputTime_ =
                                                        std::chrono::steady_clock::now();
                                                  });
    diagnosticSub_ = peer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "motor_probe/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
        {
          for (const auto &status : message->status)
          {
            for (const auto &value : status.values)
            {
              if (value.key == "phase")
              {
                phase_ = value.value;
              }
            }
          }
        });
    executor_.add_node(peer_);
    start_ = peer_->create_client<Trigger>("motor_probe/start");
    stop_ = peer_->create_client<Trigger>("motor_probe/stop");

    // posix_spawn 直接运行已安装的真实桥程序，参数只改话题前缀，保留 1.2s/300ms。
    std::vector<std::string> args = {PX4_BRIDGE_EXECUTABLE,
                                     "--ros-args",
                                     "-r",
                                     "__ns:=" + ns_,
                                     "-p",
                                     "px4_topic_prefix:=" + ns_ + "/fmu",
                                     "-p",
                                     "vehicle_status_topic:=" + ns_ + "/fmu/out/vehicle_status_v1"};
    std::vector<char *> argv;
    for (auto &arg : args)
    {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    ASSERT_EQ(
        posix_spawn(&bridgePid_, PX4_BRIDGE_EXECUTABLE, nullptr, nullptr, argv.data(), environ), 0);
    createProbe();
    ASSERT_TRUE(until(
        [this]
        { return start_->service_is_ready() && peer_->count_subscribers("thruster_command") >= 2; },
        5.0));
    pump(0.4); // 留出命令/状态接收和 DDS 图发现时间；不是修改桥的预流门限。
  }
  void createProbe(double amplitude = 0.2)
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    options.parameter_overrides(
        {rclcpp::Parameter("amplitude", amplitude), rclcpp::Parameter("channel", 2),
         rclcpp::Parameter("ramp_s", 0.15), rclcpp::Parameter("hold_s", 0.25),
         rclcpp::Parameter("arm_timeout_s", 2.3),
         rclcpp::Parameter("vehicle_status_topic", ns_ + "/fmu/out/vehicle_status_v1"),
         rclcpp::Parameter("actuator_motors_topic", ns_ + "/fmu/in/actuator_motors")});
    probe_ = std::make_shared<MotorProbeNode>(options);
    executor_.add_node(probe_);
  }
  void stopBridge()
  {
    if (bridgePid_ <= 0)
    {
      return;
    }
    kill(bridgePid_, SIGTERM);
    for (int i = 0; i < 100; ++i)
    {
      if (waitpid(bridgePid_, nullptr, WNOHANG) == bridgePid_)
      {
        bridgePid_ = -1;
        return;
      }
      std::this_thread::sleep_for(10ms);
    }
    kill(bridgePid_, SIGKILL);
    waitpid(bridgePid_, nullptr, 0);
    bridgePid_ = -1;
  }
  void TearDown() override
  {
    if (probe_)
    {
      probe_->prepareShutdown();
      pump(0.3);
      executor_.remove_node(probe_);
      probe_.reset();
    }
    stopBridge();
    if (peer_)
    {
      executor_.remove_node(peer_);
    }
  }
  void tick()
  {
    state_.timestamp += 10000;
    statusPub_->publish(state_);
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  void pump(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end)
    {
      tick();
    }
  }
  bool until(const std::function<bool()> &predicate, double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!predicate() && std::chrono::steady_clock::now() < end)
    {
      tick();
    }
    return predicate();
  }
  bool call(const rclcpp::Client<Trigger>::SharedPtr &client)
  {
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (!until([&future] { return future.wait_for(0s) == std::future_status::ready; }, 2.0))
    {
      ADD_FAILURE() << "服务未返回";
      return false;
    }
    return future.get()->success;
  }
  bool allStopped() const
  {
    return state_.arming_state == Status::ARMING_STATE_DISARMED &&
           std::all_of(lastMotors_.control.begin(), lastMotors_.control.end(),
                       [](float value) { return std::isnan(value); });
  }

  std::string ns_;
  std::string phase_;
  pid_t bridgePid_ = -1;
  bool allowArm_ = true;
  int armRequests_ = 0;
  int disarmRequests_ = 0;
  int nonzeroFrames_ = 0;
  std::array<float, 12> peaks_{};
  Status state_{};
  Motors lastMotors_{};
  Input lastInput_{};
  std::chrono::steady_clock::time_point lastInputTime_;
  std::chrono::steady_clock::time_point disarmTime_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MotorProbeNode> probe_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<Status>::SharedPtr statusPub_;
  rclcpp::Subscription<Command>::SharedPtr commandSub_;
  rclcpp::Subscription<Motors>::SharedPtr motorsSub_;
  rclcpp::Subscription<Input>::SharedPtr inputSub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticSub_;
  rclcpp::Client<Trigger>::SharedPtr start_;
  rclcpp::Client<Trigger>::SharedPtr stop_;
};

TEST_F(MotorIntegration, IdleDoesNotArmAndFullPulseStops)
{
  EXPECT_EQ(armRequests_, 0);
  EXPECT_EQ(nonzeroFrames_, 0);
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return phase_ == "FINISHED"; }, 5.0));
  ASSERT_TRUE(allStopped());
  EXPECT_GT(nonzeroFrames_, 0);
  // 原桥前四路符号为 +1；输入 2 的平台幅值应原样进入逻辑 Motor3。
  EXPECT_NEAR(peaks_[2], 0.2F, 1e-6);
  for (std::size_t i = 0; i < peaks_.size(); ++i)
  {
    if (i != 2)
    {
      EXPECT_FLOAT_EQ(peaks_[i], 0.0F);
    }
  }
  const int arms = armRequests_;
  pump(0.4);
  EXPECT_EQ(armRequests_, arms); // 完成后不会自动重启。
}

TEST_F(MotorIntegration, AcceptedArmServiceDoesNotMeanArmed)
{
  allowArm_ = false;
  ASSERT_TRUE(call(start_));
  pump(1.7); // 超过原桥的 1.2s 预流，模拟 PX4 一直未通过解锁。
  EXPECT_GT(armRequests_, 0);
  EXPECT_EQ(phase_, "ARMING");
  EXPECT_EQ(nonzeroFrames_, 0);
  ASSERT_TRUE(until([this] { return phase_ == "FAULT"; }, 2.0));
  EXPECT_TRUE(allStopped());
  EXPECT_FALSE(call(start_)); // 故障不能用另一次 start 掩盖。
}

TEST_F(MotorIntegration, ZeroAmplitudeExercisesHandshakeWithoutNonzeroOutput)
{
  executor_.remove_node(probe_);
  probe_.reset();
  createProbe(0.0);
  pump(0.5);
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return phase_ == "FINISHED"; }, 5.0));
  EXPECT_GT(armRequests_, 0);
  EXPECT_EQ(nonzeroFrames_, 0);
  EXPECT_TRUE(allStopped());
}

TEST_F(MotorIntegration, StopDuringPrestreamNeverProducesPulse)
{
  ASSERT_TRUE(call(start_));
  pump(0.2);
  EXPECT_FALSE(call(start_)); // 重复 start 不重新计时。
  ASSERT_TRUE(call(stop_));
  ASSERT_TRUE(until([this] { return phase_ == "FINISHED"; }, 2.0));
  pump(1.3);
  EXPECT_EQ(nonzeroFrames_, 0);
  EXPECT_TRUE(allStopped());
}

TEST_F(MotorIntegration, StopDuringRunningDisarms)
{
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return nonzeroFrames_ > 0; }, 4.0));
  ASSERT_TRUE(call(stop_));
  ASSERT_TRUE(until([this] { return phase_ == "FINISHED"; }, 2.0));
  EXPECT_TRUE(allStopped());
  EXPECT_EQ(lastInput_.control, (std::array<double, 8>{}));
}

TEST_F(MotorIntegration, FailsafeTerminatesAndLatchesFault)
{
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return nonzeroFrames_ > 0; }, 4.0));
  state_.failsafe = true;
  ASSERT_TRUE(until([this] { return phase_ == "FAULT"; }, 2.0));
  EXPECT_TRUE(allStopped());
}

TEST_F(MotorIntegration, SecondCommandSourceIsRejected)
{
  auto other = peer_->create_publisher<Input>("thruster_command", 10);
  pump(0.3);
  EXPECT_FALSE(call(start_));
  EXPECT_EQ(armRequests_, 0);
}

TEST_F(MotorIntegration, ProbeDisappearsBridgeWatchdogDisarms)
{
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return nonzeroFrames_ > 0; }, 4.0));
  // 故意不调用 prepareShutdown，模拟发布进程骤停；原桥仍然真实运行。
  executor_.remove_node(probe_);
  probe_.reset();
  ASSERT_TRUE(until([this] { return disarmRequests_ > 0 && allStopped(); }, 1.5));
  const double gap = std::chrono::duration<double>(disarmTime_ - lastInputTime_).count();
  EXPECT_GT(gap, 0.20);
  EXPECT_LT(gap, 1.0); // 配置 300ms；放宽到1s仅容纳 CI 调度，不是修改运行保护。
  RecordProperty("watchdog_observed_s", gap);
}

TEST_F(MotorIntegration, SecondSourceDuringRunningStopsTheBridge)
{
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return nonzeroFrames_ > 0; }, 4.0));
  auto other = peer_->create_publisher<Input>("thruster_command", 10);
  ASSERT_TRUE(until([this] { return phase_ == "FAULT"; }, 2.0));
  EXPECT_TRUE(allStopped());
}

TEST_F(MotorIntegration, MissingBridgeCannotBeReportedAsFinished)
{
  ASSERT_TRUE(call(start_));
  ASSERT_TRUE(until([this] { return nonzeroFrames_ > 0; }, 4.0));
  stopBridge();
  ASSERT_TRUE(until([this] { return phase_ == "STOPPING"; }, 2.0));
  EXPECT_EQ(lastInput_.control, (std::array<double, 8>{}));
  EXPECT_FALSE(probe_->stopConfirmed());
}

TEST_F(MotorIntegration, InvalidConfigNeverCreatesCommandPublisher)
{
  const auto before = peer_->count_publishers("thruster_command");
  for (const auto &bad :
       {rclcpp::Parameter("channel", 8), rclcpp::Parameter("channel", int64_t{4294967296LL}),
        rclcpp::Parameter("amplitude", 1.1), rclcpp::Parameter("ramp_s", 0.0),
        rclcpp::Parameter("use_sim_time", true)})
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns_});
    options.parameter_overrides({bad});
    EXPECT_THROW(std::make_shared<MotorProbeNode>(options), std::invalid_argument);
  }
  EXPECT_EQ(peer_->count_publishers("thruster_command"), before);
}

#include "rov_state_estimation/fusion_node.h"

#include <gtest/gtest.h>
#include <cmath>
#include <thread>

namespace rs = rov_state_estimation;

TEST(FusionRos, EndToEndHomeResetFramesStampsAndImuStall)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/fusion_test"});
  options.parameter_overrides({rclcpp::Parameter("frame_prefix", "fusion_test")});
  auto node = std::make_shared<rs::FusionNode>(options);
  auto io = std::make_shared<rclcpp::Node>("fusion_test_io");
  auto imu_pub = io->create_publisher<sensor_msgs::msg::Imu>("/fusion_test/imu", 100);
  auto pressure_pub =
      io->create_publisher<sensor_msgs::msg::FluidPressure>("/fusion_test/pressure", 20);
  auto camera_pub =
      io->create_publisher<geometry_msgs::msg::PoseStamped>("/fusion_test/vision/camera_pose", 10);
  geometry_msgs::msg::PoseStamped latest_home;
  diagnostic_msgs::msg::DiagnosticArray latest_diag;
  size_t home_count = 0;
  auto home_sub = io->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/fusion_test/state/pose_home", 100,
      [&](geometry_msgs::msg::PoseStamped::SharedPtr p)
      {
        latest_home = *p;
        ++home_count;
      });
  auto diag_sub = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/fusion_test/state/diagnostics", 10,
      [&](diagnostic_msgs::msg::DiagnosticArray::SharedPtr p) { latest_diag = *p; });
  auto reset = io->create_client<std_srvs::srv::Trigger>("/fusion_test/state/reset_home");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io);
  const auto pump = [&](int milliseconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < end)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };
  const auto diagnostic_value = [&](const std::string &key)
  {
    if (!latest_diag.status.empty())
    {
      for (const auto &item : latest_diag.status.front().values)
      {
        if (item.key == key)
        {
          return item.value;
        }
      }
    }
    return std::string{};
  };
  pump(400);
  // 没有任何传感器数据时，仍能收到新版诊断；不能用零误差冒充已完成的计算。
  EXPECT_EQ(diagnostic_value("diagnostics_version"), "2");
  EXPECT_EQ(diagnostic_value("gate_check_stamp_ns"), "0");
  ASSERT_FALSE(diagnostic_value("position_error_m").empty());
  EXPECT_TRUE(std::isnan(std::stod(diagnostic_value("position_error_m"))));
  ASSERT_GT(imu_pub->get_subscription_count(), 0u);
  ASSERT_GT(camera_pub->get_subscription_count(), 0u);
  int step = 0;
  constexpr int64_t epoch = 1793700000000000000LL;
  const auto publish = [&](bool with_vision)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = "base_link";
    imu.header.stamp = rclcpp::Time(epoch + static_cast<int64_t>(step) * 10000000LL);
    imu.linear_acceleration.z = 9.81;
    imu.orientation_covariance[0] = -1;
    imu_pub->publish(imu);
    pump(1);
    if (step % 2 == 0)
    {
      sensor_msgs::msg::FluidPressure pressure;
      pressure.header = imu.header;
      pressure.fluid_pressure = 101325.0;
      pressure_pub->publish(pressure);
    }
    if (with_vision && step % 20 == 0)
    {
      geometry_msgs::msg::PoseStamped camera;
      camera.header = imu.header;
      camera.header.frame_id = "fusion_test/tag_16";
      // 与几何单测同一独立解析真值，机体 (0.1,0.2,0.4)，航向零。
      camera.pose.position.x = 0.05;
      camera.pose.position.y = -0.2;
      camera.pose.position.z = -0.275;
      camera.pose.orientation.z = 1.0;
      camera.pose.orientation.w = 0.0;
      camera_pub->publish(camera);
    }
    pump(1);
    ++step;
  };
  // 不放宽上游倾斜方差门槛；5Hz观测允许最多60s模拟时间完成对齐。
  bool observed_preinit_calculation = false;
  for (int i = 0; i < 6000 && home_count == 0; ++i)
  {
    publish(true);
    if (diagnostic_value("initialized") == "false" &&
        !diagnostic_value("filter_update_count").empty() &&
        std::stoull(diagnostic_value("filter_update_count")) > 0)
    {
      observed_preinit_calculation = true;
      EXPECT_EQ(diagnostic_value("output_count"), "0");
      EXPECT_FALSE(diagnostic_value("readiness_blocker").empty());
      EXPECT_TRUE(std::isfinite(std::stod(diagnostic_value("position_error_m"))));
    }
  }
  EXPECT_TRUE(observed_preinit_calculation);
  ASSERT_GT(home_count, 0u);
  RecordProperty("first_home_simulation_s", step * 0.01);
  EXPECT_EQ(latest_home.header.frame_id, "fusion_test/home");
  EXPECT_NEAR(latest_home.pose.position.x, 0.0, 0.04);
  EXPECT_EQ(rclcpp::Time(latest_home.header.stamp).nanoseconds(),
            epoch + (step - 1LL) * 10000000LL);
  ASSERT_TRUE(reset->wait_for_service(std::chrono::seconds(1)));
  auto future = reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(1)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);
  pump(10);
  const size_t reset_count = home_count;
  // 请求以后没有新视觉：仍有 IMU，但不能用旧缓存重建 home。
  for (int i = 0; i < 30; ++i)
  {
    publish(false);
  }
  EXPECT_EQ(home_count, reset_count);
  for (int i = 0; i < 80; ++i)
  {
    publish(true);
  }
  EXPECT_GT(home_count, reset_count);
  pump(20);
  const size_t stopped_count = home_count;
  pump(400);
  EXPECT_EQ(home_count, stopped_count);
  ASSERT_FALSE(latest_diag.status.empty());
  EXPECT_EQ(latest_diag.status.front().message, "IMU_RECEIVE_STALE");

  // 恢复接收，但源时间主动跳过200ms：应锁定源时间缺口，并通过ROS保留具体间隔。
  step += 20;
  publish(true);
  pump(300);
  ASSERT_EQ(latest_diag.status.front().message, "FAULT_IMU_GAP_RESTART_NODE");
  EXPECT_EQ(diagnostic_value("fault_reason"), "imu_gap_no_invented_integration");
  // publish 的 step 已比最后发送值前进1步，再跳20步，因此间隔为21*10ms。
  EXPECT_NEAR(std::stod(diagnostic_value("fault_imu_gap_s")), 0.21, 1e-9);
  const auto fault_stamp = diagnostic_value("fault_stamp_ns");
  const auto rejected_at_fault = diagnostic_value("rejected_inputs");
  const auto gate_stamp_at_fault = diagnostic_value("gate_check_stamp_ns");
  const size_t fault_home_count = home_count;
  step += 4000;
  // 对齐到20的倍数，确保本次明确发送压力和视觉，而不只发送IMU。
  step += (20 - step % 20) % 20;
  publish(true);
  pump(300);
  EXPECT_EQ(home_count, fault_home_count);
  EXPECT_EQ(diagnostic_value("fault_stamp_ns"), fault_stamp);
  EXPECT_EQ(diagnostic_value("gate_check_stamp_ns"), gate_stamp_at_fault);
  EXPECT_EQ(diagnostic_value("rejected_inputs"), rejected_at_fault);
  EXPECT_EQ(diagnostic_value("last_input_error"), "imu_gap_no_invented_integration");
  executor.remove_node(node);
  executor.remove_node(io);
}

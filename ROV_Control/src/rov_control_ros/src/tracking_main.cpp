// 正常退出先发零再shutdown；信号回调只置标志，不在信号上下文调用ROS。
#include "rov_control_ros/point_tracking_node.h"
#include <csignal>
#include <iostream>
#include <thread>
namespace
{
volatile std::sig_atomic_t exitRequested = 0;
void onSignal(int)
{
  exitRequested = 1;
}
} // namespace
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  int result = 0;
  try
  {
    auto node = std::make_shared<rov_control_ros::PointTrackingNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !exitRequested)
    {
      try
      {
        executor.spin_some();
      }
      catch (const std::exception &e)
      {
        std::cerr << e.what() << std::endl;
        result = 1;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (rclcpp::ok())
    {
      node->prepareShutdown();
      const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(650);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < end)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << "到点节点退出：" << e.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

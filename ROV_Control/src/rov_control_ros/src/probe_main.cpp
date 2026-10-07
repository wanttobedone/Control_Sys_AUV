// 正常 Ctrl+C/SIGTERM 先归零，再关闭 ROS context；强杀进程无法执行这条路径。
#include "rov_control_ros/actuation_probe_node.h"
#include <csignal>
#include <iostream>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopRequested = 0;
void onSignal(int)
{
  stopRequested = 1;
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
    auto node = std::make_shared<rov_control_ros::ActuationProbeNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && stopRequested == 0)
    {
      try
      {
        executor.spin_some();
      }
      catch (const std::exception &error)
      {
        // 先离开运行循环，让下面的归零路径仍在有效 ROS context 中执行。
        std::cerr << "回调异常，准备归零退出：" << error.what() << std::endl;
        result = 1;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (rclcpp::ok())
    {
      node->prepareShutdown();
      // 配置允许的归零补发最多 2 s，退出时最多继续处理 2.1 s。
      const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(2100);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }
  catch (const std::exception &error)
  {
    std::cerr << "输入试验节点退出：" << error.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

// main 仅管理进程生命周期；状态机在 MotorProbeNode 中，正常退出先争取完成上锁。
#include "rov_control_ros/motor_probe_node.h"
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
    auto node = std::make_shared<rov_control_ros::MotorProbeNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    try
    {
      while (rclcpp::ok() && stopRequested == 0)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    catch (const std::exception &error)
    {
      std::cerr << "回调异常，准备停止：" << error.what() << std::endl;
      result = 1;
    }
    node->prepareShutdown();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (rclcpp::ok() && !node->stopConfirmed() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!node->stopConfirmed())
    {
      std::cerr << "退出前未取得完整上锁确认，请检查 PX4；不能把进程退出当作电机已停。"
                << std::endl;
      result = 1;
    }
  }
  catch (const std::exception &error)
  {
    std::cerr << "逐电机试验节点退出：" << error.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

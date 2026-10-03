// 入口只负责 ROS 生命周期；公式和业务回调在对应类中。
#include <exception>
#include <iostream>
#include "rov_control_ros/trajectory_preview_node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    auto node = std::make_shared<rov_control_ros::TrajectoryPreviewNode>();
    // 单线程保证订阅、服务和 timer 不会并发读写内部快照。
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    std::cerr << "预览节点退出：" << error.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

#include "rov_localization/tag_pose_node.h"
#include <exception>
#include <iostream>

// 入口只负责ROS生命周期。算法在TagPoseEstimator，消息接线在TagPoseNode。
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<rov_localization::TagPoseNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &error)
  {
    std::cerr << "ROS只读定位启动/运行失败：" << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include "rov_state_estimation/fusion_node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    // 单线程 spin：三个数据回调、reset_home 和诊断定时器不并发修改成员。
    rclcpp::spin(std::make_shared<rov_state_estimation::FusionNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("hippo_fusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

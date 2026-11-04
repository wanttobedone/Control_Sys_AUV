#pragma once

#include <Eigen/Geometry>
#include <optional>

namespace rov_state_estimation
{

// T_AB 把 B 系坐标变到 A 系：p_A = q_AB * p_B + p_AB_m。
// 所有平移单位为米；四元数在 C++ 中用 Eigen 构造顺序 w,x,y,z。
struct RigidPose
{
  Eigen::Vector3d p_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
};

struct FrameConfig
{
  // B：飞控板中心、FLU；C：光学右/下/前。用户确认的安装粗测值。
  RigidPose camera_BC{{-0.050, 0.0, -0.125}, {0.0, 0.0, 1.0, 0.0}};
  // L：原生标签系；W：标签中心、Z 向上。当前标签平贴缸底。
  RigidPose tag_WL{{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}};
};

// 无 ROS 的几何层；不做检测、滤波、时间推进或速度差分。
class FrameAdapter
{
public:
  explicit FrameAdapter(const FrameConfig &config = FrameConfig{});
  static bool isValid(const RigidPose &pose);

  // T_WB = T_WL * T_LC * inverse(T_BC)，杆臂随姿态旋转。
  RigidPose cameraToBody(const RigidPose &camera_LC) const;

  // 只接受融合后的机体位姿：固定原点与初始水平航向，保留真实倾斜。
  // 已有 home 时保持不变；机头近乎竖直时返回 false，等待下一帧。
  bool initializeHome(const RigidPose &body_WB);
  RigidPose toHome(const RigidPose &body_WB) const;
  void resetHome();
  const std::optional<RigidPose> &home() const;
  const FrameConfig &config() const;

private:
  FrameConfig config_;
  std::optional<RigidPose> home_WH_;
};

} // namespace rov_state_estimation

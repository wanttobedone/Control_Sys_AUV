#include "rov_state_estimation/frame_adapter.h"

#include <cmath>
#include <stdexcept>

namespace rov_state_estimation
{

FrameAdapter::FrameAdapter(const FrameConfig &config) : config_(config)
{
  if (!isValid(config_.camera_BC) || !isValid(config_.tag_WL))
  {
    throw std::invalid_argument("安装/标签变换必须有限且四元数为单位四元数");
  }
  config_.camera_BC.q.normalize();
  config_.tag_WL.q.normalize();
}

bool FrameAdapter::isValid(const RigidPose &pose)
{
  return pose.p_m.allFinite() && pose.q.coeffs().allFinite() &&
         std::abs(pose.q.norm() - 1.0) < 0.01;
}

RigidPose FrameAdapter::cameraToBody(const RigidPose &camera_LC) const
{
  if (!isValid(camera_LC))
  {
    throw std::invalid_argument("相机位姿无效");
  }
  const Eigen::Quaterniond q_LB = camera_LC.q.normalized() * config_.camera_BC.q.conjugate();
  // p_LC 是光心，减去“从机体中心到光心”的杆臂才得到机体中心。
  const Eigen::Vector3d p_LB_m = camera_LC.p_m - q_LB * config_.camera_BC.p_m;
  RigidPose body_WB;
  body_WB.q = (config_.tag_WL.q * q_LB).normalized();
  body_WB.p_m = config_.tag_WL.q * p_LB_m + config_.tag_WL.p_m;
  return body_WB;
}

bool FrameAdapter::initializeHome(const RigidPose &body_WB)
{
  if (home_WH_)
  {
    return true;
  }
  if (!isValid(body_WB))
  {
    return false;
  }
  const Eigen::Vector3d forward_W = body_WB.q * Eigen::Vector3d::UnitX();
  if (forward_W.head<2>().norm() < 0.1)
  {
    return false;
  }
  const double yaw_rad = std::atan2(forward_W.y(), forward_W.x());
  home_WH_ = RigidPose{body_WB.p_m,
                       Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()))};
  return true;
}

RigidPose FrameAdapter::toHome(const RigidPose &body_WB) const
{
  if (!home_WH_)
  {
    throw std::logic_error("home 尚未建立");
  }
  RigidPose body_HB;
  body_HB.p_m = home_WH_->q.conjugate() * (body_WB.p_m - home_WH_->p_m);
  body_HB.q = (home_WH_->q.conjugate() * body_WB.q).normalized();
  return body_HB;
}

void FrameAdapter::resetHome()
{
  home_WH_.reset();
}

const std::optional<RigidPose> &FrameAdapter::home() const
{
  return home_WH_;
}

const FrameConfig &FrameAdapter::config() const
{
  return config_;
}

} // namespace rov_state_estimation

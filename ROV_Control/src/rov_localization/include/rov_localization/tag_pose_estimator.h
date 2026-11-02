#pragma once

#include "rov_localization/vision_types.h"

namespace rov_localization
{
// 已标定的原始图像模型。K为像素，D为无量纲plumb_bob五参数。
// 本工具只读该模型，不在定位过程中重新拟合内参。
struct CameraModel
{
  cv::Size image_size;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::string calibration_status;
};

// 实验选值统一从tag_pose_options.yaml读取；不与标定阶段的选帧选值混用。
struct TagPoseOptions
{
  int target_id{16};
  double tag_width_m{0.080};
  double tag_height_m{0.080};
  int detector_threads{2};
  int max_hamming{0};
  double min_edge_px{35.0};
  double min_corner_margin_px{4.0};
  double max_reprojection_error_px{3.0};
  double ambiguity_error_gap_px{0.15};
  double ambiguity_rotation_deg{5.0};
  double camera_fps{30.0};
};

// L：固定标签中心为原点，四角顺序与原生det->p及离线标定完全一致。
// X从p0指向p1，Y从p3指向p0，Z=X×Y；它不是ENU/NED或机器人home。
// C：相机光学系，X右/Y下/Z前。T_CL把L中的点变到C。
// 相机在L中的位置p_LC=-R_CL^T*t_CL，旋转R_LC=R_CL^T。
struct TagPoseEstimate
{
  bool has_pose{false}; // 有数值候选；不代表该帧已通过质量/歧义检查。
  bool valid{false};    // 仅当重投影检查通过且未检测到平面歧义时为true。
  bool ambiguous{false};
  std::string reason{"target_not_found"};
  cv::Vec3d rvec_CL_rad{0.0, 0.0, 0.0};
  cv::Vec3d t_CL_m{0.0, 0.0, 0.0};
  cv::Vec3d p_LC_m{0.0, 0.0, 0.0};
  cv::Matx33d R_LC{cv::Matx33d::eye()};
  // 仅供人读：R_LC=Rz(yaw)*Ry(pitch)*Rx(roll)，不是ROV机体姿态。
  cv::Vec3d rpy_LC_deg{0.0, 0.0, 0.0};
  double height_above_plane_m{std::numeric_limits<double>::quiet_NaN()};
  double reprojection_error_px{std::numeric_limits<double>::quiet_NaN()};
  double second_error_px{std::numeric_limits<double>::quiet_NaN()};
  double solution_rotation_gap_deg{std::numeric_limits<double>::quiet_NaN()};
};

CameraModel loadCameraModel(const std::string &path);
TagPoseOptions loadTagPoseOptions(const std::string &path);

// 单帧纯几何求解：模型/配置在构造时校验并复制，之后保持只读。
// 不保存上一帧姿态，不做滤波或用历史值填补丢检；所有候选属于本次调用。
// 后续ROS节点可以直接复用该类，无需把录像、显示或ROS消息带进求解层。
class TagPoseEstimator
{
public:
  TagPoseEstimator(const CameraModel &camera, const TagPoseOptions &options);
  const CameraModel &camera() const;
  const TagPoseOptions &options() const;
  std::vector<cv::Point3d> objectPoints() const;
  TagPoseEstimate estimate(const std::vector<TagDetection> &detections) const;

private:
  CameraModel camera_;
  TagPoseOptions options_;
};
} // namespace rov_localization

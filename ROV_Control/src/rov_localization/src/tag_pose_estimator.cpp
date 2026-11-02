#include "rov_localization/tag_pose_estimator.h"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace rov_localization
{
namespace
{
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

void require(bool condition, const std::string &message)
{
  if (!condition)
  {
    throw std::invalid_argument(message);
  }
}

template <typename T> void readRequired(const cv::FileStorage &storage, const char *name, T &value)
{
  require(!storage[name].empty(), std::string("定位输入缺少字段：") + name);
  storage[name] >> value;
}

// 固定模型只在入口检查一次。convertTo统一成double，并断开调用者对Mat的共享修改。
void validateCamera(CameraModel &camera)
{
  require(camera.image_size.width > 0 && camera.image_size.height > 0,
          "相机image_width/image_height须为正");
  require(camera.camera_matrix.rows == 3 && camera.camera_matrix.cols == 3 &&
              camera.camera_matrix.channels() == 1 && camera.dist_coeffs.total() == 5 &&
              camera.dist_coeffs.channels() == 1,
          "需要3×3的camera_matrix与5项plumb_bob dist_coeffs");
  camera.camera_matrix.convertTo(camera.camera_matrix, CV_64F);
  camera.dist_coeffs = camera.dist_coeffs.reshape(1, 1).clone();
  camera.dist_coeffs.convertTo(camera.dist_coeffs, CV_64F);
  const auto &matrix = camera.camera_matrix;
  require(cv::checkRange(matrix) && cv::checkRange(camera.dist_coeffs), "相机K/D包含非有限值");
  require(matrix.at<double>(0, 0) > 0.0 && matrix.at<double>(1, 1) > 0.0 &&
              matrix.at<double>(0, 2) >= 0.0 && matrix.at<double>(0, 2) < camera.image_size.width &&
              matrix.at<double>(1, 2) >= 0.0 && matrix.at<double>(1, 2) < camera.image_size.height,
          "相机焦距须为正，主点须在原图内");
  require(std::abs(matrix.at<double>(0, 1)) < 1e-12 && std::abs(matrix.at<double>(1, 0)) < 1e-12 &&
              std::abs(matrix.at<double>(2, 0)) < 1e-12 &&
              std::abs(matrix.at<double>(2, 1)) < 1e-12 &&
              std::abs(matrix.at<double>(2, 2) - 1.0) < 1e-12,
          "K须符合OpenCV零skew针孔矩阵形式");
}

void validateOptions(const TagPoseOptions &options)
{
  require(options.target_id >= 0 && options.target_id < 587, "tag36h11 target_id须在0..586内");
  require(options.detector_threads > 0 && options.max_hamming >= 0 && options.max_hamming <= 2,
          "detector_threads须为正，max_hamming须在0..2内");
  for (const double value :
       {options.tag_width_m, options.tag_height_m, options.min_edge_px,
        options.max_reprojection_error_px, options.ambiguity_rotation_deg, options.camera_fps})
  {
    require(std::isfinite(value) && value > 0.0, "定位尺寸/最小边/误差上限/角度/帧率须有限且为正");
  }
  require(std::isfinite(options.min_corner_margin_px) && options.min_corner_margin_px >= 0.0 &&
              std::isfinite(options.ambiguity_error_gap_px) &&
              options.ambiguity_error_gap_px >= 0.0,
          "角点边距和歧义误差差值须有限且非负");
}

struct PoseCandidate
{
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Matx33d rotation;
  double error_px{0.0};
};

// 返回每个检测角点二维距离的RMS：sqrt(sum(du²+dv²)/4)，单位px。
// 同时核四角在相机前方，避免把负深度的代数解当成相机观测。
bool evaluateCandidate(const std::vector<cv::Point3d> &points,
                       const std::vector<cv::Point2d> &pixels, const CameraModel &camera,
                       PoseCandidate &candidate)
{
  if (!cv::checkRange(candidate.rvec) || !cv::checkRange(candidate.tvec))
  {
    return false;
  }
  cv::Rodrigues(candidate.rvec, candidate.rotation);
  for (const auto &point : points)
  {
    const cv::Vec3d transformed = candidate.rotation * cv::Vec3d(point.x, point.y, point.z);
    const double depth_m = transformed[2] + candidate.tvec.at<double>(2);
    if (depth_m <= 0.0)
    {
      return false;
    }
  }
  std::vector<cv::Point2d> projected;
  cv::projectPoints(points, candidate.rvec, candidate.tvec, camera.camera_matrix,
                    camera.dist_coeffs, projected);
  double sum_squared_px = 0.0;
  for (std::size_t i = 0; i < pixels.size(); ++i)
  {
    const cv::Point2d difference = projected[i] - pixels[i];
    sum_squared_px += difference.dot(difference);
  }
  candidate.error_px = std::sqrt(sum_squared_px / static_cast<double>(pixels.size()));
  return std::isfinite(candidate.error_px);
}

double rotationDifferenceDeg(const cv::Matx33d &first, const cv::Matx33d &second)
{
  const cv::Matx33d difference = first * second.t();
  const double cosine = (cv::trace(cv::Mat(difference))[0] - 1.0) / 2.0;
  return std::acos(std::clamp(cosine, -1.0, 1.0)) * kRadiansToDegrees;
}
} // namespace

CameraModel loadCameraModel(const std::string &path)
{
  cv::FileStorage storage(path, cv::FileStorage::READ);
  require(storage.isOpened(), "无法读取相机XML：" + path);
  std::string model;
  std::string distortion_model;
  readRequired(storage, "camera_model", model);
  readRequired(storage, "distortion_model", distortion_model);
  require(model == "pinhole" && distortion_model == "plumb_bob",
          "当前定位工具只接收pinhole/plumb_bob，不可混用fisheye参数");
  CameraModel camera;
  readRequired(storage, "image_width", camera.image_size.width);
  readRequired(storage, "image_height", camera.image_size.height);
  readRequired(storage, "camera_matrix", camera.camera_matrix);
  readRequired(storage, "dist_coeffs", camera.dist_coeffs);
  readRequired(storage, "status", camera.calibration_status);
  require(camera.calibration_status != "failed", "拒绝使用标定失败的相机参数");
  validateCamera(camera);
  return camera;
}

TagPoseOptions loadTagPoseOptions(const std::string &path)
{
  cv::FileStorage storage(path, cv::FileStorage::READ);
  require(storage.isOpened(), "无法读取定位配置：" + path);
  TagPoseOptions options;
  readRequired(storage, "target_id", options.target_id);
  readRequired(storage, "tag_width_m", options.tag_width_m);
  readRequired(storage, "tag_height_m", options.tag_height_m);
  readRequired(storage, "detector_threads", options.detector_threads);
  readRequired(storage, "max_hamming", options.max_hamming);
  readRequired(storage, "min_edge_px", options.min_edge_px);
  readRequired(storage, "min_corner_margin_px", options.min_corner_margin_px);
  readRequired(storage, "max_reprojection_error_px", options.max_reprojection_error_px);
  readRequired(storage, "ambiguity_error_gap_px", options.ambiguity_error_gap_px);
  readRequired(storage, "ambiguity_rotation_deg", options.ambiguity_rotation_deg);
  readRequired(storage, "camera_fps", options.camera_fps);
  validateOptions(options);
  return options;
}

TagPoseEstimator::TagPoseEstimator(const CameraModel &camera, const TagPoseOptions &options)
    : camera_(camera), options_(options)
{
  camera_.camera_matrix = camera.camera_matrix.clone();
  camera_.dist_coeffs = camera.dist_coeffs.clone();
  validateCamera(camera_);
  validateOptions(options_);
}

const CameraModel &TagPoseEstimator::camera() const
{
  return camera_;
}

const TagPoseOptions &TagPoseEstimator::options() const
{
  return options_;
}

std::vector<cv::Point3d> TagPoseEstimator::objectPoints() const
{
  const double half_width_m = options_.tag_width_m / 2.0;
  const double half_height_m = options_.tag_height_m / 2.0;
  return {{-half_width_m, half_height_m, 0.0},
          {half_width_m, half_height_m, 0.0},
          {half_width_m, -half_height_m, 0.0},
          {-half_width_m, -half_height_m, 0.0}};
}

TagPoseEstimate TagPoseEstimator::estimate(const std::vector<TagDetection> &detections) const
{
  TagPoseEstimate result;
  const TagDetection *target = nullptr;
  for (const auto &detection : detections)
  {
    if (detection.id == options_.target_id)
    {
      if (target != nullptr)
      {
        result.reason = "duplicate_target_id";
        return result;
      }
      target = &detection;
    }
  }
  if (target == nullptr)
  {
    return result;
  }
  if (target->hamming < 0 || target->hamming > options_.max_hamming)
  {
    result.reason = "hamming_rejected";
    return result;
  }
  std::vector<cv::Point2d> pixels;
  for (const auto &corner : target->corners_px)
  {
    if (!std::isfinite(corner.x) || !std::isfinite(corner.y) ||
        corner.x < options_.min_corner_margin_px || corner.y < options_.min_corner_margin_px ||
        corner.x >= camera_.image_size.width - options_.min_corner_margin_px ||
        corner.y >= camera_.image_size.height - options_.min_corner_margin_px)
    {
      result.reason = "clipped_or_invalid_corners";
      return result;
    }
    pixels.emplace_back(corner.x, corner.y);
  }
  const std::vector<cv::Point2f> contour(target->corners_px.begin(), target->corners_px.end());
  if (!cv::isContourConvex(contour))
  {
    result.reason = "nonconvex_or_small_tag";
    return result;
  }
  for (std::size_t i = 0; i < pixels.size(); ++i)
  {
    if (cv::norm(pixels[i] - pixels[(i + 1) % pixels.size()]) < options_.min_edge_px)
    {
      result.reason = "nonconvex_or_small_tag";
      return result;
    }
  }

  const auto points = objectPoints();
  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  std::vector<PoseCandidate> candidates;
  try
  {
    // IPPE返回平面的两种姿态候选。使用通用IPPE也允许真实宽高略有不同。
    // 两个解分别细化：K/D始终固定，仅优化该帧6维外参，不是再次标定。
    cv::solvePnPGeneric(points, pixels, camera_.camera_matrix, camera_.dist_coeffs, rotations,
                        translations, false, cv::SOLVEPNP_IPPE);
    for (std::size_t i = 0; i < rotations.size(); ++i)
    {
      PoseCandidate candidate;
      rotations[i].convertTo(candidate.rvec, CV_64F);
      translations[i].convertTo(candidate.tvec, CV_64F);
      if (!evaluateCandidate(points, pixels, camera_, candidate))
      {
        continue;
      }
      PoseCandidate refined = candidate;
      refined.rvec = candidate.rvec.clone();
      refined.tvec = candidate.tvec.clone();
      cv::solvePnPRefineLM(points, pixels, camera_.camera_matrix, camera_.dist_coeffs, refined.rvec,
                           refined.tvec);
      // 只采用正深度且不增加像素误差的细化结果。
      if (evaluateCandidate(points, pixels, camera_, refined) &&
          refined.error_px <= candidate.error_px)
      {
        candidate = refined;
      }
      candidates.push_back(candidate);
    }
  }
  catch (const cv::Exception &)
  {
    result.reason = "pnp_failed";
    return result;
  }
  if (candidates.empty())
  {
    result.reason = "no_positive_depth_pose";
    return result;
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const PoseCandidate &first, const PoseCandidate &second)
            { return first.error_px < second.error_px; });
  const auto &best = candidates.front();
  result.has_pose = true;
  result.reprojection_error_px = best.error_px;
  result.rvec_CL_rad = cv::Vec3d(best.rvec);
  result.t_CL_m = cv::Vec3d(best.tvec);
  result.R_LC = best.rotation.t();
  result.p_LC_m = -(result.R_LC * result.t_CL_m);
  // L的Z由角点编码顺序决定；相机可能位于负Z侧。abs(z)才是光心到平面的距离。
  result.height_above_plane_m = std::abs(result.p_LC_m[2]);
  const auto &rotation = result.R_LC;
  const double pitch_rad = std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
  double roll_rad = std::atan2(rotation(2, 1), rotation(2, 2));
  double yaw_rad = std::atan2(rotation(1, 0), rotation(0, 0));
  if (std::hypot(rotation(0, 0), rotation(1, 0)) < 1e-8)
  {
    // 欧拉角万向节位置选yaw=0；完整旋转仍由R_LC保存，不能把显示跳变当机体跳变。
    roll_rad = std::atan2(-rotation(1, 2), rotation(1, 1));
    yaw_rad = 0.0;
  }
  result.rpy_LC_deg = cv::Vec3d(roll_rad, pitch_rad, yaw_rad) * kRadiansToDegrees;
  if (candidates.size() > 1)
  {
    result.second_error_px = candidates[1].error_px;
    result.solution_rotation_gap_deg = rotationDifferenceDeg(best.rotation, candidates[1].rotation);
    // 若两次细化收敛到同一姿态，旋转差很小，不把数值重复解误报为姿态歧义。
    const double error_gap_px = result.second_error_px - result.reprojection_error_px;
    result.ambiguous = result.solution_rotation_gap_deg >= options_.ambiguity_rotation_deg &&
                       error_gap_px <= options_.ambiguity_error_gap_px;
  }
  if (result.reprojection_error_px > options_.max_reprojection_error_px)
  {
    result.reason = "reprojection_error";
  }
  else if (result.ambiguous)
  {
    result.reason = "ambiguous_pose";
  }
  else
  {
    result.valid = true;
    result.reason = "ok";
  }
  return result;
}
} // namespace rov_localization

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace rov_localization
{
// 原生 AprilTag 的输出。顺序保持 det->p[0..3]，绝不按图像左上角重新排序。
// corners_px 是原分辨率像素，decision_margin 是解码信息，不是位置精度。
struct TagDetection
{
  int id{-1};
  int hamming{0};
  double decision_margin{0.0};
  std::array<cv::Point2f, 4> corners_px{};
};

// 所有实验选值来自 calibration_options.yaml，零值默认不可直接运行。
// 单位后缀区分米、像素、秒。首版仅明确支持 pinhole，避免模型混用。
struct CalibrationOptions
{
  std::string tag_family;
  std::string camera_model;
  double tag_width_m{0.0};
  double tag_height_m{0.0};
  int target_id{-1};
  int detector_threads{0};
  double sample_period_s{0.0};
  int max_candidates_per_id{0};
  int max_selected_views{0};
  int min_fit_views{0};
  int min_holdout_views{0};
  double holdout_fraction{0.0};
  double time_group_s{0.0};
  int max_hamming{0};
  double min_edge_px{0.0};
  double min_corner_margin_px{0.0};
  double min_blur_variance{0.0};
  double min_view_distance{0.0};
  double min_image_span_x{0.0};
  double min_image_span_y{0.0};
  double min_normal_spread_deg{0.0};
  double max_rms_px{0.0};
  double max_holdout_rms_px{0.0};
  double max_relative_focal_change{0.0};
  int solver_max_iterations{0};
  double solver_epsilon{0.0};
};

// 只保存四角/元信息，不缓存全部图像。图像中心原点是左上角，u向右、v向下。
// video_time_s 由帧号/文件FPS构成抽样时间轴；不得称为真实曝光时间戳。
struct CalibrationView
{
  std::int64_t frame_index{0};
  double video_time_s{0.0};
  TagDetection detection;
  double blur_variance{0.0};
  // 中心u/v、对数尺度、对边比例与内角余弦；没有K时不把它称为真实倾角。
  std::array<double, 6> features{};
};

struct TagStatistics
{
  std::int64_t detected{0};
  std::int64_t quality_accepted{0};
};

struct ScanStatistics
{
  std::int64_t decoded_frames{0};
  std::int64_t sampled_frames{0};
  double fps{0.0};
  double reported_frame_count{0.0};
  // CAP_PROP_FRAME_COUNT可能来自时长×FPS估算，差值用于诊断，不作为损坏判据。
  double frame_count_difference{std::numeric_limits<double>::quiet_NaN()};
  std::string frame_count_warning;
  double processing_s{0.0};
  std::string video_backend;
  std::map<int, TagStatistics> tags;
  std::map<std::string, std::int64_t> rejected;
};

// 这是候选相机参数，不是map位姿或T_BC。K焦距/主点px，D顺序为
// k1,k2,p1,p2,k3；每帧rvec/tvec把标签局部L变到相机C，单位rad/m。
struct CalibrationResult
{
  bool success{false};
  std::string message;
  int target_id{-1};
  cv::Size image_size;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double relative_focal_change{std::numeric_limits<double>::quiet_NaN()};
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<CalibrationView> fit_views;
  std::vector<CalibrationView> holdout_views;
  std::vector<double> fit_errors_px;
  std::vector<double> holdout_errors_px;
  double rms_px{std::numeric_limits<double>::quiet_NaN()};
  double holdout_rms_px{std::numeric_limits<double>::quiet_NaN()};
  double image_span_x{0.0};
  double image_span_y{0.0};
  double normal_spread_deg{0.0};
};
} // namespace rov_localization

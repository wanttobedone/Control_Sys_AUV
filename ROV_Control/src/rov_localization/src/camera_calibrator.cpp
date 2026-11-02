#include "rov_localization/camera_calibrator.h"
#include "rov_localization/apriltag_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <random>
#include <sstream>
#include <stdexcept>

namespace rov_localization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

// FileStorage在缺字段时可能保留旧值，所以逐项要求字段存在；不隐式兜底实验选值。
template <typename T> void readRequired(const cv::FileStorage &storage, const char *name, T &value)
{
  if (storage[name].empty())
  {
    throw std::invalid_argument(std::string("标定配置缺少字段：") + name);
  }
  storage[name] >> value;
}

void require(bool valid, const std::string &message)
{
  if (!valid)
  {
    throw std::runtime_error(message);
  }
}

std::vector<cv::Point2f> imagePoints(const CalibrationView &view)
{
  return {view.detection.corners_px.begin(), view.detection.corners_px.end()};
}

// 每角点二维欧氏误差的RMS，分母为角点数，不是u/v标量分量数。
double reprojectionError(const std::vector<cv::Point3f> &object_points, const CalibrationView &view,
                         const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                         const cv::Mat &rvec, const cv::Mat &tvec)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
  double squared_error = 0.0;
  for (std::size_t i = 0; i < projected.size(); ++i)
  {
    const cv::Point2f difference = projected[i] - view.detection.corners_px[i];
    squared_error += static_cast<double>(difference.dot(difference));
  }
  return std::sqrt(squared_error / static_cast<double>(projected.size()));
}

double aggregateRms(const std::vector<double> &errors)
{
  double sum = 0.0;
  for (const double value : errors)
  {
    sum += value * value;
  }
  return std::sqrt(sum / static_cast<double>(errors.size()));
}

bool hasPositiveDepth(const std::vector<cv::Point3f> &points, const cv::Mat &rvec,
                      const cv::Mat &tvec)
{
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  for (const auto &point : points)
  {
    const double z_m = rotation.at<double>(2, 0) * point.x + rotation.at<double>(2, 1) * point.y +
                       tvec.at<double>(2);
    if (!std::isfinite(z_m) || z_m <= 0.0)
    {
      return false;
    }
  }
  return true;
}

// 保留实际选值便于复现实验。这里与读取列表对应，不额外创建另一套阈值。
void writeOptions(cv::FileStorage &storage, const CalibrationOptions &options)
{
  storage << "options" << "{";
  storage << "tag_family" << options.tag_family;
  storage << "camera_model" << options.camera_model;
  storage << "tag_width_m" << options.tag_width_m;
  storage << "tag_height_m" << options.tag_height_m;
  storage << "target_id" << options.target_id;
  storage << "detector_threads" << options.detector_threads;
  storage << "sample_period_s" << options.sample_period_s;
  storage << "max_candidates_per_id" << options.max_candidates_per_id;
  storage << "max_selected_views" << options.max_selected_views;
  storage << "min_fit_views" << options.min_fit_views;
  storage << "min_holdout_views" << options.min_holdout_views;
  storage << "holdout_fraction" << options.holdout_fraction;
  storage << "time_group_s" << options.time_group_s;
  storage << "max_hamming" << options.max_hamming;
  storage << "min_edge_px" << options.min_edge_px;
  storage << "min_corner_margin_px" << options.min_corner_margin_px;
  storage << "min_blur_variance" << options.min_blur_variance;
  storage << "min_view_distance" << options.min_view_distance;
  storage << "min_image_span_x" << options.min_image_span_x;
  storage << "min_image_span_y" << options.min_image_span_y;
  storage << "min_normal_spread_deg" << options.min_normal_spread_deg;
  storage << "max_rms_px" << options.max_rms_px;
  storage << "max_holdout_rms_px" << options.max_holdout_rms_px;
  storage << "max_relative_focal_change" << options.max_relative_focal_change;
  storage << "solver_max_iterations" << options.solver_max_iterations;
  storage << "solver_epsilon" << options.solver_epsilon;
  storage << "}";
}
} // namespace

CalibrationOptions loadOptions(const std::string &path)
{
  cv::FileStorage storage(path, cv::FileStorage::READ);
  require(storage.isOpened(), "无法读取配置：" + path);
  CalibrationOptions options;
  // 逐行对应YAML字段，输入和报告中都保留实际实验选值。
  readRequired(storage, "tag_family", options.tag_family);
  readRequired(storage, "camera_model", options.camera_model);
  readRequired(storage, "tag_width_m", options.tag_width_m);
  readRequired(storage, "tag_height_m", options.tag_height_m);
  readRequired(storage, "target_id", options.target_id);
  readRequired(storage, "detector_threads", options.detector_threads);
  readRequired(storage, "sample_period_s", options.sample_period_s);
  readRequired(storage, "max_candidates_per_id", options.max_candidates_per_id);
  readRequired(storage, "max_selected_views", options.max_selected_views);
  readRequired(storage, "min_fit_views", options.min_fit_views);
  readRequired(storage, "min_holdout_views", options.min_holdout_views);
  readRequired(storage, "holdout_fraction", options.holdout_fraction);
  readRequired(storage, "time_group_s", options.time_group_s);
  readRequired(storage, "max_hamming", options.max_hamming);
  readRequired(storage, "min_edge_px", options.min_edge_px);
  readRequired(storage, "min_corner_margin_px", options.min_corner_margin_px);
  readRequired(storage, "min_blur_variance", options.min_blur_variance);
  readRequired(storage, "min_view_distance", options.min_view_distance);
  readRequired(storage, "min_image_span_x", options.min_image_span_x);
  readRequired(storage, "min_image_span_y", options.min_image_span_y);
  readRequired(storage, "min_normal_spread_deg", options.min_normal_spread_deg);
  readRequired(storage, "max_rms_px", options.max_rms_px);
  readRequired(storage, "max_holdout_rms_px", options.max_holdout_rms_px);
  readRequired(storage, "max_relative_focal_change", options.max_relative_focal_change);
  readRequired(storage, "solver_max_iterations", options.solver_max_iterations);
  readRequired(storage, "solver_epsilon", options.solver_epsilon);
  validateOptions(options);
  return options;
}

void validateOptions(const CalibrationOptions &options)
{
  require(options.tag_family == "tag36h11", "tag_family当前仅支持tag36h11");
  require(options.camera_model == "pinhole", "camera_model当前仅支持pinhole；不能混用鱼眼D");
  const std::map<std::string, double> positive_values = {
      {"tag_width_m", options.tag_width_m},
      {"tag_height_m", options.tag_height_m},
      {"sample_period_s", options.sample_period_s},
      {"time_group_s", options.time_group_s},
      {"min_edge_px", options.min_edge_px},
      {"min_view_distance", options.min_view_distance},
      {"max_rms_px", options.max_rms_px},
      {"max_holdout_rms_px", options.max_holdout_rms_px},
      {"max_relative_focal_change", options.max_relative_focal_change},
      {"solver_epsilon", options.solver_epsilon}};
  for (const auto &[name, value] : positive_values)
  {
    require(std::isfinite(value) && value > 0.0,
            name + "须为有限正数，当前=" + std::to_string(value));
  }
  require(options.target_id >= -1 && options.target_id < 587,
          "target_id须为-1或tag36h11有效ID 0..586");
  require(options.detector_threads >= 1, "detector_threads须>=1");
  require(options.max_hamming >= 0 && options.max_hamming <= 2, "max_hamming须在0..2内");
  require(options.min_fit_views >= 10 && options.min_holdout_views >= 1,
          "拟合视角须>=10以支持两组各>=5的焦距稳定性检查；留出>=1，默认20/5为工程选值");
  require(options.max_selected_views >= options.min_fit_views + options.min_holdout_views,
          "max_selected_views小于min_fit_views+min_holdout_views");
  require(options.max_candidates_per_id >= options.max_selected_views,
          "max_candidates_per_id须>=max_selected_views");
  require(std::isfinite(options.holdout_fraction) && options.holdout_fraction > 0.0 &&
              options.holdout_fraction < 1.0,
          "holdout_fraction须在(0,1)内");
  require(std::isfinite(options.min_corner_margin_px) && options.min_corner_margin_px >= 0.0,
          "min_corner_margin_px须为有限非负数");
  require(std::isfinite(options.min_blur_variance) && options.min_blur_variance >= 0.0,
          "min_blur_variance须为有限非负数");
  require(std::isfinite(options.min_image_span_x) && options.min_image_span_x > 0.0 &&
              options.min_image_span_x <= 1.0 && std::isfinite(options.min_image_span_y) &&
              options.min_image_span_y > 0.0 && options.min_image_span_y <= 1.0,
          "min_image_span_x/y须在(0,1]内");
  require(std::isfinite(options.min_normal_spread_deg) && options.min_normal_spread_deg > 0.0 &&
              options.min_normal_spread_deg < 90.0,
          "min_normal_spread_deg须在(0,90)内");
  require(options.solver_max_iterations > 0, "solver_max_iterations须>0");
}

CameraCalibrator::CameraCalibrator(const CalibrationOptions &options) : options_(options)
{
  validateOptions(options_);
}

std::vector<cv::Point3f> CameraCalibrator::makeObjectPoints() const
{
  // 原生H规范的p0=(-1,+1)、p1=(+1,+1)、p2=(+1,-1)、p3=(-1,-1)。
  // 角点坐标只由编码方向决定，标签在图像中转了90度也不能重排成左上开始。
  const float half_width_m = static_cast<float>(options_.tag_width_m / 2.0);
  const float half_height_m = static_cast<float>(options_.tag_height_m / 2.0);
  return {{-half_width_m, half_height_m, 0.0F},
          {half_width_m, half_height_m, 0.0F},
          {half_width_m, -half_height_m, 0.0F},
          {-half_width_m, -half_height_m, 0.0F}};
}

bool CameraCalibrator::describeView(const cv::Mat &gray, const TagDetection &detection,
                                    CalibrationView &view, std::string &reason) const
{
  // 这层检查实际图像/观测，不在每帧重新校验固定配置。
  if (gray.empty() || gray.type() != CV_8UC1)
  {
    reason = "bad_gray_image";
    return false;
  }
  if (detection.hamming > options_.max_hamming)
  {
    reason = "hamming";
    return false;
  }
  const auto &corners = detection.corners_px;
  std::array<double, 4> lengths{};
  cv::Point2d center(0.0, 0.0);
  for (std::size_t i = 0; i < corners.size(); ++i)
  {
    const auto &point = corners[i];
    const double margin = options_.min_corner_margin_px;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < margin ||
        point.y < margin || point.x >= gray.cols - margin || point.y >= gray.rows - margin)
    {
      reason = "clipped_or_invalid_corners";
      return false;
    }
    lengths[i] = cv::norm(corners[(i + 1) % 4] - point);
    center.x += point.x / 4.0;
    center.y += point.y / 4.0;
  }
  const std::vector<cv::Point2f> polygon(corners.begin(), corners.end());
  if (!cv::isContourConvex(polygon) ||
      *std::min_element(lengths.begin(), lengths.end()) < options_.min_edge_px)
  {
    reason = "nonconvex_or_small_tag";
    return false;
  }
  const double area_px2 = std::abs(cv::contourArea(polygon));
  const cv::Rect roi = cv::boundingRect(polygon) & cv::Rect(0, 0, gray.cols, gray.rows);
  cv::Mat laplacian;
  cv::Laplacian(gray(roi), laplacian, CV_64F);
  cv::Scalar mean;
  cv::Scalar stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  view.blur_variance = stddev[0] * stddev[0];
  if (view.blur_variance < options_.min_blur_variance)
  {
    reason = "blur";
    return false;
  }
  view.detection = detection;
  // 面积取对数，使近/远投影的比例变化可比较；对边比和内角反映透视形状。
  // 这里没有使用未知K，因而这些量不能标作真实俯仰/横滚角。
  const cv::Point2f edge_a = corners[1] - corners[0];
  const cv::Point2f edge_b = corners[3] - corners[0];
  view.features = {center.x / gray.cols,
                   center.y / gray.rows,
                   0.5 * std::log(area_px2 / static_cast<double>(gray.total())),
                   std::log(lengths[0] / lengths[2]),
                   std::log(lengths[1] / lengths[3]),
                   static_cast<double>(edge_a.dot(edge_b)) / (lengths[0] * lengths[3])};
  reason.clear();
  return true;
}

void CameraCalibrator::scanVideo(const std::string &path, CalibrationResult &result)
{
  cv::VideoCapture video(path);
  require(video.isOpened(), "无法打开视频：" + path);
  statistics_.fps = video.get(cv::CAP_PROP_FPS);
  statistics_.reported_frame_count = video.get(cv::CAP_PROP_FRAME_COUNT);
  statistics_.video_backend = video.getBackendName();
  require(std::isfinite(statistics_.fps) && statistics_.fps > 0.0,
          "文件FPS无效，无法建立离线抽样时间轴；先用ffprobe检查视频");
  AprilTagDetector detector;
  detector.configure(options_.detector_threads, options_.max_hamming);
  std::mt19937 reservoir_rng(0);
  double next_sample_s = 0.0;
  cv::Mat frame;
  while (video.read(frame))
  {
    require(!frame.empty(), "解码器返回空图像");
    const std::int64_t frame_index = statistics_.decoded_frames++;
    if (result.image_size.empty())
    {
      result.image_size = frame.size();
    }
    require(frame.size() == result.image_size, "视频中分辨率变化，不能共用同一个K");
    const double time_s = static_cast<double>(frame_index) / statistics_.fps;
    if (time_s + 1.0e-9 < next_sample_s)
    {
      continue;
    }
    next_sample_s = time_s + options_.sample_period_s;
    ++statistics_.sampled_frames;
    cv::Mat gray;
    if (frame.type() == CV_8UC1)
    {
      gray = frame;
    }
    else
    {
      require(frame.type() == CV_8UC3, "视频须为8位灰度或BGR图像");
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    auto detections = detector.detect(gray);
    // 原生多线程结果可能到达顺序不同。按ID排序，保持选样可复现。
    std::sort(detections.begin(), detections.end(),
              [](const TagDetection &a, const TagDetection &b) { return a.id < b.id; });
    std::map<int, int> counts;
    for (const auto &detection : detections)
    {
      ++counts[detection.id];
    }
    if (detections.empty())
    {
      ++statistics_.rejected["no_tag_in_sampled_frame"];
    }
    for (const auto &detection : detections)
    {
      auto &tag_stats = statistics_.tags[detection.id];
      ++tag_stats.detected;
      if (counts[detection.id] != 1)
      {
        ++statistics_.rejected["duplicate_id_in_frame"];
        continue;
      }
      CalibrationView view;
      view.frame_index = frame_index;
      view.video_time_s = time_s;
      std::string reason;
      if (!describeView(gray, detection, view, reason))
      {
        ++statistics_.rejected[reason];
        continue;
      }
      ++tag_stats.quality_accepted;
      auto &group = candidates_[detection.id];
      if (group.size() < static_cast<std::size_t>(options_.max_candidates_per_id))
      {
        group.push_back(view);
      }
      else
      {
        // 固定种子的蓄水池：容量满后仍覆盖后段视频，不只留下开头的画面。
        std::uniform_int_distribution<std::int64_t> pick(0, tag_stats.quality_accepted - 1);
        const auto index = pick(reservoir_rng);
        if (index < options_.max_candidates_per_id)
        {
          group[static_cast<std::size_t>(index)] = view;
        }
      }
    }
    if (statistics_.sampled_frames % 100 == 0)
    {
      std::cout << "已解码 " << statistics_.decoded_frames << " 帧，抽样检测 "
                << statistics_.sampled_frames << " 帧\n";
    }
  }
  require(statistics_.decoded_frames > 0, "视频没有可解码图像");
  // OpenCV/FFmpeg在流nb_frames缺失时，会以时长×FPS估算CAP_PROP_FRAME_COUNT。
  // 因此即使元数据非零，也不能当成精确总数；4002与4006的差异不能证明解码损坏。
  // read=false本身也不能区分正常EOF与解码失败：保留差值/警告，但继续检查已获得的
  // 样本。无可解码帧、分辨率变化、样本不足、求解/质量检查失败仍会明确中止。
  if (std::isfinite(statistics_.reported_frame_count) && statistics_.reported_frame_count > 0.0)
  {
    statistics_.frame_count_difference =
        static_cast<double>(statistics_.decoded_frames) - statistics_.reported_frame_count;
    if (statistics_.frame_count_difference != 0.0)
    {
      statistics_.frame_count_warning =
          "已解码帧数与元数据不一致；元数据可能为时长×FPS估算，不能仅凭差值判断文件损坏";
      std::cout << "提示：实际解码 " << statistics_.decoded_frames << " 帧，元数据报告 "
                << statistics_.reported_frame_count << " 帧；记录差异，继续样本/标定检查\n";
    }
  }
}

double CameraCalibrator::viewDistance(const CalibrationView &first,
                                      const CalibrationView &second) const
{
  double squared_distance = 0.0;
  for (std::size_t i = 0; i < first.features.size(); ++i)
  {
    const double difference = first.features[i] - second.features[i];
    squared_distance += difference * difference;
  }
  return std::sqrt(squared_distance);
}

void CameraCalibrator::selectViews(const std::vector<CalibrationView> &candidates,
                                   CalibrationResult &result) const
{
  require(!result.image_size.empty(), "选帧需要原图尺寸");
  require(!candidates.empty(), "没有满足清晰度/完整性条件的观测");
  auto sorted = candidates;
  std::sort(sorted.begin(), sorted.end(),
            [](const auto &a, const auto &b) { return a.frame_index < b.frame_index; });
  const auto sharpest =
      std::max_element(sorted.begin(), sorted.end(), [](const auto &a, const auto &b)
                       { return a.blur_variance < b.blur_variance; });
  std::vector<std::size_t> chosen{static_cast<std::size_t>(sharpest - sorted.begin())};
  std::vector<double> distances(sorted.size(), std::numeric_limits<double>::infinity());
  // 最远点贪心：保留与已有样本最不相似的观测，避免把同一姿态连拍当丰富视角。
  while (chosen.size() < static_cast<std::size_t>(options_.max_selected_views))
  {
    const auto &last = sorted[chosen.back()];
    for (std::size_t i = 0; i < sorted.size(); ++i)
    {
      distances[i] = std::min(distances[i], viewDistance(sorted[i], last));
    }
    const auto farthest = std::max_element(distances.begin(), distances.end());
    if (*farthest < options_.min_view_distance)
    {
      break;
    }
    chosen.push_back(static_cast<std::size_t>(farthest - distances.begin()));
  }
  std::sort(chosen.begin(), chosen.end());
  result.fit_views.clear();
  result.holdout_views.clear();
  std::map<std::int64_t, std::vector<CalibrationView>> time_groups;
  cv::Point2f minimum(static_cast<float>(result.image_size.width),
                      static_cast<float>(result.image_size.height));
  cv::Point2f maximum(0.0F, 0.0F);
  for (const auto index : chosen)
  {
    const auto &view = sorted[index];
    const auto group_id =
        static_cast<std::int64_t>(std::floor(view.video_time_s / options_.time_group_s));
    time_groups[group_id].push_back(view);
    for (const auto &point : view.detection.corners_px)
    {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
    }
  }
  result.image_span_x = static_cast<double>(maximum.x - minimum.x) / result.image_size.width;
  result.image_span_y = static_cast<double>(maximum.y - minimum.y) / result.image_size.height;
  // 按整段时间组分离，不将同一组的相邻帧随机拆进拟合与留出。
  // 这仍是同段录像内检查，报告不得把它称为独立尺量精度验收。
  std::size_t ordinal = 0;
  for (const auto &[group_id, views] : time_groups)
  {
    (void)group_id;
    const double before = std::floor(static_cast<double>(ordinal) * options_.holdout_fraction);
    ++ordinal;
    const double after = std::floor(static_cast<double>(ordinal) * options_.holdout_fraction);
    auto &destination = after > before ? result.holdout_views : result.fit_views;
    destination.insert(destination.end(), views.begin(), views.end());
  }
  require(result.fit_views.size() >= static_cast<std::size_t>(options_.min_fit_views) &&
              result.holdout_views.size() >= static_cast<std::size_t>(options_.min_holdout_views),
          "不同视角/时间组不足：拟合=" + std::to_string(result.fit_views.size()) +
              "，留出=" + std::to_string(result.holdout_views.size()) +
              "；需分别>=" + std::to_string(options_.min_fit_views) + "/" +
              std::to_string(options_.min_holdout_views) + "。先核同ID多角度/高度/画面位置覆盖");
  require(result.image_span_x >= options_.min_image_span_x &&
              result.image_span_y >= options_.min_image_span_y,
          "角点包络覆盖不足：span_x=" + std::to_string(result.image_span_x) +
              "，span_y=" + std::to_string(result.image_span_y) + "；需让同一ID进入不同画面区域");
}

int CameraCalibrator::selectTargetId(const cv::Size &image_size) const
{
  if (options_.target_id >= 0)
  {
    require(candidates_.count(options_.target_id) != 0,
            "指定ID=" + std::to_string(options_.target_id) + "没有通过质量筛选的观测");
    return options_.target_id;
  }
  int best_id = -1;
  double best_coverage = -1.0;
  double best_scale_range = -1.0;
  std::size_t best_count = 0;
  std::ostringstream failures;
  for (const auto &[id, views] : candidates_)
  {
    CalibrationResult trial;
    trial.image_size = image_size;
    try
    {
      selectViews(views, trial);
      double scale_min = std::numeric_limits<double>::infinity();
      double scale_max = -std::numeric_limits<double>::infinity();
      for (const auto &view : views)
      {
        scale_min = std::min(scale_min, view.features[2]);
        scale_max = std::max(scale_max, view.features[2]);
      }
      // 按覆盖、尺度、数量顺序比较，不另藏一套加权系数；全相同则保留较小ID。
      const double coverage = trial.image_span_x * trial.image_span_y;
      const double scale_range = scale_max - scale_min;
      const bool better_coverage = coverage > best_coverage;
      const bool better_scale = coverage == best_coverage && scale_range > best_scale_range;
      const bool more_views = coverage == best_coverage && scale_range == best_scale_range &&
                              trial.fit_views.size() > best_count;
      if (better_coverage || better_scale || more_views)
      {
        best_id = id;
        best_coverage = coverage;
        best_scale_range = scale_range;
        best_count = trial.fit_views.size();
      }
    }
    catch (const std::exception &error)
    {
      failures << " ID " << id << ": " << error.what() << ";";
    }
  }
  require(best_id >= 0, "没有单个ID具有足够的多视角样本。" + failures.str());
  return best_id;
}

void CameraCalibrator::solveIntrinsics(CalibrationResult &result) const
{
  require(result.fit_views.size() >= static_cast<std::size_t>(options_.min_fit_views),
          "拟合样本不足，未调用求解器");
  const auto points = makeObjectPoints();
  std::vector<std::vector<cv::Point3f>> object_points(result.fit_views.size(), points);
  std::vector<std::vector<cv::Point2f>> pixels;
  for (const auto &view : result.fit_views)
  {
    pixels.push_back(imagePoints(view));
  }
  const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                  options_.solver_max_iterations, options_.solver_epsilon);
  // 首版pinhole 5项畸变，不开启rational/thin-prism等额外自由度。
  // 平面Z=0让OpenCV初始化K，每帧标签→相机外参同时求解；没有自写优化器。
  result.rms_px =
      cv::calibrateCamera(object_points, pixels, result.image_size, result.camera_matrix,
                          result.dist_coeffs, result.rvecs, result.tvecs, 0, criteria);
}

double CameraCalibrator::checkFocalStability(const CalibrationResult &result) const
{
  // OpenCV 4.6 calibration.cpp标准差分母用了total_points-nparams，四点/帧时
  // 会为负，返回NaN；K/D优化本身不受此影响。这里不修改系统库，也不伪造标准差。
  // 将已按时间排序的拟合视角交替分两组，分别调用现成求解器，测焦距的变化。
  // 这是数据子集稳定性检查，不是统计置信区间；独立米制验证仍然必需。
  double maximum_change = 0.0;
  const auto points = makeObjectPoints();
  for (std::size_t part = 0; part < 2; ++part)
  {
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> pixels;
    for (std::size_t i = part; i < result.fit_views.size(); i += 2)
    {
      object_points.push_back(points);
      pixels.push_back(imagePoints(result.fit_views[i]));
    }
    require(pixels.size() >= 5, "焦距稳定性检查的每组须至少5个视角");
    cv::Mat subset_camera;
    cv::Mat subset_distortion;
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                    options_.solver_max_iterations, options_.solver_epsilon);
    cv::calibrateCamera(object_points, pixels, result.image_size, subset_camera, subset_distortion,
                        rotations, translations, 0, criteria);
    require(cv::checkRange(subset_camera), "子集求解焦距非有限，数据约束不足");
    for (int axis = 0; axis < 2; ++axis)
    {
      const double reference = result.camera_matrix.at<double>(axis, axis);
      const double subset = subset_camera.at<double>(axis, axis);
      require(subset > 0.0, "子集求解焦距非正");
      maximum_change = std::max(maximum_change, std::abs(subset - reference) / reference);
    }
  }
  return maximum_change;
}

void CameraCalibrator::evaluateResult(CalibrationResult &result) const
{
  require(result.camera_matrix.size() == cv::Size(3, 3) && result.dist_coeffs.total() == 5 &&
              cv::checkRange(result.camera_matrix) && cv::checkRange(result.dist_coeffs),
          "求解参数为空/非有限或维度不符合pinhole模型");
  const double fx_px = result.camera_matrix.at<double>(0, 0);
  const double fy_px = result.camera_matrix.at<double>(1, 1);
  const double cx_px = result.camera_matrix.at<double>(0, 2);
  const double cy_px = result.camera_matrix.at<double>(1, 2);
  require(fx_px > 0.0 && fy_px > 0.0 && cx_px >= 0.0 && cy_px >= 0.0 &&
              cx_px < result.image_size.width && cy_px < result.image_size.height,
          "焦距非正或主点落在原图之外；核视角退化/尺寸/模型");
  result.relative_focal_change = checkFocalStability(result);
  require(result.relative_focal_change <= options_.max_relative_focal_change,
          "子集焦距相对变化=" + std::to_string(result.relative_focal_change) +
              "，超过max_relative_focal_change；检查倾角/尺度/边缘约束");
  const auto points = makeObjectPoints();
  require(result.rvecs.size() == result.fit_views.size() &&
              result.tvecs.size() == result.fit_views.size(),
          "求解外参数量与拟合视角数不一致");
  std::vector<cv::Vec3d> normals;
  result.fit_errors_px.clear();
  result.holdout_errors_px.clear();
  result.normal_spread_deg = 0.0;
  for (std::size_t i = 0; i < result.fit_views.size(); ++i)
  {
    require(hasPositiveDepth(points, result.rvecs[i], result.tvecs[i]),
            "拟合视角出现标签在相机后方的解");
    cv::Mat rotation;
    cv::Rodrigues(result.rvecs[i], rotation);
    const cv::Vec3d normal(rotation.at<double>(0, 2), rotation.at<double>(1, 2),
                           rotation.at<double>(2, 2));
    for (const auto &previous : normals)
    {
      const double cosine = std::clamp(normal.dot(previous), -1.0, 1.0);
      result.normal_spread_deg =
          std::max(result.normal_spread_deg, std::acos(cosine) * 180.0 / kPi);
    }
    normals.push_back(normal);
    result.fit_errors_px.push_back(reprojectionError(points, result.fit_views[i],
                                                     result.camera_matrix, result.dist_coeffs,
                                                     result.rvecs[i], result.tvecs[i]));
  }
  // 留出视角仅估计自身外参，保持拟合所得K/D固定；这仍非独立米制验收。
  for (const auto &view : result.holdout_views)
  {
    cv::Mat rvec;
    cv::Mat tvec;
    const bool solved = cv::solvePnP(points, imagePoints(view), result.camera_matrix,
                                     result.dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    require(solved && hasPositiveDepth(points, rvec, tvec), "留出视角无法得到有效正深度平面解");
    result.holdout_errors_px.push_back(
        reprojectionError(points, view, result.camera_matrix, result.dist_coeffs, rvec, tvec));
  }
  require(!result.holdout_errors_px.empty(), "没有留出视角，不能进行本批图像检查");
  result.holdout_rms_px = aggregateRms(result.holdout_errors_px);
  require(result.normal_spread_deg >= options_.min_normal_spread_deg,
          "法线变化只有" + std::to_string(result.normal_spread_deg) +
              "度，低于min_normal_spread_deg；补充俯仰/横滚变化");
  require(std::isfinite(result.rms_px) && result.rms_px <= options_.max_rms_px,
          "拟合RMS=" + std::to_string(result.rms_px) + "px，超出max_rms_px");
  require(std::isfinite(result.holdout_rms_px) &&
              result.holdout_rms_px <= options_.max_holdout_rms_px,
          "留出RMS=" + std::to_string(result.holdout_rms_px) + "px，超出max_holdout_rms_px");
  // OpenCV求解没有强制畸变映射可逆。抽查原图边界覆盖到的半径，拒绝径向折返。
  double max_radius_squared = 0.0;
  for (const double u : {0.0, static_cast<double>(result.image_size.width - 1)})
  {
    for (const double v : {0.0, static_cast<double>(result.image_size.height - 1)})
    {
      const double x = (u - cx_px) / fx_px;
      const double y = (v - cy_px) / fy_px;
      max_radius_squared = std::max(max_radius_squared, x * x + y * y);
    }
  }
  const double k1 = result.dist_coeffs.at<double>(0);
  const double k2 = result.dist_coeffs.at<double>(1);
  const double k3 = result.dist_coeffs.at<double>(4);
  for (int i = 0; i <= 256; ++i)
  {
    const double r2 = max_radius_squared * static_cast<double>(i) / 256.0;
    const double derivative = 1.0 + 3.0 * k1 * r2 + 5.0 * k2 * r2 * r2 + 7.0 * k3 * r2 * r2 * r2;
    require(std::isfinite(derivative) && derivative > 0.0,
            "径向映射出现折返；边缘约束/模型不足，不能只看中心低残差");
  }
}

void CameraCalibrator::saveResult(const std::string &video_path, const std::string &output_dir,
                                  const CalibrationResult &result) const
{
  namespace fs = std::filesystem;
  const fs::path directory(output_dir);
  cv::FileStorage report((directory / "calibration_report.yaml").string(), cv::FileStorage::WRITE);
  require(report.isOpened(), "无法写calibration_report.yaml");
  report << "status" << (result.success ? "candidate_requires_metric_validation" : "failed");
  report << "message" << result.message << "source_video" << video_path;
  report << "tool_version" << ROV_LOCALIZATION_VERSION;
  report << "opencv_version" << CV_VERSION << "apriltag_version" << ROV_APRILTAG_VERSION;
  report << "image_width" << result.image_size.width << "image_height" << result.image_size.height;
  report << "selected_id" << result.target_id;
  report << "decoded_frames" << static_cast<double>(statistics_.decoded_frames);
  report << "sampled_frames" << static_cast<double>(statistics_.sampled_frames);
  report << "reported_frame_count" << statistics_.reported_frame_count << "fps" << statistics_.fps;
  report << "frame_count_difference" << statistics_.frame_count_difference;
  report << "frame_count_warning" << statistics_.frame_count_warning;
  report << "video_backend" << statistics_.video_backend;
  report << "video_time_source" << "frame_index / file_fps; not hardware exposure time";
  report << "eof_limit"
         << "read=false cannot distinguish EOF from decoder failure; frame count may be estimated";
  report << "processing_s" << statistics_.processing_s;
  report << "fit_views" << static_cast<int>(result.fit_views.size());
  report << "holdout_views" << static_cast<int>(result.holdout_views.size());
  report << "rms_px" << result.rms_px << "holdout_rms_px" << result.holdout_rms_px;
  report << "image_span_x" << result.image_span_x << "image_span_y" << result.image_span_y;
  report << "normal_spread_deg" << result.normal_spread_deg;
  report << "focal_split_relative_change" << result.relative_focal_change;
  report << "focal_check_method" << "two subset refits versus full fit; not statistical stddev";
  report << "calibration_flags" << 0;
  report << "tags" << "[";
  for (const auto &[id, stats] : statistics_.tags)
  {
    const auto candidate = candidates_.find(id);
    const auto retained = candidate == candidates_.end() ? 0 : candidate->second.size();
    report << "{" << "id" << id << "detected" << static_cast<double>(stats.detected)
           << "quality_accepted" << static_cast<double>(stats.quality_accepted)
           << "retained_candidates" << static_cast<int>(retained) << "}";
  }
  report << "]" << "rejections" << "{";
  for (const auto &[reason, count] : statistics_.rejected)
  {
    report << reason << static_cast<double>(count);
  }
  report << "}";
  writeOptions(report, options_);
  report.release();

  std::ofstream csv(directory / "selected_views.csv");
  require(csv.is_open(), "无法写selected_views.csv");
  csv << "subset,frame_index,video_time_s,id,blur_variance,rms_px,x0,y0,x1,y1,x2,y2,x3,y3\n";
  csv << std::setprecision(12);
  const auto write_views = [&csv](const char *subset, const std::vector<CalibrationView> &views,
                                  const std::vector<double> &errors)
  {
    for (std::size_t i = 0; i < views.size(); ++i)
    {
      const auto &view = views[i];
      csv << subset << ',' << view.frame_index << ',' << view.video_time_s << ','
          << view.detection.id << ',' << view.blur_variance << ',';
      if (i < errors.size())
      {
        csv << errors[i];
      }
      for (const auto &corner : view.detection.corners_px)
      {
        csv << ',' << corner.x << ',' << corner.y;
      }
      csv << '\n';
    }
  };
  write_views("fit", result.fit_views, result.fit_errors_px);
  write_views("holdout", result.holdout_views, result.holdout_errors_px);
  csv.close();
  require(!csv.fail(), "selected_views.csv写入失败");
  if (!result.success)
  {
    return;
  }
  // 先写临时XML，再改名；若写盘中断，不留下看似完成的camera_wet.xml。
  const fs::path temporary = directory / ".camera_wet.tmp.xml";
  cv::FileStorage output(temporary.string(), cv::FileStorage::WRITE);
  require(output.isOpened(), "无法写相机XML");
  output << "status" << "candidate_requires_metric_validation";
  output << "tool_version" << ROV_LOCALIZATION_VERSION;
  output << "camera_model" << options_.camera_model << "distortion_model" << "plumb_bob";
  output << "image_width" << result.image_size.width << "image_height" << result.image_size.height;
  output << "camera_matrix" << result.camera_matrix << "dist_coeffs" << result.dist_coeffs;
  output << "source_video" << video_path << "tag_family" << options_.tag_family;
  output << "tag_width_m" << options_.tag_width_m << "tag_height_m" << options_.tag_height_m;
  output << "target_id" << result.target_id << "rms_px" << result.rms_px;
  output << "holdout_rms_px" << result.holdout_rms_px;
  output << "opencv_version" << CV_VERSION << "apriltag_version" << ROV_APRILTAG_VERSION;
  output.release();
  fs::rename(temporary, directory / "camera_wet.xml");
}

CalibrationResult CameraCalibrator::run(const std::string &video_path,
                                        const std::string &output_dir)
{
  namespace fs = std::filesystem;
  require(fs::is_regular_file(video_path), "输入视频不存在或不是普通文件：" + video_path);
  const fs::path directory(output_dir);
  require(!directory.empty(), "输出目录为空");
  // create_directory在已存在时返回false，避免先exists再写入导致的覆盖竞争。
  // 只创建用户明确指定的新结果目录，父目录须已存在，不自动造多层未知路径。
  require(fs::create_directory(directory),
          "输出目录已存在，拒绝覆盖；请换一个新目录：" + output_dir);
  candidates_.clear();
  statistics_ = ScanStatistics{};
  CalibrationResult result;
  const auto start = std::chrono::steady_clock::now();
  try
  {
    scanVideo(video_path, result);
    result.target_id = selectTargetId(result.image_size);
    selectViews(candidates_.at(result.target_id), result);
    std::cout << "选中ID " << result.target_id << "，拟合 " << result.fit_views.size() << "，留出 "
              << result.holdout_views.size() << "；开始求解\n";
    solveIntrinsics(result);
    evaluateResult(result);
    result.success = true;
    result.message = "图像标定检查通过；参数为候选，仍需独立尺量位移/高度/边缘验收";
  }
  catch (const std::exception &error)
  {
    result.message = error.what();
  }
  const auto finish = std::chrono::steady_clock::now();
  statistics_.processing_s = std::chrono::duration<double>(finish - start).count();
  // 成功或失败都保存诊断。保存本身的异常上抛给main，不能打印成正常完成。
  saveResult(video_path, output_dir, result);
  return result;
}
} // namespace rov_localization

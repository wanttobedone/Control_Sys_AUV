#include "rov_localization/apriltag_detector.h"
#include "rov_localization/tag_pose_estimator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <stdexcept>

namespace
{
namespace rl = rov_localization;
namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;

// 信号处理只置标志。CSV刷新、窗口和设备释放仍由主线程正常退出时完成。
void onInterrupt(int)
{
  interrupted = 1;
}

// 每个读入帧都写一行，包括丢检/歧义/质量不合格。候选位姿明确以candidate命名；
// 无候选时写nan，不会把结构体初始零或上次位置写成新的有效观测。
void writePoseRow(std::ostream &stream, std::int64_t index, double time_s,
                  const rl::TagPoseOptions &options,
                  const std::vector<rl::TagDetection> &detections, const rl::TagPoseEstimate &pose,
                  double processing_ms)
{
  const rl::TagDetection *target = nullptr;
  int target_count = 0;
  for (const auto &detection : detections)
  {
    if (detection.id == options.target_id)
    {
      target = &detection;
      ++target_count;
    }
  }
  stream << index << ',' << time_s << ',' << options.target_id << ',' << target_count << ','
         << pose.valid << ',' << pose.ambiguous << ',' << pose.reason << ','
         << pose.reprojection_error_px << ',' << pose.second_error_px << ','
         << pose.solution_rotation_gap_deg;
  const double missing = std::numeric_limits<double>::quiet_NaN();
  for (int axis = 0; axis < 3; ++axis)
  {
    stream << ',' << (pose.has_pose ? pose.p_LC_m[axis] : missing);
  }
  for (int axis = 0; axis < 3; ++axis)
  {
    stream << ',' << (pose.has_pose ? pose.rpy_LC_deg[axis] : missing);
  }
  stream << ',' << pose.height_above_plane_m << ',' << processing_ms;
  for (int row = 0; row < 3; ++row)
  {
    for (int column = 0; column < 3; ++column)
    {
      stream << ',' << (pose.has_pose ? pose.R_LC(row, column) : missing);
    }
  }
  for (std::size_t corner = 0; corner < 4; ++corner)
  {
    stream << ','
           << (target_count == 1 ? static_cast<double>(target->corners_px[corner].x) : missing)
           << ','
           << (target_count == 1 ? static_cast<double>(target->corners_px[corner].y) : missing);
  }
  stream << '\n';
}

// 显示原图和原D投影的坐标轴。不要先校正图像再重复应用原D。
// 窗口用ASCII文字，避免OpenCV putText不支持中文造成方框；终端仍使用中文说明。
void showPose(cv::Mat &frame, const rl::TagPoseEstimator &estimator,
              const std::vector<rl::TagDetection> &detections, const rl::TagPoseEstimate &pose)
{
  const cv::Scalar color = pose.valid ? cv::Scalar(0, 160, 0) : cv::Scalar(0, 80, 230);
  for (const auto &detection : detections)
  {
    if (detection.id != estimator.options().target_id)
    {
      continue;
    }
    for (std::size_t corner = 0; corner < 4; ++corner)
    {
      const cv::Point first(cvRound(detection.corners_px[corner].x),
                            cvRound(detection.corners_px[corner].y));
      const auto &next = detection.corners_px[(corner + 1) % 4];
      cv::line(frame, first, cv::Point(cvRound(next.x), cvRound(next.y)), color, 2);
      cv::putText(frame, "p" + std::to_string(corner), first, cv::FONT_HERSHEY_SIMPLEX, 0.55, color,
                  2);
    }
  }
  std::ostringstream status;
  status << "ID " << estimator.options().target_id << "  " << pose.reason;
  cv::putText(frame, status.str(), cv::Point(16, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2);
  if (pose.has_pose)
  {
    cv::drawFrameAxes(frame, estimator.camera().camera_matrix, estimator.camera().dist_coeffs,
                      pose.rvec_CL_rad, pose.t_CL_m,
                      static_cast<float>(estimator.options().tag_width_m * 0.5));
    std::ostringstream position;
    position << std::fixed << std::setprecision(3) << "candidate p_LC [m]: " << pose.p_LC_m[0]
             << "  " << pose.p_LC_m[1] << "  " << pose.p_LC_m[2]
             << "   height=" << pose.height_above_plane_m;
    cv::putText(frame, position.str(), cv::Point(16, 56), cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
    std::ostringstream angles;
    angles << std::fixed << std::setprecision(2) << "camera RPY [deg]: " << pose.rpy_LC_deg[0]
           << "  " << pose.rpy_LC_deg[1] << "  " << pose.rpy_LC_deg[2]
           << "   RMS [px]=" << pose.reprojection_error_px;
    cv::putText(frame, angles.str(), cv::Point(16, 82), cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
  }
  cv::imshow("ROV tag pose - read only (q / Esc to quit)", frame);
}

// 主流程独占采集器、检测器、CSV和计数，无ROS回调或跨线程共享位姿。
// 原生AprilTag内部工作线程由detect同步等待；读取下一帧才触发下一次求解。
int runPreview(cv::CommandLineParser &parser)
{
  const auto video = parser.get<std::string>("video");
  const auto device = parser.get<std::string>("device");
  const auto camera_path = parser.get<std::string>("camera");
  const auto config_path = parser.get<std::string>("config");
  const fs::path output = parser.get<std::string>("output");
  const bool display = parser.get<bool>("display");
  const int max_frames = parser.get<int>("max-frames");
  if (!parser.check() || video.empty() == device.empty() || camera_path.empty() ||
      config_path.empty() || output.empty() || max_frames < 0)
  {
    parser.printErrors();
    throw std::invalid_argument(
        "需要--camera --config --output及video/device二选一；max-frames须>=0");
  }
  auto options = rl::loadTagPoseOptions(config_path);
  if (parser.has("tag-id"))
  {
    options.target_id = parser.get<int>("tag-id");
    if (!parser.check())
    {
      throw std::invalid_argument("tag-id须为整数");
    }
  }
  const rl::TagPoseEstimator estimator(rl::loadCameraModel(camera_path), options);
  rl::AprilTagDetector detector;
  detector.configure(options.detector_threads, options.max_hamming);
  const bool live = !device.empty();
  cv::VideoCapture capture;
  if (live)
  {
    capture.open(device, cv::CAP_V4L2);
    // 明确请求与标定一致的分辨率；实际首帧还会复核，不按set返回值假定成功。
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FRAME_WIDTH, estimator.camera().image_size.width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, estimator.camera().image_size.height);
    capture.set(cv::CAP_PROP_FPS, options.camera_fps);
  }
  else
  {
    if (!fs::is_regular_file(video))
    {
      throw std::invalid_argument("输入录像不是可读普通文件：" + video);
    }
    capture.open(video);
  }
  if (!capture.isOpened())
  {
    throw std::runtime_error("无法打开图像输入：" + (live ? device : video));
  }
  const double input_fps = capture.get(cv::CAP_PROP_FPS);
  if (!live && (!std::isfinite(input_fps) || input_fps <= 0.0))
  {
    throw std::runtime_error("录像FPS无效，不能构造frame_index/fps时间轴");
  }
  cv::Mat frame;
  if (!capture.read(frame) || frame.empty() || frame.size() != estimator.camera().image_size)
  {
    throw std::runtime_error("首帧为空或尺寸与相机XML不一致；须使用标定时的原图模式");
  }
  // 只有输入检查通过后才创建新目录；不存在的父目录和已有结果均不自动处理。
  if (!fs::create_directory(output))
  {
    throw std::runtime_error("输出目录已存在，拒绝覆盖：" + output.string());
  }
  // 保存本次实际输入，包括命令行覆盖的ID；不修改用户XML或默认YAML。
  fs::copy_file(camera_path, output / "camera_used.xml");
  fs::copy_file(config_path, output / "options_source.yaml");
  std::ofstream csv(output / "pose.csv");
  csv.exceptions(std::ios::badbit | std::ios::failbit);
  csv << std::setprecision(12)
      << "frame_index,time_s,target_id,target_count,valid,ambiguous,reason,rms_px,second_rms_px,"
         "rotation_gap_deg,candidate_x_L_m,candidate_y_L_m,candidate_z_L_m,"
         "candidate_roll_LC_deg,candidate_pitch_LC_deg,candidate_yaw_LC_deg,height_m,processing_ms,"
         "R_LC00,R_LC01,R_LC02,R_LC10,R_LC11,R_LC12,R_LC20,R_LC21,R_LC22,"
         "u0_px,v0_px,u1_px,v1_px,u2_px,v2_px,u3_px,v3_px\n";
  const auto start = SteadyClock::now();
  std::int64_t frames = 0;
  std::int64_t valid_frames = 0;
  std::int64_t ambiguous_frames = 0;
  std::string previous_reason;
  std::string stop_reason = "end_of_video";
  std::string failure;
  std::cout << "只读定位启动：ID=" << options.target_id
            << "，p_LC为相机在标签系的位置[m]，height=abs(z)；窗口按q退出，终端Ctrl+C退出\n";
  try
  {
    while (true)
    {
      if (interrupted != 0)
      {
        stop_reason = "interrupted";
        break;
      }
      if (frame.size() != estimator.camera().image_size)
      {
        throw std::runtime_error("输入图像尺寸在运行中变化，停止使用旧内参");
      }
      const auto received = SteadyClock::now();
      // USB时间为read返回后的steady_clock间隔；它不是硬件曝光时刻。
      const double time_s = live ? std::chrono::duration<double>(received - start).count()
                                 : static_cast<double>(frames) / input_fps;
      cv::Mat gray;
      if (frame.type() == CV_8UC3)
      {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      }
      else if (frame.type() == CV_8UC1)
      {
        gray = frame;
      }
      else
      {
        throw std::runtime_error("需要8位灰度或BGR原始图像");
      }
      const auto detections = detector.detect(gray);
      const auto pose = estimator.estimate(detections);
      const double processing_ms =
          std::chrono::duration<double, std::milli>(SteadyClock::now() - received).count();
      writePoseRow(csv, frames, time_s, options, detections, pose, processing_ms);
      ++frames;
      valid_frames += pose.valid ? 1 : 0;
      ambiguous_frames += pose.ambiguous ? 1 : 0;
      if (pose.reason != previous_reason || frames % 30 == 0)
      {
        std::cout << "frame=" << frames - 1 << " time=" << time_s << "s state=" << pose.reason;
        if (pose.has_pose)
        {
          std::cout << " candidate p_LC=" << pose.p_LC_m << "m RMS=" << pose.reprojection_error_px;
        }
        else
        {
          std::cout << " seen_ids=";
          for (const auto &detection : detections)
          {
            std::cout << detection.id << ' ';
          }
        }
        std::cout << '\n';
        previous_reason = pose.reason;
      }
      if (display)
      {
        if (frame.channels() == 1)
        {
          cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);
        }
        showPose(frame, estimator, detections, pose);
        const double remaining_ms = live ? 1.0 : 1000.0 / input_fps - processing_ms;
        const int key = cv::waitKey(static_cast<int>(std::clamp(remaining_ms, 1.0, 1000.0))) & 0xff;
        if (key == 'q' || key == 27)
        {
          stop_reason = "user_quit";
          break;
        }
      }
      if (max_frames > 0 && frames >= max_frames)
      {
        stop_reason = "max_frames";
        break;
      }
      if (!capture.read(frame) || frame.empty())
      {
        if (live && interrupted == 0)
        {
          throw std::runtime_error("相机读取中断");
        }
        stop_reason = interrupted != 0 ? "interrupted" : "end_of_video";
        break;
      }
    }
  }
  catch (const std::exception &error)
  {
    failure = error.what();
    stop_reason = "runtime_error";
  }
  csv.close();
  capture.release();
  if (display)
  {
    cv::destroyAllWindows();
  }
  cv::FileStorage report((output / "run_info.yaml").string(), cv::FileStorage::WRITE);
  if (!report.isOpened())
  {
    throw std::runtime_error("无法写run_info.yaml");
  }
  report << "tool_version" << ROV_LOCALIZATION_VERSION << "source" << (live ? device : video);
  report << "camera_source" << camera_path << "config_source" << config_path;
  report << "camera_status" << estimator.camera().calibration_status;
  report << "image_width" << estimator.camera().image_size.width << "image_height"
         << estimator.camera().image_size.height;
  report << "target_id" << options.target_id << "tag_width_m" << options.tag_width_m
         << "tag_height_m" << options.tag_height_m;
  report << "max_reprojection_error_px" << options.max_reprojection_error_px
         << "ambiguity_error_gap_px" << options.ambiguity_error_gap_px << "ambiguity_rotation_deg"
         << options.ambiguity_rotation_deg;
  report << "requested_camera_fps" << options.camera_fps << "reported_input_fps" << input_fps;
  report << "time_source"
         << (live ? "steady_clock after read; not exposure time"
                  : "frame_index / file_fps; not exposure time");
  report << "frame_convention" << "L: tag center, X=p0->p1, Y=p3->p0, Z=X cross Y; C: optical";
  report << "pose_convention" << "p_LC=-R_CL^T*t_CL; RPY from R_LC=Rz(yaw)*Ry(pitch)*Rx(roll)";
  report << "metric_validation" << "not yet validated; candidate columns require valid=1";
  report << "read_limit" << "video read=false cannot distinguish EOF from decoder failure";
  report << "frames" << static_cast<double>(frames) << "valid_frames"
         << static_cast<double>(valid_frames) << "ambiguous_frames"
         << static_cast<double>(ambiguous_frames);
  report << "stop_reason" << stop_reason << "error" << failure;
  std::cout << "结束：" << frames << "帧，有效" << valid_frames << "帧，歧义" << ambiguous_frames
            << "帧；结果：" << fs::absolute(output).string() << '\n';
  if (!failure.empty())
  {
    std::cerr << failure << '\n';
    return 1;
  }
  return valid_frames > 0 ? 0 : 2;
}
} // namespace

int main(int argc, char **argv)
{
  const cv::String keys = "{help h||显示帮助}"
                          "{video||只读录像路径，与device二选一}"
                          "{device||V4L2设备路径，例如/dev/video3}"
                          "{camera||calibrate_camera输出的camera_wet.xml}"
                          "{config||tag_pose_options.yaml}"
                          "{output||尚不存在的结果目录，父目录须存在}"
                          "{tag-id||覆盖配置中的单个目标ID}"
                          "{display|true|是否打开图像窗口}"
                          "{max-frames|0|最多处理帧数，0处理到结束或手动退出}";
  cv::CommandLineParser parser(argc, argv, keys);
  if (argc == 1 || parser.has("help"))
  {
    parser.printMessage();
    return 0;
  }
  std::signal(SIGINT, onInterrupt);
  std::signal(SIGTERM, onInterrupt);
  try
  {
    return runPreview(parser);
  }
  catch (const std::exception &error)
  {
    std::cerr << "只读定位失败：" << error.what() << '\n';
    return 1;
  }
}

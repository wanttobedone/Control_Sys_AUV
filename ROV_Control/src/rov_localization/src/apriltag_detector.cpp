#include "rov_localization/apriltag_detector.h"

#include <limits>
#include <stdexcept>
extern "C"
{
#include <apriltag.h>
#include <tag36h11.h>
}

namespace rov_localization
{
AprilTagDetector::~AprilTagDetector()
{
  release();
}

void AprilTagDetector::release()
{
  // detector不负责释放family：必须先结束检测器，再释放它引用的编码家族。
  if (detector_ != nullptr)
  {
    apriltag_detector_destroy(detector_);
    detector_ = nullptr;
  }
  if (family_ != nullptr)
  {
    tag36h11_destroy(family_);
    family_ = nullptr;
  }
}

void AprilTagDetector::configure(int threads, int max_hamming)
{
  if (threads < 1 || max_hamming < 0 || max_hamming > 2)
  {
    throw std::invalid_argument("detector_threads须>=1；max_hamming须在原生库支持的0..2内");
  }
  release();
  family_ = tag36h11_create();
  detector_ = apriltag_detector_create();
  if (family_ == nullptr || detector_ == nullptr)
  {
    release();
    throw std::runtime_error("无法创建AprilTag原生资源");
  }
  apriltag_detector_add_family_bits(detector_, family_, max_hamming);
  detector_->nthreads = threads;
  // 标定优先保留原图角点信息；只用官方refine_edges，不额外叠加角点细化。
  detector_->quad_decimate = 1.0F;
  detector_->refine_edges = true;
}

std::vector<TagDetection> AprilTagDetector::detect(const cv::Mat &gray)
{
  if (detector_ == nullptr || gray.empty() || gray.type() != CV_8UC1 ||
      gray.step > static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("detect需要已配置检测器及步长可表示的8位单通道灰度图");
  }
  // 仅借用cv::Mat内存。step可能大于cols（ROI或对齐），必须传真实stride。
  image_u8_t input = {gray.cols, gray.rows, static_cast<int>(gray.step), gray.data};
  zarray_t *detections = apriltag_detector_detect(detector_, &input);
  std::vector<TagDetection> result;
  try
  {
    for (int i = 0; i < zarray_size(detections); ++i)
    {
      apriltag_detection_t *raw = nullptr;
      zarray_get(detections, i, &raw);
      TagDetection copy;
      copy.id = raw->id;
      copy.hamming = raw->hamming;
      copy.decision_margin = raw->decision_margin;
      for (std::size_t corner = 0; corner < 4; ++corner)
      {
        copy.corners_px[corner].x = static_cast<float>(raw->p[corner][0]);
        copy.corners_px[corner].y = static_cast<float>(raw->p[corner][1]);
      }
      result.push_back(copy);
    }
  }
  catch (...)
  {
    apriltag_detections_destroy(detections);
    throw;
  }
  apriltag_detections_destroy(detections);
  return result;
}
} // namespace rov_localization

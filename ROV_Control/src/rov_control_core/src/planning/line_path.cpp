/**
 * 直线路径规划与实际状态投影。
 *
 * plan()：检查请求 → 计算长度和方向 → 检查可选执行条件 → 返回路径。
 * projectProgress()：把实际位置/速度分解成沿线分量和偏离路径的分量。
 *
 * 本文件不生成时间轨迹，也不把位置误差换算成推进器控制量。
 */

#include "rov_control_core/planning/line_path.h"

#include <algorithm>
#include <cmath>

#include "rov_control_core/planning/workspace_bounds.h"

namespace rov_control_core
{

// 只在本文件使用的辅助函数，放在匿名命名空间中，避免暴露成公共接口。
namespace
{

/**
 * 检查首轮 Hippo 前向直线实验的启动条件。
 * 输入：起始姿态/速度及允许阈值，世界系路径单位方向。
 * 输出：检查结果；不修改输入，也不保存内部状态。
 */
Status checkForwardStart(const ForwardStartConstraint &constraint,
                         const Eigen::Vector3d &direction)
{
  constexpr double HALF_PI = 1.57079632679489661923;
  constexpr double DIRECTION_TOLERANCE = 1e-10;  // 单位方向的数值容差，无量纲

  // 1. 检查阈值。角度必须小于 90°，否则“近水平、沿前向”失去本阶段含义。
  if (!std::isfinite(constraint.max_start_speed_mps) ||
      !std::isfinite(constraint.max_tilt_rad) ||
      !std::isfinite(constraint.max_heading_error_rad) ||
      constraint.max_start_speed_mps < 0.0 ||
      constraint.max_tilt_rad < 0.0 || constraint.max_tilt_rad >= HALF_PI ||
      constraint.max_heading_error_rad < 0.0 ||
      constraint.max_heading_error_rad >= HALF_PI)
  {
    return {ErrorCode::INVALID_LIMITS, "前向启动的速度或角度阈值不合法"};
  }

  // 2. 姿态必须是有效的单位四元数。只容许数值舍入误差，不修正明显错误输入。
  double quaternionNorm = constraint.q_WB.coeffs().stableNorm();
  if (!constraint.q_WB.coeffs().allFinite() || !std::isfinite(quaternionNorm) ||
      std::abs(quaternionNorm - 1.0) > QUATERNION_NORM_TOLERANCE)
  {
    return {ErrorCode::INVALID_QUATERNION, "起始姿态必须为接近单位长度的四元数"};
  }

  // 3. 五次时间律按静止起步设计，不能在已有较大速度时直接套用。
  if (!constraint.v_W.allFinite())
  {
    return {ErrorCode::NON_FINITE, "起始速度必须为有限数值"};
  }
  if (constraint.v_W.stableNorm() > constraint.max_start_speed_mps)
  {
    return {ErrorCode::START_MOVING, "起始速度超过静止起步允许值"};
  }

  // 4. 机体 z 轴转到世界系后，与世界系 z 轴的夹角就是整体倾斜角。
  // clamp 将点积约束到 [-1,1]，避免浮点误差导致 acos 无定义。
  Eigen::Quaterniond attitude = constraint.q_WB.normalized();
  Eigen::Vector3d bodyZWorld = attitude * Eigen::Vector3d::UnitZ();
  double tilt = std::acos(std::clamp(bodyZWorld.z(), -1.0, 1.0));
  if (tilt > constraint.max_tilt_rad)
  {
    return {ErrorCode::START_TILTED, "初始姿态倾斜超过允许值"};
  }

  // 5. 当前执行方案只跟踪水平前向直线，不把上下运动也视为已支持。
  if (std::abs(direction.z()) > DIRECTION_TOLERANCE)
  {
    return {ErrorCode::UNSUPPORTED_DIRECTION, "首轮执行方案要求水平直线"};
  }

  // 6. 将机体 x 轴投影到世界水平面，得到当前水平朝向。
  Eigen::Vector3d heading = attitude * Eigen::Vector3d::UnitX();
  heading.z() = 0.0;
  double headingNorm = heading.stableNorm();
  if (headingNorm < DIRECTION_TOLERANCE)
  {
    return {ErrorCode::UNSUPPORTED_DIRECTION, "当前姿态无法确定水平朝向"};
  }
  heading /= headingNorm;

  // 7. 路径方向要接近当前朝向。目标在身后或侧方时不能直接纵向跟踪。
  double headingError = std::acos(std::clamp(heading.dot(direction), -1.0, 1.0));
  if (headingError > constraint.max_heading_error_rad)
  {
    return {ErrorCode::UNSUPPORTED_DIRECTION, "目标不在允许的前向路径方向内"};
  }

  return {};
}

}  // 匿名命名空间

// 构造函数只保存已经通过 plan() 检查的数据。
LinePath::LinePath(const Eigen::Vector3d &start,
                   const Eigen::Vector3d &goal,
                   const Eigen::Vector3d &direction,
                   double length)
    : start_(start), goal_(goal), direction_(direction), length_(length)
{
}

const Eigen::Vector3d &LinePath::startPosition() const
{
  return start_;
}

const Eigen::Vector3d &LinePath::goalPosition() const
{
  return goal_;
}

const Eigen::Vector3d &LinePath::direction() const
{
  return direction_;
}

double LinePath::length() const
{
  return length_;
}

Result<LinePath> LinePathPlanner::plan(const PlanRequest &request)
{
  // 1. 检查空间配置。保留具体错误原因，供上层决定怎样提示用户。
  Status boundsStatus = validateBounds(request.bounds);
  if (!boundsStatus.ok())
  {
    return Result<LinePath>::failure(boundsStatus.code, boundsStatus.message);
  }

  // 2. 检查端点是否有效，以及整条参考线段是否在允许区域内。
  if (!request.p_start_W.allFinite() || !request.p_goal_W.allFinite())
  {
    return Result<LinePath>::failure(ErrorCode::NON_FINITE, "路径端点必须为有限数值");
  }
  if (!containsSegment(request.bounds, request.p_start_W, request.p_goal_W))
  {
    return Result<LinePath>::failure(ErrorCode::OUTSIDE_WORKSPACE, "路径超出允许工作空间");
  }

  // 3. 计算位移和路径长度：delta = p1-p0，L = ||delta||。
  // stableNorm 比先平方求和更不容易在大数输入时溢出。
  Eigen::Vector3d delta = request.p_goal_W - request.p_start_W;
  double length = delta.stableNorm();
  if (!delta.allFinite() || !std::isfinite(length))
  {
    return Result<LinePath>::failure(ErrorCode::NUMERICAL_RANGE, "路径长度无法可靠表示");
  }
  if (length <= MIN_PATH_LENGTH_M)
  {
    // 不能用零长度去归一化方向；上层应把这个结果理解为“无需运动”。
    return Result<LinePath>::failure(ErrorCode::NO_MOTION, "起点与终点在数值容差内重合");
  }

  // 4. 单位方向 d = delta/L，因此 d*s 的单位是 m。
  Eigen::Vector3d direction = delta / length;

  // 5. 几何规划可以不检查执行姿态；首轮执行节点需要显式传入这组约束。
  if (request.forward_start)
  {
    Status startStatus = checkForwardStart(*request.forward_start, direction);
    if (!startStatus.ok())
    {
      return Result<LinePath>::failure(startStatus.code, startStatus.message);
    }
  }

  // 6. 检查全部通过后才创建路径。路径里不保存速度、加速度或总时间。
  LinePath path(request.p_start_W, request.p_goal_W, direction, length);
  return Result<LinePath>::success(path);
}

Result<ProgressState> projectProgress(const LinePath &path, const RobotState &state)
{
  // 输入必须已经在世界系。ROS Odometry 的机体系速度要先在适配层旋转。
  if (!state.p_W.allFinite() || !state.v_W.allFinite())
  {
    return Result<ProgressState>::failure(ErrorCode::NON_FINITE, "位置与世界系速度必须为有限数值");
  }

  // 1. 相对起点的位置，单位 m。
  Eigen::Vector3d offset = state.p_W - path.startPosition();
  ProgressState progress;

  // 2. 点积得到沿线距离和沿线速度。
  // s 可小于 0 或大于 L；裁剪它会让控制器看不到后退或超调。
  progress.s_m = path.direction().dot(offset);
  progress.v_s_mps = path.direction().dot(state.v_W);

  // 3. 从总位移中减去沿线分量，剩下的就是垂直于路径的残差。
  progress.cross_track_W = offset - path.direction() * progress.s_m;

  // 世界 z 向上，深度向下，因此深度偏差取残差 z 分量的负值。
  progress.depth_offset_m = -progress.cross_track_W.z();

  // 4. 即使输入单个数有限，点积或减法仍可能溢出，所以再检查输出。
  if (!std::isfinite(progress.s_m) || !std::isfinite(progress.v_s_mps) ||
      !progress.cross_track_W.allFinite())
  {
    return Result<ProgressState>::failure(ErrorCode::NUMERICAL_RANGE, "路径投影计算溢出");
  }

  return Result<ProgressState>::success(progress);
}

}  // 命名空间 rov_control_core

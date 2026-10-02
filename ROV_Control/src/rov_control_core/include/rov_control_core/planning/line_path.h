/**
 * 直线路径与状态投影。
 *
 * 几何表达：p(s) = p0 + d*s，d 为单位方向，s 为沿线距离 [m]。
 * 路径只回答“经过哪里”，不决定速度和总时间；时间由轨迹生成器分配。
 */

#ifndef ROV_CONTROL_CORE_LINE_PATH_H_
#define ROV_CONTROL_CORE_LINE_PATH_H_

#include "rov_control_core/types.h"

namespace rov_control_core
{

class LinePath
{
public:
  const Eigen::Vector3d &startPosition() const;  // 起点，世界系 [m]
  const Eigen::Vector3d &goalPosition() const;  // 终点，世界系 [m]
  const Eigen::Vector3d &direction() const;     // 世界系单位方向，无量纲
  double length() const;                        // 路径总长度 [m]

private:
  // 由规划器检查完输入后再构造，避免外部创建零长度或非单位方向的路径。
  // friend 只让 LinePathPlanner 可以调用这个私有构造函数。
  friend class LinePathPlanner;
  LinePath(const Eigen::Vector3d &start,
           const Eigen::Vector3d &goal,
           const Eigen::Vector3d &direction,
           double length);

  Eigen::Vector3d start_;      // 世界系起点 [m]
  Eigen::Vector3d goal_;       // 保留原始终点，避免反算端点引入舍入误差 [m]
  Eigen::Vector3d direction_;  // 从起点指向终点的单位向量
  double length_;             // 路径长度 [m]，创建后不再改变
};

class LinePathPlanner
{
public:
  // 输入：起终点、有效空间及可选前向启动约束。
  // 输出：成功时返回只读路径；失败时返回越界、无需运动等原因。
  // 无内部历史状态，因此声明为 static，无需创建一个规划器对象。
  static Result<LinePath> plan(const PlanRequest &request);
};

// 输入：路径、实际状态。这里只用世界系 p_W/v_W，不在此转换坐标。
// 输出：沿线距离、沿线速度、垂直于路径的残差及深度偏差。
// 实际距离不裁剪到 [0,L]，这样后续控制器才能看到超调或起点后退。
Result<ProgressState> projectProgress(const LinePath &path, const RobotState &state);

}  // 命名空间 rov_control_core

#endif

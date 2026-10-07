/**
 * 已知空旷工作空间的几何检查。
 * 输入边界为世界系下的轴对齐凸盒；没有障碍物搜索、碰撞模型或 ROS 依赖。
 */

#ifndef ROV_CONTROL_CORE_WORKSPACE_BOUNDS_H_
#define ROV_CONTROL_CORE_WORKSPACE_BOUNDS_H_

#include "rov_control_core/types.h"

namespace rov_control_core
{

// 输入：各轴 min/max [m]。输出：参数合法性；任何轴 min>=max 都不合法。
Status validateBounds(const WorkspaceBounds &bounds);

// 输入：允许空间、待查世界系点 [m]。输出：是否在盒内，包含边界。
// bounds由参数/规划入口经validateBounds检查；这里仅做几何包含判断。
// 有效有限边界下，NaN/Inf点自然无法通过逐轴比较。
bool contains(const WorkspaceBounds &bounds, const Eigen::Vector3d &p_W);

// 输入：允许空间、线段两个端点 [m]。输出：整条线段是否位于盒内。
// 只对当前凸盒成立：两个端点都在凸集内，它们之间的线段也在凸集内。
bool containsSegment(const WorkspaceBounds &bounds,
                     const Eigen::Vector3d &p0_W,
                     const Eigen::Vector3d &p1_W);

}  // 命名空间 rov_control_core

#endif

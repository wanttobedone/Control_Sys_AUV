/**
 * 工作空间检查实现。
 *
 * 数据来源：调用者给出的有效空间，已经扣除机器人尺寸与余量。
 * 输出用途：plan() 先用这些函数拒绝越界参考，再生成几何路径。
 */

#include "rov_control_core/planning/workspace_bounds.h"

namespace rov_control_core
{

Status validateBounds(const WorkspaceBounds &bounds)
{
  // 先排除 NaN/Inf；这些值进入大小比较后容易掩盖错误配置。
  if (!bounds.min_W.allFinite() || !bounds.max_W.allFinite())
  {
    return {ErrorCode::NON_FINITE, "工作空间边界必须为有限数值"};
  }

  // 三个轴都必须有正宽度，默认的全零边界不能直接用于规划。
  if (!(bounds.min_W.array() < bounds.max_W.array()).all())
  {
    return {ErrorCode::INVALID_BOUNDS, "工作空间每个轴都必须满足 min < max"};
  }

  return {};
}

bool contains(const WorkspaceBounds &bounds, const Eigen::Vector3d &p_W)
{
  // 无效输入直接判为不在允许空间内。
  if (!validateBounds(bounds).ok() || !p_W.allFinite())
  {
    return false;
  }

  // Eigen 的 array() 在这里表示逐元素比较，all() 表示三个轴同时满足。
  bool aboveMinimum = (p_W.array() >= bounds.min_W.array()).all();
  bool belowMaximum = (p_W.array() <= bounds.max_W.array()).all();

  return aboveMinimum && belowMaximum;
}

bool containsSegment(const WorkspaceBounds &bounds,
                     const Eigen::Vector3d &p0_W,
                     const Eigen::Vector3d &p1_W)
{
  // 轴对齐盒子是凸集，两个端点都在里面即可保证整条直线位于盒内。
  // 若未来加入障碍物，不能继续只靠这两个端点判断碰撞。
  bool startInside = contains(bounds, p0_W);
  bool goalInside = contains(bounds, p1_W);

  return startInside && goalInside;
}

}  // 命名空间 rov_control_core

/**
 * 计算结果与错误信息：供路径规划、轨迹生成共用。
 *
 * Result<T> 中的 T 表示返回值类型，例如 Result<double> 返回一个数，
 * Result<LinePath> 返回一条路径。调用者先检查 ok()，成功后再取 value()。
 * 计算失败时只返回错误信息，不生成一个看似有效的默认路径。
 *
 * 本文件只处理返回结果，不参与路径公式、ROS 通信或运行状态机。
 */

#ifndef ROV_CONTROL_CORE_RESULT_H_
#define ROV_CONTROL_CORE_RESULT_H_

#include <optional>
#include <utility>

namespace rov_control_core
{

// 程序根据错误码选择处理方式；message 提供可直接打印的中文解释。
enum class ErrorCode
{
  OK,                     // 计算成功
  INVALID_ARGUMENT,       // 输入参数的基本含义不合法
  NON_FINITE,             // 输入中有 NaN 或无穷大
  INVALID_BOUNDS,         // 工作空间的最小值不小于最大值
  OUTSIDE_WORKSPACE,       // 参考路径超出允许空间
  NO_MOTION,              // 起点与终点重合，无需生成运动轨迹
  INVALID_QUATERNION,     // 四元数无效或明显不是单位四元数
  INVALID_LIMITS,         // 速度、加速度等限制值不合法
  START_MOVING,           // 初始速度超过静止起步允许值
  UNSUPPORTED_DIRECTION,  // 目标方向不符合首轮前向直线要求
  START_TILTED,           // 初始姿态倾斜过大
  INVALID_SAMPLING,       // 采样时间、步长或点数不合法
  BUFFER_TOO_SMALL,       // 调用者提供的参考数组空间不足
  NUMERICAL_RANGE         // 数值太大或精度不足，无法可靠表示结果
};

struct Status
{
  ErrorCode code = ErrorCode::OK;
  const char *message = "成功";  // 本库传入固定字符串，错误返回时不申请堆内存

  bool ok() const
  {
    return code == ErrorCode::OK;
  }
};

template <typename T>
class Result
{
public:
  // 计算成功时保存结果。move 转移可转移的数据，避免不必要的复制。
  static Result success(T value)
  {
    return Result(std::move(value));
  }

  // 计算失败时只有错误信息，不包含 T 类型的有效值。
  static Result failure(ErrorCode code, const char *message)
  {
    return Result(Status{code, message});
  }

  bool ok() const
  {
    return value_.has_value();
  }

  const Status &status() const
  {
    return status_;
  }

  // 必须先检查 ok()。忽略检查而读取失败结果会抛出 bad_optional_access，
  // 这是调用方的编程错误，不能把它当作一次正常的规划失败。
  const T &value() const
  {
    return value_.value();
  }

private:
  explicit Result(T value)
      : value_(std::move(value))
  {
  }

  explicit Result(Status status)
      : status_(status)
  {
  }

  // optional 表示“可能有值，也可能没有值”，不是一个特殊的数学对象。
  std::optional<T> value_;
  Status status_;
};

}  // 命名空间 rov_control_core

#endif

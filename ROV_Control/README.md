# ROV_Control

自研 ROV 规划与控制学习工程。算法、ROS 节点和测试均为 C++；自有启动文件统一采用 XML 内容、以 .launch 结尾。

## 1. 当前完成到哪里（2026-09-30）

| 部分 | 状态 |
|---|---|
| Gazebo/Hippo 模型、真值反馈 | 用户此前已运行；本日核对 WSL 依赖和启动参数有效，本日未启动仿真 |
| A 批：边界、直线路径、进度投影、五次时间轨迹、未来参考 | 已实现；09-29 独立 CMake Debug 和 ROS 2 colcon RelWithDebInfo 均通过 21 个 GTest |
| B 批：ROS 状态转换、轨迹预览、XML 启动 | 函数级设计见 [总体设计 §9](docs/PATH_TRACKING_DESIGN.md#9-b-批hippo-状态接入与轨迹预览的具体设计)，待本次讲解后确认实现 |
| C/D 批：纵向 PD＋姿态保持的 Hippo 软件闭环 | B 批之后，具体执行适配和停止路径须先讲解确认 |
| MPC、Pi/PX4、真实视觉 | 后续阶段 |

**目前只有数学库和测试程序；还不能启动本工程的 ROS 预览或跟踪节点。** 预览是闭环前的接线检查，完整 SIL 要让控制输出影响 Gazebo 并重新收到状态。当前路线不经过 PX4 固件。

文档只保留三份入口：

- 本 README：当前状态、构建、使用、数据流和验证。
- [PATH_TRACKING_DESIGN.md](docs/PATH_TRACKING_DESIGN.md)：模块/函数设计、Hippo 接线、分批验收。
- [MPC_LEARNING_ROADMAP.md](docs/MPC_LEARNING_ROADMAP.md)：开源方案、MPC 数学和后续学习路线。

原阶段汇报的使用和教学内容已合并到本页与总体设计；原开发计划的有效环境/接口信息已并入总体设计。项目根目录四份规则/研究记录仍保留。

## 2. 源码与环境位置

| 内容 | 路径/约定 |
|---|---|
| 自研源码唯一来源 | Windows：C:/DeskFiles/MPC-Mamba/ROV_Control |
| WSL 访问同一份自研源码 | /mnt/c/DeskFiles/MPC-Mamba/ROV_Control |
| WSL 外部平台依赖 | ~/rov_sim/USTROV/ustrov_ws |
| 仿真环境 | Ubuntu-24.04、ROS 2 Jazzy、Gazebo Harmonic |
| USTROV 固定版本 | b1208e50becb9f74d4d0dcd8637ac11e73d3672a；09-30 两处工作树均干净 |
| 代码风格 | .h/.cpp、Allman 大括号、两空格、小驼峰函数、成员尾下划线、详细中文注释 |

ROV_Control 尚未初始化独立 Git 仓库；建议后续独立管理，USTROV 为同级固定版本依赖。package.xml 的维护者邮箱和许可证仍是明确占位。build/install/log 是本机生成物，已被 .gitignore 忽略。

## 3. 现在即可运行的 A 批示例

从 PowerShell 进入已有 WSL：

~~~powershell
wsl -d Ubuntu-24.04
~~~

在 Ubuntu 终端运行：

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/ROV_Control
./build/colcon/rov_control_core/test_path_trajectory \
  --gtest_filter=Trajectory.ExamplePrintsIndependentlyCheckedQuarterPoints
~~~

这是普通 C++ 测试可执行文件，运行它不需要 Gazebo 或 ROS source。构建产物被清理时先按下一节重建。

### 输入与结果

教学输入：起点 (0,0,-0.4) m，终点 (0.2,0,-0.4) m；有效盒 [-1,-1,-1] 到 [1,1,0] m；v_max=0.08 m/s、a_max=0.1 m/s²、T_min=0.1 s；固定单位姿态，未启用 jerk 限制。这些是数学测试参数，不是当前机器人或水池的标定。

输出总时间 T=4.6875 s：

| 时间 s | 沿线位置 m | 沿线速度 m/s | 沿线加速度 m/s² |
|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 |
| 1.171875 | 0.020703125 | 0.045 | 0.0512 |
| 2.343750 | 0.100000000 | 0.080 | 0 |
| 3.515625 | 0.179296875 | 0.045 | −0.0512 |
| 4.687500 | 0.200000000 | 0 | 0 |

这是参考轨迹，尚无机器人跟踪结果。测试使用独立代入公式得到的期望值核对这五个点。

## 4. 构建与测试

### A 批 colcon 构建

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/ROV_Control
source /opt/ros/jazzy/setup.bash

colcon build --base-paths src \
  --build-base build/colcon --install-base install \
  --packages-select rov_control_core \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon test --base-paths src \
  --build-base build/colcon --install-base install \
  --packages-select rov_control_core --event-handlers console_direct+

colcon test-result --test-result-base build/colcon/rov_control_core --verbose
source install/local_setup.bash
ros2 pkg prefix rov_control_core
~~~

A 批不依赖 USTROV。21 个 GTest 分为 6 组；colcon 汇总包含外层 CTest 入口，可能显示 22 项。09-29 实测 0 错误、0 失败。

### 独立 CMake 构建

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/ROV_Control
cmake -S src/rov_control_core -B build/core-standalone \
  -DROV_CORE_WITH_AMENT=OFF -DCMAKE_BUILD_TYPE=Debug
cmake --build build/core-standalone -j2
ctest --test-dir build/core-standalone --output-on-failure
~~~

关闭 ament 包装后使用同一份 C++/Eigen 算法。没有另外维护一套无 ROS 算法。库包当前没有 ros2 run 或 ros2 launch 节点入口。

## 5. A 批数据流与代码阅读

~~~text
PlanRequest（端点、有效空间、可选前向启动约束）
  → LinePathPlanner::plan()
  → LinePath（起终点、单位方向、长度）
  → QuinticTrajectory::create(path, limits, attitude)
       └─ chooseDuration() 计算总时间
  → sample(t) / sampleHorizon(t, dt, count, buffer)
  → ReferenceSample（同一时刻的位置、速度、加速度、姿态）

RobotState（实际状态）＋LinePath
  → projectProgress()
  → ProgressState（实际沿线进度、速度、侧向残差与深度偏差）
~~~

目前参数与时间由测试程序传入，未订阅 ROS、未输出控制。后续 ROS 层负责输入状态和时间，控制器读取内部参考对象；nav_msgs/Path 只用于显示。

短路径相对 src/rov_control_core：

| 文件 | 阅读重点 |
|---|---|
| include/rov_control_core/types.h | W/B 坐标、单位、状态与参考区别 |
| include/rov_control_core/result.h | Result<T>、Status、错误码；先 ok()，成功后才 value() |
| src/planning/workspace_bounds.cpp | 闭区间有效盒；凸集的端点检查 |
| src/planning/line_path.cpp | 长度/方向、前向约束、保留超调的进度投影 |
| src/planning/quintic_trajectory.cpp | 时间下界、五次多项式导数、端点保持和未来采样 |
| test/test_path_trajectory.cpp | 每个用例的场景与独立预期；示例入口在 ExamplePrintsIndependentlyCheckedQuarterPoints |
| CMakeLists.txt、package.xml | 编译、依赖、测试、安装与导出 |

代码风格参照用户 Glider_sim 的 ug_control：已读 PID 头文件/实现、control_node.cpp 和 mission_node.cpp；未搬入旧控制逻辑。排版约定在 .clang-format；上次按配置手工整理，未安装格式化工具。方法已从 Plan/Create/Sample 改为 plan/create/sample，头文件已从 .hpp 改为 .h，调用与安装头文件已同步。

## 6. 下一步启动方式（B 批实现后才可运行）

B 批具体职责、参数、函数和测试见总体设计 §9。自有启动文件都以 .launch 结尾，内容使用 ROS 2 XML；hippo_sim.launch 内部 include 外部 USTROV 的已有启动入口。

**以下命令是待实现接口，当前没有 rov_control_ros 和 rov_bringup 包。**

先构建：

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/ROV_Control
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash

colcon build --base-paths src \
  --build-base build/colcon --install-base install \
  --packages-up-to rov_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
~~~

终端 1 一次启动仿真与预览：

~~~bash
ros2 launch rov_bringup hippo_preview.launch \
  vehicle_name:=uuv00 start_gui:=true start_rviz:=true
~~~

终端 2 同样加载 ROS → USTROV → ROV_Control，再显式生成参考：

~~~bash
ros2 service call /uuv00/trajectory_preview/start std_srvs/srv/Trigger "{}"
ros2 topic echo /uuv00/trajectory_preview/diagnostics --once
~~~

预览节点不会驱动机器人。若已有一套 Hippo 在运行，只启动 trajectory_preview.launch，避免重复 Gazebo 和 mixer。停止预览用同名节点下的 stop 服务；故障时先排除原因，再调用 reset_fault 和 start。

记录方式：B 批采用现成 rosbag2 保存 odometry、planned_path、reference_pose、diagnostics 和 /clock；C 批再增加逐控制周期的 C++ 日志。rosbag 属于数据文件，写入 log/ 并使用不同运行目录。

## 7. 验证边界

A 批覆盖零路径、越界、前向/倾斜/起速约束、NaN/Inf、数值溢出、投影超调、解析速度/加速度峰值、jerk 时长、三维方向、差分导数、端点保持以及未来采样失败不部分写入。

这些测试不能证明 ROS 状态转换、动力学可行性或真机性能。09-30 只读确认本机 ustrov_sim、ustrov_control、rov_control_core、rviz2 包可被发现，上游 --show-args 成功；当前没有运行中的 Hippo 进程，也没有本工程 SIL 闭环结果。

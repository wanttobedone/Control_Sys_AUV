# ROV_Control

自研 ROV 规划与控制学习工程。算法、节点、测试使用 C++；自有启动文件统一为 XML 内容、`.launch` 后缀。

## 1. 系统目标与当前进度（2026-10-27）

| 部分 | 当前状态 |
|---|---|
| A：边界、直线路径、进度投影、五次时间轨迹、未来参考 | 已实现，21 个 C++ 单元测试通过 |
| B：状态适配、轨迹预览、XML 启动 | 已实现；新增 14 个单元测试通过，已接 Hippo 实际运行 |
| C1：有限输入的执行通路、死区和方向核验 | 已实现，完成 Hippo 有限脉冲和停止/暂停核验，结果见 §9.6 |
| 水平到点：规划、轨迹跟踪、姿态与物理分配 | 已实现并完成原 Hippo Gazebo 闭环；新入口与实测结果见 §10 |
| 逐电机开环工具 MotorProbeNode | 已实现；软件接线验证与使用说明见 §11，尚未连接本项目真机 |
| USTROV 真机手柄开环 | 10-27 用户报告 lab 经网线连接 Pi，运行 core + joystick 完成开环操作；本次源码核对与接入边界见 §14 |
| 相机离线标定 | lab 已生成湿光路候选 K/D，见 §13.10；在线定位及独立米制精度尚未验收 |
| MPC、真机闭环 | 尚未实现/验收；手柄开环不代表这些功能已完成 |

**现在可发布一个相对启动原点的 XY 坐标，驱动 Hippo 转向、跟踪并到点保持。直接从 §10 使用新入口。** 控制为纵向 PD/模型前馈、LOS 航向、姿态 PD；深度仅监测，MPC 尚未加入。当前使用 USTROV 真值反馈，不经过 PX4，不代表视觉/惯导融合已验收。§3–7 保留只读预览，§9 保留有限输入试验，它们是独立诊断入口。

文档分工：

- 本 README：当前实现、数据流、构建启动、验证与排障。
- [总体设计](docs/PATH_TRACKING_DESIGN.md)：文件/函数接口与后续分批设计。
- [MPC 学习路线](docs/MPC_LEARNING_ROADMAP.md)：算法推导与开源工具选型。

## 2. 架构、源码与 Git

~~~text
Control_Sys_AUV/                  Git 主仓库
├── AGENTS.md                     本地工作约定，当前被忽略
├── .gitignore
├── USTROV/                       外部 Git 子模块，提供仿真和状态桥
└── ROV_Control/
    ├── src/rov_control_core/     纯 C++/Eigen 数学，不依赖 ROS
    ├── src/rov_control_ros/      C++ 状态适配、预览、有限输入试验、水平到点节点
    ├── src/rov_bringup/          XML .launch、YAML、RViz
    ├── docs/
    ├── build/batch_b/            本机构建产物，忽略
    ├── install-b/                本次安装空间，忽略
    └── log/                      编译/仿真日志、rosbag，忽略
~~~

| 内容 | 位置 |
|---|---|
| 唯一自研源码 | C:/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control |
| WSL 访问同一份源码 | /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control |
| 已构建的 WSL 外部依赖 | ~/rov_sim/USTROV/ustrov_ws |
| 仿真环境 | Ubuntu-24.04、ROS 2 Jazzy、Gazebo Harmonic |
| 既有仿真验收版本 / 主仓库已记录子模块提交 | b1208e50becb9f74d4d0dcd8637ac11e73d3672a |
| 10-27 本地 USTROV 工作树 HEAD | f22f64746c8d2cabe756f1bc52bb961703bcec29；子模块内部干净，但主仓库尚未记录这个新指针，见 §14 |

本次沿用用户建立的主仓库 + USTROV 子模块组织，没有改写上游。路径移动前的 build/install 可能含旧绝对路径，因此本次使用新 build/batch_b、install-b；旧产物未删除。正式 C++ 测试源码仍进入 Git，忽略的是测试结果和临时验证工具。维护者邮箱和许可证仍是 package.xml 中的明确占位。

## 3. 只读预览的运行流程与数据结构

~~~text
Hippo 物理模型 → ground_truth/odometry → fake_state_estimator
    → /uuv00/odometry
    → onOdometry() → StateAdapter::convert()
    → RobotState（p_W、q_WB、v_W、omega_B、stamp_ns）
                                 │
preview.yaml → PreviewConfig     │
start 服务 ──────────────────────┘
    → createTrajectory()
    → LinePathPlanner::plan() → LinePath
    → QuinticTrajectory::create() → 固定轨迹与 t0
    → makePathMessage() → planned_path → RViz

/clock → ROS 时间 → onTimer() → sample(now-t0)
    → ReferenceSample → reference_pose → RViz
RobotState + LinePath → projectProgress() → 实际进度
参考 + 实际进度 + 时效 + 阶段 → diagnostics → CLI / rosbag
~~~

关键约定：

- W=`map`，世界 z 向上；B=`vehicle_name/base_link`。m、s、rad。
- Odometry 的 pose 在 W、twist 在 B。适配器执行 `v_W=q_WB*v_B`；角速度保留 `omega_B`。
- ROS 时间决定参考进度；steady clock 检测有效消息接收中断。仿真暂停不能继续按实际时间播放轨迹。
- `start` 快照当前点和水平机头方向。默认 yaw≈90°，终点约为当前点加 `(0,0.2,0)`。路径不会随之后的实际漂移移动。
- `Path` 只用于显示；`ReferenceSample` 才同时包含位置、速度、加速度、姿态。后续控制器应读取后者。
- 起速/倾角执行门控不在本批：漂移中也能生成预览。预览成功不能作为控制可执行性证明。

阶段：

~~~text
WAIT_STATE → 收到并检查状态 → READY
READY/FINISHED → start → PREVIEW → 参考时间到 T → FINISHED
PREVIEW/FINISHED → stop → READY（状态不新鲜则等待）
非法状态 / 时效超限 / 时间倒退 → FAULT
FAULT → reset_fault → WAIT_STATE → READY → 用户再次 start
~~~

FAULT 锁存；恢复消息、恢复仿真或调用 stop 都不能绕过 reset_fault。FINISHED 只表示参考播放结束。stop/故障发布空 Path；RViz 可能仍保留上一根 Pose 箭头，当前有效性以 diagnostics 为准。

## 4. 模块、文件与关键函数

路径相对 src/：

| 文件 | 函数/阅读重点 |
|---|---|
| rov_control_core/include/rov_control_core/types.h | 状态、规划请求、限制、参考、进度的坐标/单位 |
| rov_control_core/include/rov_control_core/result.h | 先 ok()，成功后才 value()；失败含明确错误原因 |
| rov_control_core/src/planning/line_path.cpp | plan：端点→方向/长度；projectProgress：实际沿线与横向偏差 |
| rov_control_core/src/planning/quintic_trajectory.cpp | chooseDuration、create、sample、sampleHorizon |
| rov_control_ros/include/rov_control_ros/state_adapter.h | 适配接口、上一有效时间戳 |
| rov_control_ros/src/state_adapter.cpp | convert：frame→时间→有限值/四元数→速度旋转→提交历史；reset 清历史 |
| rov_control_ros/include/rov_control_ros/trajectory_preview_node.h | PreviewConfig、PreviewStatus、节点内部快照与 ROS 对象 |
| rov_control_ros/src/trajectory_preview_node.cpp | loadConfig、onOdometry、isStateFresh、createTrajectory、makePathMessage、onStart/onStop/onResetFault、onTimer、publishReference、publishDiagnostics、clearRun |
| rov_control_ros/src/preview_main.cpp | init→单线程 spin→shutdown |
| rov_control_ros/test/test_state_adapter.cpp | 坐标、坏数据、单调时间、拒绝输入不污染历史 |
| rov_control_ros/test/test_preview_math.cpp | 实际起点/朝向、端点/时间、越界、显示点数与时间溢出 |
| rov_bringup/launch/hippo_sim.launch | include 外部 Hippo 启动，只有一套 mixer |
| rov_bringup/launch/trajectory_preview.launch | 自研节点 + 可选 RViz，已有仿真时单独使用 |
| rov_bringup/launch/hippo_preview.launch | 组合上述两个入口 |
| rov_bringup/config/preview.yaml | 参考限制、消息时效、活动盒 |
| rov_bringup/rviz/trajectory_preview.rviz | map 下显示实际里程计、完整路径、参考姿态 |

源码用中文注释展开步骤；Allman 大括号、两空格、小驼峰和成员尾下划线。节点用单线程 executor，参数只在启动时读取，运行中修改会被明确拒绝。

## 5. 构建与测试

在 PowerShell 进入 WSL：`wsl -d Ubuntu-24.04`。以下均在 Ubuntu 终端执行：

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash

MAKEFLAGS="-j2" colcon build --base-paths src \
  --build-base build/batch_b --install-base install-b \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ROS_DOMAIN_ID=83 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --base-paths src \
  --build-base build/batch_b --install-base install-b \
  --event-handlers console_direct+
colcon test-result --test-result-base build/batch_b --verbose
source install-b/local_setup.bash
~~~

B 阶段为 35 个 GTest、3 个 CTest 入口（38 项）。C1 新增测试及最终结果见 §9。C1 的 ROS 接口测试会发布人工 /clock 和状态，因此上述测试命令单独使用 ROS_DOMAIN_ID=83，与默认域中的用户仿真隔离；不要在真实设备所在的域运行这些人工消息测试。

单独查看数学例子，不需要启动 Gazebo：

~~~bash
./build/batch_b/rov_control_core/test_path_trajectory \
  --gtest_filter=Trajectory.ExamplePrintsIndependentlyCheckedQuarterPoints
~~~

L=0.2 m、vmax=0.08 m/s、amax=0.1 m/s²，得 T=4.6875 s：

| t [s] | s [m] | v [m/s] | a [m/s²] |
|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 |
| 1.171875 | 0.020703125 | 0.045 | 0.0512 |
| 2.343750 | 0.1 | 0.08 | 0 |
| 3.515625 | 0.179296875 | 0.045 | −0.0512 |
| 4.687500 | 0.2 | 0 | 0 |

仍可独立编译数学核心：`cmake -S src/rov_control_core -B build/core-new -DROV_CORE_WITH_AMENT=OFF`，然后 `cmake --build build/core-new -j2`、`ctest --test-dir build/core-new --output-on-failure`。

## 6. 只读预览的启动与使用（到点控制见 §10）

### 每个新终端先加载三层环境

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash
source /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control/install-b/local_setup.bash
~~~

对应 ROS 1 的系统 ROS → 依赖工作区 → 自己的工作区。原 .bashrc 中若仍 source 旧 ROV_Control 路径，应替换为上面的新路径；使用 `[ -f 路径 ] && source 路径` 可避免尚未构建时每次开终端报错。本次没有自动改写 .bashrc。

### 终端 1：启动一套仿真

~~~bash
ros2 launch rov_bringup hippo_preview.launch \
  vehicle_name:=uuv00 start_gui:=true start_rviz:=true
~~~

若 Hippo 已经运行，只用：

~~~bash
ros2 launch rov_bringup trajectory_preview.launch vehicle_name:=uuv00
~~~

等待终端出现 `READY：状态有效，可调用 start 预览`。不需要 roscore。自有 XML 使用 exec/namespace、$(find-pkg-share ...)、$(var ...)；外部 Python launch 由 include 复用。

### 终端 2：触发和观察

~~~bash
ros2 service call /uuv00/trajectory_preview/start std_srvs/srv/Trigger "{}"
ros2 topic echo /uuv00/trajectory_preview/diagnostics --once
~~~

RViz 中绿色是整条参考路径，参考箭头沿线移动；Actual Odometry 显示真实仿真状态。节点不会驱动机器人，所以此时实际 Hippo 不会主动追随参考。

~~~bash
# 停止预览
ros2 service call /uuv00/trajectory_preview/stop std_srvs/srv/Trigger "{}"

# FAULT 时先恢复状态来源，再复位；等待 READY 后重新 start
ros2 service call /uuv00/trajectory_preview/reset_fault std_srvs/srv/Trigger "{}"
~~~

参数修改：编辑 preview.yaml 后重建 rov_bringup 并重启；或传 `config_file:=/绝对路径/preview.yaml` 加载自己的文件。命名空间更换时设 `vehicle_name:=uuv_b`，服务和话题前缀随之改变，body_frame 由 launch 同步设置。

### 终端 3：记录数据（可选）

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
mkdir -p log
ros2 bag record -o "log/preview_$(date +%Y%m%d_%H%M%S)" \
  /clock /uuv00/odometry \
  /uuv00/trajectory_preview/planned_path \
  /uuv00/trajectory_preview/reference_pose \
  /uuv00/trajectory_preview/diagnostics
~~~

记录终端 Ctrl+C 正常结束，再 `ros2 bag info log/对应目录` 查看。完整仿真退出用启动终端 Ctrl+C；stop 服务只停止参考。

## 7. B 批预览验证结果与边界（2026-10-02）

- 三个包在 WSL Jazzy 编译通过；35 个 C++ 单元测试全部通过。三个 XML 入口被实际解析并运行。
- uuv00 下实际启动 Gazebo GUI、RViz、状态桥和唯一 mixer；RViz OpenGL 初始化成功，订阅话题和 Path 的 transient-local QoS 已核对。未把 GUI 进程启动等同于人工逐项检查画面。
- 实际参考起点约 (1.3,1.0,-0.107835) m，终点约 (1.3,1.2,-0.107835) m；101 个显示点，T=4.6875 s。实际 z 与 spawn 的 -0.5 不同，是模型被动上浮后的反馈。
- 约 0.6 s 暂停：参考时间/位置冻结；超过 2 s 无有效消息：FAULT；恢复仿真后仍锁存故障，reset_fault 后才能再次 start。
- 改名 uuv_b 后同样通过参考、暂停、故障/复位、停止清空 Path 验证；注入错误 frame 触发 FAULT。实际 ROS 图确认预览节点仅发布显示/诊断及 ROS 基础话题，无执行器发布端点；RViz 的三条业务订阅已随命名空间改变。
- rosbag 已保存并读取元信息：log/preview_b_renamed，4831 条消息，其中 Odometry 311、Path 8、参考 Pose 411、诊断 17、clock 4084。日志为预览验收数据，其中含专门注入的错误消息，不能拿来训练或评价控制性能。临时 C++ 验收客户端已清理，结果保留在 log/batch_b_renamed_probe.txt。
- 实际 Gazebo 时间重置触发了时间倒退 FAULT；但同次上游仿真报告部分 link/joint 不存在、Hippo 状态流未恢复。**这条“重置世界后只复位节点便继续”的路径没有通过；当前应退出整套 launch 再启动。** 普通暂停/恢复与 reset_fault 路径可正常使用。
- 改名实例退出时，上游 bridge 曾以 -11 退出，vehicle_spawner 删除模型超时后退出；本次自研预览节点正常退出，测试进程均已结束。这是上游关闭路径的待查问题，本批未修改 USTROV 或宣称其退出流程完全通过。
- 本批不证明运动学参考满足推力能力、闭环稳定性、视觉定位或真机安全。世界活动盒只约束参考中心点；pool.xacro 没有池壁碰撞体。
- 本次验收使用独立 ROS_DOMAIN_ID=84、GZ_PARTITION=rov_preview_batch_b；日常运行无需设置它们。如自行设置，所有终端必须一致。

此后新增的 C1 执行通路核验见 §9；已完成的水平到点控制见 §10。

## 8. WSLg 窗口空白 / WARN:COPY MODE 排查

2026-10-02 用户报告：Gazebo/RViz 只出现在 Windows 任务栏，预览空白，标题含 WARN:COPY MODE；ROS 日志已 READY，RViz 已完成 OpenGL 初始化。现场检查确认 Linux 端窗口已创建，但 /mnt/wslg/weston.log 中有 rdp_allocate_shared_memory 的 Input/output error，use_gfxredir=0；Windows RDPClient 日志也有 0x80070490 图形解码错误。

COPY MODE 表示 WSLg 退回通过 RDP 复制像素的显示方式；它本身不证明 OpenGL 或显卡驱动失效。[微软 WSLg 维护者说明](https://github.com/microsoft/wslg/discussions/312)。本机系统发行版的 /mnt/shared_memory 已存在并挂载为 virtiofs，错误出现在共享内存分配阶段。

恢复步骤（会结束全部 WSL 任务，先保存 Linux 中的工作）：

1. 启动仿真的终端 Ctrl+C，等待退出。
2. 在 Windows PowerShell 执行 `wsl --shutdown`，然后 `wsl -d Ubuntu-24.04`。
3. 重新执行 §6 的三条 source 和 hippo_preview.launch。
4. 核对两个窗口确实可见、可交互；仅进程存活或 READY 不算 GUI 验收通过。

本次经用户明确批准已执行重启；新 Weston 日志 use_gfxredir=1，未再出现此前的共享内存分配失败，新会话观察期内未见对应解码错误。GUI 是否实际可见仍需结合用户屏幕反馈确认。重启前日志保存在 log/wslg_before_restart.log，新仿真日志在 log/gui_recovery/。

启动日志里的 `Stereo is NOT SUPPORTED` 是立体显示能力提示；当前预览不发布推力/力矩，mixer 的输入超时置零是预期行为。它们不能解释本次 Windows 窗口传输故障。

## 9. C1：有限输入试验的代码、数据流和使用

本节对应用户 2026-10-03 确认的 C1。它检查执行通路、正反方向、死区和停止行为；新的闭环到点入口见 §10。原 trajectory_preview 始终保持只读职责。

### 9.1 文件与函数

| 文件 | 职责及关键函数 |
|---|---|
| rov_control_ros/include/rov_control_ros/hippo_command.h | HippoEffortCommand、ProbeConfig、ProbeTiming、ActuatorMessagePair 及接口；effort 无量纲 |
| rov_control_ros/src/hippo_command.cpp | validateProbeConfig 校验配置；samplePulse 计算有限梯形脉冲；checkProbeState 校验时序/空间/相对运动；makeActuatorMessages 完整填写两类消息；probeDuration/probeChannelName 为小型辅助函数 |
| rov_control_ros/include/rov_control_ros/actuation_probe_node.h | 节点函数和状态快照、初态、时钟、命令缓存、有限零补发截止时间 |
| rov_control_ros/src/actuation_probe_node.cpp | loadConfig→建立 ROS 对象；onOdometry/onThrusterCommand 接收状态与 mixer 输出；onStart 开始一次试验；onTimer 周期检查/采样/发送；stopOutput 归零；onStop/onResetFault；publishDiagnostics |
| rov_control_ros/src/probe_main.cpp | 单线程 executor；捕获正常 SIGINT/SIGTERM，ROS context 关闭前请求归零并继续有界 spin；强杀不保证退出处理执行 |
| rov_control_ros/test/test_hippo_command.cpp | 7 个公式、消息、坐标和非法输入测试 |
| rov_control_ros/test/test_actuation_probe.cpp | 8 个真实 ROS 接口测试，使用 C++ 假 mixer；不包含 Gazebo 动力学 |
| rov_bringup/config/actuation_probe.yaml | 参数/单位和候选值；默认 amplitude=0；实测后 hold_s 从0.3缩为0.1，保护阈值不放宽 |
| rov_bringup/launch/actuation_probe.launch | 仅起 probe，接已经运行的 Hippo |
| rov_bringup/launch/hippo_actuation_probe.launch | 组合 Hippo 与 probe，一套 Gazebo/mixer |
| rov_control_ros/CMakeLists.txt、package.xml | 增加 actuation_support、actuation_probe_node、测试及 ustrov_control_msgs 依赖；未引入姿态库 |

函数具体输入/输出、内部状态和边界见 [总体设计 §11](docs/PATH_TRACKING_DESIGN.md)。checkCommandSources 检查发布者数量/名称，makeTiming 统一时钟检查量，checkCurrentState 在 start 时进一步检查初始速度和倾斜。

### 9.2 一次运行的数据流

```text
YAML/launch → loadConfig → ProbeConfig（运行中固定）
odometry → StateAdapter::convert → latestState_ + steady 接收时刻
start 服务 → 校验状态/时钟/唯一命令源 → startState_、t0_、run_id
wall timer → checkProbeState → samplePulse(now_ros-t0)
  → publishCommand → makeActuatorMessages
  → thrust_setpoint / torque_setpoint → 上游 mixer → thruster_command
  → Gazebo → 新 odometry
各阶段/配置/状态年龄/发送值/mixer值/退出原因 → diagnostics → rosbag
```

全零基线1 s，上升0.1 s、保持0.1 s、下降0.1 s，零输入观察1 s，共2.3 ROS秒。完成后再有限补发零命令0.5现实秒。只有用户 start 才产生试验输出，launch 不自动 start。

状态机 WAIT_STATE→READY→RUNNING→FINISHED；错误进入 FAULT，恢复需要显式 reset_fault 后重新等状态和 start。FINISHED 表示输入序列结束，零命令不意味着物体因惯性/浮力产生的运动立即消失。

两类 setpoint 每次完整填写，ignore_x/y/z=false、未用轴为零。它们分两次发送，存在中间组合；全零基线确认后才进入脉冲。thruster_command 是 mixer 发送的电机命令，不是测得的推力。当前单轴幅值<=0.65，因此运行中还检查有效四路电机幅值<=0.1625及后四路为零。所有检查仅针对当前 Hippo 配置。

### 9.3 构建和测试：每条命令的含义

在 Ubuntu 终端执行；PowerShell 用户先 `wsl -d Ubuntu-24.04`，表示进入这个 WSL 发行版。

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash

MAKEFLAGS="-j2" colcon build --base-paths src \
  --build-base build/batch_b --install-base install-b \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ROS_DOMAIN_ID=83 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  colcon test --base-paths src \
  --build-base build/batch_b --install-base install-b \
  --event-handlers console_direct+

colcon test-result --test-result-base build/batch_b --verbose
source install-b/local_setup.bash
~~~

| 命令/参数 | 含义 |
|---|---|
| cd ... | 切到唯一源码工作区；/mnt/c 对应 Windows C盘 |
| 第一条 source | 加载系统 ROS 2 Jazzy |
| 第二条 source | 叠加已经构建的 USTROV 仿真/消息依赖 |
| MAKEFLAGS="-j2" | 限制本次 Make 编译并行任务数为2，降低峰值内存 |
| colcon build | 按依赖顺序构建 ROS 包，对应 ROS1 工作区构建工具的职责 |
| --base-paths src | 只从自己的 src 查包，避免误扫外部工程 |
| --build-base / --install-base | 分别指定构建产物和安装目录；沿用 batch_b/install-b 不需要另搬源码 |
| --executor sequential | 包之间顺序构建；包内部仍可按 -j2 编译 |
| --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo | 向 CMake 传递“优化并保留调试符号”的构建类型 |
| ROS_DOMAIN_ID=83 ... colcon test | 只对本条测试命令使用域83，隔离测试的人工时钟/状态 |
| ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST | 本条测试只自动发现本机 ROS 参与者 |
| --event-handlers console_direct+ | 在终端直接显示测试程序输出 |
| colcon test-result ... --verbose | 读取测试报告并列出失败详情，不能仅凭构建成功代替它 |
| 最后一条 source | 将新安装的 ROV_Control 包加入当前终端环境 |

每个用于运行/服务/记录的新终端需 source 系统 ROS、USTROV 和 install-b 三层环境。这里的命令前置环境变量只作用于该条命令；不需要为了普通仿真全局 export 域83。

### 9.4 启动、开始、停止和更换通道

**终端1：已有 Hippo 正在运行时使用：**

~~~bash
ros2 launch rov_bringup actuation_probe.launch \
  vehicle_name:=uuv00 channel:=surge amplitude:=0.0
~~~

如果尚未启动 Hippo，则用组合入口替代上面一条，二者不要一起启动：

~~~bash
ros2 launch rov_bringup hippo_actuation_probe.launch \
  vehicle_name:=uuv00 start_gui:=true channel:=surge amplitude:=0.0
~~~

`ros2 launch` 是启动工具；`rov_bringup` 是包名；后面是 XML 启动文件。`vehicle_name` 决定话题/服务前缀并同步 body_frame；`start_gui` 控制 Gazebo 窗口；`channel` 指定单轴；`amplitude` 为命令域幅值，0用于首次通路检查。反斜杠只表示命令换行，后面不要跟空格。

**终端2：等待 READY 后开始一次试验：**

~~~bash
ros2 service call /uuv00/actuation_probe/start std_srvs/srv/Trigger "{}"
ros2 topic echo /uuv00/actuation_probe/diagnostics --once
~~~

第一条向 start 服务发送空请求，服务类型为 Trigger，`{}` 表示没有请求字段；响应 success 只表示试验获准开始。第二条打印一条诊断后退出；看 state、reason/message、amplitude、observed_max_ros_step_s 和发送值。

~~~bash
ros2 service call /uuv00/actuation_probe/stop std_srvs/srv/Trigger "{}"
ros2 service call /uuv00/actuation_probe/reset_fault std_srvs/srv/Trigger "{}"
~~~

stop 中断本节点试验并归零；FAULT 时 stop 不解除故障。恢复状态来源后，等归零补发结束再 reset_fault；复位只重新等待状态，不自动开始。正常完成的 FINISHED 可再次 start，但仍会重新检查起始速度/姿态。

更换参数时，先在终端1 Ctrl+C 退出本节点，再修改幅值重启；channel 可选 surge/roll/pitch/yaw。先用 `amplitude:=0.4` 核对死区；surge/roll/pitch 的已完成短脉冲幅值为 ±0.6，yaw 使用 ±0.55（±0.6 会触发转角保护，见 §9.6）。参数启动后固定，运行中 ros2 param set 会被拒绝。改 YAML 后重建 rov_bringup，或用 `config_file:=/绝对路径/文件.yaml`；channel/amplitude 的 launch 参数覆盖 YAML 中同名项。

### 9.5 记录原始数据

另开已加载三层环境的终端，在 start 前执行：

~~~bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
mkdir -p log
ros2 bag record -o "log/probe_$(date +%Y%m%d_%H%M%S)" --topics \
  /clock /uuv00/odometry \
  /uuv00/thrust_setpoint /uuv00/torque_setpoint \
  /uuv00/thruster_command /uuv00/actuation_probe/diagnostics
~~~

`mkdir -p` 确保日志目录存在；`ros2 bag record` 订阅并存储消息；`-o` 指定新的输出目录；`$(date ...)` 把当前时间写入目录名避免覆盖；`--topics` 后列出本次要保存的话题。日志终端 Ctrl+C 会正常关闭记录并写索引。然后运行 `ros2 bag info log/实际目录名` 查看消息数量和话题类型。

launch 自带屏幕日志只记录主要阶段；rosbag 保存输入/输出数值，二者职责不同。若只是看 motion 是否变化，可额外执行 `ros2 topic echo /uuv00/odometry --once`，但单个样本不能代替整个时间序列。

### 9.6 本批实际验证结果与限制（2026-10-03）

**构建与接口测试：**三个包在 WSL/Jazzy 构建成功；原35个GTest加本批7个计算/消息测试、8个ROS接口测试，共50个GTest通过。`colcon test-result` 将5个CTest入口也统计进去，显示55项、0失败。两个新XML入口成功解析，独立probe入口已经实际运行。接口测试使用C++假mixer，覆盖显式start、有限结束、stop、暂停/复位不重启、状态中断、错误frame、时间倒退、第二命令源和非法mixer时间戳。测试与真实Gazebo结果应分开理解。

**实际模型试验：**沿用固定USTROV版本和默认Hippo模型；查询实际mixer参数并保存在下述日志。使用独立 `ROS_DOMAIN_ID=84`、`GZ_PARTITION=rov_c1_20261003_01`、实例 `c1test`，未关闭用户已有 `uuv00` GUI仿真。以下保持段均为当前默认0.1 s：

| 试验 | 实际观测 | 状态/结论 |
|---|---|---|
| 零输入 | 两类setpoint与电机输出均零 | 初轮FINISHED，通路和零基线成立 |
| 各轴effort=0.4 | 电机峰值0.1；四台推进器报告推力峰值均0 | FINISHED；与源码死区推导一致 |
| surge ±0.6 | 沿初始机头速度峰值约±0.02480 m/s；观察窗口位移约±0.01198 m | FINISHED，前向正反一致 |
| roll ±0.6 | 脉冲初段机体x角速度均值约+0.0921 / -0.0608 rad/s | FINISHED，初始响应正反一致，随后存在回摆 |
| pitch ±0.6 | 脉冲初段机体y角速度均值约+0.0841 / -0.0112 rad/s | FINISHED，初始响应正反一致；初始状态不完全相同，不能拿这两个数比较对称性 |
| yaw ±0.55 | 机体z角速度峰值约+0.1133 / -0.1123 rad/s | FINISHED，正反一致；每台推进器推力峰值约0.045616 N |
| 手动stop | 非零脉冲期间调用服务，末次两类setpoint均零，节点回READY | 实际验证通过；物体仍可惯性运动 |
| 暂停Gazebo | 状态/时钟停止后进入FAULT；末次两类setpoint为零 | 实际验证通过；恢复仿真不自动start |

转动初段均值为验收客户端收到start响应后约1.10–1.14 ROS秒的样本，存在服务/采样偏移，只用于检查符号，不是惯量或增益辨识。各轴试验并非严格相同初态。effort=0.4时仍观测到小幅滚转等运动，不能把“推进器推力为零”写成“整个模型绝对静止”，其来源尚未单独辨识。这里的推力来自Gazebo插件独立输出 `/c1test/thruster_i/thrust`，临时桥接为Float64，区别于mixer的电机命令。

**保留的失败与修订：**

- 初始保持0.3 s、幅值±0.6的pitch/yaw触发0.50 rad/s角速度保护，故将默认保持改为0.1 s，原限制保留。
- 保持0.1 s时yaw ±0.6仍触发相对10°转角保护，归零后惯性仍导致转动；改用±0.55完成本批试验。动作更强不代表更适合辨识方向。
- 有几次出现ROS时间单次增量超限，其中一条诊断记录最大增量0.336 s；对应试验在非零输出前已FAULT并归零。保留0.10 s阈值，后续独立试验可以完成。尚未定位调度/通信延迟的根因，不宣称WSL具有硬实时性或每次运行必定通过。
- ROS CLI daemon曾返回过期图信息，直接使用 `ros2 topic info /clock --verbose --no-daemon --spin-time 2` 确认唯一时钟桥仍在运行。`--no-daemon`绕过缓存，`--spin-time 2`给直接发现2秒；这解决了查询结果歧义，不等于解释0.336 s增量。
- 本次独立实例已退出；关闭时上游Gazebo/spawner在SIGINT后超时，launch升级SIGTERM后结束。上游关闭顺序仍有待查项；用户原有uuv00 GUI启动进程保留。
- 图检查存在发现延迟，两条setpoint非原子发送；有限补发是尽力发送，不证明强杀、ROS context提前关闭或网络完全中断后仍可送达。20 ms是配置周期，未完成整个控制平台的实时性验收。

**记录位置：**`log/c1_validation_20261003_01/`（Git忽略）。每次都有独立 `node.log`、`observer.log`，失败目录未覆盖；`mixer.yaml`保存实际参数。`bag`保存初轮输入/状态，`bag_forces`包含推力核验和0.336 s故障，`bag_yaw_final`包含最终yaw ±0.55及stop，已读取元信息和消息；最后一个bag共267986条消息、10类话题。`resume_*`轮次主要保留验收客户端摘要，并非所有轮次都有完整bag；部分外部观察订阅未收齐电机峰值，不把其0峰值当成实际没输出，须结合推力/状态和节点诊断。临时C++验收客户端已清理，正式回归测试和结果证据保留。

C1只证明当前模型下有限输入的通路、符号、部分非线性与停止逻辑。此后的水平闭环见 §10；仍没有MPC求解器、PX4 HIL、真实估计器或真机控制证据。

## 10. 水平 XY 到点：直接发布目标运行

### 10.1 本次完成什么

沿用原 Hippo 四推进器，可控制前向推力和三个力矩。水平目标通过“制动→转向→直线五次轨迹→纵向 PD/前馈＋LOS→姿态 PD→物理分配”执行；到点后继续抑制纵向漂移，偏离5cm重新接近。深度只监测，不生成独立升沉推力。没有地图或障碍搜索。

自研新增：core 的 `tracking/planar_tracker.h/.cpp`、`tracking/hippo_allocator.h/.cpp`；ROS 的 `point_tracking_node.h/.cpp`、`tracking_main.cpp`；两份 `.launch`、`point_tracking.yaml`、RViz配置和两份正式 C++ 测试。复用原路径/五次轨迹/进度投影、状态适配。文件、每个函数的输入输出和内部状态见 [总体设计 §6](docs/PATH_TRACKING_DESIGN.md)。没有修改 USTROV。

新的总入口绕过上游 normalized mixer，自己的 `HippoAllocator` 将 N/N·m 转换为四台电机命令。其几何和曲线专属于原 Hippo 仿真，不是用户真实 ROV 标定；不要将 `thruster_command` 再交给另一层 mixer。

### 10.2 坐标怎么理解

收到第一条有效状态时固定 `uuv00/home`：原点为当时位置，x 是当时机头水平朝向，y 向左，z 向上。转向后此坐标系不跟着转。`reset_fault` 保留原点，完整重启重新捕获；这相当于软件启动原点，不能声称真实上电瞬间已经记录。

例如默认起点约为 map 中 `(1.3,1.0,-0.5)`，初始yaw约90°。发布home目标 `(0.3,0.2,0)`，世界水平目标约为 `(1.1,1.3)`：向初始前方0.3m、左方0.2m。实际z会因正浮力改变。消息z必须填0，只表示本次不请求深度控制；非零z会被拒绝。

### 10.3 构建：在 WSL Ubuntu 终端执行

先在旧仿真的启动终端按 Ctrl+C，退出 `hippo_actuation_probe` / `hippo_preview`。每个场景只开一个总入口，避免重复世界、时钟和执行器源。

```bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash
MAKEFLAGS="-j2" colcon build --base-paths src \
  --build-base build/batch_b --install-base install-b \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install-b/local_setup.bash
```

含义：`cd`进入唯一源码；前两条source依次加载系统ROS和已构建USTROV；`--base-paths src`只发现本工程包；`-j2`限制单包编译并行数，`--executor sequential`令各包依次构建；build/install参数继续用既有产物目录；RelWithDebInfo保留调试信息并优化；最后source加载新安装包。它对应ROS1的catkin构建再source devel，但ROS2默认使用install。

其他终端运行前也加载：

```bash
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash
source /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control/install-b/local_setup.bash
```

### 10.4 终端1：启动

```bash
ros2 launch rov_bringup hippo_point_tracking.launch \
  vehicle_name:=uuv00 start_gui:=true start_rviz:=true
```

`ros2 launch`启动描述文件；`rov_bringup`是包；`.launch`内部是XML；vehicle_name决定节点/话题命名空间及body/home frame；后两项开启Gazebo GUI和RViz。无图形测试可都设false，物理仿真和控制仍运行。新入口固定use_sim_time=true，不需额外传入。

等待日志显示home已固定、`READY`。此时没有任务，不会自动发推进器命令。RViz绿色为完整计划路径，蓝色为实际轨迹，橙色为当前参考，红色为目标；深度漂移会让三维画面中的实际/参考高度不同，水平误差看diagnostics。

### 10.5 终端2：发一个点

```bash
ros2 topic pub --once /uuv00/goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'uuv00/home'}, point: {x: 0.3, y: 0.2, z: 0.0}}"
```

`topic pub`发布消息；`--once`只发一次，不需要持续发送；`/uuv00/goal`为输入话题；`PointStamped`包含坐标和坐标系；x/y单位m。未填写stamp时为0，节点允许按当前状态接收CLI目标。发布成功只证明消息发出，是否接受和是否到达须看diagnostics。

返回启动原点：

```bash
ros2 topic pub --once /uuv00/goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'uuv00/home'}, point: {x: 0.0, y: 0.0, z: 0.0}}"
```

可在运动中发布新点：先制动、转向再规划。重复同一点不重置进度。越界、坏frame、非零z或非法数字会拒绝，原任务继续。活动盒用map坐标配置；当前用户YAML中心范围x=[0,4]、y=[0,5]、z=[-2,2]。它是人为规划范围，不等于水池碰撞边界。目标header.stamp不作任务过期检查；反馈Odometry仍检查时效。

### 10.6 查看、停止、故障恢复

```bash
ros2 topic echo /uuv00/point_tracking/diagnostics --once
ros2 topic echo /uuv00/odometry_home --once
ros2 topic echo /uuv00/thruster_command --once
```

第一条显示阶段/误差/力和力矩/电机命令/时序；第二条显示相对启动原点的实际位置；第三条查看发送的8槽命令，只有前4路使用。它不是实际推力测量；READY阶段没有新电机消息，echo可能等待。

判断到达：`state=HOLDING`，距离<0.025m且水平速度<0.015m/s持续0.75s。已在目标圈内的近静止新任务可以直接进入保持。TRACKING的rms/max误差只累计带时间参考与实际XY偏差；`compute_ms`仅计算核心更新、分配和发送，不是整个ROS周期耗时。

```bash
ros2 service call /uuv00/point_tracking/stop std_srvs/srv/Trigger "{}"
```

调用服务请求取消任务、立即归零，并补发0.5 wall秒后等待新点；服务响应不保证机体位置立即静止。停止服务不能解除故障。

```bash
ros2 service call /uuv00/point_tracking/reset_fault std_srvs/srv/Trigger "{}"
```

先处理诊断中的原因（例如恢复仿真、退出重复控制节点），等归零结束后复位；home保留，旧任务不恢复，READY后重新pub。Gazebo“暂停”超过0.35s会触发FAULT；“重置世界”建议Ctrl+C后重启整套launch。整套退出用终端1的Ctrl+C，正常信号退出会尽力归零；强杀不能保证，当前直接电机通路仅限仿真。

### 10.7 记录一次完整数据流

```bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
mkdir -p log
ros2 bag record -o "log/point_$(date +%Y%m%d_%H%M%S)" --include-unpublished-topics --topics \
  /clock /tf /tf_static /uuv00/goal /uuv00/odometry /uuv00/odometry_home \
  /uuv00/point_tracking/home /uuv00/point_tracking/accepted_goal \
  /uuv00/point_tracking/planned_path /uuv00/point_tracking/actual_path \
  /uuv00/point_tracking/reference_pose /uuv00/point_tracking/diagnostics \
  /uuv00/thruster_command
```

`mkdir -p`确保目录存在；`-o`指定新的时间戳目录，避免覆盖；`--topics`列出记录的话题。`--include-unpublished-topics`允许提前订阅只有订阅端、还没有发布者的goal（已核对本机Jazzy支持）。启动仿真和录制后，等显示已订阅goal再pub，避免一次性目标在发现话题期间漏录。完成后在录制终端Ctrl+C正常关闭；`ros2 bag info log/实际目录名`查看元信息。重放用于离线分析时与在线控制域隔离，不向运行控制器重放goal/clock。

### 10.8 实测、边界与清理

验收用隔离的ROS_DOMAIN_ID=85、GZ_PARTITION=rov_point_20261003、车辆名pointtest，与用户默认域分开。日常启动无需设置这两项；如果自定义，各相关终端必须一致。

首轮Gazebo在无外加水流、真值状态下完成：

| 目标（home XY，m） | 场景 | 最终水平误差 | TRACKING水平RMSE |
|---|---|---:|---:|
| (.3,0) | 前进 | .059 mm | .553 mm |
| (.3,.25) | 换向到侧方 | .156 mm | .611 mm |
| (-.1,.1) | 身后点 | .053 mm | .563 mm |
| (0,0) | 回启动原点 | .371 mm | .613 mm |
| (.2,-.1) | 运动中换目标 | .799 mm | .842 mm |

每个到点结果还由外部验收器确认保持约2 ROS秒。最大带时间参考误差约1.313mm，观测电机幅值峰值约.148403（上限.4），深度相对首次状态上浮约.393511m。小误差属于确定性模型/真值闭环，不能外推为水下定位精度或真机效果。首轮观察器在首个/clock到达前记录起始时间，因此第一项打印duration=80.968s不是真实任务用时；不使用该列评价第一段时长，原始记录保留。

非零z、越界、错误frame均被拒绝且原任务未被替换；stop归零；暂停导致锁存FAULT并归零，恢复/复位后等待新目标。当前没有障碍避让、长期横流、传感器噪声/融合、真实电机或Pi实测结论。

最终版本开启Gazebo GUI/RViz再次完成上述序列：最终水平误差依次为.099/.176/.093/.378/.815mm，跟踪RMSE依次为.373/.439/.325/.530/.769mm，最大参考误差.955mm，观测油门峰值.147577。修正时钟就绪后的首段用时10.392 ROS秒（含额外2秒保持观测）。GUI进程及RViz OpenGL初始化成功、显示话题订阅已核对；未以进程启动代替对Windows画面的人工检查。

三个包编译通过；60个GTest全部通过，colcon连同7个CTest入口汇总为67项、零失败。新增7个core用例和3个ROS接口用例。初轮ROS测试fixture初始化早于rclcpp::init导致3个用例失败，已修正初始化顺序并完整重测通过，失败日志保留。新XML入口的--show-args与实际启动均通过。

证据保存在被Git忽略的`log/point_validation/`：`bag_first`/`bag_final`、两份CSV和`results_first.txt`/`results_final.txt`；失败构建/测试也保留在colcon日志。正式C++回归源码保留；一次性验收客户端、脚本及其专用编译目录已清理。没有新增重复阶段.md，现行函数设计已合并入总体设计。

两个bag均已读取元信息：首轮231299条、最终94745条，均为10类话题。最终bag因话题发现漏录首个goal，共9条目标消息；首轮10条。最终首段依据节点目标日志、诊断与CSV核对，不将该bag称为无遗漏记录；上述使用命令已补提前订阅选项。退出最终场景时，上游bridge再次出现-11、spawner删除模型超时；自研节点正常退出。与此前B阶段一样，这是上游退出路径问题，尚未修复，不影响本次运行阶段的目标验收结论。

上面的2026-10-03数据使用当时0.08m/s、0.06m/s²低速配置。当前用户已将YAML改为0.8m/s、0.6m/s²，不能把旧验收数据当作新配置的结果。

### 10.9 参数调节与检查职责（2026-10-06）

到点节点现在支持ROS参数服务更新。删除了validateConfig中0.2m/s、3N、0.1N·m试验上限；实际速度/角速度/倾斜/深度阈值来自配置。max_actual_speed_mps、max_angular_speed_radps、max_tilt_rad、max_depth_drift_m设置为0可关闭对应运动监测；mission_timeout_s=0可关闭任务超时。状态有效性、时间回退、执行器接口范围和显式停止继续处理。

源码改动需要按§10.3重新编译；单独改YAML可直接加载源文件，随后重启：

```bash
ros2 launch rov_bringup hippo_point_tracking.launch \
  vehicle_name:=uuv00 start_gui:=true start_rviz:=true \
  config_file:=/mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control/src/rov_bringup/config/point_tracking.yaml
```

`config_file`指定实际读取的文件，从而不依赖install目录中的配置副本。保存YAML不会自动改变正在运行的节点。

运行中显式更新、查看（ROS 1中类似dynamic_reconfigure，这里用ROS 2参数服务）：

```bash
ros2 param get /uuv00/point_tracking max_speed_mps
ros2 param set /uuv00/point_tracking max_actual_speed_mps 1.0
ros2 param set /uuv00/point_tracking max_speed_mps 0.8
ros2 param set /uuv00/point_tracking max_thruster_command 0.6
```

`get`查询已提交值，`set`提交一次修改。提高参考速度前先提高实际速度监测阈值（或明确设0关闭该项）。启动/更新日志打印生效值；冲突错误报告参数名和数值。参数服务不把新值写回YAML，需要持久化时另改配置文件。

| 参数种类 | 生效时刻 |
|---|---|
| 增益、前馈质量/阻尼、力/力矩和电机限幅、运行监测阈值 | 参数事务成功后的下一个控制周期 |
| 参考速度/加速度 | 下一次生成轨迹；当前段时间律保持连续，发布不同目标会制动后重规划 |
| world/body/home frame | 启动时固定，修改需重启；已有home不随调参重新建立 |

参数回调先检查候选副本，全部通过后才提交算法配置，拒绝的事务不改变运行值。电机业务限幅集中在HippoAllocator；发送端保留[-1,1]归一化接口检查。单帧坏/乱序反馈只丢弃，持续无有效反馈才超时。ROS图只检查实际输出话题多发布者，不依赖订阅者数量、其他话题或mixer名称。

当前max_force_N仍为1.5N。名义阻尼5.39N/(m/s)，实际0.8m/s时仅阻力前馈约4.31N；配置允许0.8不表示机器人能达到该速度。分配饱和与tracking误差应结合diagnostics观察。本次清理和逐函数职责见总体设计§6.1。

**本轮验证：**三个包构建通过；67个GTest用例通过，colcon含7个CTest入口汇总74项，零失败。覆盖高于旧数值的配置、错误参数名、原子更新回滚、动态限幅、当前轨迹保持、坏帧丢弃/持续超时、真正的电机多源冲突、stop和故障复位。`log/parameter_cleanup_20261006/`保存构建（含首次失败）、测试及仿真日志。

独立Gazebo使用ROS_DOMAIN_ID=87、GZ_PARTITION=rov_parameter_cleanup_20261006、paramtest实例；没有关闭用户已有仿真。加载源码YAML的vmax=0.8m/s、amax=0.6m/s²、Fx上限1.5N后，home目标(0.3,0.2)进入HOLDING；采样水平误差0.010674m、速度0.004822m/s，TRACKING参考误差RMSE为0.049486m。运行中将实际速度阈值改为0.5被明确拒绝，查询仍为1.0；电机限幅改0.6、参考速度改0.4成功。再发home原点进入HOLDING，采样误差0.010326m、速度0.004661m/s，TRACKING RMSE为0.049233m。上述是诊断采样值，不是长期稳态精度，也没有证明实际达到0.8m/s。

stop响应成功并返回READY。隔离场景退出后所有测试进程结束；上游spawner/SceneBroadcaster退出阶段仍有世界状态超时提示，自研节点正常退出。没有新增一次性C++/脚本文件；正式回归源码保留，日志被Git忽略。

## 11. 逐电机开环工具与项目拥有者的阅读导图（2026-10-07）

### 11.1 本批目标与范围

`MotorProbeNode`把预先指定的归一化指令变成有限单电机脉冲，复用USTROV桥完成模式切换、解锁和上锁。它用于核对接线、方向、死区及后续测力标定，不执行路径规划，也不做位置/速度闭环。

**交付边界：当前只有标定的激励端，没有测力传感器输入、稳态窗口统计、自动多幅值扫描或曲线拟合。默认保持0.5s用于短脉冲通路检查，不是已经验证足够的稳态测力时长。** 仅录制电机指令和PX4状态不能得到推力曲线。已有`HippoAllocator::forceFromThrottle()/throttleFromForce()`使用仿真模型，不能作为本机实测标定。

新增文件：`rov_control_ros/include/rov_control_ros/motor_probe_node.h`、`src/motor_probe_node.cpp`、`src/motor_probe_main.cpp`、正式测试`test/test_motor_probe.cpp`；`rov_bringup/config/motor_probe.yaml`与`launch/motor_probe.launch`。两个包的构建/依赖声明同步更新，USTROV源码不改。所有运行和测试代码为C++17；启动为XML .launch。

### 11.2 先记住三条数据链

~~~text
已有仿真闭环：
goal(home) + Odometry → PointTrackingNode
  → 路径/五次时间律/PlanarTracker → HippoAllocator → thruster_command → Gazebo

本批开环工具：
channel + amplitude + 时间 → MotorProbeNode → thruster_command
  → USTROV Px4ActuatorBridge → ActuatorMotors → DDS Agent → PX4 → ESC → 推进器
       ↑ arm/stop 服务         ↑ VehicleStatus / 桥输出观测

后续真机闭环（尚未完成）：
相机标签 + IMU + 压力 → 有效状态 → 跟踪控制器
  → 真机标定的分配/推力曲线 → 同一套PX4执行桥
~~~

第二条链读取PX4状态只是判断是否允许输出，不根据运动误差修正u(t)，因此仍是开环。三条链运行时只能有一个thruster_command来源。namespace管理名称；不会自动转换单位、坐标、时间，也不会把仿真模型变成真实模型。

### 11.3 数据结构、单位与状态归属

| 量/状态 | 含义与拥有者 |
|---|---|
| `MotorProbeConfig` | 节点启动快照：channel 0..7、无量纲amplitude、ramp_s/hold_s秒、输出Hz、等待/输出超时秒 |
| `ActuatorControls.control[8]` | 自研节点发出的输入向量，无量纲[-1,1]。不是N/RPM/PWM微秒，没有世界/机体空间坐标含义 |
| `ActuatorMotors.control[12]` | 外部桥映射后的PX4逻辑Motor1..12；前8槽由桥填入，未用尾槽NaN。物理插口由PX4输出功能配置决定 |
| `VehicleStatus` | PX4报告的模式、Armed、failsafe快照；低频快照不能作为50Hz心跳。当前桥与probe均缓存它，失联行为仍须在实际固件上验收 |
| `MotorProbePhase` | 本节点IDLE/ARMING/RUNNING/STOPPING/FINISHED/FAULT，不等于桥的内部状态枚举，更不等于电机转速状态 |
| `lastOutputTime_` | 最近桥输出的steady接收时间，用于检测桥输出中断；接收到命令输出不代表ESC或电机执行成功 |
| `runId_` / `phaseStart_` | 第几次显式试验、当前阶段起点；start增加编号，进入RUNNING才开始脉冲计时 |

`motor_input_indices[m]`决定PX4逻辑电机m读取哪个输入槽，`motor_signs[m]`再乘正负号。默认映射来自USTROV，不是当前四电机的标定。`reversible_flags`不替代电调可逆模式或PWM中位设置。

原C1的surge/roll等effort经过四路混控；本工具直接给单电机输入。两者数值处于不同接口层，不能把仿真C1测得的0.526等阈值套给真实电机。

### 11.4 谁调用函数、谁改内存

程序只有一个业务`SingleThreadedExecutor`，`main`反复`spin_some()`；订阅、服务、定时器回调串行运行。ROS中间件可能有通信线程，但它们不直接执行我们所有业务函数；业务成员的修改集中在下面的回调中，当前不需要给这些成员加互斥锁。

| 触发事件 | 函数 | 改动/输出 |
|---|---|---|
| 创建节点 | 构造函数→`loadConfig()` | 参数一次校验，建立pub/sub/client/service/wall timer；启动为IDLE，幅值默认0 |
| PX4状态到达 | `onPx4Status()` | 更新模式/解锁/failsafe缓存及到达时间 |
| 桥电机数组到达 | `onBridgeOutput()` | 更新输出流时间、帧序号及全NaN标志 |
| 用户start | `onStart()` | 检查唯一命令源/桥/已上锁，清本次结果、增加runId、异步调用bridge arm，进入ARMING |
| 解锁服务返回 | `onArmReply()` | 只记“桥接受请求”；拒绝则停止。这里不开始脉冲 |
| 约20ms wall tick | `onTimer()` | 检查动态条件，满足Armed/Offboard和新鲜桥输出后进入RUNNING；计算elapsed，调用`makeCommand()`并发布 |
| 用户stop/时间到/异常 | `beginStop()`→`requestBridgeStop()` | 归零，异步调用bridge stop，进入STOPPING；无确认就继续等待/重试，不宣称完成 |
| 命令封装 | `publishCommand()` | 给8维数组加ROS发送时间戳，发布并缓存最后一次指令 |
| 阶段变化/约5Hz | `publishDiagnostics()` | 发阶段、终止原因、runId、输入、时序和PX4状态；rosbag负责存盘 |
| Ctrl+C/SIGTERM | `prepareShutdown()` | 运行中先停止；main最多再处理2s，未确认则报错退出。进程强杀依赖桥/PX4看门狗 |

定时器不是一条额外控制线程：executor到时执行回调。节点的50Hz与桥的50Hz是两个独立调度节拍，没有严格相位同步，不承诺硬实时。脉冲时间和超时使用steady_clock；ROS时间戳用于录包关联。`dt`由相邻tick的steady时间差得到用于间隔检查，波形直接按`elapsed = steady_now - running_start`求值，不累计假定的0.02s。

同一节点在FINISHED允许新的显式start；FAULT需确认PX4上锁、处理原因后重启，不能靠再次start解除。重启清空节点缓存和runId；不改变PX4硬件状态。节点不会因为重连而自动重放脉冲。

### 11.5 波形及一个完整例子

设输入通道2、幅值0.05、ramp_s=0.5、hold_s=0.5：总时长T=1.5s。

~~~text
相对RUNNING起点   0s      0.25s      0.5s      1.0s      1.25s      1.5s
control[2]        0       0.025      0.05      0.05       0.025       0
其他输入槽        0         0          0         0           0         0
~~~

这只是指令例子，不是已确认适合真机的幅值。按当前默认映射，control[2]进入PX4逻辑Motor3；对应哪个推进器必须查实际输出配置。停止时先发零，再由桥发全NaN并请求Disarm。FINISHED表示软件观察到stop接受、其后的全NaN输出和PX4已上锁状态，不能作为桨叶已完全静止的测量。

### 11.6 构建：本机已准备的WSL工作区

下列命令针对本机；每个新终端先加载环境。硬件工具采用显式CMake选项，原仿真构建默认仍关闭它。

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/rov_sim/USTROV/ustrov_ws/install/local_setup.bash
cd /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control
source install-motor-deps/local_setup.bash

MAKEFLAGS="-j2" colcon --log-base log/motor_build build \
  --base-paths src \
  --build-base build/motor --install-base install-motor \
  --symlink-install --executor sequential \
  --packages-select rov_control_core rov_control_ros rov_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DROV_BUILD_PX4_MOTOR_PROBE=ON -DBUILD_TESTING=ON

source install-motor/local_setup.bash
~~~

- 前两次source加载系统ROS及已有USTROV消息依赖；第三次加载本次硬件依赖。
- `install-motor-deps`包含USTROV固定的px4_msgs提交`392e831c1f659429ca83902e66820d7094591410`和未修改的ustrov_core；编译源在被忽略的`log/motor_probe_deps`及外部USTROV目录。
- `--build-base/--install-base`隔离本批构建产物，保持原`install-b`仿真环境可用；`MAKEFLAGS=-j2`和sequential限制编译并行度。
- `ROV_BUILD_PX4_MOTOR_PROBE=ON`显式加入硬件节点与测试；OFF时保留原仿真目标。
- `source install-motor/local_setup.bash`把本次可执行程序、XML/YAML放进ROS搜索环境。后续终端也需加载同样的三层依赖和此overlay。

在树莓派部署时，按USTROV的underlay说明初始化其px4_msgs子模块、构建underlay和`ustrov_core`，再在Pi上编译ROV_Control并启用同一选项。不能复制WSL的x86可执行文件到ARM64；本机固定消息版本仅用于源码兼容测试，不证明与设备固件匹配。

### 11.7 先在软件隔离环境运行验证

~~~bash
ROS_DOMAIN_ID=124 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
colcon --log-base log/motor_test test \
  --build-base build/motor --install-base install-motor \
  --packages-select rov_control_ros \
  --ctest-args -R '^motor_probe$' --output-on-failure

colcon test-result --test-result-base build/motor/rov_control_ros/test_results --verbose
~~~

`-R`只运行本批正式C++集成测试；测试自动创建模拟PX4节点和真实USTROV桥子进程，结束后清理。域124和LOCALHOST避免混入真实控制域；测试程序也检查这两个变量。没有启动Agent、访问串口或仿真水动力。当前运行结果记录在本节末尾，不能把软件模拟PX4称为已经接上真机。

### 11.8 真机使用步骤与命令含义

前置项：实际PX4固件与px4_msgs匹配；Agent连接实际串口/波特率；四电机对应的输出功能、可逆ESC/中位已确认。这里不猜串口和电调参数，不修改上游看门狗。第一轮在推进器动力断开的条件下验链，再安排水中固定台架试验。

终端1，已加载上面的环境后：

~~~bash
ros2 launch rov_bringup motor_probe.launch \
  vehicle_name:=uuv00 channel:=0 amplitude:=0.0
~~~

启动probe和已有bridge，不自动启动Agent、不解锁。`channel`为输入槽下标，`amplitude=0`表示完整走流程但输入始终为零。已有同一桥时加`start_bridge:=false`；不得同时运行core.launch.py里的mixer、仿真控制器或另一个thruster_command发布者。

若DDS前缀或状态话题不同，按实际话题传入`px4_topic_prefix:=... vehicle_status_topic:=...`。`vehicle_name`只控制ROS节点/相对服务的命名空间，不会自动改PX4客户端前缀。真实通道/符号用`bridge_config_file:=实际配置绝对路径`传入，配置可放自研bringup中，不能把原8路配置直接当标定。

终端2，观察当前阶段：

~~~bash
ros2 topic echo /uuv00/motor_probe/diagnostics --once
~~~

显示phase、graph_ok、PX4状态、命令值和终止原因。`graph_ok=true`只证明发现的命令源/桥关系符合要求，尚不证明物理接线正确。

录制证据（另开终端，按实际DDS前缀修改话题）：

~~~bash
mkdir -p log
ros2 bag record -o "log/motor_probe_$(date +%Y%m%d_%H%M%S)" --topics \
  /uuv00/thruster_command /uuv00/motor_probe/diagnostics \
  /fmu/in/actuator_motors /fmu/in/offboard_control_mode \
  /fmu/in/vehicle_command /fmu/out/vehicle_status_v1
~~~

录制“想发什么”“桥实际发什么”“飞控报告什么”；不把其中任何一条当作推力测量。等录包订阅成功后再开始一次试验：

~~~bash
ros2 service call /uuv00/motor_probe/start std_srvs/srv/Trigger '{}'
~~~

返回success表示试验请求已接受。观察ARMING→RUNNING→STOPPING→FINISHED；先确认全零试验，再依据台架条件选择非零幅值。修改通道/幅值时先等上锁，Ctrl+C结束launch，再用新launch参数启动；不能在运行中改参数。

随时主动停止本次试验：

~~~bash
ros2 service call /uuv00/motor_probe/stop std_srvs/srv/Trigger '{}'
~~~

这个服务由probe记录终止原因并调用桥stop；等待确认后结束。直接调用`/uuv00/px4_actuator_bridge/stop`也能停止原桥，probe会发现模式/输出变化并退出脉冲。软件服务不替代现场断电手段。

### 11.9 项目拥有者应能解释和检查的内容

无需记住所有源码，至少能沿以下问题定位到负责模块：

| 你要回答的问题 | 到哪里看 |
|---|---|
| 当前运行的是参考预览、仿真闭环，还是单电机工具？ | launch中有哪些node；不要只看进程名称“控制” |
| 目标/状态/输入分别是什么单位？ | 消息定义和config注释；位置m、力N、归一化电机输入是三层不同量 |
| 这次谁生成非零电机指令？ | 本批makeCommand；闭环时tracker+allocator。ROS图应只有一个最终命令发布者 |
| 谁允许指令到达PX4？ | probe观察条件；USTROV桥的PRESTREAM/Offboard/Arm状态机；PX4自己的输出许可 |
| 某个参数改了，何时生效？ | probe参数为启动只读；原point_tracking部分参数可在线事务更新，二者不能混用 |
| 某条消息出现，能证明什么？ | publish不保证执行；arm服务success不保证Armed；ActuatorMotors不是机械反馈 |
| 程序停止、定位丢失、桥消失各由谁处理？ | 当前probe/bridge退出链；未来定位健康需在真机闭环入口实现，本批不订阅定位 |
| 验证覆盖到哪里？ | 本批到ROS/PX4协议模拟；PWM、电调、电机方向、推力、Pi时序仍需实机验证 |

与Codex协作时，每批按“输入输出契约→函数调用图→修改清单→可复现证据→尚未验证的部分”审阅。要能用一个数值例子跟踪从参数到最终数组的变化，再看代码的边界分支；不要仅凭编译成功或终端READY判断整机已具备能力。

### 11.10 本批验证记录

2026-10-07：三个自研包及固定版本px4_msgs/原USTROV桥编译通过。新增12个GTest通过，包含波形正负/边界/全零、只发选中通道、解锁服务接受但PX4未解锁、预流/运行中停止、failsafe、启动前/运行中多命令源、发布节点消失、桥消失不能假报完成、非法配置及64位通道编号截断拒绝。原67个用例回归通过，合计79个GTest，colcon含8个CTest入口汇总87项，零失败/跳过。

真实桥保持默认1200ms预流和300ms命令看门狗；发布节点消失的本次软件观测为0.311935569s后收到Disarm。模拟PX4按VehicleCommand修改状态并广播，不运行PX4固件、不模拟电调/水动力；不能据此承诺真机最坏上锁延迟。域125/local的XML入口启动两个节点、默认零幅值/无解锁，SIGINT后二者正常退出；测试与烟测进程已结束。

首次构建在Windows挂载路径出现小于1s的Make文件时间偏差提示；px4_msgs增量构建复核成功，全部实际运行测试通过。自研代码的ROS整数转换警告已修复，随后增量colcon编译没有stderr。构建/测试原始日志保存在`log/motor_deps_build/`、`log/motor_build/`、`log/motor_test/`；XML烟测在`log/motor_launch_smoke.log`，正式测试结果在`build/motor/*/test_results/`。只保留一份正式C++测试源码，未生成一次性测试源文件。

仍未验证：树莓派ARM64构建/时序、实际PX4消息版本与预检、Agent/串口断链、PWM插口/电调中位/方向、推力曲线和视觉定位。桥发布的ActuatorMotors回读不等于飞控接收确认；实际断串口后的失联处置必须在所用固件上核验。

### 11.11 可选：从单电机脉冲到真机推力标定

**2026-10-07优先级更正：完整逐台测力不作为当前MPC学习或首个真机闭环的前置条件。** 用户指出复杂测力超出当前起步需要，当前必做路线改为§11.12。以下保留为需要准确物理推力分配或解释模型参数时的参考；不要求先购买测力设备。运动响应辨识可以建立控制模型，但不能冒充逐台牛顿级推力标定。

目标是逐台测量正、反向静态推力`F_i = f_i(u_i, V)`，再建立控制所用的逆映射`u_i = f_i^{-1}(F_i, V)`。这里“逆映射”表示由牛顿反查指令，与推进器反向旋转是两回事。死区不能唯一求逆，零推力指定零输入；正反分支分别处理，不假定对称，不外推未测范围。

实施顺序及每一步产出：

1. **查硬件和环境。** 收集QGC固件名称/版本、Pi系统/CPU架构/ROS、Pi与Pixhawk的USB或TELEM连接、实际设备路径与波特率、推进器/ESC型号、供电、各MAIN/AUX插口及输出功能、可逆/中位/上下限/上锁输出。Pixhawk 6C不是固件名称；若是ArduSub，当前PX4 DDS桥不能直接使用。测力设备型号、量程、分辨率及采集接口也要确认。Pi上原生构建，不能复制WSL的x86安装产物。
2. **断推进器动力，验通信及输出。** Agent和PX4客户端必须配对，消息定义与固件匹配；改topic名不能修复消息布局。按实际话题查看VehicleStatus，再执行§11.8的零幅值试验。核对解锁/上锁时的真实PWM或相应输出；不能用软件数组替代接口测量。当前自研probe和原mixer不得同时运行。
3. **单推进器水中固定，验通道/方向/停止。** 其余电调动力断开；以确认过的小幅值逐个测试。记录`输入channel→桥映射→PX4 Motor功能→物理插口→推进器→正输入推力方向`。这里核对的是推进器受力方向，不能只记录桨叶旋转方向。
4. **建立测力台。** 单推进器沿推力轴与拉压传感器连接，通过传感器向固定架传递受测方向的力，不能让其他刚性固定件绕过传感器分担该力。导线留松弛，支架不遮挡进出水流；若用杠杆，按力臂比换算。传感器先校准，再在最终浸水安装状态去皮；传感器/采集器无防水能力时置于水外。内净长1.35m、宽0.65m的水缸（2026-10-19用户更正，水深待测）先用于小幅测试，改变离壁位置并检查回流影响；不据尺寸直接承诺全量程静水标定。
5. **测配对数据。** 当前工具可以逐次更换幅值、显式start，配合外部测力仪人工记录。参数启动只读，换幅值/时间需结束并重启；同设置FINISHED后可再次start。保持时间由实测稳定过程决定，选已稳定且未受明显回流影响的窗口，不能平均整个缓升/缓降脉冲。每个幅值至少做3次起步复测，回零并等待水流平息；分别记录正反向、启动/退出死区、带载电压。窗口内持续漂移时不标为稳态点。
6. **形成可验证映射。** 保存`trial_id, thruster_id, channel, input_u, mapped_px4_u, voltage_V, force_mean_N, force_std_N, steady_window, mapping_revision`，有条件加电流、PWM实测和原始时序。每台的正反分支先用单调查表/分段插值，另选未参与建表的点验收。当前没有实现采集驱动、自动拟合或向控制器加载真实曲线；这些需在测力设备接口确定后按函数设计单独实现。

第一轮可以用数显推拉力计手工保存稳态读数，不必等自动测力节点才能开始。若只有相机、IMU、深度计，没有测力装置，只能验证执行通路；自由航行时存在惯性/阻力/耦合，不能直接用位移或加速度冒充单推进器推力标定。

工程参考：[PX4 ROS 2消息兼容说明](https://docs.px4.io/main/en/ros2/user_guide)、[DDS串口与Agent](https://docs.px4.io/main/en/middleware/uxrce_dds)、[执行器输出与可逆配置](https://docs.px4.io/main/en/config/actuators)、[Blue Robotics推力台](https://bluerobotics.com/thruster-test-stand/)。读取了上述相关正文；这些是接口/实验装置参考，未验证用户设备，也不照搬参考产品的PWM范围或推力参数。

### 11.12 当前主线：电机可控、反馈可信、简单闭环、MPC（2026-10-07）

目标是Pi 4B + Pixhawk 6C上获得可以解释、复现和改进的控制闭环。内净长1.35m、宽0.65m水缸的首个位置任务为沿长轴、基本固定航向的短程前进/后退，深度与横向偏离先监测；距离按净空、定位误差和停止距离实测确定。现有仿真任意XY到点不能直接接真机，原PlanarTracker会转头朝向身后目标，HippoAllocator使用仿真N级曲线。

| 步骤 | 做什么/复用什么 | 完成标志与实现状态 |
|---|---|---|
| 1. 核实和部署硬件链 | 查固件/ROS/接线/电调/输出配置，在Pi原生编译并接DDS；复用USTROV桥 | 收到真实PX4状态；知道物理插口对应哪个逻辑电机。10-12已实读Pi环境、underlay及已构建USTROV桥；ROV_Control部署与真实飞控握手待验收，见§11.13 |
| 2. 逐台水中短脉冲 | 复用MotorProbe，断动力验链后固定推进器测通道/方向/零点/大致启动死区/停止 | 四台各有映射记录。只验证可控输入范围，不要求牛顿数据；工具已实现，硬件未验证 |
| 3. 只读状态核验 | 复用IMU/压力桥；手动转动、已知浸深变化；核查采样时间/单位/有效性 | 角速度方向和数量级正确，压力来源确为外部水压计。IMU桥orientation无效，不能据四元数默认值控制航向 |
| 4. 首个小闭环 | 在确认有偏航控制能力且浮力/姿态和转角净空允许时，先做小幅机体z角速度PI控制，输出无量纲偏航指令 | 跟踪小参考、回零无持续自激，正常/失效停止可重复；这是待设计的新节点，r=0不等于保持绝对航向 |
| 5. 机载视觉与低速状态 | 下视相机、固定标签；现成C++标定工具+标签几何，先无动力手动验证；再形成可信位置/航向/速度 | 已知位移方向尺度可复现、延迟可测、丢标明确失效。无需先做完整VIO；自研定位/适配仍待实现 |
| 6. 短程位置PD基线 | 固定航向、有符号纵向参考；位置/速度误差输出无量纲纵向指令，与航向控制组合成整组电机命令 | 小幅前进、停止和不掉头后退可重复；横向/深度超范围终止。该真机入口尚未实现，不能调用旧XY入口代替 |
| 7. 局部运动模型 | 在相同经验混控、其他轴控制和限幅配置下，录实际发送电机指令及有效状态，拟合指令到运动的离散模型 | 用另一段记录验证多步预测；只覆盖测过的低速/电压/死区分支，不宣称恢复全部物理参数；待实现 |
| 8. 一维MPC替换 | 先手算预测与QP，再C++核心/数值闭环和现有仿真适配，Pi只算不发后替换纵向PD；其他已验证环和执行桥保持同一配置 | 同参考/限幅/初始条件对照PD与MPC，记录误差、饱和和求解耗时，异常时停止；MPC尚未实现 |

初期真机采用不同于Hippo物理分配的明确接口：`归一化纵向/偏航控制量 → 经通道/方向和小幅组合响应核验的经验混控 → 电机control[8]`。各电机的不一致、死区和饱和必须记录；不把几何矩阵乘无量纲命令称为精确N/N·m分配。若可获得厂家曲线可作带来源的初值；需要物理精度或粗模型限制控制时，再启用§11.11。

纵向教学模型可取`x=[s,v_s]`、无量纲控制量`c_s`，在固定工作范围辨识`x[k+1]=A*x[k]+B*c_s[k]+d`。`s`是home固定长轴上的位置(m)，`v_s`是该轴速度(m/s)。模型输入必须对应日志中实际使用的经验混控和限幅；死区/正反分支不保证一个线性模型通用。视觉速度需要带采样时间的滤波估计，不能直接将高噪声位置差分当作真值；USTROV EKF只在回放/时间/健康接口审核后考虑复用。

硬件接入等待时可继续手算MPC与C++数值课，不把相机或精确测力作为理解预测矩阵的前置条件。新增角速度、视觉、归一化控制/混控、硬件任务入口及MPC均先函数设计再确认实现；本节是实施路线，不代表代码已存在。

### 11.13 第一次SSH实读：Pi现有环境（2026-10-12）

本次通过SSH登录主机ustrov、用户pi（当时DHCP地址192.168.1.114）。Ubuntu 24.04.4 LTS、aarch64、ROS 2 Jazzy已安装并加载；MicroXRCEAgent位于/usr/local/bin，内存报告约7.6GiB，home所在分区约20GB可用。密码不写入文档或项目文件，DHCP地址可能改变。

已有/home/pi/USTROV，检查时HEAD=676c994且工作树干净；ustrov_underlay/install中有px4_msgs，ustrov_ws/install中有ustrov_core。初次查包只加载了系统ROS，因此Package not found；加载已有工作区后发现两个桥可执行文件：px4_actuator_bridge、px4_sensor_bridge。没有发现自研rov_control_ros包，此事实不代表后续已部署MotorProbe。

新SSH终端按现有路径加载：

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/pi/USTROV/ustrov_underlay/install/local_setup.bash
source /home/pi/USTROV/ustrov_ws/install/local_setup.bash
ros2 pkg prefix ustrov_core
ros2 pkg executables ustrov_core
~~~

三层分别是系统ROS、PX4消息依赖、USTROV程序；source只影响当前终端。已确认这些安装产物存在，尚未核对其消息/桥版本与真实飞控是否兼容。

检查时没有Agent或硬件桥进程运行，ROS仅列出/parameter_events和/rosout。所查串口路径中发现/dev/ttyAMA5，没有USB串口结果；pi属于dialout组。ttyAMA5存在不证明Pixhawk接到了这个口，不能据此猜设备路径或波特率。

下一步所需信息为Pixhawk固件名称/版本，以及Pi到Pixhawk的接线（两端接口/引脚、实际波特率）。确认后再安排只读DDS数据接入与状态检查。本次没有安装软件、修改远端项目、启动Agent、请求解锁或运行电机；已经完成的是Pi环境和已安装包的核验。

### 11.14 自研仓库放到Pi：源码与构建产物分开（2026-10-12）

用户要求把Control_Sys_AUV放到树莓派，已从GitHub递归克隆到/home/pi/Control_Sys_AUV。主仓库提交d6bd13dda28933090efddffd84758889ea18d07d与电脑一致，USTROV固定为b1208e50becb9f74d4d0dcd8637ac11e73d3672a，嵌套px4_msgs固定为392e831c1f659429ca83902e66820d7094591410。电脑上的未提交文档修改及被忽略的本地AGENTS.md另行同步；不复制x86构建/安装产物或log。

复现源码下载的命令（目录已经存在时不要再次执行clone）：

~~~bash
git clone --recurse-submodules https://github.com/wanttobedone/Control_Sys_AUV.git /home/pi/Control_Sys_AUV
cd /home/pi/Control_Sys_AUV/ROV_Control
~~~

--recurse-submodules把两层外部依赖取到仓库指定提交；clone得到源代码，不等于ROS包已安装。后续在Pi原生构建，使用这套固定源码规划依赖。此前/home/pi/USTROV及其已有安装仍保留，该旧副本HEAD=676c994；不能未经核验将其产物与新子模块不同版本混为同一套已验证环境。真实固件/串口/消息兼容仍待确认，本次复制没有启动飞控控制程序。

### 11.15 QGC固件摘要与下一步只读检查（2026-10-12）

用户截图显示固件版本1.16.0rc、Generic Octocopter + geometry（Octocopter +）、系统ID=1、自定义固件版本0.0.0。版本标签已知，完整构建哈希尚未知；自定义版本字段不能证明固件源代码未修改。八旋翼软件配置与当前四推进器须区分，逐电机测试前核对逻辑Motor与物理输出、双向模式，不能据此复制八路参数。

在QGroundControl的MAVLink Console分别执行以下只读命令，将输出发回：

~~~text
ver all
uxrce_dds_client status
~~~

ver all用于取得实际固件构建信息；status用于确认DDS客户端是否运行及其通信状态。这两条在飞控控制台执行，不是在Pi SSH终端执行。同时确认Pi到Pixhawk两端的物理接口/引脚和波特率，之后再配置Agent读取状态数据。

新仓库锁定的px4_msgs=392e831c在Pi clone输出中显示v1.16.2标签；固件摘要为1.16.0rc。不能仅按版本名判断消息兼容，需要核对定义/版本及实际收发；PX4 v1.16的[官方ROS 2指南](https://docs.px4.io/v1.16/en/ros2/user_guide)说明消息定义须匹配，存在差异时可评估翻译节点。[客户端配置说明](https://docs.px4.io/v1.16/en/middleware/uxrce_dds)说明PX4客户端和Pi Agent需要按实际通道配对。本轮只有截图和文档核查，未启动控制输出。

### 11.16 实际UART/DDS遥测验证与复现（2026-10-12）

终端位置要区分：用户的QGC在另一台电脑，经USB Type-C连接Pixhawk；其中MAVLink Console执行的是飞控NSH命令，例如ver all、param show、uxrce_dds_client status。以下终端A/B则是电脑SSH进入Pi后执行的Linux/ROS命令，Pi无需安装QGC，两条链可以同时使用。

用户ver all：HW arch=PX4_FMU_V6C、PX4 version=1.16.0 c0、git=38d67f5a937f380e36cdeb8f90884e20ab1b683f，NuttX11.0.0；Build datetime为Mar31 2026。飞控客户端已经使用serial但disconnected，照片端口为TELEM2。参数截图确认SER_TEL2_BAUD=921600、UXRCE_DDS_DOM_ID=0；UXRCE_DDS_CFG值未取得，不填猜测值。

Pi实读：/boot/firmware/config.txt已enable_uart=1并在pi4段dtoverlay=uart5；/dev/ttyAMA5对应serial@7e201a00，活设备树uart5_pins为GPIO12/13，CTS/RTS默认未启用，对应getty inactive。已安装Agent库文件版本2.4.3且当时未运行。无需先改boot配置；树莓派串口名和飞控UART外设编号不是同一个命名空间。

终端A（登录Pi后），运行Agent并保持终端：

~~~bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent serial --dev /dev/ttyAMA5 -b 921600 -v 4
~~~

serial指定串口传输，--dev为Pi设备路径，-b必须与飞控波特率一致，-v4为日志级别。本次出现create_client/session established/participant created，并发现实际/fmu话题。只运行一个占用该串口的Agent；Ctrl+C停止当前Agent。验证后本次临时Agent已停止，由用户在自己的终端复现。

终端B（另开Pi SSH会话），加载已构建且确认同为392e831c的旧消息underlay，只读遥测：

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/pi/USTROV/ustrov_underlay/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list -t
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort --once
ros2 topic echo /fmu/out/sensor_combined --qos-reliability best_effort --once
ros2 topic echo /fmu/out/sensor_baro --qos-reliability best_effort --once
~~~

这里复用的是同版本消息包，不是旧USTROV执行桥程序。域0匹配用户参数；best_effort匹配PX4数据QoS，--once读取一帧结束。本次实际读取结果：

| 消息/检查 | 实际值与边界 |
|---|---|
| VehicleStatus | arming_state=1(上锁)、failsafe=false、pre_flight_checks_pass=false、system_id=1；当前不能据通信成功直接做解锁试验，应先查实际预检原因 |
| SensorCombined | 收到gyro_rad及accelerometer_m_s2；原始PX4数据为rad/s、m/s²、FRD，未通过sensor_bridge转FLU，加速度包含重力，不是已估计水平位置 |
| SensorBaro | device_id=5142033(0x4e7611)、pressure=9943.0、temperature=21.85、error_count=2；按源消息单位为Pa/°C。编码对应MS5837，具体02BA/30BA版本、压力比例/零点和已知浸深响应尚待核验，不能直接称可靠深度 |
| ActuatorMotors ROS图 | 发布者0、订阅者1；本次只有Agent和读取工具，未运行执行桥、请求解锁或发送电机命令 |

按公开Device.hpp的位域及drv_sensor.h，device_id的devtype为0x4E(MS5837)、地址0x76、I2C总线2；这识别驱动类别，不代替型号/安装和压力标定。9943Pa与正常台架环境下约100kPa的绝压量级不一致，先确认比例和已知浸深变化，不能自动乘10掩盖驱动问题。

版本检查范围：对比公开38d67f5的versioned/VehicleStatus.msg与392e831c，当前发现差异是vehicle_type的ROTARY_WING/FIXED_WING/ROVER常量分别0/1/2和1/2/3；消息读取成功不等于全部类型/枚举已兼容。实际固件导出sensor_baro，而公开同提交的默认DDS YAML未列出它，因此构建配置可能有本地修改，不能仅凭hash推断完全等同公开源码。后续控制接入前需分别核接口，本批只完成遥测通路。

同日用户复现尚未成功：Agent仅显示running/logger setup而无session established，ROS域0下无/fmu话题。再次实读Pi仅一个Agent占用ttyAMA5，921600/8N1、无流控、getty inactive。此时先保持Agent运行，在另一台PC的QGC控制台读取当前uxrce_dds_client status；根因和恢复尚未验证，不把重新安装ROS/复制代码作为已经定位的修复。

后续status截图确认飞控客户端Running/disconnected、serial、timesync false，循环已258435次。下一步重建该客户端会话，飞控保持上锁，Pi Agent保持运行，在另一台PC的QGC MAVLink Console一条完成后再执行下一条：

~~~text
uxrce_dds_client stop
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
uxrce_dds_client status
~~~

/dev/ttyS3是公开同固件hash的FMUv6C default.px4board中CONFIG_BOARD_SERIAL_TEL2映射；/dev/ttyAMA5是Pi的UART5设备，两个路径分属两块板，不能互换。stop结束当前DDS客户端，start显式按TELEM2和已读波特率重新建立会话，status检查连接。启动参数格式已核对[官方v1.16模块说明](https://docs.px4.io/v1.16/en/modules/modules_system#uxrce_dds_client)。等待数秒，目标证据是客户端connected和Agent session established，再检查实际ROS话题；仅启动成功不足以验收。本次尚未由用户执行/回传结果，不将旧会话状态猜测写成根因，也不修改持久参数。

用户随后回传成功的SensorCombined读取，timestamp=1791793969632654、gyro_rad约(0.000269,-0.000857,-0.001615)、accel约(0.3586,-0.2751,-9.7325)m/s²。数据模约9.74，符合近静止时重力响应；clip字段均0，仅一帧不代表长期偏置和质量已验收。gyro_integral_dt=4556、accelerometer_integral_dt=4429的单位µs，是积分时间窗，不直接代表ROS话题或控制周期。恢复数据已确认，但具体重启操作输出未回传，因此根因仍不作确定结论。

### 11.17 Pi原生构建与ARM软件验证（2026-10-12）

Eigen3.4、GoogleTest1.14、tf2_ros、diagnostic_msgs已在Pi安装。复用同提交392e831c的已编译消息underlay，在新固定USTROV源码只构建两个必需包，未修改外部源码：

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/pi/USTROV/ustrov_underlay/install/local_setup.bash
cd /home/pi/Control_Sys_AUV/USTROV/ustrov_ws
MAKEFLAGS="-j2" colcon --log-base log/pi_hardware_build build \
  --base-paths src --build-base build/pi_hardware \
  --install-base install-pi-hardware --symlink-install \
  --executor sequential --packages-up-to ustrov_core \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
~~~

实际结果2包完成，6min34s；安装至USTROV/ustrov_ws/install-pi-hardware。build/log已被子模块规则忽略，新增安装目录在该Pi的.git/modules/USTROV/info/exclude中局部忽略；不修改子模块的.gitignore或固定提交。

随后构建自研包和正式测试：

~~~bash
source /home/pi/Control_Sys_AUV/USTROV/ustrov_ws/install-pi-hardware/local_setup.bash
cd /home/pi/Control_Sys_AUV/ROV_Control
MAKEFLAGS="-j2" colcon --log-base log/pi_build build \
  --base-paths src --build-base build/pi_native --install-base install-pi \
  --symlink-install --executor sequential \
  --packages-select rov_control_core rov_control_ros rov_bringup \
  --event-handlers console_cohesion+ status- \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DROV_BUILD_PX4_MOTOR_PROBE=ON -DBUILD_TESTING=ON --no-warn-unused-cli
~~~

初次收到数学核心完成(1min17s)后网络中断，用户随后确认断电。重新登录核对启动时间17:05:38、无旧编译进程、原日志中断而无编译错误，再按上面命令增量续编；实际三个自研包全部完成，本次10min41s。未删除源码或已有产物，也未启动第二个并发构建。

每个新的Pi终端使用这四层环境：

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/pi/USTROV/ustrov_underlay/install/local_setup.bash
source /home/pi/Control_Sys_AUV/USTROV/ustrov_ws/install-pi-hardware/local_setup.bash
source /home/pi/Control_Sys_AUV/ROV_Control/install-pi/local_setup.bash
~~~

依次加载系统ROS、已确认同392e831c的PX4消息、新固定版本USTROV硬件包、自研包。不要在这条链中额外source旧USTROV的ustrov_ws安装；电脑WSL的install-motor与Pi的install-pi不互相复制。终端A仅运行Agent时加载系统ROS即可；包含库/节点/消息的终端按需加载对应层。

实际验证：file -L确认motor_probe_node为ELF64 ARM aarch64；ros2 pkg prefix rov_control_ros定位到install-pi；ros2 launch rov_bringup motor_probe.launch --show-args成功列出参数（这是XML解析，不是实际节点启动）。数学核心两个CTest入口通过，共21个规划/10个跟踪分配用例；另执行MotorProbe的12个正式软件测试，约30.44s完成：

~~~bash
cd /home/pi/Control_Sys_AUV/ROV_Control
ROS_DOMAIN_ID=124 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
colcon --log-base log/pi_test test \
  --build-base build/pi_native --install-base install-pi \
  --packages-select rov_control_ros --executor sequential \
  --event-handlers console_cohesion+ status- \
  --ctest-args -R '^motor_probe$' --output-on-failure
colcon test-result --test-result-base build/pi_native --verbose
~~~

结果为43个GTest用例通过；colcon包含MotorProbe的CTest入口共44项，0错误/失败/跳过。数学核心通过直接ctest执行，故没有额外dashboard入口计数。没有重跑其他ROS测试，也未验证MPC/真实电机。隔离测试使用域124、本机、C++模拟PX4和新编译的真实桥，测试前缀为motor_test_PID，未启动Agent/访问串口；与真实DDS域0不同。非法参数用例重复创建同名临时节点会有rosout重复注册提示，测试仍通过，不是实际电机命令源冲突。

构建/测试日志在log/pi_build/、log/pi_test/及USTROV/ustrov_ws/log/pi_hardware_build/；正式结果在build/pi_native/*/test_results。最后检查无MotorProbe/测试桥/Agent进程，子模块工作树干净、主仓库只有同步文档修改。编译产物和日志忽略，正式C++测试源码保留。

当前允许下一步只读核查真实VehicleStatus/预检原因及电调输出映射；安装成功不代表已完成真机解锁、PWM/电调中位/正反方向、深度比例或闭环。实际电机试验在这些条件核验后单独安排，不直接加载Hippo仿真参数或发布任意XY目标。

### 11.18 一体推进器、QGC输出映射与当前下一步（2026-10-12）

用户确认电调内置于推进器。供货图列正向31N、反向26N、7A/6S、PWM/DShot/DShot双向、PWM脉宽1～2ms；这些不是本机测力或协议验收。图中“4S～6S LIPO (7.4～22.2V)”不一致：普通LiPo的4S标称14.8V、6S标称22.2V，6S满充25.2V，需供货方确认实际允许满充电压；不据此更换电池。图中未给PWM停转中位、死区、初始化等待、允许频率、干转条件；不要仅凭1～2ms推定1500µs停转。DShot双向字样也不能代替对正反推力模式的确认。

用户随后补充AUX页面，并说明四台推进器统一接电源控制板、PWM线束由飞控AUX引出。当前功能分配如下；它说明软件对应关系，不证明电源板内部信号透传或四台的实际针脚/机体位置：

| 物理输出 | 当前功能 | 默认桥映射下的probe输入 |
|---|---|---|
| AUX1 | Motor1 | channel0 → control[0] → Motor1 → AUX1 |
| AUX2 | Motor2 | channel1 → control[1] → Motor2 → AUX2 |
| AUX3 | Motor3 | channel2 → control[2] → Motor3 → AUX3 |
| AUX4 | Motor4 | channel3 → control[3] → Motor4 → AUX4 |
| AUX5～8 | Motor5～8 | 仍有软件功能分配，不证明有实装电机或完全未接线 |
| MAIN3、MAIN4 | Motor3、Motor4 | 同一逻辑功能也分配在MAIN，是否留空需按实际接线确认 |
| 其余MAIN | Disabled | 当前无已分配功能 |

MAIN/AUX各分组为PWM400Hz，显示Disarmed1500、Minimum1100、Maximum1900（单位µs）。400Hz是物理PWM帧率，桥/probe的50Hz是ROS命令发布率，两者职责不同。Rev Range(for Servos)是舵机范围反向，不是电机双向；左侧Direction CCW描述几何旋转方向，也不是运行中正反推力开关。[官方执行器配置](https://docs.px4.io/v1.16/en/config/actuators)的Bidirectional/Reversible才是几何中的双向电机选项；电调自身还须支持并启用对应模式。先确认接线，之后再清理确实未用的MAIN及AUX5～8功能；本轮未修改配置。

当前代码路径是MotorProbe::onStart请求现有桥解锁；PublishOffboardHeartbeat把position/velocity等设false、direct_actuator设true；PublishMotorOutput按motor_input_indices和motor_signs填ActuatorMotors并带reversible_flags=255。默认映射为0..7，但第5/8逻辑电机符号为-1；这源自外部配置，尚非本机标定。QGC左侧八旋翼几何由软件机架选择产生；本工具直接给逻辑电机输入，不用该八旋翼矩阵混控，物理输出功能分配仍必须正确。

QGC截图当前为Hold/Not Ready，报告无有效位置、全球位置、高度估计及无手动输入。依据[官方v1.16 Offboard控制层表](https://docs.px4.io/v1.16/en/flight_modes/offboard)，direct_actuator这一层不要求位置估计；不能把Hold页面直接当作MotorProbe解锁结论，也不能据此保证解锁成功。PX4仍负责模式/解锁检查，桥没有force-arm参数；onStart服务success仅表示已开始请求，RUNNING才表示其观察到Armed/Offboard和桥输出。高度报错不直接证明本项目压力驱动故障，仍须分开验收。

代码中的msg.direct_actuator=true是已实现的C++赋值，不能在QGC/NSH执行。用户误输入该行得到command not found，说明它未作为命令生效，无需为此改代码。参数截图尚未返回实际数值，不能认为已查出设置；截图第一项的名称似被写成空格。下面是实际控制台命令，不要输入nsh>提示符或分号，不一次粘贴多行。

在另一台电脑的QGC MAVLink Console保持飞控上锁，先粘贴这一行、按Enter、等待输出：

~~~text
param show COM_RC_IN_MODE
~~~

收到第一项的名称和值后，再粘贴这一行并按Enter：

~~~text
param show COM_ARM_WO_GPS
~~~

show只读取、不更改。第一项确认是否仍要求RC/Joystick；官方v1.16的4表示忽略手动杆输入，设为4会失去通过杆输入接管的用途，不能当成只消除提示的修改。第二项查询GPS检查策略；v1.16参数参考定义0拒绝/1警告/2禁用，不能照旧版本说明猜当前值。[参数参考](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference#COM_RC_IN_MODE)

如果单条输入仍没有数值，可在QGC Parameters页面分别按完整参数名搜索、回传显示值。AUX截图已收到，剩余需要电源控制板PWM接口及接线近照、实际电池串数/电压、卖家确认的PWM中位/频率；据此确定单电机试验的channel及幅值。电池→电源板→内置电调是供电链；Pixhawk AUX→电源板信号接口/线束→内置电调是控制链，板上究竟如何转接待核。当前不提供start/解锁操作，不更改机架/输出/检查参数；没有运行电机或新增实现代码。下一批实验按确认中位→核对物理映射→短时低幅正反脉冲推进，完整测力仍不是起步前置。

## 12. AprilTag实物ID与旋转检查（2026-10-19，已有运行截图）

用户选择有桌面的Ubuntu电脑，USB相机直接连接该电脑。先用[AprilRobotics官方C++示例](https://github.com/AprilRobotics/apriltag/blob/master/example/opencv_demo.cc)清点标签、检查检测和旋转，本节不依赖ROS或内参；10-21另实现离线标定适配见§13.8。用户已提供多标签ID/边框截图和后续录像，尚无逐帧真实检测率/完整ID清点或米制精度结论。已有录像先离线筛选与标定检查，再做地图和只读位姿验证。

在这台Ubuntu电脑的桌面终端执行，首次安装工具：

~~~bash
sudo apt update
sudo apt install build-essential cmake git libopencv-dev v4l-utils
mkdir -p ~/vision_tools
git clone --depth 1 https://github.com/AprilRobotics/apriltag.git ~/vision_tools/apriltag
cmake -S ~/vision_tools/apriltag -B ~/vision_tools/apriltag/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON \
  -DBUILD_PYTHON_WRAPPER=OFF -DBUILD_TESTING=OFF
cmake --build ~/vision_tools/apriltag/build --target opencv_demo -j2
~~~

apt更新包索引并安装编译器/构建工具、OpenCV开发库和摄像头查询工具；mkdir创建独立工具目录，clone下载官方源码（已有该目录时直接复用并核版本，不重复clone）。cmake的-S/-B指定源码/构建目录，Release启用优化，BUILD_EXAMPLES启用示例、关闭Python包装和库测试；--target只构建opencv_demo，-j2控制并行编译任务数。不执行sudo make install，第三方工具源码和产物在个人工具目录，不写入USTROV或ROV_Control。

查询摄像头：

~~~bash
v4l2-ctl --list-devices
~~~

同一相机可能有多个video节点，选实际图像采集节点；以下以/dev/video0为例：

~~~bash
~/vision_tools/apriltag/build/opencv_demo -c 0 -f tag36h11 -x 1
~~~

- -c 0为相机索引；例如采集节点为/dev/video2时改-c 2。打开失败先核节点和是否被已有预览程序占用。
- -f tag36h11指定实物字典。
- -x 1表示检测不降采样，便于先观察细节，处理速度仍需实测。
- 示例用驱动默认图像模式，启动会打印宽高/FPS（不等于实测端到端频率）。清点ID可以先用默认模式，冻结布局前要在最终分辨率/裁切/曝光和湿光路复核。
- 弹窗内有彩色边框和数字ID；示例绿边为p0→p1、红边为p0→p3，可用红绿交点追踪同一个物理角。键盘焦点在图像窗口时按任意键退出。

检测步骤：逐张正对相机，记录ID并在背面或标签白边之外标记；每张面内旋转90°/180°/270°，正常解码时ID不变，参考边/角点随实体旋转。清点实际16张的ID、有无重复，并记录与包装范围的差异。再临时摆2×2，检查实际高度、倾角、两标签/四标签交接处的ID稳定性；先空气中测试，再最终水中复核。可静止观察约10s并旋转/移动作为人工初筛，正式检测率和最长丢检还需逐帧统计，不把人工预览当量化验收。

ID由图案编码，不需要印数字；当前地图一ID对应一个固定物理位置，排列顺序可以任意。图案也编码规范方向，检测器处理面内旋转；物理安装无强制朝上要求，但地图必须记录真实角点/朝向，不能只量中心再将全部旋转设为零。建议固定相机，选检测到的同一参考边朝同一缸边以便测量，或直接量每个有序角点的地图坐标。详细数据流与边界见总体设计§13.8。

## 13. 用现有AprilTag做水中相机标定与地图测量（2026-10-21修正）

用户明确现有16张AprilTag，无棋盘格；本节使用固定缸底标签，撤回10-20独立棋盘格路线。用户已确认检测黑框宽高均80mm，包装外100mm不作求解边长；真实ID集合仍需记录。已有多标签检测截图和用户水中录像，尚未得到真实K/D、地图或定位精度。10-21用户确认设计并授权实施：新增rov_localization离线C++适配工具，复用原生AprilTag与OpenCV，先自动选同一ID多视角；实际命令见§13.8。水缸内净1350×650mm，用户确认录像使用最终湿光路并有倾角/高度变化；视频尚未在本机处理，模型/精度仍待验收。

### 13.1 三份数据的职责

| 数据 | 产生方式 | 用途 |
|---|---|---|
| 相机模型、K、D、图像尺寸 | 最终光路中多视角拍摄已测量几何的AprilTag，调用标定求解器 | 把像素与成像方向联系起来；K焦距/主点单位px，D含义依模型 |
| ID与有序四角的map坐标 | 实测缸底标签尺寸/位置/方向 | 将识别ID对应到固定世界几何，坐标统一m |
| T_BC | 相机固定在ROV后测量/复核安装关系 | 将相机定位变成base_link定位；平移m、旋转为明确方向的矩阵/四元数 |

原始视频、实测标签几何和求解结果是实验数据；尚未生成任何正式标定YAML。后续输出包含模型/K/D/实际图像尺寸和误差，字段与ROS CameraInfo/配置须显式对应，不能只改后缀。

### 13.2 固定最终光路与图像模式

1. 正式数据使用实际水中光路：相机与标定图案之间的最终窗口/壳体/水层应与运行一致，不临时增加水缸侧壁玻璃。空气中先练习工具流程，正式参数单独用水中数据产生。IP66本身不能作为可浸水验收；如果另加壳体，带最终窗口一起标定。
2. 固定焦距、对焦、图像分辨率及裁切/缩放方式；关闭会改变几何的数字变焦/防抖，控制自动对焦（如设备有此功能）。曝光应让图案清楚且少拖影；只改曝光通常不改变K，但会改变可检测性。记录设备、模式、光路和OpenCV版本。
3. 退出占用相机的opencv_demo，再在该桌面Ubuntu终端查询。10-21用户实录确认本次/dev/video3可采图，后续插拔仍需核设备节点：

~~~bash
opencv_version
v4l2-ctl -d /dev/video3 --list-formats-ext
v4l2-ctl -d /dev/video3 --list-ctrls
~~~

opencv_version核库版本；formats核可用像素格式/尺寸/帧率；ctrls核曝光/对焦选项（没有的控制项不强设）。需要时安装采集工具/求解库：

~~~bash
sudo apt update
sudo apt install libopencv-dev ffmpeg
~~~

原opencv_interactive-calibration的棋盘格参数不适用于当前AprilTag录像，不再执行-t=chessboard或-w=9/-h=6。现有opencv_demo仍检查ID/角点；本项目离线标定工具已实现并通过软件测试，按§13.8构建运行。

### 13.3 先标相机内参，再测完整标签地图

不增加独立承载板，保持缸底标签平整、位置固定。用户10-21补充单格10mm，并进一步确认检测黑框外边宽高均80mm（0.080m），不包含外围白边；该实测尺寸可作当前单标签目标参数。内部编码格不是棋盘格内角点，不可据此把旧命令改为8×8/10mm继续使用。现有标签的浸水耐久性以实际材料/粘接验收为准。

内参与整缸地图分两步，不能把测完16张标签坐标作为所有内参标定方法的前置条件：

- **起步用单标签多视角。** 从录像选一个反复清晰出现的真实ID；只取这张标签的四个有序检测角点，以实测宽高构造其局部平面四角（m、Z=0）。相机移动、俯仰/横滚和改变距离，生成多组观测；无需先量该标签在缸底的位置、16张间距、水深或每帧相机姿态。画面允许出现其他标签，首版不把未知相对几何合并。每帧只有四点，约束与角点冗余较少，需检查视角/边缘覆盖和独立位移，不能将生成参数视为精度验收通过。
- **联合多标签时再量相对几何。** 同一帧的多张标签若作为一个目标共同求解，必须知道各ID在同一目标系中的有序角点；不能把每张标签都以自己的中心为原点后直接拼接。可以先测一小片标签，无需一次测全16张。完整缸底地图按§13.6另测，用于后续map定位。

标定的输入是已知几何点与多张图像中的对应像素，不限定为棋盘格。当前原生AprilTag检测输出可按ID/有序角点关联到实测几何，再交给OpenCV C++ calibrateCamera等求解器；不自己写非线性优化器。不同帧可以看到不同标签子集，只要对应关系正确、点分布和视角约束充分；不要求拍全16张，也不强制把它们重排成标准AprilGrid。

用于普通平面内参初始化的目标系取标签平面Z=0、长度统一m；单标签时可直接以该标签中心为原点。这不是运行map，也不改变map W的参考水面Z=0契约。运行角点仍用W中的z=-H；相机内参求解不要求先建立目标系与W的关系，米制map定位前再补齐地图。

### 13.4 录制原始标定视频

示例先选择设备明确支持的MJPG、1280×720、30fps；三者须由formats输出确认，否则按支持值替换。标定结果绑定实际图像模式，不能仅凭500万商品信息填写尺寸。创建新的数据目录，示例录像不覆盖已有数据：

~~~bash
mkdir -p ~/vision_data/calibration_wet_01
cd ~/vision_data/calibration_wet_01
ffmpeg -n -f v4l2 -input_format mjpeg -video_size 1280x720 \
  -framerate 30 -i /dev/video3 -c:v copy wet_01.mkv
~~~

- -n拒绝覆盖已有输出；第二次试验更换目录/文件名。
- -f v4l2选择Linux相机输入；input_format/video_size/framerate请求已确认的采集模式。
- -i是设备输入，-c:v copy将MJPEG直接装入视频容器，不额外有损编码；若设备只有YUYV，需另选适当采集/编码方案，不照抄此MJPG示例。
- 在FFmpeg终端按q结束并完成文件写入；可用ffplay wet_01.mkv回看清晰度。

10-21用户日志确认已录得wet_01.mkv：/dev/video3、MJPEG、1280×720、30fps、约2分13.5秒、约1.45GB；Ctrl+C后正常结束。用户确认最终水中光路及俯仰/横滚/高度变化。随后已在lab用0.1.1处理并生成候选XML，结果见§13.10；本机未读取原视频。保留现有录像，不因先前缺少XML直接重录。

标签固定在缸底，正式采集让相机在最终水中光路中移动和倾斜；保持镜头与窗口相对关系不变。采集约20～30个有差异的清晰姿态作为起步样本目标，不是算法最低数或精度承诺。每个姿态稳定约1～2s。单标签路线让选定ID完整进入中央和四周，并产生不同倾角/尺度；已测多标签路线可优先看到几张分布开的完整标签。不把单帧完整16张作为条件。

| 画面变化 | 实际操作 | 增加的约束 |
|---|---|---|
| 中央及四周 | 移动相机，让可用标签分别进入画面中央、四角/四边附近 | 覆盖不同成像位置，观察边缘畸变 |
| 不同距离 | 在可成像范围改变相机离标签平面距离，覆盖实际20～50cm计划区间；若将来在10cm运行则另验该高度 | 改变投影尺度，避免只适配一种距离 |
| 不同倾角 | 改变相机俯仰/横滚，例如约15～30°的中等倾斜，保持完整标签清晰 | 改变透视几何，改善内参估计可辨识性 |
| 独立复核姿态 | 另拍一段不同位置/距离的视频，不用于拟合 | 留作验收，避免只检查训练数据 |

不能只改变航向、只平行平移，或在同一姿态连拍几十帧。气泡、饱和反光、糊帧、严重透视压缩或无完整可解码标签的画面不用于拟合，现有有效样本不足时再补拍。联合已测多标签时不同帧的ID/角点数可以变化，实际配对由检测ID和实测目标几何确定。

### 13.5 离线求解并检查相机参数

现有opencv_demo只给出ID和四角，不会求K/D。10-21实现的calibrate_camera流程为：读原始录像→原生检测ID/有序四角→选一个样本充分的ID→用80mm宽高生成局部四角→筛选不同视角并留出时间组→调用OpenCV标定→子集焦距稳定性/法线/重投影/径向映射检查→输出候选参数/报告/选帧CSV。后续若增加联合多标签输入需另提供实测相对几何。没有重写解码器或非线性优化器。

文件/函数/状态/验证说明见PATH_TRACKING_DESIGN§13.10。独立rov_localization包支持无ROS的CMake和ament构建；原视频只读，新结果目录须尚不存在，防止覆盖。失败仍保存具体诊断但不写camera_wet.xml；成功参数也明确标为candidate_requires_metric_validation。已有录像先检查，不预设需要重拍。

复用边界补充：本地USTROV b1208e5有include外部visual_localization的入口，但没有该视觉包源码；fake_vision订阅真值Odometry，不能用于真实相机。Hippo上游已有视觉链与calibration.launch.py，但官方标定流程使用标准ROS棋盘格标定器。应优先复用Hippo/AprilTag/OpenCV，只增量补当前AprilTag录像适配；上面的新包目录是拟议组织方式，不是必须全部重写。K/D计算只需最小离线入口，无需等在线定位/滤波/控制完成；先检查已有录像，只有有效样本/覆盖不足才补拍。

用户旧命令未产生camera_wet.xml，不能把-of理解为无条件创建结果文件：该交互工具的-t=chessboard寻找9×6棋盘格内角点，不是AprilTag编码格。源码中缺少defaultConfig.xml会回退默认高级参数，非致命退出条件；保存函数仅在相机矩阵非空时写结果，正常结束还检查标定状态。现有日志不足以确定应用为何迅速退出，但目标类型不匹配已使该命令不适用；无需补造defaultConfig.xml或把按s当作解决方案。

普通平面针孔基线可调用cv::calibrateCamera，输入每帧在P中的实际角点与对应像素，初始化平面点Z=0；不使用尚未标定的K先算出“真值”几何来标定自己。若需fisheye等模型，按实际数据决定并调用匹配API，不能把不同模型D混用。K焦距/主点单位px，D依模型，目标长度m；每帧求得的相机/目标外参也不等同T_BC。

保存原始视频、实测标签几何、参数结果、相机模式和水中光路记录。检查校正后结构、图像边缘和留出数据；重投影误差不能直接证明厘米精度。若残差随位置/距离系统变化，先核测量、模式、目标平面与采样，再评估镜头模型/折射或有效工作范围。平面水界面非严格单一针孔，湿标定仍需独立位移验收。

### 13.6 建立固定水缸坐标与地图

1. 排干/降低水位便于测量，在标签之外标固定缸底内角P，X沿135cm长边、Y沿65cm短边，俯视为右手XY，Z向上。map W的XY与P对齐，Z零点沿用建图参考水面并在缸壁留固定标记；量该标记到标签表面的H。标签共面时z_W=-H，水面以后变化不重置地图，压力数据另转换。P不是机器人上电任务home。
2. 确定最终标签布局并固定。实测每张黑白交界检测边长（包装80mm仅作候选），中心位置、方向，或直接测四个检测角点；不要量100mm白边外角来替代黑框角点。使用钢尺/直角尺作垂直投影，所有坐标从同一参考量取，减少逐张累加误差；检查交叉距离、重复量取和标签平整度。
3. 对每个真实ID核有序四角：官方示例红绿边交点p0、绿边另一端p1、红边另一端p3、剩余点p2。保存顶视参考照片并记录该顺序对应哪些实际角；不能将图像左上角固定当p0。贴得与轴对齐时可由中心±半边长得到几何角点，再按实际方向排列；有斜转时按实测旋转或直接量角点。
4. 建议原始表先记毫米及参考标记：ID、边长、p0…p3的X/Y/表面高度、复测差值、照片索引；进入地图时统一m。ID与位置一一对应，未知/重复ID不补猜位置。H量到标签表面，不能以0.8m缸高代替。
5. 后续tank_tags.yaml使用每个ID的实际边长和有序P_W[4][3]；配置尚未生成，当前不要抄示例假坐标。初次一张/小片地图核轴向与尺度，再扩展全缸；标签移位/旋转后重测相应项。

### 13.7 验证地图、相机定位与机体安装关系

接入拟实现的只读定位工具后，以固定夹具/直尺导向让相机光心近似保持姿态，沿map X/Y移动已知50/100mm，在中心、边缘和标签交接处重复，记录估计位移偏差和静止散布。标定数据与验收位置分开；手持位移只查大致方向，不作毫米精度证据。图案共同旋转/定位翻转、跨ID跳变、位置随高度/画面位置系统漂移均需查明；当前opencv_demo只显示ID，不会自动输出这一步所需位置。

相机装回ROV并刚性固定后，按B=FLU、C=光学右/下/前测T_BC：光心相对机体原点的平移与三轴方向，方向通过已知姿态复核；内参标定中每帧板外参不是T_BC。PnP得到T_CW，再用T_WB=inverse(T_CW)*inverse(T_BC)得到base_link在map的位姿，T_AB始终表示B到A。最终链为图像+K/D+ID对应世界角点→相机map位姿→安装外参→机体map位姿；不提前给T_BC填单位阵。

软件工具与用户设备验收分开：离线C++工具已在本机WSL构建/测试；lab已处理真实录像并生成该相机的候选XML（§13.10），本机核对了XML/报告/CSV，未读取原视频。地图、安装外参和独立米制精度尚未验收。在线定位节点和XML launch仍待后续按复用缺口设计实施。

### 13.8 实际构建、运行与报告解读（2026-10-21）

**本批文件与职责。** src/rov_localization中vision_types.h定义像素/米/秒与观测数据；AprilTagDetector拥有原生family/detector并输出同一p0..p3顺序；CameraCalibrator负责扫描/选ID/选帧/求解/检查/保存；main只解析命令行并调用run；config/calibration_options.yaml保存80mm与实验选值；正式C++测试保留。没有新增相机ROS节点、滤波或执行器发布者，没有改USTROV。

**将源码放到lab电脑。** 此批尚未自动提交/推送，直接git pull不会带上未提交文件。可复制src/rov_localization目录，也可用当前交付的rov_localization_20261021_r2.tar.gz（工具0.1.1，不含构建/视频）；它替代旧r1包，修复帧数元数据误判。假设新包已复制到lab的~/Downloads，解压到新父目录以保留旧源码/构建：

~~~bash
mkdir -p ~/vision_tools/calibration_r2
tar --keep-old-files -xzf ~/Downloads/rov_localization_20261021_r2.tar.gz -C ~/vision_tools/calibration_r2
cd ~/vision_tools/calibration_r2/rov_localization
~~~

mkdir创建工具父目录；tar解压源码，--keep-old-files拒绝覆盖已有同名文件，若已有旧副本先换一个新父目录；cd进入带CMakeLists.txt的包根目录。已有~/vision_tools/apriltag不受影响。

**安装构建依赖并编译同一份C++，无需ROS：**

~~~bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libapriltag-dev libgtest-dev
cmake -S . -B build -DROV_LOCALIZATION_WITH_AMENT=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
ctest --test-dir build --output-on-failure
~~~

- apt update刷新包索引，install安装编译器/CMake/依赖与正式测试库。
- cmake -S .读取当前源码，-B build将产物放build；WITH_AMENT=OFF取消ROS包装；BUILD_TESTING=ON构建测试；RelWithDebInfo保留调试信息并优化。
- cmake --build实际编译，-j2最多两个并行编译任务；ctest运行测试并只在失败时打印详情。
- Ubuntu24.04的AprilTag3.3.0包有错误CMake导出路径，本项目优先通过pkg-config链接同一已安装库；不修改系统文件。程序记录实际OpenCV/AprilTag版本。

**直接处理原录像，不重新采集：**

~~~bash
./build/calibrate_camera \
  --video="$HOME/vision_data/calibration_wet_01/wet_01.mkv" \
  --config="$PWD/config/calibration_options.yaml" \
  --output="$HOME/vision_data/calibration_wet_01/results_02"
~~~

--video指定已保存录像；--config读取尺寸/筛选/模型选值；--output指定尚不存在的新结果目录，其父目录须存在。$HOME为该Ubuntu用户家目录，$PWD为当前包目录。命令使用--参数=值，路径有空格也保留引号；反斜杠只续行，后面不加空格。示例results_02若已存在则改为results_03，保留上一结果。默认target_id=-1自动比较覆盖/尺度/数量；需手动指定时在同一命令末尾加--tag-id=实际ID。--help显示入口参数。

退出码0：本批图像检查通过，写camera_wet.xml、calibration_report.yaml、selected_views.csv。XML含pinhole、plumb_bob、3×3 camera_matrix、5项dist_coeffs、实际图像尺寸、ID/尺寸/来源/版本，尚不是ROS CameraInfo文件，不只改后缀直接装入驱动。退出码2：质量/采样/求解失败，报告具体原因，无相机XML；退出码1：输入参数/输出路径/写盘等入口错误，终端显示原因。

~~~bash
cat "$HOME/vision_data/calibration_wet_01/results_02/calibration_report.yaml"
head -n 8 "$HOME/vision_data/calibration_wet_01/results_02/selected_views.csv"
~~~

cat看报告；head看CSV表头和前7条。报告先看status/message、selected_id、fit_views/holdout_views、rms_px/holdout_rms_px、image_span_x/y、normal_spread_deg、focal_split_relative_change及tags/rejections。包络span不是均匀覆盖率，解码margin不是角点协方差；CSV的frame_index/time可定位原视频样本。无标签/模糊/完整性拒绝多先查成像，同ID视角不足再针对性补拍；不要只为生成XML盲目放宽选值。

**参数修改与验证边界。** 所有实验选值位于config/calibration_options.yaml，修改后重新运行、新建结果目录。首版仅pinhole，不自动断言商品镜头模型。OpenCV4.6标准差实现对每帧四点会产生NaN，本工具仍复用其K/D求解，改用两组视角各自重拟合的焦距相对变化检查，报告不是统计标准差/置信区间。时间由文件帧号/FPS构造，仅供抽样；真实曝光时间未知。留出按视频时间组分离，仍是同段视频的图像检查，不替代§13.7独立尺量位移/高度/边缘验收。

**已验证的软件范围。** 本机WSL Ubuntu24.04、GCC13.3、OpenCV4.6.0、AprilTag3.3.0，独立CMake构建及8项GTest通过（CTest一个汇总入口）。包括真实36h11图案四向旋转、已知K/D投影恢复、静止重复帧拒绝、无标签录像失败诊断、合成MJPEG录像→检测→选样→求解→XML/报告/CSV，以及原视频字节不变/拒绝覆盖。临时测试视频/结果由正式测试夹具清理，测试源码保留。初次构建暴露并修复AprilTag包导出路径/测试utils库链接问题；初次测试6通过2失败，定位OpenCV4.6标准差自由度实现后改成子集稳定性检查，最终8通过，失败日志保留在忽略的构建目录。0.1.1测试数和lab真实录像的后续结果分别见§13.9、§13.10；本机未读取原视频，独立米制精度仍未验收。

同一份源码已通过ROS2 Jazzy的colcon/ament构建安装，并验证source对应install后ros2 run rov_localization calibrate_camera --help可发现入口。该检查未启动ROS图像/定位或真机控制。桌面Ubuntu先按上述无ROS方式运行即可；本批是离线命令行程序，不需要launch。

### 13.9 首轮真实报告与0.1.1修复（2026-10-21）

用户在lab运行r1后提供calibration_report.yaml和仅表头的selected_views.csv：OpenCV4.6.0/AprilTag3.3.0，1280×720/30fps，实际解码4002帧、抽样667帧，容器报告4006帧，耗时约101.77s。旧工具因“实际帧数+1小于报告帧数”在扫描结束抛错；selected_id=-1、fit/holdout=0，NaN只是尚未求解，不是已算出的K/D失效。

这是工具检查依据错误：OpenCV FFmpeg get_total_frames在nb_frames缺失时会由duration×fps估算帧数，不能把非零CAP_PROP_FRAME_COUNT当精确总数。相差4帧（约0.1%）本身不足以判断录像损坏，也不足以证明完整无损。0.1.1删除该致命判断，保留frame_count_difference（实际减元数据）、frame_count_warning和eof_limit；有差异只提示并继续选样/求解。read=false仍不能区分EOF与解码错误，无图/分辨率变化/样本与质量门控仍有效。若需独立排查解码，在lab运行ffmpeg -v error -i wet_01.mkv -map 0:v:0 -f null -检查错误输出，单独保留结论。

首轮报告有候选ID16=153、1=105、0=102、9=101、10=82，不能把候选数当不同视角数；当时尚不知道这些ID的有效覆盖，因此先用0.1.1重跑原视频，不重新采集。首轮CSV仅表头，无法据其补算K/D；后续重跑已有完整60行CSV和候选XML，见§13.10。

修复涉及vision_types.h（诊断字段）、scanVideo/saveResult（差值告警/报告版本）、构建元信息0.1.1和正式测试。新增用例修改临时AVI元数据为19帧、保留15张真实图像，确认报告差值-4且能进入选样阶段，不再因该差异抛错；白图仍因没有标签失败，不写假XML。修复后9项GTest通过，原合成录像到XML路径回归通过；临时用例文件自动清理。真实录像未在本机读取；lab后续选样/求解结果见§13.10，独立精度尚未验收。初版8项测试记录保留在§13.8，当前版本测试数为9。

### 13.10 lab真实湿标定候选结果与下一步（2026-10-21）

本节依据用户提供的camera_wet.xml、calibration_report.yaml和完整selected_views.csv，三份输出已核对。工具0.1.1、OpenCV4.6.0、AprilTag3.3.0，lab处理约36.82s，4002解码/667抽样帧；原始录像未在本机读取。用户将该次实验配置max_holdout_rms_px从2.0改为2.3后重跑，XML与报告状态为candidate_requires_metric_validation。此为本次实验选值，仓库默认配置仍为2.0；不是算法误差降低或已完成独立定位验收。

| 检查量 | 实际结果 | 解释 |
|---|---|---|
| 标签与视角 | ID16；47拟合、13留出 | 只使用该ID的80×80mm已知局部几何；没有求出16张标签的地图 |
| 拟合RMS | 1.476898px（门槛1.5） | 用于求K/D的47个视角；接近本次门槛 |
| 留出RMS | 2.213650px（本次门槛2.3） | 固定K/D，估计13个留出视角外参后重投影；同段视频验证，不是独立米制真值 |
| 角点包络 | 横向0.958967、纵向0.957954 | 角点最小/最大包络约占图宽高95.9%/95.8%；不代表均匀覆盖该面积 |
| 法线最大夹角 | 56.18795° | 视角存在变化，不是相机安装俯仰标定值 |
| 两组焦距重拟合最大变化 | 6.5347%（门槛25%） | 相对全组fx/fy的变化，不是统计置信区间 |
| 其余图像检查 | 正焦距/图内主点、正深度、径向导数抽样检查通过 | 径向抽样不证明完整二维畸变映射可逆，也不替代水下系统误差验收 |

候选参数绑定1280×720原图和该湿光路。K单位px，D按plumb_bob的k1,k2,p1,p2,k3顺序、无量纲：

~~~text
K = [675.434437,   0.000000, 677.439238;
       0.000000, 671.130192, 349.137555;
       0.000000,   0.000000,   1.000000]
D = [0.746329310, -0.205327268, -0.018293589,
     0.066863663, -0.015357379]
~~~

fx/fy是像素焦距，不是镜头标称2.8mm；cx/cy是主点。D不能因为商品称“无畸变”而丢弃。原图+K/D交给PnP，或图像校正后使用匹配的K_rect和零畸变，不能对已校正图再次应用原D。更换裁切/输出模式/调焦/光路时重新核对；XML是OpenCV格式，尚不是可直接装入ROS驱动的CameraInfo YAML。

CSV独立重算留出RMS与报告一致。留出组中位数1.596191px，帧2634/87.8s为5.686647px，占该组平方误差总和50.7635%；其余显著样本含33.0s/2.852343px、120.8s/2.467809px。拟合组68.2s/5.084252px和39.8s/4.430897px也值得回看。残差本身不能确定是反光、运动成像、角点误差、平面姿态解或模型偏差；不据此自动删帧、换模型或要求重录，保留原始记录。

10-21报告分析后确定先做固定标签局部系的只读位姿、重投影叠加与独立50/100mm位移/高度验收，再建立地图及T_BC。11-02用户确认并实施最小工具，见下节；相机在实际光路中的精度仍待验证。单个完整标签就能起步，内参可用于同一相机模式下其他ID，不需要每张重标相机；全缸坐标还需各ID在共同地图中的真实几何。标签平面歧义、边缘/高度相关偏差和跨ID一致性仍需验证。

### 13.11 单标签只读定位：录像与USB相机实验（2026-11-02）

**已完成什么。** 用户确认实施前一轮的最小定位范围，rov_localization更新至0.2.0。新增tag_pose_preview普通C++入口，直接复用已有AprilTagDetector与OpenCV，不要求ROS、全缸地图或T_BC。读取已得到的湿光路XML，从指定ID四角求相机在标签局部系中的位姿；支持原录像和桌面Ubuntu的V4L2 USB相机，显示原图/角点序号/坐标轴/候选位置姿态，记录每帧CSV。此处“实时”仅指读相机，不是ROS图像链或闭环控制。

**改动与函数链。** 新增tag_pose_estimator.h/.cpp、tag_pose_preview_main.cpp、config/tag_pose_options.yaml、test/test_tag_pose.cpp，更新CMakeLists/package.xml；旧标定求解与默认门槛保持原实现。详细接口/状态见总体设计§13.11，调用顺序为runPreview→loadCameraModel/loadTagPoseOptions→构造TagPoseEstimator/AprilTagDetector→VideoCapture::read→detect→estimate→writePoseRow/showPose。每帧独立，未滤波或补旧位姿。

estimate保持K/D固定，调用solvePnPGeneric(IPPE)取得两个平面解，分别solvePnPRefineLM，仅保留正深度且误差未增加的细化结果；按原图重投影误差选候选，两个明显不同姿态误差太接近时标ambiguous_pose。该检查是实验启发式，不保证排除所有平面翻转或水下系统偏差，需看真实静止/已知位移记录。

**坐标和单位。** L以固定ID标签中心为原点，X从p0指向p1，Y从p3指向p0，Z=X×Y；轴跟随标签编码，可看窗口的p0…p3和红X/绿Y/蓝Z。L不是水缸map、ENU/NED、机体或上电home。C为光学右/下/前。PnP输出T_CL，程序取R_LC=R_CL转置、p_LC=-R_LC*t_CL；界面位置和CSV候选x/y/z为相机光心在L中的坐标m。相机可能位于负Z侧，height_m=abs(z_L)是光心到标签平面的垂距，不等于斜视时的t_CL.z。RPY从R_LC按Rz(yaw)*Ry(pitch)*Rx(roll)展开，单位度，只是相机光学系姿态显示，不能当成ROV机体航向；完整R_LC也保存在CSV。

#### 13.11.1 在lab的Ubuntu电脑构建

交付包rov_localization_20261102.tar.gz只含本包源码/配置/正式测试，不含录像、build或伪造内参。复制到Ubuntu的~/Downloads。已有源码位于主仓库时可直接cd到ROV_Control/src/rov_localization，执行下方cmake部分。使用交付包时创建新的父目录，不覆盖旧r2源码：

~~~bash
mkdir -p ~/vision_tools/tag_pose_01
tar --keep-old-files -xzf ~/Downloads/rov_localization_20261102.tar.gz \
  -C ~/vision_tools/tag_pose_01
cd ~/vision_tools/tag_pose_01/rov_localization

cmake -S . -B build -DROV_LOCALIZATION_WITH_AMENT=OFF \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
ctest --test-dir build --output-on-failure
~~~

mkdir创建目录；tar解压源码并拒绝覆盖已存在文件；cd进入包。沿用已安装的g++/cmake/pkg-config/libopencv-dev/libapriltag-dev/libgtest-dev，无需重装ROS。WITH_AMENT=OFF构建普通C++程序，BUILD_TESTING=ON启用正式测试，-j2限制编译并行。ctest应显示两个汇总入口通过，合计18个GTest；无需source工作区，也无需launch。

#### 13.11.2 先回放旧录像

下面假定内参保存在上次results_03，若实际目录不同，只替换--camera路径；输出目录每次换新名称，父目录~/vision_data已有：

~~~bash
./build/tag_pose_preview \
  --video="$HOME/vision_data/calibration_wet_01/wet_01.mkv" \
  --camera="$HOME/vision_data/calibration_wet_01/results_03/camera_wet.xml" \
  --config="$PWD/config/tag_pose_options.yaml" \
  --output="$HOME/vision_data/tag_pose_video_01"
~~~

--video指定原始录像；--camera读取已标定K/D；--config默认ID16/80mm及实验选值；--output是新结果目录。默认打开窗口，q/Esc结束；Ctrl+C也会请求主循环退出并保存数据。没有桌面时加--display=false；只想先试一段时加--max-frames=300。无需重新录标定视频，也不必让ID16每帧可见，消失的帧会明确记录。

#### 13.11.3 USB实时只读实验

先退出占用相机的FFmpeg/opencv_demo，确认当前设备；不要把历史/dev/video3当永久编号：

~~~bash
v4l2-ctl --list-devices
~~~

若本次采集节点仍为/dev/video3：

~~~bash
./build/tag_pose_preview \
  --device=/dev/video3 \
  --camera="$HOME/vision_data/calibration_wet_01/results_03/camera_wet.xml" \
  --config="$PWD/config/tag_pose_options.yaml" \
  --output="$HOME/vision_data/tag_pose_static_01"
~~~

--device替换--video，二者只选一个。程序请求MJPG和XML中的1280×720、配置中的30fps；首帧实际尺寸不符即报错，不用错误内参继续算。保持调焦、图像模式与湿光路不变；设备接受请求不证明真实曝光率或时延，实测另看记录。

先固定相机看完整ID16、静止约20秒，检查位置散布/姿态翻转/丢检。之后换新--output如tag_pose_move_x_01，保持高度和朝向，用直尺/简单导向沿标签X方向移动50mm、停稳，再移动50mm、停稳，记下各停稳时段；必要时沿Y重复。比较同一标签系下各停稳段位置均值之差：50mm=0.050m、100mm=0.100m，而不是逐帧运动轨迹总长度。另选不同高度和画面位置核系统偏差。此阶段只测一张标签，不先量全16张；能否满足实验精度由实际记录和任务要求判断。

#### 13.11.4 输出、选值与已验证边界

每次结果目录有pose.csv、run_info.yaml、camera_used.xml和options_source.yaml。后两者是原XML/配置的只读副本；若--tag-id覆盖配置，实际ID以run_info和CSV为准。run_info记实际输入/版本、相机候选状态、尺寸/目标几何、帧数/有效数/歧义数、选值与退出原因。程序退出码0只表示该次有有效几何帧，2表示没有有效帧，1表示参数/输入/运行错误，不表示米制精度验收通过。

CSV重点看time_s、valid、reason、candidate_x_L_m/y/z、height_m、rms_px；其余包含相机RPY、完整R_LC、原角点和处理耗时。仅valid=1的行用于初步定位统计；ambiguous_pose/reprojection_error保留候选值供排查，无候选/丢检写nan；不得把失败行或上次值当观测。target_not_found时终端会列seen_ids，便于判断是否看到了其他标签。固定target_id避免未知地图下切换ID导致坐标原点跳变；手动--tag-id=其他ID可单独试验，但会更换局部参考系。

所有试验选值在config/tag_pose_options.yaml：max_reprojection_error_px默认3.0为单帧门槛，不是之前标定留出总RMS的2.3；ambiguity_error_gap_px=0.15且两解旋转差>=5°时标歧义；min_edge_px=35、边距4、max_hamming=0。改YAML后重启程序、新建结果目录；不在运行中热加载，不靠放宽门槛替代真实误差检查。

录像time_s=frame_index/文件FPS；相机time_s为read返回后steady_clock相对首帧的时间，不是硬件曝光时刻或ROS时间。processing_ms含灰度转换/检测/PnP，不含采集等待、显示和落盘，不等于端到端时延。录像read=false仍无法区分正常EOF与解码错误。

验证：WSL Ubuntu24.04/GCC13.3/OpenCV4.6.0/AprilTag3.3.0，9个新增定位GTest和9个旧标定GTest全部通过，CTest两个汇总入口；含非零旋转的逆变换/RPY还原、100mm位移、报告K/D强畸变/正视合成投影、歧义/坏帧/丢检、真实36h11图案合成MJPEG→CLI→CSV、尺寸拒绝和不覆盖。显示/画轴/waitKey用Qt offscreen通过，不能称实际桌面/USB验收。Jazzy colcon构建与ros2 run rov_localization tag_pose_preview --help通过，仅证明安装入口，不是ROS节点。临时录像/结果由正式夹具清理；未读取用户原视频、未接实际相机、Pi或推进器，真实精度待上述实验。

## 14. 当前 USTROV 手柄开环与 ROV_Control 接入边界（2026-10-27）

**证据范围。** 用户报告平台已搭建，Linux 操作站 lab 经网线连接 Pi，运行以下 USTROV 入口实施手柄开环。本次读取本地子模块 HEAD `f22f647`；相对历史验收的 `b1208e5`，主要新增有线 DDS 部署、只读 OLED 和手柄四轴增益 51.42，core/bridge/mixer 主逻辑未变。本次没有远程读取两台设备的实际安装版本、重跑控制或修改参数；用户报告不能替代逐项标定记录。旧 §11.13–11.18 是 10-12 的历史检查，不再作为“平台至今未能开环”的判断。

### 14.1 两台电脑上的进程与命令

Pi 的 SSH 终端加载 `/opt/ros/jazzy/setup.bash`、`~/USTROV/ustrov_underlay/install/setup.bash`、`~/USTROV/ustrov_ws/install/setup.bash` 后，运行 `ros2 launch ustrov_bringup core.launch.py`。这启动外部 MicroXRCEAgent 进程（默认 `/dev/fcu_data`、921600）、`px4_actuator_bridge`、`px4_sensor_bridge` 和 `actuator_command_mixer`。串口别名在设备上的实际指向本次未读，不能直接认定等于旧 `/dev/ttyAMA5`。core 默认 vehicle_name 为空、PX4 前缀 `/fmu`，不启动手柄、EKF、视觉定位或轨迹控制器。

lab 本地终端加载 `/opt/ros/jazzy/setup.bash`、`~/USTROV/install/setup.bash` 后，运行 `ros2 launch ustrov_remote_control joystick_control.launch.py`。一个 `joystick_container` 进程加载 `joy::Joy`（节点 `/joystick`）和 `JoyStick`（节点 `/joystick_mapper`），对应 ROS1 的 nodelet 组合思路。source 只改变当前终端环境，不启动节点；SSH 后执行的位置是 Pi。

网络模板见外部 [deployment/README.md](../USTROV/deployment/README.md)：lab `192.168.50.1/24`、Pi `192.168.50.2/24`、domain 0，Fast DDS 只使用指定有线/回环地址。已部署时两端各自加载 `~/ustrov-network/dds-wired-env.sh`（可由各自 bashrc 加载）。SSH 能连通不等于 DDS 已发现；ROS2 的跨电脑消息不通过 SSH，也不以 MicroXRCEAgent 作为 ROS1 master。

### 14.2 消息、函数和时序

~~~text
lab: USB手柄 → joy::Joy → /joy
     → JoyStick::OnJoy → ComputeThrust/ComputeTorque
     → /thrust_setpoint + /torque_setpoint
              ↓ 以太网 DDS
Pi: ActuatorMixerNode::OnThrustSetpoint/OnTorqueSetpoint
     → SimpleMixer::Mix → /thruster_command
     → Px4ActuatorBridge::OnThrusterCommand（缓存）
     → OnOutputTimer（50 Hz，状态机允许后发出）
     → /fmu/in/actuator_motors → Agent → 串口 → PX4 → PWM → 推进器
~~~

`mapping.use_4dof=true` 的含义是 `[Fx,Mx,My,Mz]`，不是 XYZ+航向。按当前轴编号，axes[1]→Fx、axes[3]→Mx、axes[4]→My、axes[0]→Mz；各乘 51.42。Fy/Fz 为零。`JoyStick::OnParams` 支持这个参数运行时更新；参数不因 `ros2 param set` 自动保存回 YAML。

死区另需核实：launch 固定 `deadzone=0.5`。检查到的 [官方 joy ros2 源码](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/src/joy.cpp) 在构造函数缓存死区，没有运行时更新回调；因此 `ros2 param set /joystick deadzone 0.05` 返回成功或 get 显示 0.05，不足以证明实际轴处理改变。lab 安装版本本次未读，应检查该版本或将期望值放入启动配置后验证；本次没有改启动代码。

`ActuatorSetpoint` 为 header、xyz、ignore_xyz；当前属于暂用模型下的逻辑控制量，不能把 51.42 尤其三个转动通道直接解释为实测 N/N·m。mixer 的输入顺序是 `[Mx,My,Mz,Fx,Fy,Fz]`，8×6 矩阵只使用前四行，后四路为零。`SimpleMixer::Mix` 做矩阵乘法、Hippo 二次推力曲线逆解、除以 50 rps，再在超过幅值 1 时对全部通道同比缩放。`input_limits` 虽读取存储，但没有参与 `ApplyInput` 运算，不能视为生效限幅。

例：在最终 `/joy` 消息 axes[1]=0.2、其他相关轴为零时，Fx=10.284，前四路矩阵结果为 [-2.571,+2.571,-2.571,+2.571]，逆曲线得到约 19.714 rps 的模型中间量，归一化输出约 [-0.3943,+0.3943,-0.3943,+0.3943]。这不是实际转速/推力测量，也不是 PWM 微秒值。

joy 启动配置自动重复 20 Hz，也可被手柄事件触发。mapper 每次 Joy 回调计算；mixer 收到任一 setpoint 就计算并发布，所以两个 setpoint 不是原子更新，也不是固定 50 Hz 计算。桥另以 50 Hz 定时发送最新缓存。mixer 每 300 ms 检查各输入是否超过 300 ms 未更新，超时归零并在变化时发布；桥独立检查 300 ms 命令新鲜度。多级检查、定时调度和飞控状态响应决定实际停止时延，不能承诺从网络断开起恰好 300 ms 全机停止。

解锁命令的正确格式是 `ros2 service call /px4_actuator_bridge/arm std_srvs/srv/SetBool "{data: true}"`。这是启动序列的请求：IDLE→PRESTREAM（默认 1200 ms 零输出和心跳）→WAITING_FOR_OFFBOARD→WAITING_FOR_ARMED→ACTIVE。服务成功仅表示接受请求，实际状态由 PX4 回报确认；ACTIVE 后输出当前缓存。data=false 或 `~/stop` 请求停止/上锁。`direct_actuator=true` 表示直接电机层接口，不提供可默认依赖的 PX4 姿态/角速度内环。

### 14.3 反馈和自研接口

`Px4SensorBridge::OnSensorCombined` 将 PX4 FRD 的角速度(rad/s)/加速度(m/s²)翻为 ROS FLU，发布 `/uuv00/imu`；orientation_covariance[0]=-1，表示没有姿态测量。`OnSensorBaro` 将压力(Pa)/温度(°C)转为 `/uuv00/pressure`、`/uuv00/barometer_temperature`，不计算深度。时间戳首次按接收时刻建立固定偏移，之后保留源时间差，不是高精度相机/IMU同步验收。

`ustrov_state_estimation` 确有 IMU+压力+视觉 EKF 实现，`OnImu` 驱动预测/输出，`OnBaro` 启动平均标零后换算深度，`OnVision`接收位姿；这些节点没有被当前 core 启动。不能把传感器桥等同定位，也不能假定当前已有 `/uuv00/odometry`。sensor 配置中的输出话题是绝对路径，修改 vehicle_name 不会自动改这些名称。

ROV_Control 可选择在 setpoint 层接 USTROV mixer，或自己完成分配后在 `thruster_command` 层接执行桥，两者须明确择一。现有 `PointTrackingNode` 已用 `HippoAllocator` 并直接发布电机消息，`checkGraph` 会拒绝多发布者；其配套 `hippo_point_tracking.launch` 本来就排除了默认 mixer。未来真机入口可复用 `ustrov_core/launch/px4_hardware.launch.py` 加独立 Agent，但必须先完成真实状态适配、模型/控制量定义、标定与模式接管设计，不能仅改 namespace 后执行旧仿真入口。当前 core 不提供控制源仲裁器。

本次只读审查和同步已有文档；没有新增实现、测试文件、编译或远端硬件动作，也没有提交/改写子模块指针。后续函数设计与用户确认规则继续有效。

### 14.4 从混控数组到 PWM 的阅读补充（2026-10-28）

USTROV 本地仍为 f22f647。本次继续读取手柄、混控、执行桥及包构建声明；未运行电机。PX4 设备所报提交的源码在线获取失败，以下输出层依据官方 v1.16.0 的 [FunctionMotors.hpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/mixer_module/functions/FunctionMotors.hpp) 和 [mixer_module.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/mixer_module/mixer_module.cpp)，不视为设备固件及参数已核验。

三层映射必须分别理解：mixer_matrix 把机体逻辑控制量变成各槽模型推力；bridge 的 motor_input_indices/motor_signs 把槽位映射到 PX4 逻辑 Motor；PX4 输出 Function 把逻辑 Motor 映射到 AUX/MAIN 物理通道，再由 PWM 范围和反向配置产生脉宽。实际线束才决定物理通道对应哪个推进器。direct_actuator 绕过运动控制/分配，不绕过输出范围、解锁门控和输出层非线性处理。

官方 v1.16.0 的 FunctionMotors::updateValues 在 THR_MDL_FAC 有效且非零时还会做非线性变换；因此 USTROV 逆曲线得到的归一化模型转速，与 PX4 输出层采用的控制量语义必须核对，不能默认两处补偿可以直接叠加。reversible_flags 表达带符号电机输入，不等于 PWM Rev Range 或 ESC 已完成双向配置。

条件例子：已解锁并结束启动渐变、PWM 输出、可逆输入、THR_MDL_FAC=0、Min=1100/Max=1900 µs、无输出反向时，output_limit_calc_single 的线性映射为 PWM=1500+400u。§14.2 的 u≈±0.394287 对应约 1342/1658 µs；启用该通道输出反向时二者交换。若沿用10-12截图AUX1..4→Motor1..4，则逻辑电机按此进入AUX1..4；不能把旧截图当当前配置。1500来自本例范围中点；Disarmed参数是另一种状态下的输出，不能凭其数值证明电调实际停转点。50 Hz 是桥更新命令频率，400 Hz 若仍配置则是PWM波形重复频率，二者不同。

若未来仅替代手柄生成“向前开环”输入，可按明确的无量纲等级s输出Fx=51.42s、其余通道为零，持续发布两条setpoint并更新stamp/ignore字段；这是接口设计例子，尚未新增入口。只允许一个命令来源。向前0.2m/s需要速度反馈控制器，向前0.2m需要位置参考/轨迹与状态反馈，不能把这两种目标直接填入thrust_setpoint.x。

包与运行时的对应示例：ustrov_core目录通过package.xml声明包身份；CMakeLists把main.cpp和px4_actuator_bridge.cpp编译成一个px4_actuator_bridge可执行文件。启动该文件产生进程，main创建Px4ActuatorBridge节点并spin。一个包可有多个可执行文件、库或仅消息/配置；一个节点实现可分散在多个文件。手柄是另一种部署：joy与ustrov_remote_control两个包的组件，装进同一个component_container进程，成为joystick与joystick_mapper两个节点，对应ROS1 nodelet的部署思路。

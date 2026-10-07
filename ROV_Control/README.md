# ROV_Control

自研 ROV 规划与控制学习工程。算法、节点、测试使用 C++；自有启动文件统一为 XML 内容、`.launch` 后缀。

## 1. 系统目标与当前进度（2026-10-07）

| 部分 | 当前状态 |
|---|---|
| A：边界、直线路径、进度投影、五次时间轨迹、未来参考 | 已实现，21 个 C++ 单元测试通过 |
| B：状态适配、轨迹预览、XML 启动 | 已实现；新增 14 个单元测试通过，已接 Hippo 实际运行 |
| C1：有限输入的执行通路、死区和方向核验 | 已实现，完成 Hippo 有限脉冲和停止/暂停核验，结果见 §9.6 |
| 水平到点：规划、轨迹跟踪、姿态与物理分配 | 已实现并完成原 Hippo Gazebo 闭环；新入口与实测结果见 §10 |
| 逐电机开环工具 MotorProbeNode | 已实现；软件接线验证与使用说明见 §11，尚未连接本项目真机 |
| MPC、真机闭环、真实视觉 | 后续阶段 |

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
| USTROV 固定版本 | b1208e50becb9f74d4d0dcd8637ac11e73d3672a |

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

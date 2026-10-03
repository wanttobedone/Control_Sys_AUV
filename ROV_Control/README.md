# ROV_Control

自研 ROV 规划与控制学习工程。算法、节点、测试使用 C++；自有启动文件统一为 XML 内容、`.launch` 后缀。

## 1. 系统目标与当前进度（2026-10-02）

| 部分 | 当前状态 |
|---|---|
| A：边界、直线路径、进度投影、五次时间轨迹、未来参考 | 已实现，21 个 C++ 单元测试通过 |
| B：状态适配、轨迹预览、XML 启动 | 已实现；新增 14 个单元测试通过，已接 Hippo 实际运行 |
| C/D：纵向 PD + 姿态保持软件闭环 | 下一批，执行接口和停止路径须先讲解确认 |
| MPC、Pi/PX4、真实视觉 | 后续阶段 |

**本批完成从 Gazebo 状态到参考显示的通路。机器人尚未跟踪参考，预览节点不输出控制命令。** 当前仿真使用 USTROV 的真值反馈，不经过 PX4，也不代表视觉/惯导融合已验收。

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
    ├── src/rov_control_ros/      C++ 状态适配、只读轨迹预览
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

## 3. 运行流程与数据结构

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

colcon test --base-paths src \
  --build-base build/batch_b --install-base install-b \
  --event-handlers console_direct+
colcon test-result --test-result-base build/batch_b --verbose
source install-b/local_setup.bash
~~~

本次是 35 个 GTest：A 的 21 + 状态适配 7 + 预览数学 7。colcon 还统计 3 个 CTest 入口，因此汇总显示 **38 tests，0 errors，0 failures**。

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

## 6. 启动与使用

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

## 7. 本次验证结果与边界

- 三个包在 WSL Jazzy 编译通过；35 个 C++ 单元测试全部通过。三个 XML 入口被实际解析并运行。
- uuv00 下实际启动 Gazebo GUI、RViz、状态桥和唯一 mixer；RViz OpenGL 初始化成功，订阅话题和 Path 的 transient-local QoS 已核对。未把 GUI 进程启动等同于人工逐项检查画面。
- 实际参考起点约 (1.3,1.0,-0.107835) m，终点约 (1.3,1.2,-0.107835) m；101 个显示点，T=4.6875 s。实际 z 与 spawn 的 -0.5 不同，是模型被动上浮后的反馈。
- 约 0.6 s 暂停：参考时间/位置冻结；超过 2 s 无有效消息：FAULT；恢复仿真后仍锁存故障，reset_fault 后才能再次 start。
- 改名 uuv_b 后同样通过参考、暂停、故障/复位、停止清空 Path 验证；注入错误 frame 触发 FAULT。实际 ROS 图确认预览节点仅发布显示/诊断及 ROS 基础话题，无执行器发布端点；RViz 的三条业务订阅已随命名空间改变。
- rosbag 已保存并读取元信息：log/preview_b_renamed，4831 条消息，其中 Odometry 311、Path 8、参考 Pose 411、诊断 17、clock 4084。日志为预览验收数据，其中含专门注入的错误消息，不能拿来训练或评价控制性能。临时 C++ 验收客户端位于被忽略的 ../tmp/live_probe，结果在 log/batch_b_renamed_probe.txt。
- 实际 Gazebo 时间重置触发了时间倒退 FAULT；但同次上游仿真报告部分 link/joint 不存在、Hippo 状态流未恢复。**这条“重置世界后只复位节点便继续”的路径没有通过；当前应退出整套 launch 再启动。** 普通暂停/恢复与 reset_fault 路径可正常使用。
- 改名实例退出时，上游 bridge 曾以 -11 退出，vehicle_spawner 删除模型超时后退出；本次自研预览节点正常退出，测试进程均已结束。这是上游关闭路径的待查问题，本批未修改 USTROV 或宣称其退出流程完全通过。
- 本批不证明运动学参考满足推力能力、闭环稳定性、视觉定位或真机安全。世界活动盒只约束参考中心点；pool.xacro 没有池壁碰撞体。
- 本次验收使用独立 ROS_DOMAIN_ID=84、GZ_PARTITION=rov_preview_batch_b；日常运行无需设置它们。如自行设置，所有终端必须一致。

下一批先讲清 **沿线 PD 计算、上游姿态计算库、归一化命令单位、限幅/启动/停止/超时、真实控制日志**，再进入 C/D 的闭环实现。

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

# 路径规划与轨迹跟踪：从直线基线到 MPC

设计日期：2026-09-28；状态更新：2026-10-02。**A/B 批经用户确认后已实现并验证；C–G 批仍为待确认设计。**已完成部分见 [使用说明](../README.md)。本文件 C–G 的类、节点和话题仍是规划，不能据此认为已存在。算法、节点、测试均为 C++17；XML/YAML/CMake 为声明文件。

命名与排版于 2026-09-29 按用户 Glider_sim 工程同步：自研头文件用 `.h`、函数用小驼峰、源码写中文解释。下文未实现接口也使用这一约定，上游既有接口保留原名。

## 1. 当前目标与范围

用户决定：视觉闭环保留到后续，优先直接在真实相机、最终防水窗和水环境中验证，不要求先实现视觉仿真。室内浅池、下视 AprilTag、相机＋镜头＋防水预算 2500 元以内的需求保留。视觉接入时再做采集、标定、定位、融合、失效处理；控制器通过统一状态接口接收结果。

当前先在 Gazebo 真值反馈下建立可解释的规划与跟踪链。它验证算法、接口、时序和动力学响应，不代替真机标定，也不是 Pixhawk HIL。现已完成模型/状态通路与自研参考预览，尚无本工程轨迹跟踪结果。

### 三个概念

| 层 | 问题 | 输入 → 输出 | 初版 |
|---|---|---|---|
| 路径规划 | 从哪里走到哪里，几何上经过哪里？ | 起点、终点、可用空间 → 曲线 p(s) | 已知空旷区域中的单条直线，拒绝越界；没有障碍搜索 |
| 轨迹生成 | 什么时候走到哪里，速度/加速度是多少？ | 路径、运动限制 → p(t)、v(t)、a(t) | 五次时间律，静止出发、静止结束 |
| 轨迹跟踪 | 现在有误差，应怎样驱动？ | 实际状态、带时间参考 → 控制命令 | 先纵向 PD + 姿态保持，再以 MPC 替换纵向 PD |

几何路径跟随可以只要求靠近曲线；本任务先做有明确时序的轨迹跟踪。A*、RRT、障碍地图、在线重规划不属于第一批，待确有障碍任务后再引入。

## 2. 源码证据与复用边界

核查版本：USTROV b1208e50becb9f74d4d0dcd8637ac11e73d3672a；本轮为 Windows 源码只读检查，未执行控制试验。

| 现有部分 | 本阶段处理 | 依据及限制 |
|---|---|---|
| ustrov_sim、hippo3、动力学插件、static_bridge | 复用 | 提供仿真被控体和执行通路；参数属于 Hippo，不能当作真机标定 |
| fake_state_estimator → odometry | 复用作仿真反馈 | 当前转发真值，不是 INS/VINS/EKF 验证 |
| ustrov_control::attitude_control::GeometricController | 在 ROS 适配包中封装、调用其计算库 | CMake 导出了 attitude_control 库；不启动另一个姿态控制节点。Update 本身未做限幅，封装必须检查输出 |
| 已运行 actuator_command_mixer | 首轮受限仿真复用，保持唯一实例 | 当前 spawn_hippocampus 已创建；不叠加上游整套 attitude launch |
| TrackingControllerNode | 阅读参考，首版不直接连接 | 位置取世界系，速度直接取 twist；当前插件 twist 是机体系。输出 attitude_target 与默认 geometric 节点的 heading_target 接口也不同 |
| src/nodes/attitude_control_node.cpp | 不据文件存在就接入 | 有 attitude_target 订阅，但当前 CMake 未把这个源文件列入构建；不默认可启动 |
| path_planning::PathFollowerNode | 不接入 | 上游示例 launch 引用了它，当前仓库未包含该包 |
| 视觉、真正估计器、PX4 桥 | 后续阶段 | 本轮不改桥、安全状态机或估计器；旧独立俯仰测试不读、不复用 |

需要特别理解：

1. 当前 Hippo 的直接输入为 body-x 推力和三个力矩；没有独立 body-y/body-z 推力。真实四推进器布局仍未知，两者不能等同。
2. 初版直线沿初始水平朝向，主动控制沿线位置/速度及姿态；**侧向和深度只监测，不构成完整三维轨迹跟踪或定点保持**。模型有正浮力，深度可能漂移。漂移使有限时段试验无法通过时，先分析浮力/深度控制方案，不暗改模型让结果好看。
3. 首轮终点要求沿线速度降至零，但不能承诺三维悬停。试验结束即退出到不输出控制的状态。
4. 当前水池 xacro 只有可视网格，未定义池壁碰撞体。可视水池不能承担碰撞约束；需显式配置经核验的可用空间。
5. normalized mixer 的 thrust/torque 是命令域量；初版称为 effort，无单位，不能标 N/N·m。实际推进器输出仍有非线性、滞后和耦合饱和。

## 3. 第一版系统结构与数据流

```text
YAML：目标相对位移、空间边界、速度/加速度限制、PD、命令限幅
                            ↓
/uuv00/odometry → StateAdapter → RobotState
                                  ↓ 首次显式 start 且状态有效
                         LinePathPlanner::plan
                                  ↓ LinePath
                      QuinticTrajectory::create
                                  ↓ 一条不可变的轨迹
每个有效控制周期：
ROS 时间 → 轨迹时间 → sampleHorizon → 带时间参考
RobotState → projectProgress → 沿线位置/速度
              ↓                         ↓
        IProgressController::compute（先 PD，后 MPC）
                                  ↓ surge_effort_requested
RobotState → HippoActuationAdapter（上游姿态库 + 显式命令限幅）
                                  ↓
           thrust_setpoint + torque_setpoint → 唯一 mixer
                                  ↓ thruster_command
                         static_bridge → Gazebo
                                  ↓ odometry

RunGuard 对整次输出统一放行/终止；每周期记录状态、参考、请求/发送命令和原因。
Path / Pose 等 ROS 消息用于显示；控制核心直接使用同一份 C++ 轨迹对象。
```

C/D 阶段计划用一个 C++ tracking_node 承载算法对象，单线程 executor 顺序处理回调。类之间通过函数调用交换一致快照，先不把每个类拆成 ROS 节点。这样参考位置、速度和加速度不会来自互不同步的三个订阅缓存。

### 坐标与时钟

- W = map，米、秒、弧度；仿真 z 轴向上，水下 z 通常为负，深度为正的传感器值须在其适配处转换。
- B = uuv00/base_link，ROS 机体系；q_WB 表示从 B 到 W 的旋转。
- 当前 Odometry：位置/姿态在 map；线速度和角速度在 child_frame_id。StateAdapter 计算 **v_W = R(q_WB) v_B**，保留 omega_B。
- 第一版只接受明确配置的这两个 frame，不把未知 frame_id 直接改字符串当作完成转换。未来不同 frame 经 TF/外参适配后输入核心。
- 轨迹时间用 ROS 时钟；仿真 use_sim_time=true。接收间隔和卡死检测用 steady clock。仿真暂停不继续推进轨迹；长时间无数据或时间倒退锁存停止，恢复后需要重新 start，不自动继续旧轨迹。
- 50 Hz 仅为与当前模型配置对齐的起步候选；正式周期根据实测消息率、抖动和处理时间确定。状态新鲜度阈值为显式配置。

依据：已读 ROS 官方 Jazzy [Odometry.msg 全文](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg)和 [Path.msg 全文](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Path.msg)。Path 包含 PoseStamped 数组，没有显式速度/加速度字段，故不作为完整控制参考的内部结构。

## 4. 核心数据结构（A 批相关部分已实现）

下表为总体接口。A 批已定义 RobotState、WorkspaceBounds、MotionLimits、PlanRequest、ReferenceSample、ProgressState 及错误返回；LinePath 为 line_path.h 中的只读类。控制命令、运行状态和日志类型留到后续批次，未创建空实现。Eigen 向量/四元数只用于数学核心，不包含 rclcpp、Gazebo、USTROV 消息。

| 类型 | 字段、单位与含义 |
|---|---|
| RobotState | stamp_ns；p_W[m]、q_WB、v_W[m/s]、omega_B[rad/s]。时间戳的时钟域由适配层保证，核心不自行读系统时钟 |
| WorkspaceBounds | 允许机体中心进入的 min_W/max_W；这些界限已减去机体半径、挂载及余量，不能直接填池子外形尺寸 |
| MotionLimits | v_max[m/s]、a_max[m/s²]、可选 j_max[m/s³]、min_duration[s]；均为正数且有限 |
| PlanRequest | p_start_W、p_goal_W、起始朝向/速度、WorkspaceBounds；首版执行模式还要求目标在初始水平前向直线上，且起速足够小 |
| LinePath | p0_W、单位方向 d_W、长度 L[m]；s∈[0,L] |
| ReferenceSample | t_from_start[s]、s/v_s/a_s，p_W/v_W/a_W，q_WB_des、omega_B_des；首版姿态常值，期望角速度为零 |
| ProgressState | s=d·(p−p0)、v_s=d·v_W；横向误差、深度偏差作为诊断。实际 s 不裁剪到 [0,L]，否则会隐藏超调 |
| ControlContext | dt[s]、previous_sent_surge_effort；后者为上一周期实际发送的请求，不能声称为真实测得推力 |
| ProgressCommand | requested_surge_effort、valid、status；明确为 normalized 命令域 |
| HippoEffortCommand | surge_effort、torque_effort_B；全为命令域量，映射到现有 normalized mixer |
| RunStatus / GuardResult | IDLE、PREVIEW、RUNNING、FINISHED、FAULT；放行布尔值、原因枚举。FAULT 锁存 |
| StepRecord | run_id、序号、状态/控制时间、状态年龄、参考/实际状态、误差、原始/限幅命令、饱和标志、周期耗时、模式/故障原因 |

所有构造/计算都返回显式成功或错误；NaN、零范数四元数、非法限幅、零长度路径等不能默默继续。第一版零长度目标返回“无需运动”，不进入姿态/推力计算。

## 5. 算法：先把最小闭环算明白

### 5.1 路径与时间分开

直线：p(s)=p0+d·s，d=(p1−p0)/L。已有空旷凸盒区域内，检查缩小后的边界包含两个端点，即能保证线段在盒内；这不等于一般环境碰撞检测。

令 τ=t/T，0≤τ≤1：

- h(τ)=10τ³−15τ⁴+6τ⁵
- s(t)=L·h(τ)
- v_s(t)=L/T·h′(τ)，a_s(t)=L/T²·h″(τ)
- p_ref=p0+d·s，v_ref=d·v_s，a_ref=d·a_s

由解析峰值选时间：T≥max(T_min, 1.875L/v_max, sqrt((10√3/3)L/a_max))。若启用 jerk 限制，再要求 T≥cbrt(60L/j_max)。这是直线路径的运动学约束，不证明驱动能力可实现。

t<0 采样起点，t>T 采样终点，速度/加速度为零。五次曲线与静止段连接为位置/速度/加速度连续，jerk 不保证连续；不把它称为最优轨迹。首版拒绝运行中突然改目标，需要停止后重建，避免参考跳变。

### 5.2 纵向 PD 基线

e_s=s_ref−s，e_v=v_ref−v_s。

effort_requested=k_p·e_s+k_d·e_v。

- k_p、k_d 的单位对应“命令域量/米”和“命令域量/(米/秒)”；不是物理力模型参数。
- 速度项直接用状态速度，不对位置误差做噪声敏感的差分。
- 先不用积分或未经验证的加速度前馈；ReferenceSample 仍保留加速度供后续模型前馈/MPC 使用。
- 封装层按幅值与变化率限制纵向 effort，并单独限制姿态 effort。首版要求朝向接近路径方向，偏离超过容限立即退出，不能靠除以接近零的 cos(angle) 追求不可能的瞬时运动。
- 反向纵向 effort 用于减速前必须在 Gazebo 验证符号及实际响应；推进器布局/曲线未经核验时不写死真实映射。

### 5.3 MPC 的位置

后续保留路径与轨迹生成器，替换 IProgressController 的实现。MPC 一次读取未来 N 个时间点的参考，而 PD 只使用第一个参考。

先验证局部命令域模型，例如 x=[s,v_s] 或加入推进器滞后状态，x(k+1)=A x(k)+B effort(k)。若指令到力的关系在试验区间明显非线性，应重选线性化工作点/映射或升级模型，不能默认从零油门附近得到恒定非零控制增益。质量/附加质量不从 xacro 不加核验地抄入。

依次讲解：离散模型 → 预测展开 → Q/R 与输入变化惩罚 → 幅值/变化率约束 → OSQP → 求解状态/失败行为 → 同轨迹与 PD 对比。此时再冻结 model/prediction/qp_builder/osqp_backend/linear_mpc 的函数签名，详见 MPC_LEARNING_ROADMAP.md §7；不在首批创建空实现。

## 6. 拟议文件架构与每个函数职责

保留三个包。纯计算的规划和控制暂在同一库中，以子目录区分；规模增大后可以拆包，无需一开始建立很多节点/消息包。

```text
ROV_Control/
  docs/                              本设计 + MPC 学习路线
  # USTROV 版本由父级主仓库的 Git 子模块固定，不另建 dependencies.repos
  src/
    rov_control_core/
      CMakeLists.txt, package.xml
      include/rov_control_core/
        result.h                    A 批共用结果与错误返回
        types.h
        planning/workspace_bounds.h
        planning/line_path.h
        planning/quintic_trajectory.h
        tracking/progress_controller.h
        tracking/longitudinal_pd.h
        runtime/run_guard.h
      src/                           对上述有实现的 .h 各配同名 .cpp
      test/
        test_path_trajectory.cpp
        test_tracking_guard.cpp
    rov_control_ros/
      CMakeLists.txt, package.xml
      include/rov_control_ros/
        state_adapter.h
        trajectory_preview_node.h     B：仅做状态接入与参考显示
        hippo_actuation_adapter.h
        run_logger.h
        tracking_node.h
      src/                           对上述 .h 各配同名 .cpp
        preview_main.cpp              B：已实现；跟踪入口后续另行设计
      test/test_state_adapter.cpp
      test/test_preview_math.cpp
    rov_bringup/
      CMakeLists.txt, package.xml
      launch/hippo_sim.launch          B：封装外部仿真入口
      launch/hippo_preview.launch      B：仿真 + 预览组合入口
      launch/trajectory_preview.launch
      launch/hippo_line_tracking.launch
      config/preview.yaml              B：预览参数
      rviz/trajectory_preview.rviz     B：显示配置
      config/line_demo.yaml
      config/hippo_sim.yaml
```

文件与函数表为计划中的完整 V0 接口，不承诺一次全部实现。types.h 放状态与请求等数据结构，result.h 放错误码、Status 与 Result<T>；progress_controller.h 仅接口，无需为它创建空 .cpp。测试均用 C++。包元数据负责声明依赖、安装头文件/库/配置及测试，不含算法。

### 6.1 规划核心（A 批已完成）

实施细化：PlanRequest 的前向启动条件放入可选 ForwardStartConstraint，几何规划可省略，后续首轮执行必须设置。projectProgress 为独立纯函数。类型/函数的实际名称和错误语义以源码及 ../README.md 为准。

| 文件 / 类 | 函数：输入 → 输出 | 内部状态与调用关系 |
|---|---|---|
| planning/workspace_bounds.h/.cpp | validateBounds(bounds)→结果；contains(bounds,p)→bool；containsSegment(bounds,p0,p1)→bool | 纯函数；边界已包含机体余量，由 plan 调用 |
| planning/line_path.h/.cpp / LinePathPlanner | plan(request)→LinePath 或错误；projectProgress(path,state)→ProgressState | 无缓存；plan 检查有限值、非零长度、范围及首版方向/起速条件；投影供控制与日志 |
| planning/quintic_trajectory.h/.cpp / QuinticTrajectory | create(path,limits,attitude)→轨迹或错误；chooseDuration(L,limits)→T；sample(t)→ReferenceSample；sampleHorizon(t,dt,count,out)→结果；duration()→T | 保存不可变路径、T、常值姿态；Horizon 填预分配容器，避免每控制周期分配内存；count/dt 非法返回错误 |
| test/test_path_trajectory.cpp | 测试端点、解析速度/加速度峰值、不同长度/时间尺度、时间越界、越界路线和零长度请求 | 独立用边界条件/解析峰值作期望，不仅验证实现返回了自己算的结果；首批不发执行器命令 |

### 6.2 跟踪与运行状态

| 文件 / 类 | 函数：输入 → 输出 | 内部状态与调用关系 |
|---|---|---|
| tracking/progress_controller.h / IProgressController | compute(state,horizon,context)→ProgressCommand；reset() | 接口只约束纵向控制；不冒充所有未来六轴控制的通用接口；horizon 非空且时间有序 |
| tracking/longitudinal_pd.h/.cpp / LongitudinalPd | 构造参数 kp/kd；compute(...)→PD 请求；reset() | 只保存增益，首版无积分状态；使用 horizon 首元素；有限值检查；限幅交给唯一执行适配处 |
| runtime/run_guard.h/.cpp / RunGuard | start(checks)→成功/拒绝；evaluate(state,reference,timing,errors)→GuardResult；finish()；stop(reason)；resetFault() | 保存 RunStatus、故障原因；输入时钟/年龄由 ROS 层传入。检查缺状态、时间跳变、异常 dt、边界、姿态误差、侧/深度漂移；阈值必须显式配置 |
| test/test_tracking_guard.cpp | 验证错误符号/零误差、异常输入拒绝、FAULT 不自动恢复、FINISHED 不继续输出 | 后续补验证参考越界、超时、手动停止、不同控制器替换；不以单纯接口覆盖率为验收 |

### 6.3 ROS 适配（保持算法核心独立）

| 文件 / 类 | 函数：输入 → 输出 | 内部状态与调用关系 |
|---|---|---|
| state_adapter.h/.cpp / StateAdapter | convert(odom,expected_frames)→RobotState 或错误；reset() | 检查 frame、时间戳递增、有限值、四元数范数；仅容许小数值误差归一化；转换 v_B→v_W。保存上一时间戳用于检测重置；接收时刻在节点保存 |
| hippo_actuation_adapter.h/.cpp / HippoActuationAdapter | configure(gains,limits)→结果；reset(q_target)；compute(state,reference,progress_command,dt)→HippoEffortCommand；toMessages(command,stamp,base_frame)→两条 ActuatorSetpoint；notifySent(command) | 持有上游 GeometricController 和上一周期已发送 effort；设目标姿态/零角速度→Update→检查→限幅。notifySent 只在成功调用 publish 后更新记录，不代表执行器确认。构造时全部初始化 |
| run_logger.h/.cpp / RunLogger | open(run_metadata)→结果；enqueue(StepRecord)→结果；flush()；close() | 有界队列和独立写文件线程；控制回调不做阻塞磁盘写入，禁止队列无限增长。记录落盘故障/丢记录计数；试验记录失效时结束本次实验 |
| tracking_node.h/.cpp / TrackingNode | loadConfig()→经校验参数；onOdometry(msg)→最新状态快照；onStart()→服务响应；onStop()→响应；onResetFault()→响应；onControlTick()；onWatchdogTick()；onThrusterCommand(msg)；publishPreview()；stopOutput(reason) | 持有核心对象、轨迹、t0、最新状态及其 steady 接收时间、run_id、预分配 horizon。首版运行中禁止更改模型/控制参数；两个定时器和订阅在同一 executor 顺序执行 |
| main.cpp | main(argc,argv)→初始化、创建节点、spin、清理返回码 | 不包含控制公式 |
| test/test_state_adapter.cpp | 非单位/无效四元数、错误 frame、乱序时间；yaw=90° 时 body-x 速度应转成 world-y；消息转换/限幅/变化率的额外测试随适配批次加入 | 必须覆盖会改变真实控制方向的错误；不能只测恒等姿态 |

notifySent 的“成功”指 ROS publish 调用未报错，不能证明 mixer 或推进器已执行。onThrusterCommand 记录 mixer 的八路输出及其接收时间，不把不同序号异步消息强行宣称为同周期匹配。

### 6.4 节点控制流程与话题

本节为 C/D 的控制节点设计；B 批使用 §9 的独立只读预览节点，不提前创建执行适配器。

onStart：检查模式/状态/边界/参数 → 从当前有效位置和初始水平朝向构造目标 → plan → create → 固定 t0 → reset 控制器/执行适配器并进入 RUNNING。C/D 的 TrackingNode 承担控制，B 的 TrajectoryPreviewNode 单独承担只读显示。运动启动条件包括小速度、接近水平、参考路径方向可执行；失败只返回原因，不发送运动命令。

onControlTick：读取同一快照 → Guard → 采样当前和未来参考 → projectProgress → compute → 姿态计算/命令限幅 → 发布两条命令 → notifySent → 记录完整 StepRecord。任何一环失败统一 stopOutput。

| ROS 接口（均相对 vehicle namespace） | 类型/语义 |
|---|---|
| odometry（订阅） | nav_msgs/Odometry；首轮真值来源，后续可用经适配估计状态 |
| thrust_setpoint、torque_setpoint（发布） | ustrov_control_msgs/ActuatorSetpoint；body frame、全部轴显式赋值，ignore 全 false；Fy/Fz=0 |
| thruster_command（只读订阅） | 记录 mixer 输出，不作为本节点额外发布入口 |
| ~/planned_path、~/reference_pose（发布） | nav_msgs/Path、geometry_msgs/PoseStamped；显示/诊断，不回流为控制输入 |
| ~/diagnostics（发布） | diagnostic_msgs/DiagnosticArray；状态年龄、运行状态、退出原因、记录丢失等 |
| ~/start、~/stop、~/reset_fault（服务） | std_srvs/Trigger；start 为一次实验启用，不等同 PX4 arm |

preview 模式不创建执行器发布者。sim_control 模式默认 IDLE，显式 start 后才控制；不是启动 launch 就自动运动。Odometry 采用兼容传感数据发布者的 QoS；命令与上游 SensorDataQoS 对齐，控制数据不使用可重放旧命令的 transient-local。

到时限 t=T：采样终点，进入有限 settling 时间，仅继续纵向/姿态控制；沿线位置/速度满足容差且持续规定时间则 FINISHED，否则超时为失败。侧向/深度随时越界先触发 FAULT。完成、故障或 stop 时，两组命令各发一次零值，然后停止周期命令，不持续以零刷新下游看门狗。清除故障后仍回 IDLE。

当前 mixer 两个回调分别立即发布，**两组命令不具备原子性**，同一时间戳也不能改变这一点。首轮只用于受限仿真，需记录最终输出；多轴/真机前要另行设计同一周期控制分配与统一执行器消息。原有 mixer/PX4 看门狗保留，本节点的 guard 不能被宣称为经过验证的真机安全保护。

### 6.5 配置与启动文件

| 文件 | 内容与边界 |
|---|---|
| config/line_demo.yaml | 相对目标位移、v/a/可选 jerk、空间边界、方向/起速条件、误差容限、settling 超时；空间和容限在运行前结合模型核验，不把真机池尺寸直接套到 Gazebo |
| config/hippo_sim.yaml | frame、namespace、模式、period、状态年龄阈值、PD/姿态增益、normalized effort/变化率限制；不存相机内参或实机推进器参数 |
| launch/hippo_sim.launch | B：XML 入口，include 外部 USTROV top_hippocampus_complete.launch.py，传 vehicle_name/use_sim_time/start_gui；只启动一套仿真与 mixer |
| launch/trajectory_preview.launch | B：只启动 trajectory_preview_node 与可选 RViz，订阅已有仿真状态；不启动 Gazebo/mixer |
| launch/hippo_preview.launch | B：组合以上两个 .launch，供首次一键启动；已有 Hippo 时只用前一行的预览入口 |
| launch/hippo_line_tracking.launch | 连接已有仿真，启动一个 own tracking_node；复用计算库，不额外启动姿态节点/mixer；use_sim_time=true，初始 IDLE |
| 父级 .gitmodules + USTROV gitlink | 固定 USTROV 版本；采用用户已建立的 Git 子模块，不另建 dependencies.repos |

预览没有控制输出；后续闭环必须确认控制命令来源唯一，不能重复启动跟踪节点或 mixer。构建依赖：ROS 2 + USTROV 安装环境 → ROV_Control overlay；自己的实现只维护一个权威源码目录，WSL 构建从该来源产生，不能同时维护 Windows/WSL 两份独立编辑版本。

## 7. 后续扩展与真机迁移

### 多航点/曲线

纵向直线通过后，再设计水平弧线和多航点：路径应含切向/曲率，时间分配需同时限制向心加速度 v²κ 和转向速度。直线接直线的尖角不能以非零速度硬通过；选择停点转向或连续曲线需结合机动能力。届时新增曲线路径/制导函数，并升级受控自由度，不能复用首版标量控制器就称为平面位置跟踪。

水平/空间轨迹阶段，先解决侧向误差如何通过航向/俯仰与前向推力间接纠正、浮力与深度如何协调、控制输入约束和可实现终点状态。真实四推进器几何将决定是否继续使用这种欠驱动结构。八推进器版本需重新验证分配矩阵秩、条件数、饱和可达集合和模型；不只是改数组长度。

### 真机

1. 保留 C++ 几何规划、时间采样和合适的控制算法；在 Pi 4B 上重新构建并测周期耗时。
2. 用实际 IMU/深度/DVL/后续视觉形成可用状态，经 StateAdapter 输出统一格式；没有可信位置观测时不开展真机绝对位置闭环。
3. HippoActuationAdapter 是仿真专用边界。真机要按实际布局、推力曲线、坐标和归一化规则实现硬件适配，结合 PX4 direct_actuator 验证。Pi 仍承担所需控制环，不能默认 PX4 自动给出姿态内环。
4. 真机配置、时间源、ROS 版本、传感质量、保护条件与使能流程逐项核验；命名空间/remap 只解决连接命名。
5. 视觉在真实设备上先采集/标定/离线或只读估计，再小范围闭环。仿真可用状态噪声、延迟、丢包故障注入检查控制接口，这不要求仿真真实水下图像。独立评价真值与反馈来源区分。

## 8. 分批实现与验收

| 批次 | 先解释/再实现 | 验收 |
|---|---|---|
| A（已完成） | 数据类型、边界、直线路径、五次时间律 + C++ 核心测试 | 2026-09-29 独立 CMake 与 ament/colcon 构建测试通过，21 个 GTest 全通过；教学讲解见 ../README.md；尚不据此判断用户已经掌握 |
| B（已完成） | ROS 状态适配与轨迹预览，只连读状态 | 2026-10-02 新增 14 个 C++ 测试通过；Hippo 默认/改名实例、参考、暂停/故障、复位、rosbag 验证完成；世界时间重置后的上游状态恢复限制见 README §7 |
| C | 小输入方向核验、姿态库封装、PD、限幅、日志与运行状态机 | 唯一命令源、符号正确、沿线响应可解释；暂停/过期/越界停止有效；记录完整 |
| D | 受限短直线 PD 跟踪 | 报告沿线误差、横向/深度漂移、饱和占比、周期耗时与退出原因，不把投影误差冒充全空间误差 |
| E | 模型辨识/核验 → MPC 数学与 QP/OSQP → 数值再 Gazebo | 同参考/初始条件/限幅与 PD 比较；记录求解失败、超期、预测误差 |
| F | 可实现转弯、侧向/深度控制、多航点 | 新受控自由度逐步通过；未满足时不追加复杂规划器掩盖控制缺口 |
| G | Pi/实际布局适配、状态估计、后续真实视觉闭环 | 按设备与观测条件分级验证，保留桥原有保护；不是改 namespace 后直接下水 |

2026-09-29 用户确认并完成 A 批；09-30 细化 B 批设计并清理重复文档；10-02 用户确认 B 批后完成实现与接线验证。B 是闭环前的状态/时序验收；C/D 才让控制输出影响仿真。首轮数值示例不冻结真机参数。沿用用户已建立的 Control_Sys_AUV 主仓库和 USTROV 子模块，本次没有自动提交。


## 9. B 批：Hippo 状态接入与轨迹预览的具体设计

设计日期 2026-09-30；2026-10-02 用户“开始执行”确认后已实现。本批完成状态、坐标、仿真时间与参考显示接通；首轮真正的 SIL 闭环继续按 C/D 的纵向 PD＋姿态保持推进。新增 14 个 C++ 测试通过，实际 Hippo 的预览、暂停、故障/复位及改名实例已运行；已发现的世界重置限制和结果统一见 README §7。

### 9.1 系统目标与复用范围

从运行中的 /uuv00/odometry 得到统一 RobotState；用户 start 后，从当前点沿当前水平朝向生成一条 0.2 m 的教学参考轨迹；RViz 显示完整路径和随仿真时间变化的参考姿态，同时显示原始 Odometry。

复用 A 批全部数学接口与 Result<RobotState>；几何/轨迹公式不变。复用 USTROV 仿真、状态桥、TF 和已有唯一 mixer。B 批只订阅状态，不创建 thrust_setpoint、torque_setpoint、thruster_command 发布者。视觉、估计器切换、PX4 与旧独立俯仰测试不进入本批。

B 不使用“机器人是否静止”判断参考图形能否生成；其参考允许在机器人被动漂移时预览，明确标为 PREVIEW。C 的真实控制启动必须补齐前向、起速、姿态和工作空间条件；不能拿 B 的预览成功当作可执行性证明。

### 9.2 完整数据流

~~~text
preview.yaml + launch 参数
    → loadConfig() → PreviewConfig
/clock → ROS 时钟                     steady clock → 接收间隔/暂停过久检测

Hippo 真值 → fake_state_estimator → /uuv00/odometry
    → StateAdapter::convert()（检查 frame/数值/时间，v_B 转 v_W）
    → latestState_ + 接收时刻 + 状态有效标志
    → start 服务
    → createTrajectory()（当前点、水平朝向、目标位移、空间范围）
    → plan() → create() → 保存 trajectory_ 和 t0_
    → makePathMessage() → planned_path（RViz）

每次 onTimer()
    → 检查状态年龄与时钟
    → t = now_ros - t0
    → trajectory.sample(t)
    → publishReference() → reference_pose（RViz）
    → projectProgress() → 实际沿线进度/参考进度等诊断
    → publishDiagnostics() → diagnostics
    → rosbag2 保存输入状态、参考、诊断和 /clock
~~~

参数只在创建节点时读取，本批运行中不支持热改参数。状态、参考计算、服务与计时回调使用单线程 executor，避免读取半更新的快照。Path/参考 Pose 是显示输出，后续控制仍直接读取 C++ ReferenceSample。

### 9.3 数据结构与内部状态

| 类型/成员 | 内容及单位 |
|---|---|
| RobotState / ReferenceSample / ProgressState | 复用 types.h，W=map，B=vehicle_name/base_link，m/s/rad |
| PreviewConfig（放在预览节点头文件） | world/body frame、相对长度 m、MotionLimits、WorkspaceBounds、显示点数、wall timer 周期、ROS 状态年龄与 wall 接收间隔阈值 |
| PreviewStatus（同一头文件） | WAIT_STATE、READY、PREVIEW、FINISHED、FAULT；PREVIEW 无控制权限 |
| StateAdapter::lastStampNs_ | 上一份已接受的消息时间戳，初始为空；拒绝的消息不能更新时间状态 |
| latestState_ / lastReceiveSteady_ | 最新有效状态和该有效消息的接收时刻；非法数据不得刷新“状态新鲜” |
| trajectory_ / t0_ | start 时一次性创建的轨迹和仿真起始时间；不跟着实际位置漂移而重建 |
| lastRosTime_ / nextDiagnosticSteady_ | 检测时间倒退、限制诊断发布频率；不把 wall 时间当作轨迹进度 |
| status_ / reason_ | 当前阶段和中文原因，提供服务响应、屏幕日志与诊断 |

### 9.4 文件、类和每个函数

以下是 B 批实际新增的 C++ 实现文件；已有数学核心只被链接，不增加 ROS 依赖。

| 文件/类/函数 | 输入 → 输出 | 内部状态、步骤及调用 |
|---|---|---|
| rov_control_ros/include/rov_control_ros/state_adapter.h + src/state_adapter.cpp / StateAdapter(worldFrame, bodyFrame) | 期望坐标名 → 适配器 | 保存 frame 名与可空上一时间戳，不订阅话题 |
| StateAdapter::convert(odom) | nav_msgs::msg::Odometry → Result<RobotState> | 检查 frame、有限值、四元数、时间戳格式/单调性；小误差归一化；旋转 v_B；成功才保存时间戳。失败复用现有错误码并给具体中文原因 |
| StateAdapter::reset() | 无 → 清除时间历史 | 用于显式故障复位/仿真新时间段；不改变 frame 约定 |
| trajectory_preview_node.h + src/trajectory_preview_node.cpp / TrajectoryPreviewNode(options) | NodeOptions → ROS 节点 | 调 loadConfig；创建适配器、订阅、显示发布者、服务、wall timer；初始 WAIT_STATE |
| loadConfig() | 已声明的 ROS 参数 → 经校验 PreviewConfig | 检查数组长度、有限值、正周期/限制、有效边界、非空 frame、显示点数；非法则拒绝节点初始化，不静默换默认 |
| onOdometry(msg) | 一条消息 → 更新有效快照或置 FAULT | 调 convert；成功保存状态与 steady 接收时间；坏数据使状态失效，活动预览停止；不得沿用旧状态伪装新消息 |
| isStateFresh(nowRos, nowSteady) | 两种当前时间 → Status | 检查有状态、ROS 时间年龄、明显未来时间、wall 接收间隔；只读检查，用于 start 和 timer |
| createTrajectory(state, config)（命名空间纯函数） | 当前有效状态＋配置 → Result<QuinticTrajectory> | 水平投影 q_WB*[1,0,0] 并归一化为 d；p_goal=p_current+length*d；调用 A 批 plan/create；水平朝向近零时拒绝；无持久状态 |
| onStart(request, response) | Trigger → 成功/拒绝原因 | 只在 READY/FINISHED 且状态有效时调用 createTrajectory；固定 t0、保存轨迹、发布完整 Path 后进入 PREVIEW；FAULT 必须先 reset_fault |
| onStop(request, response) | Trigger → 停止结果 | 调 clearRun；回 READY 或 WAIT_STATE，不发布执行器消息 |
| onResetFault(request, response) | Trigger → 复位结果 | 清轨迹和旧状态、调用适配器 reset、重新等待有效消息；不自动开始预览 |
| onTimer() | wall tick → 参考/状态更新 | 先检查 ROS 时间倒退与状态时效；ROS 时间不前进则不推进参考；活动期间按 ROS 时间 sample，末端发布一次后 FINISHED；诊断降频发布 |
| makePathMessage(trajectory, epoch, count, frame)（命名空间纯函数） | 轨迹＋开始时间＋显示点数＋世界 frame → Result<nav_msgs::msg::Path> | 检查点数/采样结果；仅 start 时均匀采样含首尾的显示点；位置/姿态来自同一轨迹；不在 timer 重建数组 |
| publishReference(sample, stamp) | 单点参考＋ROS 时间 → PoseStamped 发布 | 按 world frame 填位置/四元数；当前参考速度等通过诊断给出，不谎称 Pose 包含速度 |
| publishDiagnostics() | 阶段、快照、时序、参考/投影 → DiagnosticArray 和状态转换日志 | 包含阶段、原因、两种年龄、s/v_s 及参考值、T；常态约 1 Hz，状态转换时立即发布 |
| clearRun(nextStatus, reason) | 下一阶段及原因 → 清除活动轨迹 | 清轨迹/t0，发布空 planned_path 撤销旧路径，更新阶段；保存有效反馈与否由调用方明确处理 |
| src/preview_main.cpp / main(argc, argv) | 命令行 → 返回码 | init → 创建节点 → 单线程 spin → shutdown；无计算公式 |
| test/test_state_adapter.cpp | 构造 Odometry → 断言 | yaw=90° 时 body-x 变 world-y；错误 frame/四元数/NaN/时间顺序；错误不污染上一有效时间；reset 后接受新时间段 |
| test/test_preview_math.cpp | 已知姿态、位置、轨迹 → 断言 | 水平目标方向与长度、Path 首尾和 frame/时间；将无 ROS 状态的计算辅助函数做成可直接调用的方法，避免只测试 private 回调实现细节 |

createTrajectory 和 makePathMessage 在同一节点头文件中声明为命名空间纯函数，源码定义；由节点与测试共用，不另外建立“框架”类。最终签名按上述输入输出落实，算法核心仍不依赖消息类型。

新增构建声明：rov_control_ros/CMakeLists.txt 和 package.xml，链接 rov_control_core，依赖 rclcpp、rcl_interfaces、nav_msgs、geometry_msgs、diagnostic_msgs、std_srvs 及测试工具。rcl_interfaces 用于拒绝运行时参数修改，避免参数显示值与内部配置不同。rov_bringup/CMakeLists.txt 和 package.xml 只安装 launch/config/rviz，声明预览节点与仿真/RViz/launch_xml 运行依赖。此处不提前链接 GeometricController，C 批执行适配器再链接 ustrov_control。

### 9.5 启动、参数和 ROS 1 迁移

| 新增声明文件 | 职责 |
|---|---|
| launch/hippo_sim.launch | ROS 2 XML 包装，include 外部 top_hippocampus_complete.launch.py；设置 vehicle_name、use_sim_time=true、start_gui |
| launch/trajectory_preview.launch | 只启动自研 trajectory_preview_node 与可选 RViz；节点名固定 trajectory_preview，namespace=vehicle_name |
| launch/hippo_preview.launch | include 上述两个 .launch，一次启动一套 Hippo 和预览 |
| config/preview.yaml | 参数类型与教学起步值：length=0.2 m、v_max=0.08 m/s、a_max=0.1 m/s²、T_min=0.1 s；world=map；显示点数 101；其余时效/边界按本机模型明确配置 |
| rviz/trajectory_preview.rviz | Fixed Frame=map，显示 Odometry、planned_path 和 reference_pose；命名空间通过启动配置/话题重映射处理，验证 uuv00 后再验证改名实例 |

空间边界已按模型和生成位姿核验后写入：pool.dae 的 unit=0.2、Z_UP，平移 (1,2,-1.5) 后可视外包围约 x[-0.101,2.101]、y[-0.101,4.101]、z[-1.515,0.05] m。预览盒选 [0.8,0.5,-1] 至 [1.8,1.7,0]，只作为起始位置附近的参考中心活动范围；没有把可视网格当作碰撞/安全验证。0.2 m 等数值仅是预览例子，闭环增益/限幅另行设计。timer=0.02 s，ROS 状态最大年龄=0.25 s、未来容差=0.05 s、steady 接收最大间隔=2 s，均在 YAML 中说明。

本机 /opt/ros/jazzy/lib/python3.12/site-packages/launch_xml/parser.py 的 get_file_extensions() 明确包含 launch.xml、xml、launch，所以 .launch 后缀可直接使用。所有自有入口采用 .launch；USTROV 作为外部依赖保留其现有 Python 入口，由 XML include 复用，不把重写第三方启动作为学习前置项。

| ROS 1 习惯 | ROS 2 XML |
|---|---|
| node 的 type | exec |
| node 的 ns | namespace |
| $(find 包名) | $(find-pkg-share 包名) |
| $(arg 参数) | $(var 参数) |
| rosparam file | node 内 param from |
| roslaunch 包 文件.launch | ros2 launch 包 文件.launch |
| 全局 use_sim_time | 每个需要仿真时间的节点设置 use_sim_time |

迁移依据：已读 [ROS 2 Jazzy 官方迁移说明相关章节](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.rst)；后缀依据本机已安装源码。launch API 的具体接法在实现后通过 --show-args、实际启动和节点/话题核验。

### 9.6 时钟、QoS 和失效行为

- Odometry 订阅使用 SensorDataQoS、keep_last(1)，及时消费状态；显示输出和诊断采用可靠 QoS，完整 Path 用 transient-local 保存最新图形，便于稍后开启 RViz。
- 假设输入 pose 在 map、twist 在 vehicle/base_link；帧名不匹配则报错，不能通过改字符串冒充完成坐标变换。本批不提供任意 TF 查找。
- 消息四元数接近单位时只修正舍入误差；零值、明显非单位、NaN/Inf 直接拒绝。q 和 -q 表示同一旋转，验收看旋转结果。
- 轨迹使用 ROS 时间；wall timer 只负责定期检查与发布。短暂停仿真时参考冻结，超过显式 wall 接收间隔则 FAULT。仿真恢复/时间倒退后需显式复位与 start，不静默恢复旧轨迹。
- 到 T 后参考停在终点，阶段为 FINISHED；它只表示参考播放完成，不表示机器人已到达。B 不用位置误差判定任务成功。
- start 的“成功”只表示生成预览参考成功；C/D 中的控制使能和 PX4 arm 是不同概念。

### 9.7 如何运行和判断这一批通过

已验证的构建/启动命令集中在 README §5–6，不重复维护第二套。操作：source 三层工作区 → colcon 构建新包 → hippo_preview.launch → start 服务 → 看 RViz 与 diagnostics → stop；已有 Hippo 时只启动 trajectory_preview.launch。路径移动后本次产物放 build/batch_b 和 install-b。

验收按实际顺序：

1. C++ 状态转换测试通过，特别是 yaw≈90°；默认 Hippo 朝世界 +y，前向目标因此约为 p_current+(0,0.2,0)，不固定沿世界 +x。
2. --show-args 能解析自有 .launch，launch 后仅一套 Gazebo、bridge、state_estimator、mixer，以及一个预览节点。
3. planned_path 起点与 start 时的实际位置一致，终点/姿态和核心采样一致；RViz Fixed Frame=map。
4. /clock 停止时参考不推进，长暂停/时间回退/非法状态给明确诊断，恢复不自动开始。
5. 预览节点在执行器话题没有发布者端点，排除把“无位置变化”误当作无命令证据。
6. rosbag 可读到原始 odometry、参考 Pose 和诊断；记录明确为预览验收，不写“闭环跟踪成功”。

之后进入 C/D：先逐通道核对小输入的方向与零命令/超时行为，再接已讲解的纵向 PD、上游姿态计算库、命令限幅、运行状态机和周期日志。首轮报告沿线误差、速度、横向/深度漂移、命令饱和、周期耗时与退出原因；以这些证据确认 SIL 闭环是否成立。

## 10. 合并保留的环境、算法细节与维护约定

### 环境与模型

- 09-30 核对 Windows USTROV 和 WSL ~/rov_sim/USTROV 均为 b1208e50，工作树干净；ustrov_sim、ustrov_control、rov_control_core、rviz2 可发现，上游 --show-args 成功。当时无 Hippo 进程；未把这些检查当作新闭环结果。
- 默认启动树：top_hippocampus_complete → start_gazebo（empty.sdf、pool、/clock）＋spawn_hippocampus（hippo3、唯一 mixer）→ spawn_vehicle（状态桥、真假估计、TF）；相机/AprilTag 默认关闭。
- 初始生成位置为 (1.3,1.0,-0.5) m，yaw=1.5708 rad。运行后位置会因浮力等改变，规划起点必须取有效实时状态。
- world/empty.sdf 为世界与步长配置；pool.xacro 是可视模型、没有池壁碰撞；hippo3.xacro 与 hippo3_params.xacro 为机器人与参数；上述值属于 Hippo，不能当作真实四推进器 ROV 的标定。
- 真值/压力配置目标频率分别约 50/100 Hz；不能把配置值当实测消息率。模型有 IMU/压力/真值；当前默认没有 DVL/GPS/声呐测量来源。fake_vision 不依赖图像；真实 estimator 输出接口需另适配。
- 附加质量参数在已读水动力插件中用于科氏项，未见对应 M_A*νdot 加速度项；MPC 建模前需做受控激励核验，不直接把参数相加称为有效惯量。
- 自研算法不依赖具体仿真引擎；当前用 Gazebo 建立反馈与控制工程。MuJoCo/MarineGym 在有明确并行学习需求后再评估；真实视觉直接在最终成像设备上验证，预算和场地需求按根目录记录保留。

### A 批实现中应理解的细节

- 有效盒已减去机器人尺寸/余量，contains 包含边界；只有在空旷凸盒内才能靠两端点判断线段不越界。
- LinePath 私有构造使外部不能跳过 plan 检查；L≤1e-9 m 返回 NO_MOTION，不构造零方向路径。
- projectProgress 不裁剪实际 s；cross_track_W=(p-p0)-d*s，depth_offset_m=-cross_track_W.z()。后者为相对投影点的偏差，不是传感器绝对深度。
- chooseDuration 使用 long double 中间量，并在转换到 double 向下舍入时用 nextafter 向上修正；这些是浮点计算处理，不是额外物理余量。
- sample 的位置多项式后半段使用 h(τ)=1-h(1-τ) 减少消减误差；两端外侧返回精确起终点与零速度/加速度。时间字段仍保留查询 t。与静止段拼接为位置/速度/加速度连续，jerk 不保证连续。
- sampleHorizon 要求调用者先设置 buffer.size≥count；先全部检查，再写前 count 个元素，失败不部分改写；不 resize，尾部不变，超过 T 的参考保持终点。
- Result<T> 成功才可读取 value()；读取失败结果会抛 bad_optional_access，属于调用者编程错误。库不读取 ROS 时钟、不自行恢复运行。
- 21 个测试的具体范围、0.2 m 数字例子及运行命令统一在 README；不用另开逐批汇报文件重复维护。

### Git 与文件维护

自研包使用 rov_ 前缀，依赖方向为 ROV_Control → USTROV。2026-10-02 用户已将两者放入 Control_Sys_AUV 主仓库，USTROV 为固定 commit 的 Git 子模块；沿用这一组织，不再另加重复的版本清单。必要的上游修复独立维护，本次无上游修改。仿真子集不需 PX4 underlay，完整硬件工程再按依赖加载。保留一份自研源码，通过 WSL /mnt/c/DeskFiles/MPC-Mamba/Control_Sys_AUV/ROV_Control 构建，不另建会分叉的编辑副本。主仓库 AGENTS.md 按用户要求暂时忽略；正式 C++ 单元测试源码继续受版本管理。

文件约定：README 放当前可执行步骤与结果；本文件维护设计和下一批接口；MPC_LEARNING_ROADMAP 保留理论/选型。根目录规则、证据和决策分别维护。删除重复过程稿前迁移有效信息并修复链接；不新增仅复述进展的日报/批次 .md。

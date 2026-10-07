# 路径规划与轨迹跟踪：从水平到点到 MPC

更新：2026-10-06。参数与检查职责整理见 §6.1。A/B/C1 及水平 XY 到点闭环已实现。用户授权完成在环部分，并明确选择沿用 Hippo 四推进器、深度仅监测。本文 §1–8 是当前实现；§9–11 保留预览和有限输入入口的具体设计。原先拟议的空控制接口、normalized effort PD 和上游姿态库封装方案已由当前物理分配方案替代，不创建这些空类。

## 1. 系统目标

向 `/<vehicle>/goal` 发布一次 `PointStamped`，机器人在水平面到达该点并进入容差保持。坐标原点取本节点收到的第一条有效 Odometry，x 沿当时机头的水平投影、y 向左、z 向上；之后固定。它是软件启动原点，不声称等于真实硬件通电瞬间。完整重启重新建立，故障复位保留。

当前是 Gazebo + 自研 C++ 控制器的软件在环，没有 PX4 固件在环。状态由真值桥提供，不是 VINS/INS 融合验证。目标 z 必须为0，表达“只请求水平运动”；深度不闭环。真实浅池下视 AprilTag、相机/镜头/防水合计2500元内的需求保留到后续真实设备验证。

## 2. 整体架构与复用边界

- `rov_control_core`：Eigen/C++17 纯计算，无 ROS 时钟、话题或文件读写。
- `rov_control_ros`：状态适配、home、回调、运行保护、命令和显示发布。
- `rov_bringup`：XML `.launch`、YAML、RViz。
- USTROV 固定版本 `b1208e50becb9f74d4d0dcd8637ac11e73d3672a`：复用世界、原 Hippo、浮力/水动力/推进器插件、状态桥和 TF。本轮没有修改上游。
- 新入口直接 include `spawn_vehicle.launch.py`，不 include 会启动 mixer 的 `spawn_hippocampus.launch.py`。四路物理分配由自己的 `HippoAllocator` 完成。
- 原预览和 C1 probe 仍各自保留，不能与到点节点同时控制同一实例。原 geometric 姿态库、未实现的 PathFollower、旧 pitch_mpc、PX4、相机不接入这次闭环。

```text
PointStamped(home) → onGoal → HomeFrame::toWorld → PlanarTracker::setGoal
Odometry → onOdometry → StateAdapter::convert → RobotState
YAML → loadConfig → PlanarConfig / 运行监测阈值
/clock → ROS时间增量 ──────────────────────────┐
wall 20ms → onTimer → checkState/checkGraph   │
  → PlanarTracker::update(state,dt) ←─────────┘
    制动 → 转向 → LinePathPlanner + QuinticTrajectory
    sample + projectProgress → 纵向PD/前馈 + LOS航向 + 姿态PD
  → TrackingOutput(Fx, Mx, My, Mz)
  → HippoAllocator::allocate → ActuatorControls(前4路，后4路零)
  → thruster_command → 外部bridge → Gazebo推进器/机体
  → 新Odometry → 下一控制周期
参考/实际轨迹/目标/home/误差/控制量 → RViz、diagnostics、rosbag
```

## 3. 运行流程、时钟与失效边界

```text
WAIT_STATE → READY → 发布有效目标 → BRAKING → TURNING → TRACKING → HOLDING
运动中换目标 → BRAKING → 重新转向/规划
HOLDING 偏离5cm → BRAKING → 再接近
stop → STOPPING(有限归零) → READY
状态/时序/输出源异常 → FAULT(锁存、归零)
恢复数据 + reset_fault → WAIT_STATE → READY → 需要重新发布目标
```

- BRAKING：前向速度阻尼，水平速度<0.015m/s、|yaw角速度|<0.08rad/s连续0.25s才转向；已在目标圈内则直接保持。
- TURNING：速度阻尼仍在，航向误差<0.04rad、|yaw角速度|<0.08rad/s、水平速度<0.02m/s连续0.3s后，从当前实测位置规划。
- TRACKING：五次参考到终点后，水平误差<0.025m、速度<0.015m/s连续0.75s才报告HOLDING。参考结束和实际到达分开判断。
- HOLDING：沿最终朝向抑制纵向漂移；横向没有直接推力，超过0.05m重新接近。不承诺横流中精确悬停或任意扰动下收敛。
- ROS dt 推进参考，wall timer 检测停顿。状态年龄、接收/时钟停顿、最大正dt均由YAML指定，dt上限仅在ROS层检查。实际速度/倾斜/角速度/深度漂移也从YAML读取，0可关闭对应运动监测；原0.5m/s硬阈值已移除。单帧坏/重复/乱序Odometry被丢弃，不刷新有效接收时间；持续无有效反馈才按超时停止。时间倒退、活动盒越界仍归零并锁存FAULT。
- 任务时限由mission_timeout_s指定，0关闭；保持期间不按任务超时退出。实际电机话题多发布者每1s检查，接受目标前再查。订阅者是否已发现、节点叫什么名字、无关setpoint话题均不作为许可条件；ROS图检测不是分布式互斥锁。
- 故障/stop立即发零并补发0.5 wall秒；随后释放输出权限。正常SIGINT/SIGTERM在ROS context有效时完成归零。强杀/崩溃不能保证此路径；新入口无上游mixer watchdog，因此仅用于仿真，真机需独立下层看门狗。

## 4. 数据结构与接口约定

| 类型 | 坐标/单位、内容 | 谁创建、谁消费 |
|---|---|---|
| RobotState | p_W[m]、v_W[m/s]、q_WB、omega_B[rad/s]、stamp_ns | StateAdapter → tracker/运行检查 |
| HomeFrame | origin_W；q_WH只含初始yaw | 首条有效状态 → 目标转换、局部里程计、静态TF |
| PlanarConfig | 活动盒、速度/加速度、PD/前馈、姿态增益、容差/超时 | YAML + 名义模型常量 → tracker |
| LinePath | p0_W、单位方向、长度L | planFrom中的LinePathPlanner → 时间轨迹/投影 |
| ReferenceSample | s/v_s/a_s、p_W/v_W/a_W、期望姿态 | QuinticTrajectory::sample → 控制/显示 |
| ProgressState | 实际s、沿线速度、未裁剪投影及横向误差 | projectProgress → 纵向控制、LOS |
| TrackingOutput | Fx[N]、torque_B_Nm、参考、误差、阶段、限幅 | update → allocator/诊断 |
| AllocationOutput | 单桨物理力[N]、8槽油门、缩放系数、分配后wrench | allocate → ROS消息/诊断 |

W=`map`，B=`vehicle/base_link`，H=`vehicle/home`。q_WB把机体系转到世界系；Odometry的twist在B，适配器执行v_W=q_WB*v_B。目标转换为p_W=p_home_W+R_WH*p_H；当前姿态变化不改变R_WH。只改frame字符串不算坐标转换。

目标QoS为reliable/volatile，防止节点重启重放旧目标。目标是持续任务，header.stamp不用于过期检查；反馈Odometry仍必须满足时效要求。错误frame、非零z、非有限数、越界目标被拒绝并保留原任务；重复同一XY目标不重置轨迹。

Path为显示消息，不包含完整速度/加速度；控制直接调用C++轨迹对象。planned_path/actual_path/home/accepted_goal保留最新消息供晚启动的RViz读取，诊断/显示名义10Hz；actual_path最多2000点。odometry_home的twist仍在B；当前真值零协方差直接保留，真机非零协方差需完整变换。

## 5. 算法与中间量

### 5.1 直线路径与五次时间轨迹

空旷凸活动盒中，p(s)=p0+d*s，d=(p1-p0)/L。两个端点在盒内才能建立路径；不包含地图、A*、RRT或障碍避让。

令τ=t/T，h=10τ³−15τ⁴+6τ⁵：s=Lh，v=L/T*h'，a=L/T²*h''。时间取
`T >= max(Tmin, 1.875*L/vmax, sqrt((10*sqrt(3)/3)*L/amax))`，启用jerk限制时再加对应项。参考从静止出发、以零速度/加速度结束。运动中换目标先制动，因此不将旧的非零参考速度硬拼到新轨迹。

### 5.2 纵向PD与航向

实际进度s=d·(p−p0)，v_s=d·v_W。控制为
`Fx = m*a_ref + d_surge*v_ref + kp*(s_ref−s) + kd*(v_ref−v_s)`。

当前名义m=2.31kg、d_surge=5.39N/(m/s)，kp=4N/m、kd=4.5N/(m/s)，均可配置，属于当前模型近似而非真机辨识。结构体缺省vmax=0.08m/s、amax=0.06m/s²；当前用户YAML为0.8m/s、0.6m/s²，Fx请求仍裁至±1.5N。配置加载成功不代表实际可达峰值速度；例如0.8m/s的名义阻力项约4.31N，已经超过1.5N限幅。

路径左法向n=(-sinψ_path,cosψ_path)，横向误差e_y=n·(p−p_projected)。LOS航向修正为`clamp(-atan2(e_y,0.20),-0.6,0.6)`，即位于路径左侧时向右偏头。航向差大于0.3rad时冻结参考时间并制动。该策略围绕近对准工况实现，不提供横流/强扰动的稳定性保证。

roll/pitch目标为0，yaw目标为路径航向加LOS修正。`M=Kp*[-roll,-pitch,wrap(yaw_des-yaw)]−Kd*omega_B`，然后逐轴限幅。wrap处理±π，身后目标可转向；节点拒绝大倾斜，避免将小倾斜欧拉反馈泛化成全姿态控制。默认Kp=[.035,.05,.10]，Kd=[.028,.06,.08]，力矩上限=[.006,.025,.025]N·m。

### 5.3 原Hippo物理分配和死区逆映射

令a=0.097/√2m，每台物理正推力f_i均定义为沿body+x。由力臂r×F及桨反作用力矩，

```text
[ Fx ]   [ 1     1     1     1   ] [f0]
[ Mx ] = [-.01  +.01  -.01  +.01 ] [f1]
[ My ]   [ a     a    -a    -a   ] [f2]
[ Mz ]   [-a     a     a    -a   ] [f3]
```

固定满秩矩阵在构造时求逆；max_thruster_command配置电机限幅（缺省.4，归一化接口范围(0,1]），转换成单桨推力上限后四路同比缩小以保留wrench方向。上游单桨曲线为`F=max(0,.003313*n²+.1086*n−.8576)`，n=50*|油门|。解正根得到期望物理力所需油门；物理力趋近0时油门趋近约.13154的死区边缘，精确零力返回零。最后乘电机符号[-1,+1,-1,+1]，后4槽置零。发送端只检查有限值与归一化接口范围，不重复配置限幅。

这是对已核验仿真曲线的静态逆映射，不修改原插件；未消除真实推进器滞后、换向、电压与水流影响。不要再把该输出交给normalized mixer。ROS端一条消息装8槽，外部bridge仍逐路转发Gazebo消息，不能称端到端原子执行。

## 6. 文件、类与函数级调用

路径相对`src/`；头文件在包的`include/<包名>/`，实现位于`src/`。

| 文件 | 函数：输入 → 输出；内部状态/调用者 |
|---|---|
| core/types.h、result.h | 已有共享状态/限制/参考与显式错误；value只能在ok后读取 |
| core/planning/line_path.h/.cpp | plan(PlanRequest)→LinePath；projectProgress(path,state)→ProgressState，无内部运行状态 |
| core/planning/quintic_trajectory.h/.cpp | chooseDuration、create、sample、sampleHorizon；保存不可变路径和时间限制 |
| core/tracking/planar_tracker.h/.cpp / HomeFrame | fromState→原点+初始yaw；toWorld/toHome做固定坐标转换 |
| 同上 / PlanarTracker | validateConfig→Status；setConfig更新已校验参数但保留任务；setGoal校验后进入BRAKING；planFrom生成线段/轨迹；update(state,dt)推进阶段并输出物理请求；reset清任务计时/轨迹 |
| core/tracking/hippo_allocator.h/.cpp | 构造保存B/B逆；forceFromThrottle/throttleFromForce对应单桨曲线；allocate(Fx,M)→AllocationOutput，无上一周期状态 |
| ros/state_adapter.h/.cpp | convert检查frame/时间/有限值/单位四元数、旋转速度；保存上一有效时间戳；reset清历史 |
| ros/point_tracking_node.h/.cpp | 创建算法对象；onOdometry丢弃坏帧/更新有效状态、首次建home；onGoal检查并启动任务；onTimer检查→update→allocate→sendMotors |
| ros/point_tracking_parameters.cpp | loadConfig读取启动参数；readParameterValues更新候选副本；validateParameters检查候选；onParameters只校验，applyParameters在ROS提交后更新tracker/allocator；logConfig打印生效值 |
| 同上 | checkGraph检查唯一电机发布者和无setpoint源；checkState检查时序/运动/空间；sendMotors边界复查后发布 |
| 同上 | stopOutput立即归零+有限补发；onStop取消任务；onReset等待归零后清故障/状态，保留home；prepareShutdown正常退出共用归零路径 |
| 同上 | publishHome发Pose和静态TF；publishDisplay降频发布参考/路径；publishDiagnostics发布状态/误差/控制/时序，阶段变化写终端 |
| ros/tracking_main.cpp | init→单线程executor→正常信号置标志→prepareShutdown→有界spin→shutdown |
| bringup/config/point_tracking.yaml | 增益、任务容差、空间/时效阈值、执行器限幅；ROS参数服务可更新，保存文件本身不触发更新 |
| bringup/launch/hippo_point_tracking.launch | 世界+原版Hippo无mixer+point_tracking.launch；用户日常总入口 |
| bringup/launch/point_tracking.launch | 自研节点+可选RViz，只可接无mixer的Hippo实例 |
| bringup/rviz/point_tracking.rviz | 实际里程计、计划/实际路径、参考/目标，Fixed Frame=map |

### 6.1 参数和检查的职责（2026-10-06）

参考本地USTROV的`actuator_mixer_node_params.cpp`和`geometric_control_node_params.cpp`：模块使用配置限幅，控制增益可由参数回调更新。未修改上游，也没有移植PX4固件。

- 参数入口：数组维度、有限值、正/非负范围、活动盒上下界与参数关系检查一次。删除0.2m/s、3N、0.1N·m及状态/深度参数共同≤1等试验上限。允许零增益关闭相应项。Status持有字符串，错误带参数名、数值和要求。
- ROS参数事务：onParameters在config/limits副本上验证，无副作用；ROS提交后applyParameters更新实际对象。无新线程，无定时读YAML。增益、力/力矩及电机限幅下周期生效；已生成的五次轨迹保持原时间律，速度/加速度在下一段planFrom生效。home和任务进度不重置。
- 实际运动监测：max_actual_speed_mps、max_tilt_rad、max_angular_speed_radps、max_depth_drift_m为0时关闭该项。启用实际速度检查时其阈值必须大于参考上限；时效参数必须为正。坐标接口为read_only，仿真期间use_sim_time保持true。
- 消息入口：StateAdapter保留坐标、时间单调性、有限值和四元数检查。节点丢弃单帧错误，不刷新健康时间；超时、时间回退、多电机发布源仍停止归零。C1还保留全零基线确认、单通道和有限脉冲流程，这些属于试验本身的要求。
- 算法：保留零路径、数值溢出、四元数、采样区间等数学检查以及制动/转向阶段条件。contains不再每周期重验边界配置；samplePulse不再每周期重验固定ProbeConfig。core的dt仅检查有限且>0，ROS配置max_ros_step_s负责调度容差。
- 执行器：HippoAllocator维护由max_thruster_command算出的单桨最大推力，饱和同比缩放；sendMotors仅校验归一化接口。1是接口最大幅值，0.4是可改的默认值。归零、状态超时及下层看门狗不因本次整理取消。
- C1/预览：删除额外写死的试验范围和订阅者/节点名门槛；配置仍在启动时固定，当前在线调参仅对到点节点开放。预览显示点数的资源上限、ROS时间戳可表示范围仍保留。

回归验证覆盖高于旧门槛的配置、事务回滚、热改限幅、当前轨迹连续性、单帧错误/持续丢失、真实多源冲突及停止归零；Gazebo结果和具体命令统一记入README，不将接口假状态测试当成动力学验证。
| core/test/test_planar_tracking.cpp | 坐标、独立力矩平衡、曲线逆映射/限幅、身后目标、换目标、保持、LOS与时间边界 |
| ros/test/test_point_tracking.cpp | 人工状态/clock+假bridge；目标、无任务无输出、stop、暂停锁存、复位不重启、多源冲突 |

core/ros均为现有包内新增文件；CMake安装节点并登记正式测试，ROS包新增tf2_ros依赖。没有新增空包或将每个算法类拆成ROS节点。

## 7. 扩展与真机迁移

- MPC：保留home、状态适配、路径/轨迹、执行分配和运行检查；先将update中的纵向物理力公式独立为可替换策略，再做离散模型、预测矩阵、QP/OSQP及失败回退。此时再冻结接口，不提前放空实现。
- 多航点：目前逐个发布目标，运动中可替换；连续曲线/非零端速的多段拼接尚未实现。
- 真机：另做状态与执行器适配，核验FLU/FRD、ENU/NED、速度所属坐标、深度符号、时间、有效性、实际分配矩阵与推力标定、Pi周期及PX4权限/看门狗。命名空间只能隔离话题名，不能完成这些迁移。
- home：真实上电时刻若要跨控制节点重启保留，需要由独立定位/任务管理层建立并保存，当前原点属于单次节点生命周期。
- 视觉：实机下视AprilTag定位结果经标定、融合、时效/失效检查后进入状态适配；不将图像直接塞入当前跟踪器。

## 8. 验证与当前完成范围

已完成纯数学、ROS接口以及原Hippo实际Gazebo闭环三层验证。验收覆盖前进、斜向、身后目标、返回原点、运动中换目标、错误目标保留任务、停止/暂停归零与复位不自动重启；正式测试和数值结果统一维护在README §10，避免重复两份数字。

该结果属于无外加水流、真值反馈、原模型的水平基线。未验证避障、长期横流、传感器融合、PX4 HIL或真机性能。pool.xacro无池壁碰撞体，活动盒只约束中心点。暂停世界恢复可显式复位，重置世界仍建议重启完整launch。

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
| onOdometry(msg) | 一条消息 → 更新有效快照或丢弃该帧 | 调 convert；成功保存状态与 steady 接收时间；坏帧不刷新有效接收时间，持续无有效数据才超时停止 |
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

后续 C1 的方向/死区核验已完成，见 §11；当前水平闭环见 §1–8。实现采用自己的物理姿态 PD 与推力分配，没有接入原拟议的上游姿态库封装。

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

## 11. C1：执行通路、死区和方向核验（2026-10-03，已确认并实现）

### 11.1 源码证据与实施顺序

先核读本地固定版本的 mixer、消息、GeometricController、generic 推进器模型及 Hippo 引用关系，用户确认后实现并进行输入试验：

1. Hippo 的 `hippocampus_normalized_default.yaml` 输入为 `[Mx,My,Mz,Fx,Fy,Fz]`；单个有效通道对每台推进器的系数绝对值为 0.25，Fy/Fz 列为零。此配置的推力到转速转换为恒等映射；未饱和时单通道 effort=u，对应电机命令绝对值 0.25|u|。
2. generic 推进器最大转速 3000 rpm=50 rev/s，先计算 `F(n)=0.003313*n*n+0.1086*n-0.8576`，再将负值裁成零。正根约 6.57718 rev/s，因此非零稳态推力要求电机命令超过约 0.13154，对应单通道 effort 超过约 0.52617。
3. 上述阈值是源码推导，条件为仅一个通道非零、其余清零、当前模型及混控配置。C1 已核对运行参数：effort=0.4 时四台推进器报告推力均为零；|effort|=0.6 时峰值各约 0.14325625 N，与公式一致。这是两个幅值点的验证，没有用密集扫描测定精确阈值；不外推到多轴叠加或真实设备。
4. `SimpleMixer::ApplyInput()` 没有使用配置中的 `input_limits` 做逐通道限幅；`GeometricController::Update()` 也没有限幅。自己的发送边界必须校验命令，不能只看配置名或头文件说明。

因此先比较零输入、阈值以下输入、刚超过阈值的正反输入，再选 PD 增益。保持上游模型和混控不变，不通过删掉常数项或改混控系数掩盖死区。

### 11.2 系统目标与复用范围

C1 新增一个 C++ 节点，发送有限时长的单通道脉冲，记录实际运动、命令与 mixer 输出，验证方向和停止行为。它是执行通路试验，尚无参考误差反馈控制。

- 复用 StateAdapter、RobotState/Result、Hippo 仿真和唯一 mixer；原预览入口保留，不借预览 start 使能控制。
- 本批不接 PD、姿态保持、MPC、真实估计器、PX4 或相机。姿态/位置反馈用于监测和终止试验，不用于持续稳定控制。
- 只选 surge/roll/pitch/yaw 中一个非零通道，其他通道与 Fy/Fz 显式置零。每次 start 只执行一次配置好的脉冲，不自动遍历所有轴。
- 当前水平闭环复用 C1 的方向证据，另建物理力/力矩分配入口，不复用 normalized effort 消息封装；probe 保持独立试验职责。

### 11.3 数据流、时间和消息语义

```text
actuation_probe.yaml + XML 参数 → loadConfig → 固定 ProbeConfig
odometry → StateAdapter::convert → latestState_ + steady 接收时刻
  → 检查状态/命令来源 → READY → start → 保存起始状态与 t0

wall timer（名义20 ms）→ 时效/时钟/边界/相对运动检查
  → ROS 时间前进时 samplePulse(elapsed, config)
  → HippoEffortCommand（无量纲）→ makeActuatorMessages
  → thrust_setpoint + torque_setpoint → 唯一 mixer
  → thruster_command → static_bridge → Gazebo → 新 odometry

thruster_command → onThrusterCommand → 缓存输出和接收时间
阶段、配置、时效、故障 → diagnostics
rosbag2 → /clock、odometry、两类 setpoint、thruster_command、diagnostics
```

两条 ActuatorSetpoint 的 ignore_x/y/z 都为 false，所有字段显式赋值，防止保留旧缓存。两次 publish 不是原子操作：第一条可能与上一周期另一类输入组合；先在基线期发完整零组合、观察 mixer 回零，记录中保留实际混控输出。thruster_command 是发送的电机命令，不是执行确认或测得推力；方向验收依据实际状态变化。

脉冲进度使用 ROS 时间；wall timer 在暂停时继续检查。ROS 时间不前进时不继续发非零命令，接收超时进入锁存 FAULT，并由 wall timer 有限次补发全零。上游 watchdog 使用 ROS 时间，仿真暂停时不能依赖其计时。

### 11.4 数据结构和内部状态

这些类型放在下述两个 ROS 头文件中，不给数学核心增加 ROS 依赖：

| 类型/成员 | 内容 |
|---|---|
| HippoEffortCommand | surge_effort、torque_effort_B；四个无量纲命令，默认全零 |
| ProbeConfig | frame、通道、幅值、基线/上升/保持/下降/观察时间、幅值上限、工作空间、时效/位移/转角/速度阈值 |
| ActuatorMessagePair | thrust/torque 两条消息，共用时间戳和机体系 frame；不表示同步送达 |
| ProbeTiming | ROS 时间增量、状态 ROS 年龄、steady 接收间隔/时钟停顿时长、时钟倒退标志，由节点计算 |
| ProbeStatus | WAIT_STATE、READY、RUNNING、FINISHED、FAULT；RUNNING 内含基线、脉冲、零输入观察阶段 |
| 节点成员 | latestState_、startState_、t0_、lastRosTime_、lastReceiveSteady_、最近 mixer 输出/接收时刻、有限零补发截止时间、试验序号、阶段/原因 |

WAIT_STATE→READY→显式 start→RUNNING→FINISHED。FINISHED 仅表示输入试验结束。FAULT 锁存，新的有效状态不能自动恢复；reset_fault 清旧状态和时间历史后重新等待消息，不自动 start。

### 11.5 文件、类和函数级接口

以下接口已实现；C++17、详细中文注释、.h/.cpp、两空格 Allman。头文件位于 `rov_control_ros/include/rov_control_ros/`，实现位于该包 `src/`。

| 文件/类 | 函数：输入 → 输出 | 内部状态、调用关系 |
|---|---|---|
| hippo_command.h/.cpp | validateProbeConfig(config)→Status | 纯函数：通道、有限值、时间、幅值、空间、阈值检查；loadConfig 调用 |
| 同上 | samplePulse(elapsed,config)→Result<HippoEffortCommand> | 纯函数：基线零→线性上升→保持→线性下降→零观察；仅填所选通道，结束后始终零；负/非有限时间拒绝 |
| 同上 | makeActuatorMessages(command,stamp,bodyFrame,limit)→Result<ActuatorMessagePair> | 纯函数：检查有限值/frame/幅值；完整赋值；越限拒绝，避免悄悄改变试验幅值；正常输出/归零共用 |
| 同上 | checkProbeState(state,startState,timing,config)→Status | 纯函数：检查时效、时间、边界、相对位移/转角、速度；q/-q 等价，不用欧拉角直接相减处理跨 ±π |
| actuation_probe_node.h/.cpp / ActuationProbeNode | 构造函数、loadConfig() | 读取并固定配置；要求 use_sim_time=true；创建适配器、订阅/发布、Trigger 服务、wall timer，拒绝运行中改参数 |
| 同上 | checkCommandSources()→Status | 检测setpoint/电机话题实际多发布者冲突；不依赖固定mixer名称或订阅数。基线收到全零电机反馈才允许脉冲。图发现有延迟，此检查不是分布式互斥锁 |
| 同上 | onOdometry(msg) | convert 成功才更新快照/接收时间；单帧非法输入丢弃并限频告警，持续无有效状态才由定时器超时归零；FAULT 等显式复位 |
| 同上 | onThrusterCommand(msg) | 保存 mixer 输出和接收时刻，检查有限值/范围；不把输出称为实际推力 |
| 同上 | onStart(request,response) | 仅 READY/FINISHED 且零补发已结束；检查时效、起速、倾斜、唯一命令源；保存初态/t0/试验序号，进入全零基线 |
| 同上 | onTimer() | 检查状态/时钟/图→有新 ROS 时间时采样并发送；基线期确认 mixer 回零；结束归零；FAULT 时仅有限零补发及诊断 |
| 同上 | publishCommand(command,stamp)→Status | 封装并发布两类消息，记录发送值/时间；publish 不表示送达确认；异常结束试验 |
| 同上 | stopOutput(nextStatus,reason) | 放弃非零序列，立即发全零并启动有限 wall 补发，保存原因；故障归零跳过正常脉冲下降段 |
| 同上 | onStop(request,response)、onResetFault(request,response) | stop 调 stopOutput，不能解除 FAULT；reset 等零补发结束后清状态/时间、adapter.reset，回 WAIT_STATE |
| 同上 | publishDiagnostics() | 阶段变化即时、常态降频发布配置/命令/时效/相对运动/原因；rosbag 保留原始话题，不用字符串诊断替代原始状态 |
| probe_main.cpp | main(argc,argv) | init→单线程 spin→正常退出前尽力归零→shutdown；被强杀不能保证退出处理会执行 |
| test/test_hippo_command.cpp | C++ 计算/消息测试 | 已知时间的脉冲值、不同轴、结束全零、ignore 标志、NaN/越限、q/-q、运动/时间异常 |
| test/test_actuation_probe.cpp | C++ ROS 接口测试 | 未 start 无非零输出、有限时长、停止/暂停/过期/坏状态/多源、故障锁存、复位不重启 |
| rov_bringup/launch/actuation_probe.launch | XML：已有仿真时仅启动 probe | 参数 vehicle_name、channel、amplitude、config_file；节点名 actuation_probe，无额外 mixer/姿态节点 |
| rov_bringup/launch/hippo_actuation_probe.launch | XML：include hippo_sim + actuation_probe | 一套仿真和 probe；start_gui 可选；原 B 入口保留 |
| rov_bringup/config/actuation_probe.yaml | 配置初始试验/监测参数 | 只含仿真值，说明每个阈值用途，不写真实设备标定 |

已修改 ROS 包 CMakeLists/package.xml，增加支持库、程序、测试和 ustrov_control_msgs 依赖；C1 不链接姿态控制库。bringup 安装新配置/launch。继续使用 build/batch_b、install-b，目录名不限制新增内容。实际可执行步骤、逐条命令解释及失败记录集中在 README §9。

### 11.6 当前参数与验证方法

以下是经过有限输入试验修订的仿真配置，不是 PD/MPC 增益或真机标定：

- 单次一个通道：先 0，再 0.4；surge/roll/pitch 使用 ±0.6，yaw 使用 ±0.55；默认effort_limit为0.65，可在YAML配置至归一化接口边界1。上述数值是原试验记录，不自动递增到最大命令。
- ROS 时间：全零基线 1 s，上升 0.1 s、保持 0.1 s、下降 0.1 s，随后全零观察 1 s，总计 2.3 s。初次候选保持 0.3 s 导致 pitch/yaw 角速度超限，故缩短；yaw 短脉冲 ±0.6 仍触发转角保护，故降低该轴幅值。保护阈值未放宽；故障/手动停止立即归零。
- wall 周期候选 20 ms；状态 ROS 年龄上限 0.10 s、steady 接收间隔 0.15 s、未来容差 0.05 s、单次正 ROS 增量上限 0.10 s。启动需看到时钟前进；故障零补发 0.5 s wall 后停止发送。接通时根据实测校验阈值，变更说明原因并记录。
- 起始线速度 <0.03 m/s、角速度 <0.10 rad/s、倾斜 <5°；活动期间相对位移 <0.10 m、相对转角 <10°、线速度 <0.20 m/s、角速度 <0.50 rad/s，并位于明确配置的活动盒。浮力导致条件不满足时等待或终止，不能将漂移直接归因于脉冲。
- 正/反输入从满足启动条件的状态开始，必要时重启整套场景；记录基线漂移和沿初始机头的速度/位移、机体系角速度。只有 mixer 消息不计为运动方向通过。
- 测试前核对实际 mixer 参数和加载模型，与源码推导对应；试验期间不热改 mixer。ROS 图检查不能证明混控参数相同。

验证顺序：

1. 构建、C++ 数学/消息/接口测试；先证明无 start 无非零输出、脉冲必结束、错误归零。
2. 独立仿真实例中确认唯一命令源和模型/参数；零输入验证通路与记录。
3. 先 surge，再 roll/pitch/yaw，分别比较 0.4 和 ±0.6 的有限脉冲；监测超限即终止并保留失败记录。
4. 验证 stop、暂停、状态中断、时间倒退、错误 frame、多命令源。零命令后惯性/浮力运动仍可能存在，不能以位置立即静止作为归零判据。
5. rosbag 保存 /clock、odometry、两类 setpoint、thruster_command、probe diagnostics，逐次新建忽略的 log 子目录；正式 C++ 测试源码保留。
6. 交付讲解：改动文件→每周期数据流→构建/测试/启动/服务/记录命令及每个参数含义→实测结果和边界。未实施命令不写成已经可运行。

C1 没有加入死区补偿；后续水平闭环已按 §5.3 对原模型做静态曲线逆映射，并通过实际 Gazebo 验证。它不等于真实推进器标定。MPC 仍需建模实际输入/响应关系；真机需独立状态/执行适配和看门狗。

## 12. 真机迁移与透明水缸视觉方案（视觉未实施；10-07逐电机工具见§12.10）

### 12.1 系统目标与已知条件

先实现真实传感器驱动的小范围水平闭环：发布相对本次任务 home 的 XY 目标，由视觉、IMU、深度提供有效状态，在实际控制权限允许时沿用当前路径/时间律/跟踪思想。视觉直接使用真实水缸成像链，不增加视觉仿真前置条件。第一批仅定位与数据记录，不启用推进器。MPC仍未实现，不把换状态来源称为完成MPC。

用户确认：Pi 4B、Pixhawk 6C、Ubuntu 24.04对应ROS 2（按Jazzy规划，未读取设备）、深度计接Pixhawk、无DVL。四推进器在侧面且与Hippo相近；未有精确几何和推力曲线。已有HSK USB单目相机，传图正常；用户提供商品规格为500万IMX335、2.8mm、120°、“无畸变”、IP66、2m线，真实投影模型/近焦/帧率/浸水能力未核验，不再直接断言是鱼眼。透明水缸1.5×0.5×0.8m，暂按长宽高，内/外尺寸和实际水深未知；尺寸限制见§12.8。用户现已选择相机固定在ROV上，按下视/缸底标签设计。

### 12.2 当前代码能复用到哪一步

| 模块 | 已有事实 | 迁移工作 |
|---|---|---|
| LinePathPlanner / QuinticTrajectory / HomeFrame | C++几何、时间律、软件原点已完成 | 保留算法，重设实际空间与运动限制；home只在定位有效后建立 |
| PlanarTracker | 前向力、姿态力矩，横向误差通过转向修正 | 用真实质量/阻尼/增益；核验需要的Fx/Mx/My/Mz是否可达，不能用四电机数量替代能力检查 |
| HippoAllocator | 固定Hippo安装几何、反作用力矩与推力曲线 | 真机另做可配置分配，实测方向/力臂/曲线/饱和，原仿真类保留 |
| PointTrackingNode | 当前loadConfig强制use_sim_time=true，状态来自仿真Odometry | 不删除保护来冒充迁移；另建真机入口，复用计算层，补定位健康和解锁握手 |
| USTROV PX4 bridge | ActuatorControls→ActuatorMotors，显式arm/stop、prestream、命令超时、通道映射，direct_actuator=true | 校验固件/px4_msgs、实际PWM输出功能、可逆ESC和未装通道；避免叠加两次电机符号 |
| USTROV sensor bridge | SensorCombined FRD→FLU；SensorBaro压力转发；启动时估计时间偏移 | 确认外部水压计驱动/实例/device_id和时间对齐，现有代码未按device_id筛选 |
| USTROV C++ EKF | SetImuData/SetBaroData/SetVisionData及融合代码和可链接ekf库 | 先回放审核，再包装统一Odometry；补同一时刻的状态/协方差及健康契约 |
| Hippo visual_localization | 上游AprilTag36h11，Python视觉EKF、C++PX4 bridge；本地无该包 | 借鉴标签地图与外参变换，采用C++检测/几何；不原样复制Python滤波和桥 |

证据范围：USTROV core README记载Armed/Offboard与停止命令后约318ms上锁，本轮仅阅读，未现场复测。Gazebo已有结果是真值闭环。`Estimator::PublishPose()`协方差仍为零占位，`OnImuWatchdog()`复位TODO、`UpdateDeadreckoningStatus()`空实现；不能凭节点有输出就认定定位健康。`PublishState()`取延迟融合状态而打当前IMU时间，不能与当前速度/姿态任意拼接。

当前本地point_tracking.yaml已含速度0.8m/s和加速度0.6m/s²等用户调参，全部仍为仿真配置；真机不直接加载。参数已支持部分运行时更新，不能继续沿用早先“一律不支持热改”的描述。

### 12.3 两种相机布置与建议

| 项目 | 机载下视，缸底固定标签 | 缸外固定，机器人携带标签 |
|---|---|---|
| 必须固定的几何 | 各标签在map中的位置/方向；相机到机体外参 | 相机在map中的外参；标签板到机体外参 |
| 工程代价 | 相机防水窗口、安装/线缆；近底视场和可见标签覆盖 | 刚性支架，隔缸壁标定；机体转动时避免标签被遮挡 |
| 对当前USB相机 | 需确认防水；完整湿光路标定 | 可先不改防水，验证定位；仍存在空气/玻璃/水折射 |
| 迁移价值 | 接近后续机载定位，限于有标签覆盖区域 | 适合室内控制试验，依赖外部设施/通信 |

2026-10-06用户已选择机载相机，替代此前缸外优先的试验建议。后续围绕刚性下视安装、最终防水窗口、缸底已知标签地图开展只读定位验收。缸外布置仅保留为比较资料，不是当前首阶段任务。用于反馈的同一视觉输出不算独立真值。

### 12.4 相机模型、标签与坐标契约

1. 先核相机输出是原始图还是内部矫正图，固定分辨率/焦距/曝光，保持Image与CameraInfo一致。“120°/无畸变”不是镜头模型证明；按标定和独立误差检验选择针孔加畸变或fisheye模型。若采用fisheye，先生成合适视场的虚拟针孔图，再检测/PnP；虚拟K与零畸变供后级使用，不能将原鱼眼D套给普通针孔PnP。去畸变映射表初始化时缓存。
2. 默认候选AprilTag36h11以对齐Hippo，若用户已印ArUco则配置正确字典；不能混用图案和字典。测量实际标签边长，固定到平整、防水、哑光硬板。多标签几何可提高覆盖与冗余，但不是免除平面位姿歧义/折射的保证。
3. 确认最终空气/窗口/水光路后湿标定，并用不同位置/高度/姿态的已知位移独立验证。鱼眼畸变校正不能消除平面窗口折射；若近似模型在所需区域失败，先缩小视场/工作区域或进一步采用折射模型，不能调控制增益掩盖系统定位偏差。
4. 坐标约定：W为水缸固定map、Z向上（建议水面为z=0，缸底为-H），B为机体FLU，C为光学系右/下/前。T_AB将B坐标变换到A。

机载：已知T_WM和T_BC，图像得到T_CM，计算 `T_WB=T_WM*inverse(T_CM)*inverse(T_BC)`。

缸外：已知T_WC和T_BM，图像得到T_CM，计算 `T_WB=T_WC*T_CM*inverse(T_BM)`。

单目尺度来自实测标签尺寸/地图。home再由有效T_WB固定建立；不把相机原点、标签原点和home混为一谈。深度零点与map的高度基准对齐，补压力传感器相对base_link的杆臂；只改frame字符串无效。

### 12.5 规划中的数据流与时序

~~~text
相机Image + CameraInfo → 按实际镜头模型校正 → 标签检测/已知几何PnP → 质量门控
    → vision_pose(map中的base_link，采集时刻、协方差) + 视觉健康
Pixhawk IMU → USTROV sensor bridge → imu(FLU、rad/s、m/s²)
已确认的水压实例 → pressure(Pa) → 水面零点/杆臂 → 深度约束
    → C++融合状态（候选复用USTROV ekf，经回放审核）
    → Odometry + StateHealth → StateAdapter → RobotState
goal(home) → LinePathPlanner → QuinticTrajectory → PlanarTracker
    → 物理力/力矩请求 → 真机分配/曲线 → 执行许可与限幅
    → 唯一thruster_command → USTROV actuator bridge → DDS → PX4 → ESC
~~~

所有传感器输入、观测/拒绝原因、状态健康、参考、物理请求、分配后可达力、归一化指令、PX4状态及延迟记录到rosbag。相机处理独立进程，最新帧优先、队列有界；状态节点按IMU采样时间预测，观测按采集时间融合，控制节点定时计算。初始目标视觉约20Hz、控制50Hz，均需Pi实测P95/最坏延迟再定；频率是目标不是已达到的结果。

真机use_sim_time=false；控制dt用单调时钟，观测保留源时间并统一时间基准。缸外PC相机与Pi分机时须核验时钟同步和网络延迟；同机处理也不能以处理结束时间冒充曝光时间。视觉丢失时允许估计器短时预测，但StateHealth必须降级并由控制停止；不得不断重打时间戳把旧观测伪装成新定位。

### 12.6 暂定文件与函数级接口（均未创建）

自研C++遵循现有.h/.cpp中文注释风格。新增一个`rov_localization`包隔离OpenCV/相机依赖；控制数学仍在core，运行入口在ros，配置启动在bringup。以下每个.h/.cpp成对，main仅init/spin/shutdown。

| 文件/类 | 函数与输入→输出 | 内部状态与调用关系 |
|---|---|---|
| localization/camera_geometry.h/.cpp，CameraGeometry | configure(CameraInfo,输出视场)→校正配置；rectify(Image)→图像和虚拟K | 保存标定版本/校正映射；标定或分辨率变化使旧结果失效 |
| localization/fiducial_localizer.h/.cpp，FiducialLocalizer | loadGeometry(标签尺寸/地图或机体标签外参)；detect(图像)→ID/角点；estimateBodyPose(角点,K,外参)→PoseObservation；checkQuality(观测)→通过/原因 | 保存固定几何和上一有效观测；PnP正深度、重投影、跳变/歧义检查；拒绝帧不能刷新last_valid |
| localization/fiducial_localization_node.h/.cpp | onCameraInfo缓存模型；onImage→rectify/detect/estimate/check→publishPose/publishHealth | 相机回调为唯一视觉状态写入者；只发观测，不做二次惯导融合 |
| localization/real_state_node.h/.cpp，RealStateNode | onImu→SetImuData/Update；onPressure→校零/SetBaroData；onVision→门控/SetVisionData；makeOdometry→统一时刻的p/q/v/omega；evaluateHealth；reset | 候选持有已有Ekf库；保存源时间、初始标定、有效观测年龄/质量。输出v_B=R_WBᵀv_W，omega_B使用时间对齐且正确去偏的IMU |
| core/tracking/thruster_allocator.h/.cpp，ThrusterAllocator | configure(r_i,d_i,曲线,限幅)→秩/可达轴报告；allocate(物理力旋量)→各电机N/指令/实际可达力及残差 | 几何与标定持久化；未装通道明确处理；饱和后必须报告实际可达量，不把请求冒充输出 |
| ros/hardware_tracking_node.h/.cpp，HardwareTrackingNode | onOdometry/onHealth缓存；onGoal→setGoal；onEnable请求零预流与bridge arm；onPx4Status确认模式；onTimer→health→tracker→allocator→publish；onStop归零/请求上锁 | 独立真机任务/执行许可状态；单线程executor序列化。唯一thruster_command发布者，未ARMED+Offboard/定位有效前只允许零预流；重连不自动续跑 |
| bringup/config/{camera,tank_tags,camera_extrinsics,real_vehicle,real_tracking}.yaml | 相机模型、标签几何、外参、真实推进器/控制配置 | 记录测量来源/单位；实测配置与Hippo配置分开，不提前填假标定 |
| bringup/launch/{real_sensors,vision_localization,real_state_preview,real_point_tracking}.launch | 纯XML：只读传感器→视觉→状态预览→真机控制逐级启动 | 首批只实现到只读定位；真机控制入口需后续确认。不可include默认core.launch同时又启动自己的分配器，因为前者还会启动原mixer |

拟用数据结构：PoseObservation保存stamp、T_WB、6×6测量协方差、标签数/重投影误差/拒绝原因；StateHealth保存初始化、最近有效视觉/压力/IMU时间、协方差/融合拒绝、时间重置和可控制轴。配置只读标定项，运行增益更新策略另批明确。

USTROV EKF存在延迟输出与协方差导出问题，包装不是自动修复：先用真实录包验证时间、静态漂移、坐标、初始化/丢标行为，补所需接口；若须改上游库，另提最小补丁或替代估计器设计并确认，不本轮私自改子模块。初版优先选择树莓派侧一条融合链；是否改用PX4 EKF融合视觉要等水压支持和固件版本确认，不同时堆两套滤波且把相关输出当独立观测。

### 12.7 水缸实验分阶段验收

1. **只读硬件核验**：读取OS/ROS/固件与DDS版本、IMU/压力设备。使水压探头产生已知10cm浸深差，淡水应约增加981Pa；检查device_id、单位和压力实例。这不要求移动整个未验密封的机器人，也不启动电机。
2. **机载相机定位台**：相机刚性安装且暂不启用推进器，缸底固定标签，水缸外贴尺和固定参考点。先空气后最终湿光路；独立记录已知5/10/20cm位移与旋转、中心/边缘和不同高度，统计尺度/静态抖动/延迟/丢标。具体距离受水缸尺寸限制；校准点和验证点分开。
3. **无动力整机状态**：手动平移/转动、改变深度，比较视觉与融合Odometry。主动遮挡标签/断图验证健康降级；启动/重启不能先用未初始化零位姿建立home。录包回放不发布执行命令。
4. **推进器台架与分配**：先断动力验证输出通道/解锁/断链，再用水中固定测力装置测正反推力、死区、电压和可达力矩。核实roll反作用力矩是否可用，不能默认Hippo系数适配。
5. **受限首闭环**：依据实际Fx/Mz等权限选择直线/航向→小幅XY；确认浮力/姿态稳定和离底可视条件。没有Fz权限时，深度计只观测，不能声称定深。运动限幅按测力/净空/定位误差决定，不能直接复制当前仿真0.8m/s。标签丢失/时间异常/断通信→零命令与上锁策略必须实测；零推力不等于立刻静止或定深。

水缸中心点活动范围需要扣除机体包络、转弯扫掠、定位误差和停止距离。建议先关注几厘米到十几厘米的位移，但尺寸/制动能力未知前不冻结数值。外部同源视觉只是反馈；后续定量控制评价使用独立标尺定位工装或独立标定的第二观测通道。缆绳张力和池壁回流应记入试验条件。

本轮只读核查与设计记录，没有新增实现/测试脚本，没有改USTROV，没有连接/解锁真机，也没有新的运行验证结果。

### 12.8 水缸尺寸对第一阶段的约束（2026-10-06补充）

用户提供1.5×0.5×0.8m，暂按长宽高；0.8m为缸高，实际水深另测。以下是基于旧机体约0.4×0.3m的条件估算，不作为当前真机净空验收：

- 长轴对齐时，若缸宽为净宽0.5m，左右各约0.1m，尚未扣推进器突出、标签、线缆和定位误差。
- 矩形机体偏航角为theta时，横向包络为 `0.4*abs(sin(theta))+0.3*abs(cos(theta))`。45°时约0.495m，最大值0.5m；完整掉头几乎没有侧向余量。实际圆筒/推进器构型应测真实包络，不能把这个矩形近似当已测几何。
- 当前首阶段按用户选择采用机载下视、缸底固定标签；手动沿长轴移动，先验证5/10cm等已知位移。应检查最低离底高度时的近焦、校正后可见范围、窗口折射和姿态歧义。此前缸外优先建议已被替代；下视安装不消除水缸宽度与转向包络限制。
- 首轮控制目标收窄为纵向低速短程和小角度航向修正，距离/限幅待实测；不直接安排斜向/身后目标或原地180°转向。
- 现有`PlanarTracker::update()`在制动后以`atan2(delta.y(),delta.x())`设置航向并进入TURNING。虽然允许负Fx制动，仍不等同固定航向倒车规划。若要长轴往返，需要另设计有符号纵向参考、倒推曲线/控制和横向越界终止；不得直接发布负home-X验证倒退。继续遵循先函数设计、确认后编码。
- 侧壁间距小，壁面/回流影响需记录；这里测得的动力学参数不能直接作为开阔水域参数。没有Fz权限时，水深测量不能保证保持视野或不触底，浮力/姿态稳定需先确认。

本次只更新场地事实和规划，没有更改控制器或运动参数。

### 12.9 用户指定论文代码、标签制作与标定（2026-10-06补充，未实施）

用户参考IEEE文献9341051，题名为Towards Micro Robot Hydrobatics: Vision-based Guidance, Navigation, and Control for Agile Underwater Vehicles in Confined Environments，IROS2020。本次只取得官方出版/会议元信息，未读论文全文；以下实现判断来自实际读取的[作者代码](https://github.com/DanielDuecker/microAUV-localization)，不据论文摘要推断性能。

- 旧代码是catkin/rospy，`start_localization.launch`串联usb_cam→image_proc→apriltag_ros→ekf_node.py→MAVROS，字典配置在外部apriltag_ros包，不能从此仓库冻结其family。
- `calibration_tags.py::callback_april()`保存相机看到的标签位置；`callback_gantry()`结合龙门架位置及固定旋转累积世界标签坐标；结束后写CSV。它不是镜头内参标定，作者的calibration_tank.csv不能用于本水缸。
- `ekf_node.py::callback()`取标签相对位移的范数作为距离观测，并含1.05/1.1经验尺度系数；`ekf_class.py`状态为[x,y,z,机体前向速率]，姿态由另一路更新、角速度用于预测，输出姿态存在仅yaw构造。不能把它称为通用六自由度VIO或直接照搬经验参数。
- 当前工程采用已知多标签角点的几何位姿观测→独立融合状态。AprilTag36h11作为新方案候选，以便和较新Hippo视觉对齐；OpenCV ArUco字典也可用，但生成/检测/尺寸约定必须一致。标定板建议ChArUco或棋盘格，与运行定位标签可分别选择。

三类标定必须区分：

| 标定 | 产出 | 工具/方法 |
|---|---|---|
| 相机内参及畸变 | K(3×3，焦距/主点为像素)、D、图像尺寸、模型名 | 复用OpenCV C++ camera_calibration / interactive calibration / ChArUco示例；若确属fisheye，调用其专用标定与校正API。发行版未必安装示例可执行文件，实施时核版本与构建 |
| 相机相对机体 | T_BC(平移m、旋转) | 测量刚性安装外参，再用已知姿态/位置工装复核；相机内参软件不会自动给出该外参 |
| 固定标签地图 | ID、标签尺寸m、各角点W坐标或T_WM | 在平整硬板按已知布局制作并实测，定义W为沿缸长/缸宽/Z向上；压力零点另做高度变换。无需作者龙门架即可建立初版规则地图 |

先固定最终窗口、分辨率、焦距/数字裁切；空气中验证工具链，再在最终水中光路采集不同位置、倾角、距离的标定图像，不能只平行平移一张板。湿标定为工程近似，平面折射不一定能由单一K/D精确描述。用未参与拟合的已知位移、中心/边缘和不同高度验收；RMS重投影误差小不等于水下尺度准确。

水缸板的初始试样可选8cm边长、不同ID的标签，先少量测试，再根据实际离底高度/可用像素布成两排长条地图。例：2×8个、中心间距14cm，图案包络约1.06×0.22m，四周另留白；仅为布局草案，不保证当前镜头能同时看到足够标签。黑色外边长作为所选AprilTag36h11检测尺寸时须和检测库定义一致，外部白边不混入尺寸；打印关闭缩放、实测尺寸、防反光并保持平整。IP66不能证明长期浸水能力，须明确壳体/窗口/出线密封；商品“250–6000mm”不作为镜头焦距，近焦能力需实测。

首个可运行视觉交付只到Image/CameraInfo→标签角点→T_WB及质量，不接电机。沿用§12.6的CameraGeometry/FiducialLocalizer/Node设计，标定计算调用现成C++库，不另写一套优化器。

### 12.10 树莓派逐电机开环：有限脉冲实现（2026-10-07）

现有链为`ActuatorControls.control[8] → Px4ActuatorBridge::OnThrusterCommand → 50Hz OnOutputTimer → PublishMotorOutput → /fmu/in/actuator_motors → XRCE-DDS → PX4 → 已配置电机输出 → ESC`。开环不需要相机、DVL、Odometry或路径跟踪器；桥本身仍依赖VehicleStatus、显式arm和命令新鲜度。

- 输入是无量纲[-1,1]，不是N、RPM或PWM微秒。`control[0]`仅在当前映射下对应PX4逻辑Motor1；物理PWM口还取决于PX4输出功能。`motor_input_indices[motor]`选择输入，`motor_signs[motor]`再乘符号。8槽不代表有8台推进器。
- 默认配置：输出50Hz，命令300ms未更新触发上锁，预流1200ms。OnThrusterCommand按接收now()计新鲜度，不用header.stamp；拒绝NaN/Inf并裁剪有限值至[-1,1]。输入字段可省header；8元素数组保留。
- `OnArmService()`先检查并进入PRESTREAM，返回success只表示接受流程。桥持续发零→请求Offboard→等待实际Offboard→请求Arm→等待实际Armed→ACTIVE才转发非零。`direct_actuator=true`意味着不自动提供PX4姿态/深度内环。
- `OnStopService()`切DISARMING，全部输出NaN并请求普通Disarm；源码输入层拒绝NaN，不能由上游直接发NaN模拟停止。零指令的实际中位输出、负向命令均需可逆ESC及PX4配置匹配。
- 原`core.launch.py`还启动mixer，会成为thruster_command的另一个源，不能与逐电机发布者并用。使用桥单节点或仅硬件桥的外部launch；新增自有启动仍XML .launch。仿真C1的actuation_probe是Fx/力矩输入通路，不直接复用为真机单电机试验。
- 串口/波特率尚未核验；core.launch默认/dev/fcu_data、921600，而core README示例/dev/ttyPX4、115200，不能把任一当用户已确认接线。固件px4_msgs、VehicleStatus的_v1后缀、消息DDS暴露、四台实际输出/方向/ESC模式逐项只读核实后，再断动力核对解锁与输出。

2026-10-07用户确认函数设计并要求执行，已按下表增加C++有限脉冲节点，避免手工切换publisher造成300ms断流；复用原PX4桥，不修改子模块。视觉实现仍待后续。

| 新增文件 | 类/函数、输入输出与状态 |
|---|---|
| rov_control_ros/include/rov_control_ros/motor_probe_node.h、src/motor_probe_node.cpp | MotorProbeNode；loadConfig()校验启动参数；makeCommand(config,elapsed)纯计算8维梯形脉冲；onPx4Status()缓存模式/解锁；onBridgeOutput()记录实际桥输出及全NaN帧；onStart()请求已有bridge arm；onArmReply()只表示服务接受；onTimer()在PX4 Armed/Offboard与新鲜非全NaN桥输出齐备后进入RUNNING计脉冲；onStop()/beginStop()/requestBridgeStop()归零并异步上锁。阶段、配置、消息缓存归本节点的单线程回调修改 |
| rov_control_ros/src/motor_probe_main.cpp | rclcpp init/spin/shutdown；单线程executor，状态仅由上述串行回调修改；启动默认IDLE不解锁，不自动重试非零脉冲 |
| rov_bringup/config/motor_probe.yaml | ramp_s/hold_s/output_rate_hz/arm_timeout_s/bridge_output_timeout_s及服务名；channel/amplitude由launch传入，幅值缺省0。参数启动只读，改试验设置时先结束并重启；不填猜测的真机方向 |
| rov_bringup/launch/motor_probe.launch | XML，启动唯一probe与已有bridge可执行程序，start_bridge=false可复用已有实例；不启动原core.launch中的mixer或额外传感器节点。Agent另按实际串口配置启动，不猜波特率。支持DDS前缀、状态话题及bridge_config_file |
| rov_control_ros/test/test_motor_probe.cpp | 正式C++测试：纯波形边界；模拟PX4状态/VehicleCommand响应，启动实际USTROV桥程序，检查预流/拒绝解锁/有限脉冲/停止/冲突/断流。隔离域124、localhost，不用Agent或真实飞控 |

运行阶段为IDLE→ARMING→RUNNING→STOPPING→FINISHED/FAULT。每周期输入PX4状态快照、桥输出时效、配置与单调时间，输出8维command与diagnostics；原始输入/桥输出/VehicleCommand/VehicleStatus通过rosbag关联。图发现约5Hz检查唯一命令源，不能代替跨进程锁；输出约50Hz。桥内部状态没有发布接口，因此不声称读取到了它的ACTIVE枚举；以arm服务接受、PX4 Armed/Offboard和连续有效桥输出建立运行条件。桥可能还需一个tick进入ACTIVE，首个脉冲采样为零。

STOPPING取得stop接受、停止请求之后的全NaN桥输出和PX4已上锁状态后才进入FINISHED或FAULT；没有确认则继续零输入/每秒重试stop。FINISHED可显式start新试验；FAULT需检查并重启。结束后出现迟到的Armed状态会再次请求stop，不自动恢复脉冲。停止确认只代表协议观察，不能证明电机机械静止。Ctrl+C/SIGTERM在ROS仍有效时最多继续spin两秒；强杀则由原桥300ms命令看门狗和PX4失联策略处理，实际固件仍待验收。

CMake选项ROV_BUILD_PX4_MOTOR_PROBE默认OFF，显式ON才要求px4_msgs并编译硬件工具，避免原仿真环境被硬件依赖阻塞。运行参数采用启动只读策略，区别于原point_tracking的动态参数；不混用两者调参方法。所有回调串行；异步服务future不在executor中同步等待，退出/重试清理挂起请求。

验证顺序：纯C++波形/ROS与实际桥协议测试→真机断动力验链→水中固定推进器短脉冲、方向/死区→测力装置标定正反推力。本批实际编译/运行证据和可执行步骤集中于README§11；没有真机接入、PWM/ESC和水动力验证。原仿真C1/到点实现保持独立入口。

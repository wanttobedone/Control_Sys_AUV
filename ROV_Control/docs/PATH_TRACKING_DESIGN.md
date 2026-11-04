# 路径规划与轨迹跟踪：从水平到点到 MPC

更新：2026-11-03。视觉原型实验的现行顺序与下一批函数设计见 §13.13；旧静止/尺量方案是可选检查，不再作为 ROS 接入的前置门槛。参数与检查职责见 §6.1。A/B/C1 及水平 XY 到点仿真闭环已实现，深度仅监测。本文 §1–8 是当前实现；§9–11 保留预览和有限输入入口的具体设计。真机闭环尚未实现，不由此次视觉设计自动授权执行电机。

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

## 12. 真机迁移与透明水缸视觉历史方案（当前视觉实现见§13，逐电机工具见§12.10）

### 12.1 系统目标与已知条件

当前目标按2026-10-07用户反馈收敛为：硬件与逐电机核验→IMU/压力只读状态→条件允许时小幅角速度闭环→机载视觉状态→固定航向的纵向短程PD→局部运动辨识→一维MPC。完整逐台测力改为可选，不作为首闭环前置；逐步完成标志集中README§11.12。后续可扩大为相对home的XY到点，但当前窄缸不安排任意转向。视觉直接使用真实水缸成像链，其首批仅定位与记录、不启用推进器。MPC仍未实现，不把换状态来源称为完成MPC。

用户确认：Pi 4B、Pixhawk 6C、Ubuntu 24.04对应ROS 2（按Jazzy规划，未读取设备）、深度计接Pixhawk、无DVL。四推进器在侧面且与Hippo相近；未有精确几何和推力曲线。已有HSK USB单目相机，传图正常；用户提供商品规格为500万IMX335、2.8mm、120°、“无畸变”、IP66、2m线，真实投影模型/近焦/帧率/浸水能力未核验，不再直接断言是鱼眼。透明水缸于2026-10-19确认内净长1.35m、宽0.65m，旧0.8m为缸高描述，实际水深仍未知；尺寸限制见§12.8。用户现已选择相机固定在ROV上，按下视/缸底标签设计。

### 12.2 当前代码能复用到哪一步

| 模块 | 已有事实 | 迁移工作 |
|---|---|---|
| LinePathPlanner / QuinticTrajectory / HomeFrame | C++几何、时间律、软件原点已完成 | 保留算法，重设实际空间与运动限制；home只在定位有效后建立 |
| PlanarTracker | 前向力、姿态力矩，横向误差通过转向修正 | 首轮真机另设计固定航向、有符号纵向参考的归一化指令控制器；原类保留仿真及未来物理控制参考，不能将其N级输出直接当电机指令 |
| HippoAllocator | 固定Hippo安装几何、反作用力矩与推力曲线 | 原仿真类保留；首轮真机另设计经验混控，核验真实通道/方向/组合响应/饱和。准确N/N·m分配和逐台测力作为后续可选路线 |
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
2. 使用与实物一致的AprilTag36h11字典；若换ArUco则配置对应字典，不能混用。测量实际标签边长，按10-19用户选择直接固定到缸底，保持标签平整、不移位和清晰白边；不要求额外底板。多标签几何可提高覆盖与冗余，但不是免除平面位姿歧义/折射的保证。
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
goal(home长轴) → 有符号纵向参考/五次时间律 → 纵向PD(后换MPC) + 航向控制
    → 无量纲纵向/偏航指令 → 经核验的经验混控 → 执行许可与限幅
    → 唯一thruster_command → USTROV actuator bridge → DDS → PX4 → ESC
~~~

所有传感器输入、观测/拒绝原因、状态健康、参考、无量纲请求、实际发送电机数组、限幅标记、PX4状态及延迟记录到rosbag。只有以后启用物理分配模型时才记录其估计可达力，不能把无量纲输入标作牛顿。相机处理独立进程，最新帧优先、队列有界；状态估计保留采样时间，控制节点定时计算。初始目标视觉约20Hz、控制50Hz，均需Pi实测P95/最坏延迟再定；频率是目标不是已达到的结果。归一化控制/混控的新接口须在实施前补完整函数级设计，下面旧物理路线表不能直接当作该批已确认设计。

真机use_sim_time=false；控制dt用单调时钟，观测保留源时间并统一时间基准。缸外PC相机与Pi分机时须核验时钟同步和网络延迟；同机处理也不能以处理结束时间冒充曝光时间。视觉丢失时允许估计器短时预测，但StateHealth必须降级并由控制停止；不得不断重打时间戳把旧观测伪装成新定位。

### 12.6 暂定文件与函数级接口（均未创建）

本节保留整体真机分层；2026-10-07首批机载视觉的具体实施范围、文件、回调及验收以§13为准，融合与执行入口仍属后续批次。

自研C++遵循现有.h/.cpp中文注释风格。新增一个`rov_localization`包隔离OpenCV/相机依赖；控制数学仍在core，运行入口在ros，配置启动在bringup。以下每个.h/.cpp成对，main仅init/spin/shutdown。

| 文件/类 | 函数与输入→输出 | 内部状态与调用关系 |
|---|---|---|
| localization/camera_geometry.h/.cpp，CameraGeometry | configure(CameraInfo,输出视场)→校正配置；rectify(Image)→图像和虚拟K | 保存标定版本/校正映射；标定或分辨率变化使旧结果失效 |
| localization/fiducial_localizer.h/.cpp，FiducialLocalizer | loadGeometry(标签尺寸/地图或机体标签外参)；detect(图像)→ID/角点；estimateBodyPose(角点,K,外参)→PoseObservation；checkQuality(观测)→通过/原因 | 保存固定几何和上一有效观测；PnP正深度、重投影、跳变/歧义检查；拒绝帧不能刷新last_valid |
| localization/fiducial_localization_node.h/.cpp | onCameraInfo缓存模型；onImage→rectify/detect/estimate/check→publishPose/publishHealth | 相机回调为唯一视觉状态写入者；只发观测，不做二次惯导融合 |
| localization/real_state_node.h/.cpp，RealStateNode | onImu→SetImuData/Update；onPressure→校零/SetBaroData；onVision→门控/SetVisionData；makeOdometry→统一时刻的p/q/v/omega；evaluateHealth；reset | 候选持有已有Ekf库；保存源时间、初始标定、有效观测年龄/质量。输出v_B=R_WBᵀv_W，omega_B使用时间对齐且正确去偏的IMU |
| core/tracking/thruster_allocator.h/.cpp，ThrusterAllocator（后续可选物理路线） | configure(r_i,d_i,曲线,限幅)→秩/可达轴报告；allocate(物理力旋量)→各电机N/指令/估计可达力及残差 | 首轮不实施；物理精度确有需要时再标定与设计，不能把请求或模型估计冒充实测 |
| ros/hardware_tracking_node.h/.cpp，HardwareTrackingNode（需按新主线细化） | onOdometry/onHealth缓存；onGoal→有符号参考；onEnable请求零预流与bridge arm；onPx4Status确认模式；onTimer→health→归一化控制→经验混控→publish；onStop归零/请求上锁 | 独立真机任务/执行许可状态；单线程executor序列化。唯一thruster_command发布者，未ARMED+Offboard/定位有效前只允许零预流；重连不自动续跑；不能直接复用原PlanarTracker的N级输出 |
| bringup/config/{camera,tank_tags,camera_extrinsics,real_vehicle,real_tracking}.yaml | 相机模型、标签几何、外参、真实推进器/控制配置 | 记录测量来源/单位；实测配置与Hippo配置分开，不提前填假标定 |
| bringup/launch/{real_sensors,vision_localization,real_state_preview,real_point_tracking}.launch | 纯XML：只读传感器→视觉→状态预览→真机控制逐级启动 | 首批只实现到只读定位；真机控制入口需后续确认。不可include默认core.launch同时又启动自己的分配器，因为前者还会启动原mixer |

拟用数据结构：PoseObservation保存stamp、T_WB、6×6测量协方差、标签数/重投影误差/拒绝原因；StateHealth保存初始化、最近有效视觉/压力/IMU时间、协方差/融合拒绝、时间重置和可控制轴。配置只读标定项，运行增益更新策略另批明确。

USTROV EKF存在延迟输出与协方差导出问题，包装不是自动修复：先用真实录包验证时间、静态漂移、坐标、初始化/丢标行为，补所需接口；若须改上游库，另提最小补丁或替代估计器设计并确认，不本轮私自改子模块。初版优先选择树莓派侧一条融合链；是否改用PX4 EKF融合视觉要等水压支持和固件版本确认，不同时堆两套滤波且把相关输出当独立观测。

### 12.7 水缸实验分阶段验收

1. **只读硬件核验**：读取OS/ROS/固件与DDS版本、IMU/压力设备。使水压探头产生已知10cm浸深差，淡水应约增加981Pa；检查device_id、单位和压力实例。这不要求移动整个未验密封的机器人，也不启动电机。
2. **机载相机定位台**：相机刚性安装且暂不启用推进器，缸底固定标签，水缸外贴尺和固定参考点。先空气后最终湿光路；独立记录已知5/10/20cm位移与旋转、中心/边缘和不同高度，统计尺度/静态抖动/延迟/丢标。具体距离受水缸尺寸限制；校准点和验证点分开。
3. **无动力整机状态**：手动平移/转动、改变深度，比较视觉与融合Odometry。主动遮挡标签/断图验证健康降级；启动/重启不能先用未初始化零位姿建立home。录包回放不发布执行命令。
4. **推进器核验与经验混控**：先断动力验证输出通道/解锁/断链，再用水中固定的单推进器短脉冲确认方向、零点、大致启动死区和停止。核验小幅组合响应及其他轴耦合；不要求先测牛顿级曲线，不假定Hippo的roll反作用力矩系数适配。
5. **受限首闭环**：先依据实际偏航权限、IMU质量和姿态/浮力条件做小幅机体z角速度PI；视觉有效后做固定航向纵向短程PD，不直接扩大到任意XY。没有Fz权限时，深度计只观测，不能声称定深。运动限幅按实际响应/净空/定位误差决定，不能直接复制当前仿真0.8m/s。标签丢失/时间异常/断通信时的停止策略须实测；零推力不等于立刻静止或定深。以上条目按验证对象分组，实际优先顺序以README§11.12为准。

水缸中心点活动范围需要扣除机体包络、转弯扫掠、定位误差和停止距离。建议先关注几厘米到十几厘米的位移，但尺寸/制动能力未知前不冻结数值。外部同源视觉只是反馈；后续定量控制评价使用独立标尺定位工装或独立标定的第二观测通道。缆绳张力和池壁回流应记入试验条件。

本轮只读核查与设计记录，没有新增实现/测试脚本，没有改USTROV，没有连接/解锁真机，也没有新的运行验证结果。

### 12.8 水缸尺寸对第一阶段的约束（2026-10-06补充）

2026-10-19用户确认水缸内净长1.35m、宽0.65m，替代此前1.5×0.5m的长宽估计；旧0.8m为缸高描述，实际水深仍需另测。以下是基于旧机体约0.4×0.3m的条件估算，不作为当前真机净空验收：

- 长轴对齐且居中时，按0.65m净宽与0.3m机宽估算，左右各约0.175m，尚未扣推进器突出、标签、线缆和定位误差。
- 矩形机体偏航角为theta时，横向包络为 `0.4*abs(sin(theta))+0.3*abs(cos(theta))`。45°时约0.495m，最大值0.5m；在0.65m缸宽内若中心固定，最大包络时理论上两侧各约0.075m。不再沿用旧0.5m缸宽下“完整掉头几乎没有余量”的结论；也不据此宣称实际掉头已可行。实际圆筒/推进器构型应测真实包络，不能把这个矩形近似当已测几何。
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
- 当前工程采用已知多标签角点的几何位姿观测→独立融合状态。使用实物AprilTag36h11；生成/检测/尺寸约定必须一致。2026-10-21明确使用现有16张缸底标签兼作内参标定目标，不再要求独立ChArUco/棋盘格；原棋盘格命令已撤回，C++标定采样/几何配对适配待实现。

三类标定必须区分：

| 标定 | 产出 | 工具/方法 |
|---|---|---|
| 相机内参及畸变 | K(3×3，焦距/主点为像素)、D、图像尺寸、模型名 | 复用OpenCV C++ camera_calibration / interactive calibration / ChArUco示例；若确属fisheye，调用其专用标定与校正API。发行版未必安装示例可执行文件，实施时核版本与构建 |
| 相机相对机体 | T_BC(平移m、旋转) | 测量刚性安装外参，再用已知姿态/位置工装复核；相机内参软件不会自动给出该外参 |
| 固定标签地图 | ID、标签尺寸m、各角点W坐标或T_WM | 直接固定在缸底并实测几何，定义W为沿缸长/缸宽/Z向上；压力零点另做高度变换。无需作者龙门架或额外底板即可建立初版地图 |

先固定最终窗口、分辨率、焦距/数字裁切；空气中验证工具链，再在最终水中光路采集不同位置、倾角、距离的标定图像，不能只平行平移一张板。湿标定为工程近似，平面折射不一定能由单一K/D精确描述。用未参与拟合的已知位移、中心/边缘和不同高度验收；RMS重投影误差小不等于水下尺度准确。

水缸板的初始试样可选8cm边长、不同ID的标签，先少量测试，再根据实际离底高度/可用像素布成两排长条地图。例：2×8个、ID0～15、中心间距14cm，黑色图案包络约1.06×0.22m；每片四周各留1cm白边后，100×100mm成品整体排列包络为1.08×0.24m。16个是可放入水缸的草案，不是通过全域可见性计算得到的最少数量；此前6个是初测样品，可纳入最终地图。黑色外边长作为所选AprilTag36h11检测尺寸时须和检测库定义一致，外部白边不混入尺寸；打印关闭缩放、实测尺寸、防反光并保持平整。IP66不能证明长期浸水能力，须明确壳体/窗口/出线密封；商品“250–6000mm”不作为镜头焦距，近焦能力需实测。

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

验证顺序：纯C++波形/ROS与实际桥协议测试→真机断动力验链→水中固定推进器短脉冲、方向/死区→按README§11.12接状态和简单闭环。测力装置标定正反推力改为物理精度确有需要时的可选工作。本批实际编译/运行证据和可执行步骤集中于README§11；没有真机接入、PWM/ESC和水动力验证。原仿真C1/到点实现保持独立入口。

## 13. 机载视觉定位实施计划（分批实施：标定/只读ROS已完成，机体状态与闭环待后续）

### 13.1 目标、硬件与完成边界

用户选择相机直接浸水，预计镜头离底20～50cm。首批目标：在固定标签覆盖区域，用真实图像输出机体在固定map中的位置/姿态，附采集时间和观测质量；在无动力整机移动时完成独立尺量验证。只有这一目标通过，才进入IMU/压力融合和后续控制。

尚需准备或确认：

| 项目 | 用途与当前条件 |
|---|---|
| 相机浸水能力或浸水外壳 | 当前仅用户提供IP66规格；需允许浸深、持续时间和线缆出口密封依据。离底20～50cm不是相机距水面的浸深。若需加外壳，最终窗口一起标定 |
| 刚性下视支架、线缆固定 | 相机相对base_link位姿固定，线缆不进入视野；固定后量T_BC，不在运行中挪动 |
| 已知尺寸的标定几何 | 10-21用户明确只有16张AprilTag，直接使用缸底固定标签；测真实ID、码边长、相对位置/有序角点，不要求新增棋盘格或承载板。同一物理目标可同时服务内参标定和地图定位，但求解输入/用途不同 |
| 固定标签及缸底安装 | 包装36h11、码80mm/外100mm、ID0～16；直接贴缸底，不要求额外底板。用户报告空气中10cm图像可见，尚未检测或湿验证；按实际运行最低高度、倾角和轨迹交接处验收后冻结布局，行列间距可以不同 |
| 标尺/直角尺、位置挡块或简易刚性导向工装 | 测安装外参和地图，提供已知位移与角度核验。需控制倾斜/偏摆，不能把手持尺读数当高精度运动捕捉真值 |
| 照明、Pi供电/散热/已有有线通信 | 先使用实验室现有条件；反光/拖影或欠压/过热时针对性改善，不预先采购GPU、DVL或第二相机 |

20、35、50cm作为光学检查点，不保证在50cm离底时整个ROV仍能在0.8m缸高内完全浸没；实际水深、相机位置与机体包络要一起测量。纯视觉定位使用相机和已知标签即可，Pixhawk/IMU/压力在后续融合批次接入。

### 13.2 软件复用与分批交付

采集复用ROS2 `usb_cam`；AprilTag编码检测复用官方原生库，在自研C++类中调用；标定、去畸变与PnP复用OpenCV。自研部分负责参数/地图/坐标变换/质量和ROS输出。先在Linux PC处理录包验证，再在Pi原生Ubuntu/Jazzy构建部署并测负载；WSL用于编译与回放，不把USB设备透传作为首个相机接入前置条件。后续上Pi保持同样接口和标定模式，重新测性能。

| 阶段 | 工作与产出 | 进入下一阶段的依据 |
|---|---|---|
| V0 采集核验 | 枚举设备/输出格式/实际帧率、固定一种模式、记录分辨率与曝光策略；20cm近焦检查。依赖版本按实际Jazzy环境固定 | 连续图像、无持续积压，图案清晰；相机物理安装/浸水条件确定 |
| V1 内参标定 | 10-21已实现§13.10离线适配；lab真实录像已得1280×720候选K/D，ID16、47拟合/13留出，RMS=1.476898/2.213650px，用户本次留出门槛2.3px；详README§13.10 | 候选参数已生成，图像检查不替代独立位移；地图/T_BC/在线链未验收，重新安装窗口/调焦/裁切重核 |
| V2 地图与外参 | 固定并测量ID/边长/方向/中心或角点；量相机T_BC；建立与压力参考一致的map | 实际尺寸有记录，所有ID唯一；X/Y/Z和旋转方向可手动验证 |
| V3 几何定位 | 实现CameraGeometry/TagMap/FiducialLocalizer/ROS节点与XML启动；先静态/手动移动的空气图像回放 | 检测ID、角点对应及T_WB正确；单位/外参逆变换与异常帧处理测试通过 |
| V4 湿光路与整机验收 | 20/35/50cm、中心/边缘、短程平移、小角度转动；遮挡/断图/重连；Pi运行录包 | 报告误差、抖动、有效率/最长丢标、延迟与CPU/温度；圈定实际可用区域 |
| V5 状态融合设计与实施 | 在真实录包上核验已有USTROV EKF，设计状态输出/协方差/健康与时间契约；输出统一Odometry | 位置、姿态、速度同一时刻，丢标可识别、速度经独立检查，才供真实控制入口使用 |

V0～V4为拟确认的首批视觉实现范围；V5在真实观测具备后细化和确认。标定优化使用官方工具，不另写优化器。未标定时可以检测ID/显示角点，但不得发布冒充米制定位的有效机体位姿。

### 13.3 坐标、接口与数据流

沿用§12.4：W为固定水缸map，X沿缸长、Y沿缸宽且与Z组成右手系，Z向上，不强行对应地理东/北；以建立地图时的水面高度作z=0，标签板z=-H，H需实测并记录参考水位。B为uuv00/base_link，FLU；C为uuv00/camera_optical_frame，右/下/前。长度m、图像坐标px、内部角度rad。

多标签PnP输入世界角点P_W与图像点u,v，求解的T_CW将世界点变到相机系，因此 `T_WB=inverse(T_CW)*inverse(T_BC)`。先核已知合成几何及角点顺序，再接真实图像；不能直接把PnP的tvec当机体世界位置。标签全部共面时使用适用平面多点的方法并检查候选解；四点正方形专用IPPE_SQUARE不能直接用于整张多标签地图。

~~~text
usb_cam：Image + 同步CameraInfo
  → CameraGeometry::configure/rectify：校正灰度图 + K_rect
  → FiducialLocalizer::detect：AprilTag ID + 四角像素
  → TagMap::collectWorldCorners：对应的已知世界角点
  → FiducialLocalizer::estimateBodyPose：多点PnP → T_CW → T_WB
  → checkQuality：匹配/重投影/正深度/歧义/跳变与图像年龄
  → PoseWithCovarianceStamped + Diagnostics + 可选叠加图/RViz地图
  → rosbag：输入、输出、观测拒绝原因和耗时
~~~

| 拟定话题 | 类型与语义 |
|---|---|
| /uuv00/camera/image_raw、camera_info | sensor_msgs/Image、CameraInfo；同一图像模式与时间，K/D对应输入图；未标定标明缺失 |
| /uuv00/vision_pose | geometry_msgs/PoseWithCovarianceStamped；header.frame_id=map，内容固定为base_link位姿，stamp继承图像时间。无child_frame_id，因此该约定写入接口文档 |
| /uuv00/vision/diagnostics | diagnostic_msgs/DiagnosticArray；初始化/有效/丢标/歧义原因、有效ID数、像素误差、处理耗时、最后有效图像年龄 |
| /uuv00/vision/debug_image、tag_map | sensor_msgs/Image、visualization_msgs/MarkerArray；调试图按需低频，地图供RViz查看 |

视觉不发布速度为零的Odometry；后续融合入口输出的Odometry必须满足当前StateAdapter：map中的pose、机体系linear/angular twist。第一批只发布Pose及显示数据，后续由唯一融合节点拥有map→base_link动态TF，避免多源TF冲突。

PoseObservation包含stamp_ns、p_W[3]、q_WB[4]、covariance[6×6]、tag_count、reprojection_error_px、processing_ms、valid/reason及协方差来源标记。TagGeometry保存ID、边长m、四角P_W；CameraModel保存model、image_size、K/D及输出K_rect。ROS协方差平移单位m²、角度rad²；前期用保守可配置观测噪声并标记为假设，结合独立湿光路误差修订，不能写全零并宣称高精度，也不能只用重投影残差代表水下系统偏差。

### 13.4 在线视觉候选文件、函数与状态所有权（离线子集见§13.10）

2026-10-21复用范围澄清：下表是功能分层候选，不代表这些功能都应重写。用户要求基于Hippo/USTROV增量开发，先核上游可复用组件，再落地缺口。本地USTROV b1208e5有include外部visual_localization的launch入口，但其源码包不在该工作区；Hippo官方文档已提供image_decoder/rectifier/apriltag_node/vision_ekf链和calibration.launch.py。上游标定launch包装标准ROS棋盘格标定器，不能直接用于现有AprilTag录像。后续在线图像/检测优先使用匹配版本的现成组件，位姿/质量/融合仅在坐标、接口或C++要求确有缺口时补；不并行运行两套检测器，也不将离线工具存在作为整套视觉自研的理由。

`rov_localization`为单独ament_cmake包：库使用C++/OpenCV/Eigen/AprilTag，ROS节点使用rclcpp/cv_bridge/image_transport；控制核心不引入图像依赖。所有自研源码沿用.h/.cpp、详细中文注释及现有clang-format。

| 文件（位于ROV_Control/src） | 类/函数及输入→输出 | 持有状态/调用关系 |
|---|---|---|
| rov_localization/include/rov_localization/vision_types.h | CameraModel、TagGeometry、TagDetection、PoseObservation；明确像素/米/旋转/协方差约定 | 纯数据，无ROS订阅和时钟 |
| rov_localization/include/rov_localization/camera_geometry.h、src/camera_geometry.cpp | CameraGeometry::configure(CameraModel)生成映射；rectify(image)返回灰度校正图/K_rect | 缓存不可变模型和映射；针孔/fisheye分支按真实标定配置。更换图像模式需重新配置，不每帧重建 |
| rov_localization/include/rov_localization/tag_map.h、src/tag_map.cpp | TagMap::loadYaml(path)加载/校验；collectWorldCorners(detections)返回配对P_W/像素及ID | 持有只读ID到四角的映射；统一AprilTag角点编号，未知/重复ID和坏几何不用于位姿 |
| rov_localization/include/rov_localization/fiducial_localizer.h、src/fiducial_localizer.cpp（按上游缺口决定新增范围） | 候选configure/detect/estimateBodyPose/checkQuality/reset职责；先评估Hippo现有图像/检测输出能否直接使用，必要时只适配地图/坐标/质量接口 | 在线检测来源唯一；可消费现成检测结果，或在确需原图四角时复用库，不能同时重复检测。上一接受观测仅成功时更新；时间倒退/长丢标清历史 |
| rov_localization/include/rov_localization/fiducial_localization_node.h、src/fiducial_localization_node.cpp | loadConfig()；onCameraFrame(Image,CameraInfo)同步输入→校正→检测→求解→门控；publishPose()；publishDiagnostics()；publishDebug()/publishMap()；onDiagnosticsTimer()检查断图 | 节点持有上述类和计数/时间，单线程executor串行修改；错误帧只输出原因，只有有效解发布新pose |
| rov_localization/src/localization_main.cpp | init→构造→spin→shutdown | 不再封装算法；相机驱动和定位各为独立进程 |
| rov_localization/CMakeLists.txt、package.xml | 构建/依赖/安装、正式测试入口 | 显式依赖版本，不把机器上的隐式依赖当可复现环境 |
| rov_localization/test/test_fiducial_geometry.cpp、test_localization_interfaces.cpp | 已知位姿投影/角点顺序/外参逆变换；帧和标定不匹配、断图/乱序、重放后输出 | 正式C++验证，测试几何不等于水下视觉仿真或硬件精度验收；真实图像另走录包回放 |
| rov_bringup/config/camera.yaml、camera_calibration.yaml | 设备/输出模式；测得的模型/K/D/图像尺寸 | 正式标定文件待数据产生，不提前填写假K/D。来源、日期、窗口/输出模式写入配置注释 |
| rov_bringup/config/tank_tags.yaml、camera_extrinsics.yaml、vision_localization.yaml | 标签ID/实际几何；T_BC；检测/质量/调试开关 | 标定几何启动只读；阈值来自试验选择，参数入口检查，避免每帧反复校验固定配置 |
| rov_bringup/launch/camera.launch、vision_localization.launch；rviz/vision_localization.rviz | XML启动现成采集器和自研定位、只读显示，参数按车辆名组织 | 不启动fake_vision、fake_state_estimator或执行器发布者；避免真/假位姿同名混合 |

直接调用AprilTag原生库的原因是需要四角像素统一参与地图PnP；不自己写标签解码，也不同时再启动apriltag_ros重复检测。官方apriltag_ros保留为接口/对照参考。最初允许单标签检测/求解检查，多标签作为覆盖目标；有效位姿需通过平面歧义检查，不能认为看见两张共面标签就自动消除歧义。最少标签数、重投影阈值等通过初测确定和记录，不把任意经验值伪装理论条件。

### 13.5 运行时、时间与初始化

图像到达触发计算，不用50Hz定时器重复处理旧图。输入使用同步图像/CameraInfo，队列有界、最新帧优先；过载时丢旧帧并报告年龄，不累积长延迟。CameraModel在首个有效Info到来时建立；尺寸/模型与配置不符时停止有效pose输出，显式修正后重启。

初始状态WAIT_CAMERA/CALIBRATION，第一批不建立任务home。成功观测进入TRACKING；看不到标签、歧义或无图时输出诊断，旧pose不重打时间戳。新图像恢复后重新接受几何/时间一致的解；长中断清除上一位姿连续性约束，再定位不隐式改变地图或未来控制任务home。

姿态和位置继承图像采样时间；处理耗时与无图看门狗使用steady_clock。USB驱动时间戳含义要核验，未证实硬件曝光时间时报告为驱动时间，不称硬件同步。真机use_sim_time=false；离线rosbag回放可显式用仿真时间重现采样间隔，但绝不接执行输出。

单线程executor保证节点状态串行；AprilTag可有内部检测线程，但不允许它们直接改ROS节点状态。诊断定时器仅检查/发布健康信息，不生成新定位。目标处理频率先按约15～20Hz评估，选择设备支持的分辨率，再测Pi端到端延迟；20Hz是设计目标，不是已实测性能。

### 13.6 验收与后续接控制的边界

以下是后续精度评价的可选方法，11-03起不再作为ROS原型前置：在不同高度/位置做固定静止或已知位移，检查偏差、散布、方向/尺度、丢标和延迟。手动移动可查方向与量级，不能给出独立绝对精度；16张标签也不自动证明全缸覆盖。当前实际实验顺序以§13.13和README§13.14为准。

此前1cm绝对误差/5mm静止散布/2°航向/100ms延迟只是未确认的讨论预算，现不作为原型门槛。按目标行程、可用余量和实测延迟选择合理容差与低速；例如0.05m/s与100ms相乘为5mm运动量，只是预算示例。先核方向与尺度量级、真实更新和断图行为，再依据任务结果改善精度/速率。

遮挡标签、断开图像、重复/倒退时间戳、错误CameraInfo、未知ID时，确认没有持续伪新定位；恢复后检查绝对地图定位与跳变。平面位姿翻转或湿光路尺度偏差不能靠低通滤波隐藏；先改观测模型、工作区域或安装/照明。当前无DVL不阻止已知标签可见时的定位，但标签完全丢失后不能宣称长时间准确水平定位。

后续融合候选复用USTROV C++ Ekf。已有Estimator::OnVision只复制数值、没有坐标变换；PublishPose协方差仍TODO，IMU超时复位也未实现，不能直接rename话题就交给控制器。RealStateNode的详细接口和速度/协方差输出在V5确认，最终满足StateAdapter的map/FLU/时间契约。现有PointTrackingNode仍限制仿真时间，真机另按README§11.12固定航向短程路线接入；不在本视觉批次直接启动它。

10-07此处为完整视觉设计；10-21经用户确认只实施§13.10离线标定子集，构建/8项测试结果见README§13.8。在线相机、定位、地图、滤波等依Hippo复用缺口后续确认，未自动展开V0～V4；真实图像/米制验收仍需设备与录像。

### 13.7 AprilTag直接贴缸底：间距与成像条件（2026-10-19）

用户明确选择不加独立底板，标签直接固定到缸底。此前1200×350mm底板和140mm中心距只是未验证的施工示例，已被本方案替代；35cm板宽来自24cm两排标签包络加两侧各5.5cm施工余量，没有成像最优性依据。用户补充的“10cm离底能看见角点”已澄清为空气中图像可见，未运行检测、未验证水下，不等于已输出稳定ID、角点或位姿。原20～50cm为计划工作区间，若实际要在10cm运行，则将10cm纳入湿验证。

包装标注36h11、ID0～16、外尺寸100×100mm、码尺寸80×80mm；10-21用户明确实际现有16张，实际ID集合仍需识别记录，不再推定第17张或ID16必为备用。最终行列和间距按实测视野决定。相机内参与运行地图是不同数据，同一组标签可兼作两者的目标；不要求额外承载板，也不要求固定网格节距，X列间距与Y行间距可以不同。

**相机和算法要求分层：**

- 图像容纳图案：只证明在该高度/姿态能看到，不保证解码成功。应保留完整编码、黑框及周围可辨白边，避免裁切、遮挡、反光、失焦和运动模糊；最低高度侧重视野，最高高度/最大倾角侧重有效像素与清晰度。
- 检测：正确36h11字典，软件持续输出正确ID和同一标签有序四角；不能把零散可见的不同标签角点当作已完成解码。没有适用于所有镜头/曝光/分辨率的统一最小像素数或角度阈值，按正式图像模式实测检测率与最长丢检时间。
- 位姿：一个已知尺寸的完整标签、相机内参/畸变模型及其有序角点可以用于位姿求解，仍需处理平面解歧义和误差。不要求同屏16个或固定至少两个标签；多个有良好空间分布的完整标签用于冗余/改善约束，不是无条件精度保证。固定map定位还需实测标签位置/方向，机体位姿另需T_BC。
- 运动连续性：对计划经过的区域、实际高度与倾角检查标签交接；若全部标签丢失，该帧没有直接标签位置观测，IMU短时预测不等同持续绝对定位。定位误差用独立已知位移检验，不能只看ID成功率或重投影误差。

**间距的简化几何检查（本项目推导，非厂家指标）：** 用完整成品宽a=10cm作保守覆盖尺寸，外边净隙g，中心距d=a+g。相机正下视、图像轴与网格对齐、沿相邻标签中心连线移动时，在两标签中点要让相邻两张完整进入视野，需要该方向有效地面视野F≥2a+g；这也使一维相邻“至少一张完整”的可见区间衔接。有效F应剔除不清晰/校正裁切边缘，实际还须留余量；这不保证整个运动区始终同时有两张标签。二维网格分别检查F_x、F_y，并另验实际边界、偏航/俯仰/横滚。g=4/6/10cm时，对应几何下限F=24/26/30cm，不能用空气中120°商品视角代替湿光路实测。

物理容纳另算：沿135cm长边放8张10cm成品，排长=80+7g cm。不留端部余量时g≤55/7≈7.86cm；g=6cm排长122cm，居中端距各6.5cm，g=8cm排长136cm已放不下。两排横向间距可以另外增大，但可见性仍须单独验证；不把上述尺寸可容纳当作检测通过。

**直接安装与测量：**

1. 选一个固定缸底内角为施工参考P，X沿135cm长边、Y沿65cm短边、Z向上。在标签之外做可清除的测量基线；以实际ID记录，不以包装顺序代替ID。先用少量标签临时摆设，记录候选g_x/g_y。
2. 在最终图像分辨率、裁切/去畸变设置下，测实际工作高度的有效地面视野；先空气检查工具链，再用最终湿光路验证。把相机放在标签正上方、两标签中间和四标签之间，横纵移动并施加预期倾角，记录正确ID、完整角点、丢检和延迟。最低和最高运行高度都检查。
3. 根据标签与玻璃/缸底材料选可长期浸水的粘接方式，按产品要求干燥施工/固化；保持平整、不移位、无气泡翘边和完整白边，胶不覆盖图案，不撕破本体防水层。运输包装及遮挡纸签移除。相机浸水能力仍按最终装置单独确认。
4. 固定后复测每张边长、中心或四角、方向和标签表面高度，形成后续tank_tags.yaml输入。图案朝向统一仅方便施工，仍核对检测器角点顺序，不把全部标签旋转默认写成单位阵。没有独立底板时不再沿用旧板角坐标。
5. P用于测量，不自动等同任务home。若W的XY原点/轴与P对齐，W的z=0为建图参考水面，则标签平面z_W=-H，H量到实际标签表面；否则显式记录T_WP。标签移位后重新测量地图。

本节为布置/几何设计；10-21另已实现离线适配并用合成数据验证，尚未处理实际湿录像或完成米制定位。官方依据（阅读相关正文，非新增论文证据）：[AprilTag检测/位姿](https://github.com/AprilRobotics/apriltag#pose-estimation)、[OpenCV Board](https://docs.opencv.org/4.x/db/da9/tutorial_aruco_board_detection.html)、[OpenCV PnP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)。

### 13.8 官方C++示例的ID、角点与朝向检查（2026-10-19）

用户确认测试设备为有桌面的Ubuntu电脑，相机直连该电脑；不把WSL USB透传或Pi无桌面显示作为此步前提。复用官方example/opencv_demo.cc，运行命令和每项含义集中README§12。此步无需K/D或标签米制边长，仅检查检测，不创建本项目实现代码。

已阅读示例main函数：getopt_parse取得-c相机索引、-f字典、-x降采样参数→VideoCapture打开相机→tag36h11_create创建字典→apriltag_detector_add_family注册→循环cap>>frame取得图像→cvtColor转灰度→image_u8_t引用图像内存→apriltag_detector_detect返回各检测→det->id用于屏幕文字、det->p[4][2]为四角像素→line/putText画参考边和ID→imshow显示→每帧销毁detections。示例不读地图、不求相机/机体米制位姿，也不输出本工程ROS消息。仅人工预览，不自带项目验收用的逐帧统计/日志。

官方quick_decode_codeword会检查四个90°编码旋转并返回匹配ID/rotation；正常面内旋转不改变物理标签的ID。有序角点依据编码规范方向，不等于每帧图像几何上的左上角。示例绿边p0→p1、红边p0→p3，红绿交点为p0。固定相机观察同一张旋转时可验证ID不变，p0仍跟随同一物理角，不能以彩色边仍在画面某一侧作为要求。

地图要求实际ID、尺寸和角点/姿态相互一致。当前一ID只映射一个物理位置，ID相邻与空间相邻没有强制关系。可在图案/完整白边之外或背面标记解出的数字和参考方向。安装可旋转任意角度；若中心不动而图案转90°，T_WM旋转或有序世界角点也必须相应改变。最易测量的做法是把每张解码后的同一参考边对齐同一缸边，或直接量四个有序角点；不能凭不同图案的外观猜规范“上方”。

首步人工检查：每张单独识别并清点→面内旋转四方向检查同ID→少量标签交接/高度/倾角检查。用户随后提供多张标签带数字ID和彩色四边形的运行截图，证明该帧检测成功；图中具体拍摄高度、完整ID清点、连续性和湿光路未经验证。正式成功率/最长丢检、时间与湿光路定位精度仍需后续实现和实测。当前有官方工具的用户运行截图，没有自研定位运行或精度验收结果。

### 13.9 使用现有AprilTag湿标定与地图实测的数据契约（2026-10-21修正）

具体现场步骤/命令集中README§13。用户明确只有16张AprilTag，无棋盘格；本节撤回10-20独立棋盘格路线，以现有标签兼作标定目标。10-21已确认并实施§13.10离线适配，不填假参数；无需另打印/购买或给棋盘格做防水。完整地图/在线定位仍待后续。

复用FFmpeg V4L2保留原始MJPEG录像（仅设备支持模式时用-c:v copy），原生AprilTag检测提供真实ID和有序像素角点，再调用OpenCV C++标定接口；单标签采样/配对/保存已在§13.10实现。opencv_demo仅检测，旧opencv_interactive-calibration -t=chessboard命令不用于此流程；缺少defaultConfig.xml会回退默认参数，不是该错误目标类型的修复入口。

起步内参路线使用选定的同一真实ID：以实测黑框宽高构造标签局部Z=0四角，多位置/尺度/俯仰/横滚观测逐帧生成objectPoints/imagePoints。无需先测16张缸底位置、间距、水深或每帧相机姿态；每帧四点的冗余较少，视角/边缘覆盖与独立验证不足时不得仅凭低残差验收。联合多标签是后续可替换输入：需提供各ID在同一目标系中的实测角点，不能将不同标签各自原点的四角直接拼为一块目标；不同帧可见不同标签子集，不必全16张入画或重排标准AprilGrid。完整map测量单独服务运行定位，不作为单标签内参的前置门槛。

10-21已有用户录制日志：/dev/video3、MJPEG、1280×720、30fps、wet_01.mkv约2分13.5秒；用户确认最终湿光路且改变了俯仰/横滚/高度。用户报告单格10mm，并进一步确认检测外框宽高均80mm（0.080m），不将内部编码格当棋盘格角点。先复用录像筛选，不因旧工具没输出而要求重拍；尚未读取视频内容验证采样质量。

数据分别保存：原始视频与相机/标签目标/湿光路记录；模型/K/D/实际图像尺寸/采样与误差；手工实测ID/码边长/有序目标角点；相机刚性安装后独立测量的T_BC。目标长度统一m，K焦距/主点单位px，D依模型。拟CameraModel适配显式处理字段/模型；无数据不写占位参数。

地图沿用§13.3固定W，XY从固定缸底角P投影，Z零点为建图参考水面标记，标签表面z=-H；水位变化不隐式改地图。内参目标可直接选标签局部系、Z=0，无需预先求出它与W的关系；联合已测目标也用Z=0的平面坐标初始化，不能把未经转换的z=-H点直接当普通零平面初始化输入。角点按编码规范排序；贴正可由中心±半边长构造后排序，斜贴显式旋转或实测角点。几何来自独立尺量，不从尚未标定的视觉位姿生成“真值”反标自身。

验证分开：重投影误差核图像拟合，留出数据及独立50/100mm位移核方向/尺度/高度与边缘系统误差；相机刚性固定到机体后再用T_BC验证base_link位姿。当前opencv_demo没有这一步米制输出，拟定位节点仍未实现。平面水界面非严格针孔，首版湿标定是限定工作范围内的工程近似，不能承诺所有距离/边缘精度。

### 13.10 单标签录像离线标定工具：已确认设计与实现（2026-10-21）

**目标与复用。** 从已有wet_01.mkv选一个ID的多视角观测，使用已确认的80×80mm求相机内参候选并输出可追溯报告。复用AprilRobotics tag36h11检测、OpenCV视频/标定/FileStorage；自研C++17薄封装、选样和报告。沿用已有规划的rov_localization包；本批不创建定位/融合/相机ROS节点，不接USTROV、控制核心、PX4、推进器或Gazebo，不测完整map或T_BC。

**已有功能核对。** 本地USTROV b1208e5的top_lemniscate_demo_offboard.launch.py include外部visual_localization/top_localization.launch.py，但本地无该package.xml；fake_vision.cpp从ground_truth/odometry复制pose并延迟发布，不处理真实图像。Hippo官方Camera Calibration文档明确calibration.launch.py使用标准ROS棋盘格标定器，无法直接处理当前tag36h11录像。当前增量是原生检测输出到OpenCV标定输入的离线适配，最终复用清单优先于上一条完整新包目录；不需要先完成在线定位/融合工程才算K/D。录像先检测/选样，不承诺无需补拍。

**构建选择。** 沿用rov_control_core/CMakeLists.txt已有模式，ROV_LOCALIZATION_WITH_AMENT默认ON供colcon，设OFF可用普通CMake在桌面Ubuntu构建，不要求该电脑安装ROS。当前库仅OpenCV/AprilTag依赖。AprilTagDetector只封装本次离线调用；后续在线定位优先复用Hippo现成图像/检测组件，此类是否复用按真实缺口决定，不强制另写完整在线检测链。

| 已新增文件（相对ROV_Control/src/rov_localization） | 职责、输入输出/状态 |
|---|---|
| CMakeLists.txt、package.xml | 构建纯C++库/calibrate_camera、可选ament导出、正式CTest入口；记录实际依赖版本 |
| include/rov_localization/vision_types.h | TagDetection、CalibrationOptions、CalibrationView、CalibrationResult纯数据；无ROS类型/线程 |
| include/rov_localization/apriltag_detector.h、src/apriltag_detector.cpp | AprilTagDetector拥有官方family/detector，灰度图→真实ID/有序像素角点/解码信息；不求位姿、不需K/D |
| include/rov_localization/camera_calibrator.h、src/camera_calibrator.cpp | CameraCalibrator持有本次配置、候选观测、选定帧、结果/诊断；负责下表流程 |
| src/calibrate_camera_main.cpp | 解析视频/配置/新结果目录，构造校准器并run，打印路径与退出码；无优化公式 |
| config/calibration_options.yaml | tag36h11、width_m=height_m=0.080、自动/指定ID、选样/质量/覆盖与模型/求解选项；不写假K/D。FileStorage输入说明其YAML头要求 |
| test/test_camera_calibration.cpp | 正式C++几何/角点旋转/合成投影恢复/退化与错误输入/不覆盖输出测试；不把合成测试叫水中精度验证 |

**数据契约。** TagDetection含id、corners_px[4]、hamming、decision_margin。直接复制原生det->p[0..3]，不按图像左上角重排；decision_margin是解码信息，不当角点协方差。CalibrationView含frame_index、video_time_s/来源、id、原图尺寸、四角px、中心/面积/四边形形状与清晰度；时间仅用于去密集重复，不当曝光时间/控制dt。候选数量有界，不缓存整段视频。Options持有路径、80mm宽高、目标ID与所有筛选/求解配置，入口统一检查。Result含model、image_size、K/D、逐帧目标→相机外参、RMS/逐帧误差/覆盖诊断、ID/帧号、依赖版本和候选状态。

K为3×3，fx/fy/cx/cy单位px；首版明确采用pinhole模型，D顺序k1,k2,p1,p2,k3，不因120°宣传强行称鱼眼。角点/每帧tvec单位m、rvec单位rad。标签局部系L以码中心为原点，按官方H与p[0..3]映射到(-w/2,+h/2,0)、(+w/2,+h/2,0)、(+w/2,-h/2,0)、(-w/2,-h/2,0)，w=h=0.080m。该轴随编码规范定义，绝不随图像上下改变。L与逐帧外参不是home、map或T_BC，内参不需先量L→map。

| 类/函数 | 输入 → 输出 | 状态、调用关系与失败边界 |
|---|---|---|
| main / loadOptions | CLI路径、YAML → Options或具体错误 | 核文件可读、宽高/选值合法、结果目录不存在；原视频只读、拒绝覆盖旧结果 |
| AprilTagDetector::configure / detect | 配置 / 原尺寸8位灰度图 → detections | 创建/析构官方资源，复制检测结果再释放；不预先用未知K去畸变，不自写解码或无解释二次角点修正 |
| CameraCalibrator::run | Options → Result/失败报告与退出码 | 清空本次计数/候选，按扫描→选ID→选帧→求解→检查→保存顺序调用 |
| scanVideo | 视频路径、检测器 → 按ID候选、逐帧/拒绝统计 | VideoCapture顺序读取；实际尺寸固定，变化即报错；记录帧号/时间来源。CAP_PROP_FRAME_COUNT可能估算，差异仅记录告警，不能据此判损坏。read=false的EOF/解码错误区分限制显式报告；坏检测跳过并记原因 |
| selectTargetId | 候选组、自动/指定ID → 一个ID或不足原因 | 自动按有效二维覆盖/尺度变化、数量与ID作确定性排序；指定ID不替换。同帧重复ID不拼接；不能因样本不足偷偷把未知几何多标签合并 |
| selectViews | 选定ID观测与配置 → 拟合/留出帧、覆盖报告 | 核清晰/完整/几何、中心/尺度/形状差异与密集重复；二维形状只作多样性代理，无K时不声称已测真实倾角。约20～30个不同清晰视角为起步目标，非数学下限/精度承诺；按片段/视角分组留出，不随机拆相邻帧充当独立验收 |
| makeObjectPoints / solveIntrinsics | 宽高、对应像素 → L四角、K/D、逐帧外参 | 同一L四角用于各帧，cv::calibrateCamera联合求内参与每帧外参；明确pinhole及flags。首版不自动切模型或把fisheye D混入 |
| evaluateResult | 结果、拟合/留出组 → 数值/覆盖/退化诊断、逐帧px误差 | 核参数有限、正焦距、样本与求解状态；projectPoints核残差。留出组仅求外参，不回调K/D；报告单一正视/尺度、边缘覆盖不足、系统残差等。低重投影误差不等于厘米精度 |
| saveResult | 数值有效候选、诊断、新目录 → XML/报告/CSV | camera_wet.xml写模型/K/D/尺寸/来源/候选状态，非直接CameraInfo文件；calibration_report.yaml记录统计/拒绝/质量；selected_views.csv记录ID/帧/误差。不足/失败只写原因，不造参数；写盘失败报错，不覆盖旧数据 |

**运行与状态所有权。** 单个离线进程、一个顺序控制流程。每次读到视频帧触发检测，扫描结束后选样并求解；无ROS回调/定时器/topic/service。原生检测器内部工作线程可配置，但detect同步等待、结果复制后进入下一帧。CameraCalibrator独占候选/结果，无跨回调共享状态；构造/run开始初始化本次状态，每次新运行重新开始，无热加载。耗时用steady_clock，视频时间不当控制时钟。

**输出/维护。** 新结果目录建议~/vision_data/calibration_wet_01/results_01/，已有wet_01.mkv保持只读。配置与正式源码/测试入Git，原始视频/参数试验结果/CSV在工作区外数据目录，构建/log按既有规则忽略。只更新README和本设计，不新增重复阶段.md。真实测得K/D以后显式转换到ROS CameraInfo/运行配置；不通过改后缀或frame字符串冒充适配。

**验证顺序与边界。** 已用已知K/D和多倾角合成投影核角点/拟合/退化拒绝，再用真实36h11图案旋转核ID/p序，正式测试保留。lab随后实际处理了原视频并提供候选XML/报告/CSV，结果见README§13.10；本机未读取原视频，不把候选XML当水中定位验收通过。独立尺量和高度/边缘检查可用于精度评价；11-03不再将其作为ROS原型前置，当前顺序见§13.13。

用户随后明确“继续执行”，本节范围已授权并实现。真实使用与软件验证结果集中README§13.8；在线视觉各节点仍是候选设计，未自动扩展实施。新增源代码经过clang-format，详细中文注释；没有修改USTROV或控制链。

实现细化：CameraCalibrator::describeView从图像提取完整性/最小边/清晰度/二维特征；scanVideo使用固定种子有界蓄水池，按帧号/文件FPS抽样并记录后端/EOF限制。selectViews使用特征空间最远点贪心，再按时间组留出，未将随机相邻帧拆分冒充独立验收。所有试验选值在YAML，读取时逐项要求字段存在并校验；min_fit_views至少10，支撑两组各至少5视角的焦距检查，默认20/5不是理论保证。

兼容修订：Ubuntu24.04的AprilTag3.3.0包CMake导出含错误路径，优先pkg-config、缺.pc时回退原生CONFIG，头目录显式查找；测试按需链接Debian单独的apriltag-utils。OpenCV4.6 calibration.cpp计算标准差使用total_points-nparams，在每帧四点时为负并返回NaN，而已知投影K/D恢复正确；本工具不依赖该标准差。evaluateResult新增内部checkFocalStability：拟合视角交替拆两组，各调用cv::calibrateCamera，与全组fx/fy比较，取最大相对变化对照YAML的max_relative_focal_change；这是子集一致性，不是统计不确定度。其余法线、留出残差、径向折返检查保留。没有新增优化器或修改系统库。

0.1.1（r2）实际报告驱动修复：首轮lab报告4002帧/容器4006帧，旧scanVideo提前误判，CSV仅表头，尚未选帧/求解。依据OpenCV4.6 FFmpeg get_total_frames源码，将元数据差异改为诊断：ScanStatistics保存signed frame_count_difference与frame_count_warning，saveResult记录tool_version，eof_limit覆盖有元数据时也不能区分结束原因。修复本身没有改选样/求解/质量选值；新增临时AVI15帧/元数据19帧回归，共9项测试通过。lab已重跑并生成候选参数；用户随后单独将本次实验留出门槛从2.0改为2.3px，完整结果见README§13.10。其状态为candidate_requires_metric_validation，尚无地图/T_BC或独立米制验收。

### 13.11 单标签只读位姿实验：确认范围与函数设计（2026-11-02）

用户在前一轮最小工具说明后明确“直接开始改”，本批实施录像/桌面USB采集→已有检测→固定内参PnP→相机在标签系位姿→显示/CSV。复用AprilTagDetector、OpenCV采集/IPPE/LM/投影；不另写检测器/优化器。只做实验必要通路，不新增地图、融合器、ROS图像节点、控制发布器或通用插件框架；USTROV和执行器链保持原样。后续ROS集成可复用本批数学类，但属于另批。

| 文件/函数 | 输入→输出 | 内部状态与调用/边界 |
|---|---|---|
| tag_pose_estimator.h：CameraModel、TagPoseOptions、TagPoseEstimate | K/D/尺寸/候选状态；实验选值；has_pose/valid/ambiguous/reason、T_CL、p_LC/R_LC/RPY/误差 | 单位在字段后缀；无ROS类型。has_pose只代表有候选，不能代替valid |
| tag_pose_estimator.cpp：loadCameraModel(path) | 原camera_wet.xml→CameraModel | 只读；核pinhole/plumb_bob五参数、尺寸/K/D数值，不重新标定 |
| loadTagPoseOptions(path) / validateOptions | tag_pose_options.yaml→选值 | 入口核尺寸/ID/解码/误差/帧率；不逐帧重复检查固定参数 |
| TagPoseEstimator构造 / camera / options | 模型与选值→只读求解器/访问器 | 深复制K/D，持有固定模型与选值，无历史位姿或滤波状态 |
| objectPoints() | 80mm宽高→L四角 | p0(-w/2,+h/2)、p1(+,+)、p2(+,-)、p3(-,-)，Z=0；沿用标定规范 |
| estimate(detections) | 本帧原生ID/四角→TagPoseEstimate | 只接指定ID，拒绝同帧重复ID/坏角/小边/解码超限；局部变量保存两解和细化结果，丢检不补上一帧 |
| evaluateCandidate / rotationDifferenceDeg | 某候选外参/实测角点/K/D→正深度与RMS/两解旋转角差 | solvePnPGeneric(IPPE)两个平面解分别RefineLM；只采纳正深度且原图误差不增加的细化结果；K/D固定 |
| estimate内坐标变换 | 最佳T_CL→p_LC=-R_CL^T*t_CL、R_LC=R_CL^T | L标签局部、C光学右下前；abs(p_LC.z)为到平面距离。RPY仅显示，完整旋转保留 |
| tag_pose_preview_main.cpp：main / runPreview | CLI输入/新目录→进程退出码和实验结果 | 一个顺序循环独占VideoCapture、AprilTagDetector、求解器、CSV/计数；USB或视频二选一，首帧/每帧尺寸核对 |
| writePoseRow | 时间/检测/本帧结果→每帧CSV | 所有读入帧记录valid/reason，失败无候选时写nan；保留候选、角点与R_LC供复查，不把失败候选当测量 |
| showPose | 原图/结果/K/D→角点编号/XYZ轴/数值窗口 | X红/Y绿/Z蓝；未校正图配原D，不二次去畸变。ASCII叠字、中文终端 |
| onInterrupt / runPreview退出 | q/Esc或SIGINT→结束循环/释放并保存 | signal只置sig_atomic_t标志，主线程落盘。run_info记录来源/实际ID/选值/帧数/停止原因；输出新目录拒绝覆盖 |
| config/tag_pose_options.yaml | 实测边长/ID/可调实验筛选→启动配置 | 默认ID16/80mm；改后重启，不热加载。实际命令行覆盖ID另记run_info |
| test/test_tag_pose.cpp、CMakeLists/package.xml | 正式几何/图案/CLI测试、构建/安装→软件证据 | 新增9项定位测试及原9项标定回归；版本0.2.0，GUI只链接预览入口，几何库不依赖highgui |

**采样/状态。** read成功的一帧触发一次detect和estimate，顺序计算、写行、显示；无ROS定时器/回调/额外控制线程。detect内部线程同步等待返回。视频time_s由帧号/文件FPS，相机为read返回后steady_clock相对首帧；都不是曝光时间或控制时钟。本批不计算速度/控制dt。processing_ms不含采集等待、GUI和落盘。进程每次构造初始化状态，不存在位置归零服务或跨ID自动home。

**假设/验证。** 标签固定且80mm几何可信，原图模式/光路与XML一致；L不是ENU/NED、map或机体。平面两解误差差<=0.15px且旋转差>=5°则valid=false/ambiguous_pose，选值只是启发式而非可观测性保证，不借历史滤波隐藏翻转。11-03用户明确以原型效率为先：独立50/100mm和固定静止记录改为可选精度检查，直接推进§13.13的ROS位姿接入；闭环前仍须确认方向、尺度量级和数据新鲜度。前一轮K/D与本轮合成测试不能替代真实水下精度。操作和已运行范围集中README§13.11，不新增重复阶段文档。

### 13.12 机载相机部署接法更正（2026-11-02）

用户已明确相机固定ROV、USB接Pi、Linux笔记本有线连接Pi。运行位置改为Pi本机：VideoCapture读Pi的/dev/videoX，原detect/estimate/CSV函数链保持原样，笔记本SSH仅操作/显示终端并scp取数据。aarch64须由Pi原生编译源码，不复制x86 build；设备路径现场确认。默认display=false不要求Pi有桌面；可选ssh -X以现有showPose显示窗口，X11条件和性能需现场确认。本轮未新增或改动C++/ROS接口，只补README§13.12分终端操作。

相机刚性安装只改变实验操作：可手动平移整台ROV检查位姿变化，不要求提前测T_BC；旋转会引入杆臂位移，机体中心位姿仍需安装外参。数据原点仍是固定标签L，未因相机上机变成home或map。随后用户Pi水中采集/求解已得到真实记录（README§13.13），X11、端到端时延及米制精度仍未验证；后续部署方案见下节。

### 13.13 面向实际水缸的最小原型计划与函数设计（2026-11-03，本批已实施）

**任务和范围。** 用户要求基础可运行、误差满足实验需要，不以计量级精度工程为目标。现有湿标定XML、80mm标签和97帧连续有效记录足以继续ROS接入；固定静止统计、100mm专门试验、全16张地图、复杂滤波和完整测力均不作本批前置。最终首个运动任务是沿135cm长边、小范围、近固定航向到点，深度先监测。水缸宽65cm，按当前Hippo式控制接口不预设独立横移推力；实际安装轴向仍待核对，暂不安排大转弯、任意XY轨迹或三维保持。5～10cm前向目标是拟用起步量级，实际取值须落在完整标签视野和机体可用余量内。

**批次与复用。**

1. 本批：实时单标签相机位姿→ROS→笔记本RViz，推进器不动作。复用AprilTagDetector、TagPoseEstimator、loadCameraModel/loadTagPoseOptions，继续用ID16及原K/D，不重写检测或PnP。
2. 后批：配置粗测相机安装T_BC，把相机位姿变成机体位姿；定义Z向上的局部世界W、固定标签T_WL，建立第一次有效机体定位的home。复核USTROV IMU/压力和C++ EKF接口，再选择最低限度的速度/角速度来源；不能把缺少的速度填零接控制器。
3. 后批：只读运行真机控制计算，再验证低幅方向与停机，最后5～10cm纵向到点。既有PointTrackingNode参数入口强制use_sim_time=true，且自带Hippo物理分配；保留该仿真入口，真机另做薄适配，复用已核实的USTROV执行桥。一次只能有一个命令源。
4. 需要跨标签运动时才测相邻2～4张ID的位置/朝向，加入局部地图；不为了第一步测齐16张，也不切ID就重置原点。

**实际部署选择。** Hippo当前官方文档使用Pi发布压缩图像、板外PC检测定位。考虑实测Pi现有工具约2.3Hz，本批按同一部署方式设计：Pi复用现成C++ usb_cam采集，lab运行自研薄ROS包装与RViz。11-03用户日志确认Pi的usb_cam 0.8.1拒绝旧配置mjpeg并退出；上游raw_mjpeg直通也有已报告的分支不匹配，当前改用mjpeg2rgb解码为RGB，再由compressed_image_transport编码JPEG发送。分辨率、源header及lab接口保持，Pi多一次解码和编码，性能待实测；不再将当前路径称为原始MJPEG直通。现有本地USTROV工作区仍无visual_localization源码包，不能直接声称整套Hippo视觉EKF已经运行。这里复用Hippo部署思路及现有AprilTag/OpenCV求解；暂不引入上游Python视觉EKF或另一个重复检测节点。

~~~text
Pi: USB相机MJPEG → usb_cam解码RGB → 压缩插件编码JPEG（1280×720，保持湿标定光路）
                   ↓ /uuv00/vertical_camera/image_raw/compressed
                   ↓ 以太网DDS，保留源图像header.stamp
lab: TagPoseNode::onImage → JPEG解码 → 灰度 → detector.detect
                                          → estimator.estimate
     camera_wet.xml + tag_pose_options.yaml ──────┘
             ├ valid → /uuv00/vision/camera_pose（PoseStamped）+ TF
             ├ 每帧状态/处理耗时 → /uuv00/vision/diagnostics
             └ 原图叠角点 → /uuv00/vision/debug_image（仅lab本地显示）
     RViz/rqt查看；rosbag按需记录图像、位姿和诊断
~~~

**文件、类和函数。用户已确认并实施为0.3.0，下表为本批实现；后续机体变换/融合/控制仍另批。**

| 新增或修改文件（相对src） | 函数/入口 | 输入、输出与内部状态 |
|---|---|---|
| rov_localization/include/rov_localization/tag_pose_node.h；src/tag_pose_node.cpp | TagPoseNode构造、loadConfig() | 读取XML路径、现有TagPoseOptions路径、frame_prefix、超时及调试开关；由prefix与目标ID派生frame名；初始化只读求解器、唯一检测器、订阅器、发布器和250ms健康定时器。参数启动只读，构造失败报告具体配置，不假造内参 |
| 同上 | onImage(CompressedImage) | 检查源frame、递增时间和JPEG/尺寸，解码为原始BGR/灰度，调用detect→estimate；单帧局部数据不跨帧复用。更新最后输入时间/计数/当前原因，通过检查才调用publishPose；每次处理后publishDiagnostics，按开关publishDebugImage |
| 同上 | publishPose(estimate, source_header) | p_LC与R_LC→米、单位四元数PoseStamped；header.frame_id=uuv00/tag_16，时间沿用输入。TF为tag_16→vision_camera_optical，使用独立显示子帧避免与USTROV base_link→camera的静态TF发生双父冲突 |
| 同上 | publishDebugImage(image, detections, estimate, header) | 原图叠ID/角点序号和有效状态，用sensor_msgs/Image供lab本地查看；不去畸变后再用原D计算。不重写检测/求解，图像状态与本帧一致 |
| 同上 | publishDiagnostics()、onHealthTimer() | 发布ID、valid/reason、收到/处理数量、解码/检测/PnP耗时及最近输入距今时间。steady_clock检查断图；定时器只更新状态，不重新发布旧位姿或TF |
| rov_localization/src/tag_pose_node_main.cpp | main(argc,argv) | rclcpp::init→构造节点→单线程spin→shutdown；不封装算法，无控制输出 |
| rov_localization/CMakeLists.txt、package.xml | ROS构建/安装 | 保留WITH_AMENT=OFF的两个离线CLI；ament模式增加上述节点及rclcpp、sensor_msgs、geometry_msgs、diagnostic_msgs、tf2_ros等必要依赖；OpenCV解码用imgcodecs；数学库仍不依赖ROS |
| rov_bringup/config/usb_camera.yaml | usb_cam参数 | Pi设备/dev/video0、1280×720、MJPEG、受设备支持的帧率、optical frame；配置不证明实际帧率。原分辨率保持，避免随意缩图后误用K |
| rov_bringup/config/tag_pose_ros.yaml | ROS包装参数 | 原XML/选值文件的路径、frame、话题、断图提示和调试开关；不复制一套K/D或质量门槛。driver若无已标定CameraInfo，本节点不使用其零内参，明确以XML为模型源 |
| rov_bringup/launch/usb_camera.launch | XML启动 | 仅在Pi启动usb_cam，图像话题按vehicle_name组织；不启动core/解锁/控制 |
| rov_bringup/launch/tag_pose_view.launch；rviz/tag_pose.rviz | XML启动与RViz | lab启动TagPoseNode和可选RViz，固定帧tag_16、显示相机Pose/TF/调试图；不把tag_16叫map/home |
| rov_bringup/package.xml | 运行依赖 | 声明新增定位与相机启动依赖；已有CMake安装launch/config/rviz目录可复用 |
| rov_localization/test/test_tag_pose_ros.cpp | 正式ROS接口测试 | 合成JPEG+已知K/D输入→位姿坐标/四元数/源时间；无标签/重复时间/尺寸错误/断图不产新位姿，恢复后可继续。不把这些软件测试写成实机精度证据 |

**时间、状态和并发。** 相机驱动是独立进程。定位单线程executor；onImage与onHealthTimer串行访问成员，原生AprilTag内部线程同步返回，不直接修改ROS状态。订阅采用sensor-data、keep_last(1)处理积压时的最新样本；这限制本地队列但不保证驱动/网络无积压。只保留最后输入源时间、接收steady时间、最后有效接收时间、计数和诊断原因，不维护预测轨迹。启动清空这些状态；换ID/光路参数重启；本批无reset服务、home或控制周期。首帧开始正常观测；坏帧停止新位姿输出并说明原因；断图定时报告。RViz可能保留最后显示，应同时看诊断/时间戳。

驱动时间不预先称曝光时间。pose保留源图像stamp，处理耗时用本机steady_clock；Pi/lab未确认时钟一致前不把now-stamp当端到端延迟。后批接反馈前核对两机时间、输入新鲜度与实际更新间隔；可将同一节点迁回Pi，无需重写数学层。本批不发布Odometry，不填零速度/零协方差冒充完整状态；ROS标量姿态以四元数为主，角度文本仅显示。

**下一批外参的明确边界。** T_AB定义为把B系点变到A系。现有求解输出T_LC；相机安装T_BC实测/粗测后，机体位姿T_LB=T_LC·inverse(T_BC)，在约定世界系中为T_WB=T_WL·T_LB。标签局部L并不保证Z向上；进入Hippo估计器前必须通过T_WL对齐坐标和高度基准，不能直接重命名为map。home取第一次有效机体位姿位置与初始水平航向，和硬件通电时间区分。先用尺量安装平移和确认安装朝向即可，不要求复杂手眼标定。

11-03用户提供安装粗量并确认FLU（此前FRU为笔误）：定位机体B暂以飞控板中心为原点，+X以Pixhawk前向箭头为准；镜头位于后方5cm/下方12.5cm，竖直向下，图像上方对应机体右侧。在FLU中t_BC=(-0.050,0,-0.125)m；光学X图像右/Y图像下/Z镜头前给出X_C=-X_B、Y_C=+Y_B、Z_C=-Z_B，因此R_BC=diag(-1,1,-1)，ROS顺序四元数(x,y,z,w)可取(0,1,0,0)。此安装模型由用户粗量与朝向描述确定，不是独立外参标定。飞控板中心并不自动等于质心或内部IMU测量点。后续位置换算p_LB=p_LC-R_LB·t_BC，R_LB=R_LC·R_BCᵀ；必须先把安装偏移旋转到L，不能把B中的三维偏移直接加到标签系位置。机体/home函数设计见§13.14，尚未创建实现。

**本批实际实验。** 仍用ID16完整可见的约25cm附近位置：手动小幅前/后、左/右、转动，lab看图像与位姿方向；粗量一个位移只查尺度量级，不要求单独精密台架或反复CSV。遮挡一次应停止新pose，恢复可继续；记录实际更新率与明显延迟。先提出3～5cm级的小范围任务误差作为后续原型讨论预算，而非实测精度或固定验收标准；若目标行程同样只有5cm，必须有更小的不确定性或改成更有分辨意义的行程。几何错误、旧数据冒充新数据和电机方向错误不能靠放宽精度预算掩盖。

用户确认后完成0.3.0实现：三个C++源/头文件及正式ROS测试、两个XML launch/两份配置/RViz、构建依赖。数学求解和旧CLI保持原实现。消息使用标准类型；调试BGR图直接填Image，不额外引入cv_bridge；frame_prefix启动只读。5项ROS接口测试与18项旧测试共23项通过，两个launch参数解析及只读定位节点实际启动/退出通过；临时数据清理，正式C++测试保留。尚未接用户USB/Pi或验证跨机/RViz真实窗口，操作与证据见README§13.15。最终Pi独立计算仍是用户目标，运行位置可切换，不把当前约2.3Hz当硬件上限。

### 13.14 机体观测、Hippo EKF 与融合 home（2026-11-03）

用户已确认实施。原纯视觉 home 设计由本节替代：唯一正式 home 建立在融合状态之后。自研新增独立 `rov_state_estimation` 包，几何、算法适配和 ROS 接口分层；已有 `rov_localization` 的检测/PnP 保持职责，不复制或改写上游 EKF。`tag_pose_view.launch` 仍能单独预览相机，新 `hippo_fusion.launch` 组合视觉与融合。当前只读定位，不接电机或仿真 PointTrackingNode。

**坐标与数据契约。** `T_AB` 把 B 系坐标变到 A 系；平移米，角度弧度，四元数 ROS/YAML 为 xyzw，Eigen 构造为 wxyz。

- B：飞控板中心，FLU，X 前/Y 左/Z 上，前方以 Pixhawk 箭头为准。
- C：相机光学 X 右/Y 下/Z 前。已知相机后5cm、下12.5cm、镜头向下、图像上方朝机体右；`t_BC=(-.050,0,-.125)m`，`R_BC=diag(-1,1,-1)`。
- L：ID16 原生标签系；W：该标签中心的局部 Z 向上坐标，`R_WL=diag(1,-1,-1)`，`t_WL=0`。平贴水平缸底为当前假设，W 不赋予东/北地理方向。
- H：融合初始化通过后的首个机体位置和水平航向；Z 保持向上，启动 roll/pitch 仍保留。它是软件完成初始化的原点，不是从电源上电瞬间追溯的位置。

~~~text
R_LB = R_LC * R_BC.transpose()
p_LB = p_LC - R_LB * t_BC
R_WB = R_WL * R_LB
p_WB = R_WL * p_LB + t_WL
p_HB = R_WH.transpose() * (p_WB - p_WH)
R_HB = R_WH.transpose() * R_WB
~~~

压力初始化寻找源时间差不超过 `pair_tolerance_s` 的视觉机体高度 z0 与压力 P0，后续 `z = z0 - (P-P0)/(rho*g)`：P 为 Pa，rho 为水密度 kg/m³，g=9.81m/s²，z 为标签平面上方高度 m。因此允许直接在水中启动，无须把首次压力误认为水面。当前把压力感测点相对飞控中心的高度变化近似为一致；不包含压力安装杆臂补偿或传感器比例标定，大角度姿态时应另核该误差。

**输入输出。** 以下位于 `/uuv00` 命名空间，车辆名前缀可配置。

| 话题 | 类型、坐标与单位 | 用途 |
|---|---|---|
| `vision/camera_pose` | PoseStamped，L 中的光心位姿 | 原 TagPoseNode 发布，源时间来自 Pi 图像 |
| `imu` | Imu，FLU，比力 m/s²、角速度 rad/s | USTROV 桥已完成 FRD→FLU，忽略其无效 orientation |
| `pressure` | FluidPressure，Pa | 原桥压力观测，来源/比例需现场确认 |
| `vision/body_pose` | PoseStamped，W 中的飞控板中心 | 纯视觉观测，供比较 |
| `state/pose` | PoseStamped，W 中的融合机体 | 初始化通过后发布；观测过期时可继续惯性预测，诊断降级 |
| `state/pose_home` | PoseStamped，H 中的融合机体 | home 初始化后发布 |
| `state/velocity_body` | TwistStamped，B 中的线速度 m/s 与角速度 rad/s | 线速度从 EKF 世界速度旋转；角速度为 IMU 减 EKF 陀螺偏置 |
| `state/diagnostics` | DiagnosticArray | 状态、输入时间、压力对齐、home、拒收原因、倾斜方差 |
| `state/reset_home` | Trigger 服务 | 清 home，等待请求后的新视觉与就绪融合输出重建，不重置 EKF/压力基准 |

没有发布 Odometry：原库不导出完整 P 矩阵，不能以全零协方差冒充精度，也没有为了调用仿真控制器伪造其状态契约。将来控制接入可直接消费已有 pose/twist 并增加对应新鲜度和权限检查，另批设计。

TF：`vision_world → tag_16 → vision_camera_optical` 为视觉显示支路；`vision_world → home → fused_body` 为融合支路。tag 变换静态发布；home 值在重设前固定，但随融合状态源时间动态发布，支持显式重设。使用 fused_body 避免与 USTROV 已有 base_link 的父子关系冲突。

**文件和函数。** 路径相对 `ROV_Control/src/rov_state_estimation`。

| 文件/类 | 关键函数 | 输入、输出、内部状态与调用关系 |
|---|---|---|
| `include/.../frame_adapter.h`、`src/frame_adapter.cpp` / FrameAdapter | `cameraToBody` | 上列外参链，输入 T_LC、输出 T_WB；无检测/滤波 |
| 同上 | `isValid`、构造函数 | 检查有限值、单位四元数和固定安装变换 |
| 同上 | `initializeHome`、`toHome`、`resetHome`、`home`、`config` | 仅保存固定配置和可选 T_WH；航向水平投影小于0.1时等待，不用初始倾斜定义水平系 |
| `include/.../hippo_fusion.h`、`src/hippo_fusion.cpp` / HippoFusion | 构造函数 | 校验参数，持有上游 Ekf、观测缓存、时间/积分累积量、初始化和故障状态 |
| 同上 | `setVision`、`setPressure` | 检查有限值、单调源时间与相对 IMU 时间窗口，缓存新观测；不积分、不重发旧观测 |
| 同上 | `tryAlignPressure` | 选相近时间的一对视觉高度/压力，保存 z0/P0，仅建立一次 |
| 同上 | `pushImu` | 首帧只设时间基准；后续按实际dt分段积分，补齐严格10ms后调用updateFilter；例如250Hz的4+4+4ms被分为10ms窗口与2ms余量 |
| 同上 | `updateFilter` | 每个10ms窗口：SetImuData→feedObservations→Ekf::Update→读取输出→updateReadiness；返回最新融合结果 |
| 同上 | `feedObservations` | 逐条送入未处理压力及最新未处理视觉；压力换高度，视觉同半球四元数与时间兼容补偿，调用 SetBaroData/SetVisionData |
| 同上 | `updateReadiness` | 观测新鲜、上游 AttitudeValid、等待启动时间、融合接近视觉持续0.5s后置 initialized；之后丢观测只降级，不重建home；观测恢复且融合/视觉重新接近才恢复READY |
| 同上 | `reject`、状态/getter 函数 | 统计拒收原因；IMU时间缺口超限或非有限滤波结果锁定故障，需要重启节点 |
| `include/.../fusion_node.h`、`src/fusion_node.cpp` / FusionNode | `loadConfig`、构造函数 | 只读参数、三订阅、四状态输出、诊断、两种TF、服务、250ms健康定时器 |
| 同上 | `onCameraPose` | 核frame→cameraToBody→setVision→发布视觉机体观测，保留相机源stamp |
| 同上 | `onPressure`、`onImu` | 核frame，分别缓存压力/推动EKF；只有新IMU推动状态发送，忽略Imu.orientation占位 |
| 同上 | `publishState` | 发布源时间状态；把世界线速度旋到B；就绪时初始化home；发布home位姿与TF |
| 同上 | `publishStaticTag`、`publishDiagnostics`、`onResetHome` | 静态地图关系、健康检查、显式home重设；定时器不补发旧pose |
| `src/fusion_node_main.cpp` | `main` | init→构造→单线程spin→shutdown；启动参数错误报告退出 |
| `test/test_hippo_fusion.cpp` | 正式几何/EKF测试 | 独立解析真值、真实上游库、延迟5Hz视觉、200/250Hz IMU、压力、运动/遮挡/时序 |
| `test/test_fusion_ros.cpp` | 正式ROS测试 | 合成数据经DDS进入实际节点，检查home服务、frame/stamp、停止IMU后不补状态 |

Bringup 新增 `config/hippo_fusion.yaml`、`launch/hippo_fusion.launch`、`launch/sensors_readonly.launch`、`rviz/hippo_fusion.rviz`；package.xml 增加新包运行依赖。前一个入口复用原视觉节点；后一个直接启动现有只读传感器桥，可选 Agent，与已有 core 二选一。

**时间、线程和适配边界。** 三类 header 均须处于 Pi 的 ROS 系统时间基准。已有桥用首次 PX4 boot time 与 Pi now 的差建立固定偏移；相机同在 Pi。lab 的接收时刻仅用于通信健康检查，不参与积分。Pi 桥重启/时钟跳变后应重启融合。原库按10ms周期构建状态与协方差，本层按源时间把IMU间隔切分并积累到严格10ms，区间采用当前采样值矩形积分；200Hz/250Hz输入均已测试，实际低频/大抖动输入仍需核验，不宣称配置100Hz就是实测100Hz。

原 `SetVisionData` 会减 `vision_delay_us + 5ms`，且 buffer 长度又依赖该参数。本层保留 `history_s` 长度，并在库入口补回相同量，使库中观测时间仍是相机源时间，不重复补偿延迟；超出历史窗口的视觉明确拒收。库内部视觉超时仍沿用其实现，本层独立以原始源时间给出READY/DEGRADED，避免把预测当新视觉。

所有回调/服务/诊断由单线程 executor 串行执行，只有它修改节点和融合缓存；上游 EKF 同步调用，没有另建控制线程。AprilTag 原有内部线程属于另一视觉进程，本包不参与。第一次有效输出初始化home；reset_home仅清其存储并记录当时视觉stamp，必须等更新观测后重建；遮挡恢复保留home。IMU缺口超过0.10s锁故障，不虚构缺失积分。

### 13.15 复用的上游实现与保留边界

本轮使用本地 USTROV f22f647 的 `ustrov_state_estimation` 计算库；未修改子模块源码或指针。`Estimator` 原ROS包装未启动，因其压力首30帧按水面归零、输出硬编码map/协方差未实现/速度无frame，与当前实验契约不一致。

`Ekf::PredictState` 使用角速度积分姿态、比力积分速度并扣重力；`UpdateHeightFusion` 默认压力高度；`UpdateVisionFusion` 用视觉XY与完整四元数校正。`FuseOrientation` 仍使用固定0.01四元数分量噪声，本层 `vision_position_std_m` 只调位置噪声，不冒称能调其姿态权重；本层在输入处处理 q/-q 同姿态符号。它是当前库的16维四元数分量EKF，不称为新写的15维ESKF或VINS。

压力真实单位/来源、相机粗外参、真实处理延迟仍由现场只读实验检查；本轮的软件验证及操作集中在README§13.17。单标签可见区是当前定位范围，遮挡期间惯性预测会漂移；没有全16标签地图、真实水下真值精度验收或自动控制。本次不新增这些实验前置条件。

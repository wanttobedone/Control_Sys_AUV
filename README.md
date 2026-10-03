# Control_Sys_AUV

ROV 规划、轨迹与控制开发仓库。当前以 ROS 2 Jazzy + Gazebo Harmonic 的 Hippo 仿真作为工程接入平台。

| 目录 | 职责 |
|---|---|
| [ROV_Control](ROV_Control/README.md) | 自研 C++ 算法、ROS 节点、单元测试、XML `.launch`；构建与使用从这里开始 |
| USTROV | 外部 Git 子模块，提供模型、仿真插件、状态桥与基础控制工具 |

本仓库管理自研源码和 USTROV 的固定提交。已有 WSL underlay 位于 `~/rov_sim/USTROV/ustrov_ws`；应保持其提交与子模块一致。新克隆时用 `git clone --recurse-submodules <本仓库地址>`，或在克隆后执行 `git submodule update --init --recursive`。

构建、安装、测试结果、日志和 rosbag 被 `.gitignore` 忽略；`src/*/test/*.cpp` 测试源码保留。`AGENTS.md` 是本地工作约定，按当前要求暂不提交，因此其他机器克隆后不会自动获得该文件。

设计和学习资料统一见 [总体设计](ROV_Control/docs/PATH_TRACKING_DESIGN.md) 与 [MPC 学习路线](ROV_Control/docs/MPC_LEARNING_ROADMAP.md)，不另维护一份重复的实施计划。

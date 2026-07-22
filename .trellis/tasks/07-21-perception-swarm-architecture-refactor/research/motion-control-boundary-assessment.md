# Research: 本机运动意图与飞控边界

- Query: 下一阶段重构是否实现飞控，还是只定义本机规划到外部控制器的可替换边界？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前 `SingleDroneExplorerNode` 发布 `geometry_msgs/msg/PoseStamped` 的 `motion_goal`，`FakeOdomNode` 直接把它转换为确定性 `PoseSegmentTrajectory`。
- 当前命令不是通用飞控契约：只执行 XY + yaw，`position.z` 不作为高度命令，高度由 FakeOdom 内的 AltitudeAdapter 单独仲裁；last-goal-wins、停止确认和速度限制也由当前仿真节点约定。
- workspace 没有 PX4、MAVROS、MAVSDK、Gazebo controller 或物理飞行控制实现；现有设计也把完整物理飞控和动态避障列为范围外。
- 本机安全需要在下发前拒绝不安全意图，并根据执行反馈确认 Accepted/Executing/Holding/Reached/Rejected/Failed；单向 PoseStamped 无法完整表达命令 identity、取消、确认和失败原因。
- 不同外部执行器可能接受 pose、velocity、trajectory 或 action，父级不能把某一种飞控协议写入探索算法。

## Implication

推荐本轮定义 ROS-free `MotionIntent` 与 `ExecutionFeedback` 逻辑契约，并通过可替换 adapter 连接 FakeOdom、未来 Gazebo controller 或真实 autopilot。local planner/safety 负责生成经过 known-free 检查的意图和 Hold/Cancel，adapter 负责映射具体控制协议并回报执行状态；本轮不实现姿态控制、电机混控、物理飞控或厂商协议。当前 PoseStamped goal 仅作为 FakeOdom 兼容 adapter，不是长期公共飞控接口。

## Evidence

- `docs/phases/phase-03-swarm.md:908`
- `docs/phases/phase-03-swarm.md:913`
- `ws/src/drone_scanner/src/FakeOdomNode.cpp:184`
- `ws/src/drone_scanner/src/FakeOdomNode.cpp:245`
- `docs/decisions/perception-and-swarm-architecture-refactor.md:362`

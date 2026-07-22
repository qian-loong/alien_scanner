# Research: 本机运动控制权威与抢占

- Query: coordinator、local executor、安全层和外部 failsafe 谁可以产生最终 MotionIntent？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前 global allocator 发布 ExplorationTask，本机 SingleDroneExplorer 再发布 motion_goal；因此现有正常路径已经是“中央给任务、本机给运动命令”。
- `FakeOdomNode` 使用 last-goal-wins，重复目标只按数值去重，没有 command authority、intent epoch、cancel acknowledgment 或旧 publisher fencing。
- KnownFreePathChecker 和停止确认是本机安全契约；让 coordinator 直接写 actuator command 会绕过本机地图 freshness 和 body/segment/path 检查。
- 稀疏链路会延迟、重复和乱序；多个命令 source 使用最后到达者获胜会使旧命令在重连后复活。
- 真实 autopilot 还可能有硬件 failsafe、急停或人工接管，其优先级必须高于探索意图，但具体厂商协议属于 adapter。

## Implication

推荐每个 vehicle session 只有一个 active local execution authority。Coordinator 只发布 TaskLease/role，不直接发布 MotionIntent；local executor 在本机安全检查后生成 intent。优先级固定为 external emergency/autopilot failsafe > local safety Hold/Cancel > active local execution intent。接管或重启推进 control-authority epoch，旧 epoch 的 intent/feedback 一律拒绝；Relay 只路由任务和状态，不获得 actuator authority。

## 与无人机编队表演的关系

典型无人机表演通常采用“离线编排轨迹 + 中央任务/时间管理 + 机载高频跟踪 + 遥测/故障处置”的分层：中央侧生成并校验带时间参数的轨迹、编队时序和起停命令；每架无人机接收任务或轨迹片段，在本机控制器内以更高频率完成位置、速度、姿态和执行器控制；中央侧接收状态、心跳和到达/失败反馈，而不是持续通过网络发送电机控制量。

这与本项目的共同原则是：控制权威分层、消息带 session/epoch/sequence、低频监督消息与本机高频闭环分离、急停/失联/failsafe 优先级高于正常任务。差异在于表演路径通常预先规划、环境和目标相对确定；本项目的任务由未知地图和 frontier 动态产生，必须持续经过本机 known-free、安全和 freshness 检查，不能直接套用表演的“预加载后按时间执行”假设。

因此，表演系统的可借鉴部分是消息生命周期、时间同步、轨迹/任务分发、遥测和故障抢占机制；不应照搬其中央预编排轨迹作为未知洞穴探索的安全依据。

## Evidence

- `docs/phases/phase-03-swarm.md:848`
- `docs/phases/phase-03-swarm.md:903`
- `docs/phases/phase-03-swarm.md:996`
- `ws/src/drone_scanner/src/FakeOdomNode.cpp:246`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/prd.md:96`

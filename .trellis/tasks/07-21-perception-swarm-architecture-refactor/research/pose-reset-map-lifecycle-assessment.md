# Research: Pose reset 与本机地图生命周期

- Query: 定位 source 重启、frame 改变或 pose discontinuity 时，旧 authoritative local map 如何处理？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前 FakeOdom 轨迹连续，现有 OctoMap builder 没有 pose source session 或 reset epoch，因此基线没有覆盖 estimator 重启和位姿跳变。
- Local occupancy voxel 的空间含义依赖 observation 时刻的 pose chain；新 pose epoch 的坐标不能静默追加到旧 revision chain。
- D-013 已规定 shared alignment 改变时不对旧 aggregate contribution 做首版增量重投影，而是失效旧贡献、请求 keyframe 并重建。
- D-015/D-023 已有 source session 和 reset epoch，可用于确定性识别不连续；但尚未定义它们如何推进 local map epoch、任务状态和 shared contribution。
- 原地重投影旧三维 occupancy map 需要后端相关的插值、冲突与 free/occupied 证据重算，会破坏当前后端无关契约并扩大 C2/C3。

## Implication

推荐首版 fail closed：pose source session 改变、frame 改变或 reset epoch 推进时，立即停止向旧 authoritative map chain 提交观测并暂停受影响任务；旧 local/shared contribution 保留在旧 pose/alignment epoch 下并按 Frozen/Removed 策略处理。定位重新 Ready 且新 alignment 提交后，启动新的 local map source epoch，从空状态或合法新 keyframe 重建，再完成 shared-map/task resync。首版不原地重投影旧地图；若未来 estimator 能证明跨重启连续性，可作为独立 continuity protocol 扩展。

## Evidence

- `ws/src/drone_scanner/src/FakeOdomNode.cpp:424`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:337`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:376`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:153`

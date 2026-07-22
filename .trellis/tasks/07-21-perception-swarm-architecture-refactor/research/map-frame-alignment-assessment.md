# Research: 本机地图与共享地图坐标对齐

- Query: 当前代码是否已经支持每机局部 map frame 的跨机对齐？
- Scope: internal
- Date: 2026-07-21

## Findings

- Phase 3 多机 launch 为所有本机地图传入同一个 `map` frame，并通过模拟 TF 直接建立每机 odom 到该共享 frame 的关系。
- `OctoMapBuilderNode` 在配置的 `map_frame` 中构图并发布完整 OctoMap，因此 source 发送前已经完成坐标变换。
- `GlobalMapMergerNode` 要求输入 `header.frame_id == map_frame`；frame 不一致时拒绝 source，没有配准、对齐质量或 alignment epoch。
- Phase 3 文档明确把“每机独立 OctoMap + 共享 `map`”作为 3-8 前提。这是仿真基线，不足以覆盖真实多机 SLAM 各自局部 frame、漂移和重定位。
- 下一阶段 map update 已计划引入 source epoch/revision 和 delta。若 source-to-shared transform 可变化但没有独立 alignment epoch，同一 voxel key 的含义会在 revision 链中悄然改变，无法正确撤销旧贡献或判断任务坐标是否仍有效。

## Implication

父级必须固定坐标对齐的所有权和版本边界。具体点云/地图配准算法可下放或延期，但 shared view 不能继续把“所有 source 天然在同一 frame”作为隐式契约。

## Evidence

- `docs/phases/phase-03-swarm.md:1365`
- `docs/phases/phase-03-swarm.md:1734`
- `ws/src/swarm_controller/src/OctoMapBuilderNode.cpp:236`
- `ws/src/swarm_controller/src/OctoMapBuilderNode.cpp:276`
- `ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:416`

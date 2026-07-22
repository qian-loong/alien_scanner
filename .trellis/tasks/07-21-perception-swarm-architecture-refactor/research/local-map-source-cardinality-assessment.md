# Research: 本机地图 source 数量与权威性

- Query: 一架无人机是否允许多个本机地图 producer 同时成为协作与安全数据源？
- Scope: internal
- Date: 2026-07-21

## Findings

- Phase 3 每架无人机只有一个 `/drone_i/octomap`；launch 按无人机索引为 merger 生成一条 source topic，当前 topic 名同时承担寻址和 source identity。
- 新父设计已经把 `vehicle_id`、`component_id/source_id` 和各自 session 分层，因此逻辑上能够表达同一 vehicle 下的多个 mapping producer，但尚未定义它们是否都具有相同权威性。
- 多个雷达不要求多个地图 source：一个 mapping pipeline 可以按独立 origin 消费多个 `LidarObservation`，再产生一个一致的本机 occupancy state。
- 若多个使用相同物理观测的 mapper 同时作为 shared-map contributor，会产生重复证据、冲突归属、freshness 门控和任务撤销歧义；本机安全也无法在没有明确选择规则时决定使用哪张图。
- 二次开发和算法 A/B 对比可以通过固定 observation replay 与 conformance suite 完成，不要求在生产无人机上并行运行 alternate mapper。

## Implication

父级应固定本机地图的 authority/cardinality 语义，C2 再设计具体 C++ API、配置和状态机。每个 vehicle session 只运行一个 active mapping pipeline，并产生唯一 authoritative local occupancy source；多个雷达作为该 pipeline 的独立 observation 输入。替代建图算法在启动时按配置选择，通过固定 replay 和 conformance suite 比较，首版不引入实时 shadow mapper。切换算法不属于运行中热切换，需要新 vehicle session 和 map resync。

## Evidence

- `ws/src/swarm_controller/launch/multi_drone_exploration.launch.py:213`
- `ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:224`
- `docs/phases/phase-03-swarm.md:1738`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:104`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:361`

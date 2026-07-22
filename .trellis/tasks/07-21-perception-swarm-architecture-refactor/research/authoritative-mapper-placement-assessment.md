# Research: 权威建图的部署位置

- Query: C1-C8 是否允许将 authoritative mapper 从无人机本机卸载到 Relay/EdgeAggregator/中央节点？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前每机 OctoMap builder 与传感器/odom 位于同一 namespace 和感知栈，本机 explorer 直接消费本机地图。
- 父设计当前写明原始 LiDAR 默认本机处理，跨机默认传 LocalMapUpdate、摘要、位姿、任务和诊断，但“默认”尚未形成强制范围边界。
- EdgeAggregator 已定义为聚合多个 source 的 map update，不是消费原始 scan 的远程 mapper；Relay 只透传领域消息。
- 远程 authoritative mapping 需要把高频 LaserScan/PointCloud2、TF/pose、descriptor、同步和 free-ray 语义纳入 fleet 数据面，并使地图 freshness 依赖链路；本机安全仍需要低延迟本机环境状态，通常会导致第二条本机建图/避障链。
- 用户希望后续支持更多无人机、Relay 和边缘聚合，但当前没有要求资源受限无人机必须卸载原始感知计算。

## Implication

推荐 C1-C8 强制 authoritative mapper 位于 vehicle-local compute domain，可以与传感器分进程但不经过 fleet sparse link。`G_map` 的跨机边界从 revisioned `LocalMapUpdate` 开始；EdgeAggregator 聚合地图更新，不解释原始 LiDAR。原始数据可本地录 bag，并可在未来通过独立 raw-observation transport/remote analytics 扩展上传，但远程结果不能成为首版唯一的本机安全或 authoritative local map source。

## Evidence

- `ws/src/swarm_controller/launch/multi_drone_exploration.launch.py:213`
- `docs/phases/phase-03-swarm.md:1745`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:123`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:278`

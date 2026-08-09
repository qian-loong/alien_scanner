# Research: C3 当前边界与可复用证据

- Query: C3 可以直接复用哪些 C2/Phase 3 能力，哪些能力必须新建，哪些应留给 C4+
- Scope: internal
- Date: 2026-08-03

## C2 已提供

- `LocalObservationMapper` 是唯一权威本机 occupancy writer。
- `MapIdentity` 已包含 `vehicle_id`、mapper session、`map_epoch` 和 `revision`。
- revision 只在 backend 原子 mutation 成功后推进；epoch reset 把 revision 归零。
- `CommitReceipt` 与 `acquire_read_transaction(receipt)` 可以锁定精确 revision；旧 receipt
  返回 `Superseded`，防止 metadata/content 混合。
- `MapReadTransaction` 提供 deterministic known-cell iteration、point/region query、geometry
  与 known bounds。
- 当前 `LocalMapState` 是 metadata-only ROS 消息；当前 OctoMap 输出是 visualization-only
  snapshot，不是同步协议。

证据：

- `.trellis/spec/backend/local-observation-map-contract.md`
- `ws/src/alien_perception/perception_local_map/include/perception_local_map/MapTypes.hpp`
- `ws/src/alien_perception/perception_local_map/include/perception_local_map/LocalObservationMapper.hpp`
- `ws/src/alien_perception/perception_local_map/src/LocalObservationMapper.cpp`
- `ws/src/alien_perception/perception_interfaces/msg/LocalMapState.msg`

## 当前 backend 能力缺口

- `OctoMapBackend::capabilities()` 为 point query、bounded query、reset、known bounds 和
  serialization；`supports_dirty_region=false`。
- `DeterministicVoxelBackend` 同样没有 dirty-region/serialization 能力。
- 因此首版后端无关 delta 不能假设 native dirty set；可行的最小基线是精确 revision
  canonical snapshot comparison。

证据：

- `ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp`
- `ws/src/alien_perception/perception_local_map/test/DeterministicVoxelBackend.cpp`

## Phase 3 可复用 oracle

- 无损 source-level replay 保存三路约 2 Hz 完整 OctoMap，可用于 full -> delta -> full
  离线等价对比。
- 当前文档已确认相邻 snapshot 可推导 added/removed/flipped、dirty bounds 和 content hash，
  但不能表达 session/epoch、gap、乱序或路由行为。
- `OctoMapMerger` 已有 deterministic normalization、snapshot compare、added/removed/flipped、
  资源预检、受控 commit failure 和 revision 保持测试，可作为行为 oracle，不能直接成为
  C3 的公共协议或后端无关容器。

证据：

- `docs/phases/phase-03-swarm.md` 的 source-level replay 收口段落
- `ws/src/swarm_controller/include/swarm_controller/OctoMapMerger.hpp`
- `ws/src/swarm_controller/src/OctoMapMerger.cpp`
- `ws/src/swarm_controller/test/TestOctoMapMerger.cpp`

## C3 与后续边界

- C3：ROS-free 更新语义、canonicalization/hash、producer/applier、gap/resync 状态、
  C3 map-update ROS message、本机短 resync service、OctoMap/reference adapter、直接本地
  topic 闭环、replay 等价和资源边界。
- C4：routed envelope、跨机 resync 路由/重试、QoS、队列/优先级/背压、链路故障和
  传输指标；不重新定义 C3 payload 语义。
- C5：Relay/route/EdgeAggregator/contributor manifest/Frozen contribution 的生产实现。
- C8：N=5 shared/global view、静态 alignment 多进程与 RViz 总验收。

## 性能与调度证据

- `PerceptionLocalMapNode::on_observation()` 在 `submit_observation()` 成功后同步调用
  `publish_octomap()`；当前完整 snapshot serialization 位于 mutation callback 路径。
- `OctoMapBackend::apply()` 以三态 before/after 变化统计 `changed_cell_count`，但只要
  backend 返回 `Applied`，`LocalObservationMapper` 就推进 revision。因此 C3 必须支持
  三态内容不变的 revision-only 空 delta，不能把它判断为缺包。
- C2 bounded 基线的 callback p99 为约 15.6 ms；地图扩展到约 1.81M cells 时 callback p99
  约 93.0 ms，占 100 ms 周期预算的约 93%，其中 snapshot serialization 是主导项。
- 现有 `swarm_controller::LatestSnapshotSlot` 已提供可复用的 pending/in-flight/ready、
  latest-wins、superseded 和 bounded 状态模式；C3 可借鉴其语义，但不应未经边界评审
  直接跨包依赖该内部 helper。

证据：

- `ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp`
- `docs/perception-real-chain-retest.md`
- `docs/local-map-resource-profiling.md`
- `ws/src/swarm_controller/include/swarm_controller/LatestSnapshotSlot.hpp`

## 建议

先用 canonical snapshot comparison 建立后端无关的真实 delta 与恢复语义基线。把 native
dirty-region 作为同一 producer contract 的可选优化能力；只有在 C3 基线性能证据证明 O(n)
生成不可接受时，才单独修改 OctoMap backend mutation 路径。

canonical hash 首版可使用容器已安装的 OpenSSL 3 `EVP` SHA-256。该选择提供成熟、稳定的
跨平台实现，避免项目自写 hash；具体 encoding/hash 范围由 `design.md` 固定并用 golden
vector 验证。

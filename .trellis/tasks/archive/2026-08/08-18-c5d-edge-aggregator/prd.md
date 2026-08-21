# C5d EdgeAggregator 聚合运行期

## 目标

在 C5c Explorer/Relay 运行期之上增加一个真实 EdgeAggregator，跑通 2 Explorer -> 2 Relay -> 1 EdgeAggregator -> 中央接收端的 N=5 数据链。EdgeAggregator 对中央只表现为一个 aggregate source，同时保留有界的 contributor manifest 和各来源恢复状态。

## 范围

- 消费 C4 `RoutedMapUpdate`，每个 contributor 使用独立 ingress/applier。
- 合并 canonical occupancy cells，使用确定性的 Occupied 优先冲突规则。
- 通过现有 `MapUpdateProducer` 生成 aggregate keyframe/delta/remove，并通过现有 `AggregateManifest` 描述 contributor revision/hash/active 状态。
- contributor 的新 session/map epoch、删除、重新加入、resync 和 stale/冲突消息必须 fail closed。
- EdgeAggregator 仅在 `RuntimeSnapshotCache::service_admission(ServiceKind::Aggregation)` 通过时接收新聚合工作。
- 增加薄 ROS 节点、N=5 launch/integration test、聚合状态可视化和独立证据目录。

## 非目标

- 不修改 C4 `MapUpdate v2`、Merkle/chunk、revision、resync 或 trust wire 语义。
- 不在线做地图 alignment；首版要求所有 contributor geometry/frame 一致。
- 不实现 C6 region/task allocator，不把聚合逻辑放入 `PureRelay`。

## 验收标准

- [x] 2 个来源可经任意 Relay 到达 EdgeAggregator，中央只收到一个 aggregate source。
- [x] aggregate revision 与 manifest hash 原子一致；同 revision manifest 冲突不会部分提交。
- [x] contributor keyframe/delta、remove、session/epoch 切换、重新加入和 resync 有确定性测试。
- [x] 同 voxel 冲突结果在重复运行中一致，manifest contributor 列表严格按 vehicle id 排序。
- [x] contributor 数、cell 数、消息历史和诊断计数均受 limits 约束。
- [x] EdgeAggregator 无 AggregationService 或健康门失败时拒绝新输入，不改变已提交输出。
- [x] N=5 launch test 覆盖两来源、多 Relay、EdgeAggregator、中央 aggregate receiver、旧 route 拒绝和恢复 keyframe。
- [x] RViz/MarkerArray 配置显示 EdgeAggregator、贡献者、aggregate revision、active/degraded/resync 状态；本次以静态配置核验和运行期 Marker 链路为证据，未重复进行 GUI 人工截图目检。

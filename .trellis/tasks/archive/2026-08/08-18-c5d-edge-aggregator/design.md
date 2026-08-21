# C5d EdgeAggregator 设计

## 数据流

```text
Explorer 0 -> Relay 0/1 \
                         -> EdgeAggregator -> AggregateMapUpdate -> central receiver
Explorer 1 -> Relay 0/1 /
```

Relay 只转发 `RoutedMapUpdate`。EdgeAggregator 先验证 envelope、producer、route 和 source session，再将消息交给该 contributor 的 `MapUpdateIngress`。每个 contributor 的最终 canonical view 只保留一份有界 snapshot；聚合器不保存逐消息历史。

## 核心状态

- `ContributorState`: source identity、独立 ingress、active、最后合法 revision/hash、geometry 和 bounded diagnostics。
- `aggregate_cells`: 按 `VoxelIndex` 排序的 canonical cells。
- `MapUpdateProducer`: 以 EdgeAggregator 自身 `SourceIdentity` 作为 aggregate source，独立产生 keyframe/delta/remove。
- `AggregateManifest`: contributor source/revision/content_hash/active 严格排序；manifest hash 在完整 aggregate update 构造后计算。

输入更新只在 contributor ingress 成功应用且聚合快照、producer update、manifest hash 全部成功后原子提交；失败时保留旧 aggregate revision、manifest 和 cells。

## 决定性规则

- geometry/frame 必须与首个有效 contributor 一致；不一致直接拒绝。
- 同一 voxel 多来源状态冲突时 `Occupied` 优先，结果按 `(x,y,z)` canonical comparator 排序。
- contributor remove 仍保留 manifest entry，`active=false`，并生成新的 aggregate revision。
- 新 session 或 map epoch 只允许由底层 applier 的单调 source admission 接受；旧 session 永久进入 bounded retired window。

## ROS 边界

`edge_aggregator` 订阅多个 Relay 输出 topic，完成 ROS conversion 后调用核心；发布 `AggregateMapUpdate` 与 bounded `LinkDiagnostic`。`aggregate_receiver` 复用 `AggregateIngress` 做中央验证。节点从 runtime snapshots 读取 `AggregationService` 门控，不复制 role 判断。


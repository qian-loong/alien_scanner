# Phase 3：多机探索与地图融合——决策与答疑

> 本文记录 Phase 3 当前有效的设计解释、答疑和决策边界。
> 内容按功能步骤组织，不按时间线记录未提交改动；正式变更以 `docs/phases/phase-03-swarm.md`
> 和提交记录为准。

关联文档：

- [Phase 3 实施方案与验收](../phases/phase-03-swarm.md)
- [项目总计划](../xenomorph-scanner-plan.md)

## 1. 文档范围

| 主题 | 阶段归属 | 当前结论 |
|------|----------|----------|
| 本机 unknown / free / occupied 形成 | Step 3-3 | 由雷达射线写入本机 OctoMap |
| 多机 peer 状态可视化 | Step 3-7 | 每台 explorer 的 MarkerArray 包含自身状态和本机看到的 peer 输入 |
| 多来源体素混合 | Step 3-8 | 完整快照替换、occupied 优先、无隐式重采样 |
| 全局图显示 | Step 3-8 | `/global_map` 当前用于融合结果观察，不接入 3-7 本机规划 |

## 2. Step 3-3：本机体素状态

### Q：unknown、free、occupied 如何产生？

**A：** 每台无人机的 `OctoMapBuilder` 对每条观测射线执行：

1. 射线中间经过的体素写为 `free`。
2. 命中 endpoint 写为 `occupied`。
3. 同一条射线的 endpoint 从 free 集合中移除，避免命中点同时被写成 free。
4. 未被射线观测到的位置不写入，保持 `unknown`。

本机 OctoMap 使用 OctoMap 自身的 occupancy log-odds 累积，多帧重复观测会改变本机节点的
占用概率。全局融合时不再平均这些概率，而是只读取每个来源最终判定的 free / occupied 状态。

### 决策结论

- `unknown` 不是障碍，也不是自由空间；它表示当前没有足够观测。
- 未知空间不能被当作 free 直接规划穿过。
- 本机建图负责概率累积，策略层负责 known-free 安全检查。

## 3. Step 3-7：Peer 状态与 MarkerArray

### Q：为什么 `/drone_0/exploration_markers` 中有 1、2 号的位置和目标？

**A：** 该话题表示“0 号无人机的决策上下文”，不是“只包含 0 号自身对象”。

每台 explorer 会订阅其他无人机的 `/odom` 和 `/motion_goal`，形成自己的 peer snapshot。随后
同一个 MarkerArray 同时发布：

- 0 号自己的 selected goal、checked path、first blocked 和状态文本；
- 0 号看到的 `peer_positions`；
- 0 号看到的 `peer_active_goals`。

这样可以直接检查分散策略的输入是否接线正确、坐标是否正确、peer 是否新鲜，以及 0 号为何
避开某个目标。

### Q：只勾选 `ExploreMarkers0`，为什么仍能看到 1、2 号内容？

有两个独立原因：

1. `ExploreMarkers0` 自身会绘制 0 号看到的 peer 位置和目标。
2. RViz 顶层 `TF` 是一个共享显示，默认显示所有无人机 frame；它不随 `Odom1/2` 或
   `ExploreMarkers1/2` 的开关关闭。

因此这不是 MarkerArray 话题串线。要严格只观察 0 号自身，需要关闭全局 TF，并关闭
`peer_positions` / `peer_active_goals` namespace；更完整的后续方案是将 self markers 与 peer
markers 拆成独立 topic 或显示组。

### Q：红色立方体代表什么？

红色立方体是 `first_blocked_position`，表示最近一次 body/path known-free 检查发现的第一个
阻塞采样点。它可能对应：

- `UnknownBlocked`：本机观测不足；
- `OccupiedBlocked`：本机 OctoMap 判定为占用。

它不是 truth collision 标记，也不是 `/global_map` 的 occupied voxel。进入 Rescanning 或
ExplorationStalled 后，最近一次失败位置会继续作为诊断上下文保留，直到下一次重新选路刷新。

### 决策结论

- Peer 订阅和 peer 数据仍属于 3-7 算法必需输入，不删除。
- 当前暂不拆分 MarkerArray；可视化隔离属于后续 RViz 组织改进，不改变分散策略。

## 4. Step 3-8：全局体素混合

### Q：多台无人机如何合并同一个体素？

全局融合器对每个 max-depth key 维护来源贡献计数：

```text
free_sources(k)     = 标记 k 为 free 的来源数量
occupied_sources(k) = 标记 k 为 occupied 的来源数量
```

派生全局状态为：

```text
occupied_sources(k) > 0  -> Occupied
否则 free_sources(k) > 0 -> Free
否则                       -> Unknown
```

因此：

| 来源状态 | 全局结果 |
|----------|----------|
| Free + Unknown | Free |
| Free + Free | Free |
| Free + Occupied | Occupied |
| Occupied + Occupied | Occupied |
| Unknown + Unknown | Unknown |

### Q：为什么 occupied 优先，而不是多数投票或概率平均？

这是安全侧的保守规则。即使多个来源认为某处 free，只要有一个来源发现 occupied，就不能让
free 覆盖潜在障碍。全局融合不做来源可信度、距离、时间或概率加权，也不平均各机的 log-odds。

### Q：来源更新是增量叠加还是完整替换？

是完整快照替换。每个来源保留上一份规范化快照，新快照与旧快照逐 key 差分：

- 新快照新增的 key，加入该来源贡献；
- 新快照缺失的旧 key，删除该来源贡献；
- 同一 key 状态翻转，更新该来源的 free / occupied 计数；
- 完全相同的快照只返回 `AcceptedUnchanged`。

因此来源重建或重置后不会永久留下旧体素 ghosting。

### Q：压缩的 pruned leaf 如何混合？

浅层 leaf 会覆盖一个立方体区域，不能只取中心点。融合器按：

```text
level = tree_depth - leaf_depth
width = 2^level
展开数量 = width^3
```

将其展开为全部 max-depth key，然后排序、去重。同一来源内部重复覆盖时 occupied 优先。
展开前会检查乘法溢出和单来源上限，失败时整条来源更新原子拒绝。

### Q：Unknown 是否会覆盖 Free 或 Occupied？

不会。Unknown 表示该来源没有该 key 的记录，不是一个显式“清空命令”。只有当所有来源都
不再贡献该 key 时，全局状态才回到 Unknown。

### Q：一个来源停止发布后，旧贡献会自动消失吗？

不会。stale 只产生诊断 WARN，不自动删除地图贡献，因为“没有新消息”不等于“旧观测无效”。
来源必须：

1. 发布合法空快照；或
2. 发布不再包含旧 key 的新完整快照；或
3. 被显式 `removeSource()`。

### Q：`source_revision` 和 `global_revision` 有什么区别？

- `source_revision`：任一来源快照内容变化就增加。
- `global_revision`：全局派生状态实际变化才增加。

例如 A 从 free 改成 occupied，但 B 已经将同一 key 标为 occupied：A 的 source revision 增加，
但 global revision 不增加，因为全局显示状态未变。

### Q：全局节点如何处理高频来源消息？

回调只做 envelope 检查并锁存每个来源最新 pending 快照；timer 再执行反序列化和融合。相同
来源在一个融合周期内的中间快照被 coalesce，不会重复累计概率或贡献。

输入要求：

- `frame_id` 必须为 `map`；
- 只接受 `binary=false` 的完整 `OcTree`；
- resolution 必须一致；
- 时间戳严格递增；
- serialized bytes、单来源展开体素数、全局体素数均受上限保护。

### 决策结论

- 全局融合是确定性的三态集合合并，不是概率融合。
- occupied 优先是安全契约。
- `/global_map` 当前用于共享地图展示和后续能力预留，3-7 explorer 仍使用各自本机 OctoMap。
- 当前不做全局 frontier 分配、全局路径规划或 stale 自动删图。

## 5. 当前阶段边界

当前文档记录的结论不意味着已经实现以下能力：

- 全局地图驱动的任务分配；
- 多机在全局 frontier 上的唯一所有权；
- 动态障碍物时间预测；
- 3D 全局路径规划；
- 将 peer 可视化与自身 MarkerArray 完全拆分。

这些能力属于后续步骤或独立可视化改进，不应与当前体素混合契约混淆。

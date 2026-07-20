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
| 全局图使用边界 | Step 3-8/3-9 | `/global_map` 用于全局任务区域检测，不替代本机运动安全图 |
| 全局 frontier 任务分配 | Step 3-9 | support-v2 已让真实 bag 产生 Region；Component 语义尚未选定最终修复；freshness 后台管线及 revision 级时延诊断已实现，等待真实负载归因与验收 |

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
- `/global_map` 由 3-9 allocator 用于全局 frontier 区域检测；3-7/3-9 explorer 的运动安全检查
  仍只使用各自本机 OctoMap。
- 3-8 本身不承担任务分配、全局路径规划或 stale 自动删图。

## 5. Step 3-9：全局 frontier 与任务分配诊断

### Q：如何在 RViz 中证明正在执行 3-9，而不是 3-7 自由探索？

飞行路线彼此分开不能证明任务分配生效，因为 3-7 的 peer 分散也可能产生类似结果。应同时
检查任务模式、全局 Marker 和 allocator diagnostics：

| explorer 文本 | 含义 |
|---------------|------|
| `task=0 id=0` | `LocalFallback`，执行 3-7 本地探索 |
| `task=1 id=N` | `Assigned`，接受非零全局任务 ID |
| `task=2 id=0` | `Standby`，有可执行边但区域已分配给其他无人机 |
| `LocalFallback(expired/unavailable)` | allocator 不可用或任务租约失效 |

`/global_task_markers` 中：

- `global_frontier_regions` 的绿色球体表示已经成熟的稳定 region；
- `global_task_assignments` 的蓝色连线表示无人机到所属任务目标的所有权关系，不是实际飞行路径。

只有 `status.message=Coordinated`、至少两台无人机 `task=1`、非零且不重复的 task ID，以及蓝色
owner 连线相互一致，才能作为人工任务分配证据。入口只有零或一个稳定区域时保持
`LocalFallback` 是设计内行为。

### Q：为什么 GlobalTaskMarkers 显示 Ok，却没有绿色球体和蓝色连线？

allocator 每周期都会发布两个 Marker namespace，即使其中的点列表为空。RViz 的 `Status: Ok`
只表示收到过格式合法的 MarkerArray，不表示当前存在 stable region、Assigned task，或发布节点
仍然存活。

绿色球体为空表示没有可显示的稳定 region；所有 explorer 为 `task=0 id=0` 时，蓝色连线按定义
必然为空。RViz 还会保留发布者退出前的最后一帧，因此 launch 结束后 Marker 仍可能显示 Ok。

### Q：本次 rosbag 实测得到什么结论？

验证数据位于本地 `rosbags/swarm_3_9_20260716_091008`，有效录制 222.3 秒、50372 条消息。
三台无人机各收到 166 条任务消息，全部为：

```text
mode=0, task_id=0, revision=1
```

`Coordinated` 样本为 0，`detected_regions`、`tracked_regions`、`eligible_edges` 和
`matching_cardinality` 全程为 0，因此本轮运动全部来自 3-7 本地探索。真值审计 18297 条均为
`Clear`，没有真实穿墙。

### Q：OctoMap leaf、voxel 和 detector column 分别是什么？

- voxel 是几何空间中的立方体；
- leaf 是 OctoMap 八叉树中不再细分的终端节点，保存 free/occupied 概率状态；
- unknown 通常表示对应节点不存在，不是一种显式 leaf；
- merger 将来源 leaf 规范化为 0.1 m max-depth key；
- column 是 detector 创建的二维聚合单元，不是 OctoMap 原生节点。

默认 `column_stride_voxels=2`，所以一个 column 的 XY 底面为 0.2 m × 0.2 m；同一 XY 范围内
不同 Z 高度的 free leaf 被聚合到同一 column。当前 `raw_frontier_columns` 实际统计所有包含
free leaf 的 sampled column，名称不能解释为“真正的 frontier 数量”，后续诊断应更名并增加
`frontier_candidate_columns`。

### Q：垂直检测主要检查什么？

垂直检测检查 free/unknown 边界是否具有足够高度，而不是判断无人机上下方碰撞或实际路径安全。
同一个 column、同一个水平 unknown 方向必须满足：

```text
不同 Z key 数量 >= 5
Z 跨度 >= 0.4 m
```

该条件用于过滤单帧倾斜环形成的薄扫描边缘。前倾环会使不同 Z 证据同时发生 XY 偏移，因此
单帧证据可能分散到相邻 column；但移动、yaw 重扫和三机融合会在累计地图中补充垂直层。

envelope-v1 历史基线样本中，8127 个具有 unknown 邻接的候选 column 里有 6303 个通过垂直阶段，
通过率约 77.6%。因此该基线已证明垂直条件不是当时的最强瓶颈。

### Q：envelope-v1 历史基线中的 440 个支撑体素如何产生？

旧算法对通过垂直检查的 frontier column，根据 unknown 方向向已知 free 内侧检查一个长方体：

```text
深度 0.8 m：8 层
宽度 1.0 m：11 个采样点
高度 0.4 m：5 层
8 × 11 × 5 = 440 个体素
```

每个被访问的体素都必须存在且为 free；遇到 unknown、occupied 或越界即拒绝并提前退出。
该检查用于证明 frontier 内侧存在已观测自由支撑，不检查 unknown 外侧，也不等同于某台无人机
的本机 first-hop 路径安全检查。

旧版 440 个查询体素不单独显示。RViz 的 OctoMap 只显示原地图，无法指出某个 support box、
第一个失败体素或失败类型。完整调试应只发布有界前 N 个 support box/失败点，不能显示全部
候选，否则会进一步增加 Marker 与处理负担。

### Q：envelope-v1 历史基线当时最大的 detector 瓶颈是什么？

该历史样本的检测漏斗为：

```text
具有 unknown 邻接的候选 column：8127
通过垂直检测：                  6303
通过支撑检测：                    27
形成 region：                       0
```

支撑阶段拒绝 6276 个，支撑通过率约 0.43%。剩余 27 个 supported column 又无法组成满足列数、
面积、跨度和方向一致性的连通 component。因此当时首先需要定位 support rejection 的
unknown/occupied/越界类型和空间分布，不能把零 Region 主要归因于 Z 层数量条件。support-v2 已解除
这一层阻断；当前待决策瓶颈是 Component 连通/方向语义。此前同步执行造成的 freshness 抖动已在独立
后台执行管线中修复，不能与 Component 行为修改混合验收。

### Q：merger 耗时是否导致 allocator 一步错、步步错？

不是状态污染意义上的“一步错、步步错”，而是吞吐与时效契约不匹配。bag 中全局地图消息年龄
中位数为 3.566 秒、最大 4.701 秒，merger 最大融合耗时为 4.012 秒，而 allocator freshness
timeout 为 5 秒。allocator 又在单线程 timer 中同步反序列化并扫描最高约 142 万 known voxel，
导致订阅和 timer 回调积压。

结果表现为同一 update sequence 先短暂 `global_map_fresh=1`，随后又变成 0，并安全退回
`LocalFallback`。地图没有损坏，也没有资源拒绝，但协调活性无法稳定维持。envelope-v1 detector
当时还会因 region 为零而无法分配，所以语义与执行管线是两个独立阻断；evidence-ray-v2 已解除前者
的第一层 support 阻断。独立的有界后台 decode/detect 管线已经修复“重型工作直接阻塞 ROS timer/订阅
回调和无界积压”这一执行问题，并由 invalid/recovery、负载、clock reset 和 shutdown 用例覆盖；但这不
等于真实三机负载下 5 s 端到端 freshness 已经闭环。

### Q：这里的 allocator 管线、revision 和 freshness 分别是什么？

管线是地图从 ROS 输入到任务发布的执行路径：envelope 校验 -> latest-wins pending -> worker claim ->
OctoMap decode -> global Frontier detect/local tree decode -> ready -> timer apply -> allocator update -> 发布。
它只描述执行方式，不改变 Component、Region、匹配或 owner 算法。

管线 revision 是每个来源的内部单调版本，用于区分：

- `latest`：最新已接受输入；
- `pending/in_flight/ready`：等待、处理中、等待应用；
- `consumed`：结果已确认消费，包括失败结果；
- `applied`：成功替换最后合法状态的结果。

latest-wins 可以合并中间 pending revision，因此 revision 用于证明处理进度和版本身份，不保证每个版本
都执行。它也不同于成功 global map 才推进的 `global_update_sequence`，以及任务语义变化使用的 task
revision。

freshness 回答“最后成功应用的数据现在是否仍可用于分配”。global map 同时检查成功应用后的 receive
age 和消息 `header.stamp` 的 ROS 时间年龄；global diagnostics、local map 和 odom 也各有自己的 timeout。
revision 很新并不代表 fresh：例如 `latest=194/applied=193/in_flight=194` 表示 worker 正在追赶，但若
applied 193 的任一年龄超过 5 s，allocator 仍必须回到 `LocalFallback`。

### Q：为什么后台管线修复后，真实运行仍出现 `global_map_fresh=0`？

2026-07-19 的带 RViz 长时间运行和关闭 RViz 的受控复现均显示：节点、回调和 revision 持续推进，
`global_map_processing_failures=0`、`global_map_worker_failures=0`，说明没有重现 executor 阻塞。但最后
成功应用 map 的 receive age 约为 3～4 s，global Detector 约为 1～2 s；上游 merger 的历史最大融合
耗时约为 2.6～5.1 s，单 worker 还要轮询三份 local decode。receive age 尚未超过 5 s 时 freshness
仍可能为 0，说明 applied revision 的 header stamp age 也已超限。

revision 级有界时延诊断现已补齐。global map 与每份 local map 都发布最后一次 consumed revision 的：

- accepted 到 worker claim 的 queue wait；
- decode、global detect、worker 总耗时；
- worker complete 到 timer claim/apply 的 apply wait；
- receive 到 consume 的总延迟；
- receive 时和 consume 时的 header age。

queue/decode/detect/apply/total 使用 steady clock，header age 使用 ROS clock。当前 applied revision 另行
发布 receive age 和 header age；失败结果会推进 consumed timing，但不会覆盖 applied revision/age。诊断
只保留每个来源最后一次 consumed timing 和当前 applied 状态，不保存无界历史。

自动测试已覆盖旧 header、失败 consumed-but-not-applied、真实 backlog 下的 pending coalescing，以及
ROS clock rollback 后恢复前的清空中间态。该实现仍未改变单 worker round-robin、5 s freshness timeout、
Component/方向语义或本机探索算法。下一步应在默认三机真实负载中采集这些字段，连续多个周期比较
queue、local decode、global detect、apply wait 与 header lag 后，再决定是否需要独立评审调度修改。

2026-07-19 随后完成关闭 RViz、开启 truth audit 和 detector stage timing 的 180 秒稳态采集。按
`last_consumed_revision` 去重后共有 38 个 global revision；revision 从 72 推进到 110，
`processing_failures=0`、`worker_failures=0`，但 181 个 allocator 周期全部为 `global_map_fresh=0`。
关键中位数为：

| 阶段 | 中位耗时/年龄 |
|------|--------------:|
| merger 实际重融合周期 | 4.07 s |
| global map 到达 allocator 时的 header age | 4.80 s |
| global queue wait | 2.56 s |
| global decode | 0.28 s |
| global detect | 3.41 s |
| global apply wait | 0.76 s |
| global receive -> consume | 7.09 s |
| global consume 时 header age | 11.87 s |

Detector 的 3.34 秒内部总耗时中，leaf scan 中位 3.17 秒；vertical、support、component 分别只有
0.08、0.05、0.02 秒。三份 local decode 中位数分别为 0.11、0.08、0.18 秒，相加约 0.38 秒。
因此 local round-robin 会增加延迟，但不是当前主要瓶颈。即使理想化消除全部 local decode、queue 和
apply wait，4.80 秒的到达 header age 再加 global decode/detect 仍明显超过 5 秒。当前证据不支持先做
local 加权调度或仅增加 worker；下一方案必须同时处理上游 merger 全量重融合和 Detector 近百万 free
voxel 的全量 leaf scan，且不能通过重写 header stamp 或增大 timeout 隐藏观测年龄。

同轮 truth audit 的 8007 个状态全部为 `Clear`。Detector 已从 1 个增长到 6 个 accepted Region，但因
allocator 输入全程不健康，`eligible_edges` 和 `matching_cardinality` 仍为 0，不能用本轮判断最终任务
匹配效果。原始本地证据保存于 `ws/log/allocator_freshness_capture.json`，该目录属于 colcon 忽略输出，
不进入 Git。

### Q：merger 的历史 4.07 秒具体耗在哪里，是否应先改通信拓扑？

A-only 已增加独立 `merge_cycle_id` 和 source/cycle 分阶段诊断，不修改融合或发布语义。Release 默认三机
连续采集 `181.49 s`，得到 182 个连续 cycle，全部 accepted；关键 p50/p90 为：

| 阶段 | p50 | p90 |
|------|----:|----:|
| cycle total | 0.561 s | 0.631 s |
| 三份 source decode | 0.134 s | 0.159 s |
| decode cleanup | 0.088 s | 0.104 s |
| normalize | 0.203 s | 0.230 s |
| `updateInnerOccupancy` | 0.027 s | 0.043 s |
| serialize | 0.091 s | 0.119 s |

`decode + normalize` 占 cycle p50/p90 约 `60.1% / 61.6%`，是当前 merger 主成本；snapshot compare、
delta/apply 和 inner update 不是主要部分。accounting remainder ratio p90 仅 `0.028%`，诊断覆盖完整。

本次固定为 Release；历史 `4.07 s` 采集没有与本次冻结成相同 build profile/运行窗口，因此不能将差异
解释为 A 的优化，A 本身不改变性能行为。Release 本轮 182 个 allocator 样本全部 `global_map_fresh=1`，
processing/worker/resource/envelope failures 为 0，truth audit 21,801 次全部 `Clear`。

这说明未来 source delta 或并行 source preparation 可能有效，但仅改 peer/Relay 拓扑不会自动减少中央
merger 的完整 source 处理量。当前不存在 Relay 类型，本轮不实现拓扑、增量地图或边缘聚合。原计划是
先完成 B，再用 A+B 同配置复测决定 C；该计划已被 2026-07-20 的参考基线收口决策取代，B/C 延期。

2026-07-20 的 Sol 复核后，诊断实现又完成了边界收紧：cycle 前后 source/global revision 已发布到
`DiagnosticArray`；`coalesced_before_claim` 是当前 pending slot 的计数，下一次单帧 claim 会回到 0；
`fullMapToMsg()` 的返回失败和抛异常都保留为 `merge_applied=1/published=0/serialization_failure`，不会
退出 merger。单一旧 key 跨位置替换时先处理新增状态，再处理删除状态，并增加反向、多 key 和最终 leaf 数
测试，避免空 root 继承旧 occupied。

异常边界遵循 3-8 既有契约：会返回 `Invalid` 的 envelope/normalize/resource/delta-preflight 以及受控
commit hook 均在逻辑变更前拒绝；提交阶段实际 `bad_alloc` 不是可恢复业务错误，节点不捕获后继续运行，避免
出现 contributions/tree 已变而 source snapshot/revision 未同步的“继续服务”状态。为避免诊断改变被测性能，
未采用每个 cycle 全量复制三份状态的方案；该方案在短 Release smoke 中把 cycle p50 从 `0.561 s` 推到
`0.928 s`，且 accounting remainder p90 达 `13.86%`。最终增量候选的 56-cycle 短 smoke 中 cycle
p50/p90 为 `0.651/0.739 s`，contribution/tree apply p50 低于 `0.001 s`、source commit p50 约
`7 us`、remainder ratio p90 为 `0.035%`；57 个 allocator 样本全部 fresh 且失败计数为 0。正式性能
结论仍以固定配置的长窗口复测为准。

clock launch 测试现在先确认回拨前合法 watermark，再在回拨后发送低于旧 watermark 的 stamp，断言
envelope rejection、cycle ID、revision 和 output stamp 不变，随后用更高 stamp 验证恢复。它与纯 helper
共同覆盖 future/epoch age invalidation，不改变原有 admission 语义。

### Q：绿色球为什么与无人机不在同一岔道，三机又为什么停止？

绿色球来自 `/global_task_markers` 的 `global_frontier_regions`，表示 stable Region representative，
不是某架无人机的 Assigned target。现场 `global_task_assignments.points` 为空，三机任务均为
`task_mode=0/task_id=0`，所以不存在应当连向绿色球的 owner-target 任务线。后续将新增仅对 Assigned
任务发布的独立 target Marker，并保持 Region marker 使用不同 namespace/颜色。

三机最终状态均为 `ExplorationStalled/NoSafeCandidate`，body 为 `Safe`，truth audit 为 `Clear`；这是
3-7 固定规模前向 known-free 短跳在当前本机地图中耗尽后执行的安全 Hold，不是 allocator 将无人机派往
错误岔道。本轮 freshness 诊断不修改 `SingleDroneExplorer`、`KnownFreePathChecker` 或本机候选语义。

### Q：什么是“真实地图语义校准”？

这里的真实地图是当前倾斜环、OctoMap Builder 和 merger 实际生成的 `/global_map`，不是
`/cave/points`、洞穴中轴线或分支真值。校准目标是让 detector 正确解释运行时体素形态：

```text
单环扫描薄边       -> 不形成虚假多任务
持续直廊前沿       -> 可形成一个 region，但不启动多机协调
成熟双分支         -> 形成两个稳定 region
unknown/occupied   -> 不成为直接运动目标
```

全局 detector 只证明“这是值得分配的持续探索区域”；本机 first-hop 继续证明“该无人机当前能
安全开始朝它前进”。不得通过校准放宽本机 body/segment known-free 安全契约。

### Q：现有 bag 如何用于分阶段语义校准？

第一轮先补齐诊断，不改变任何选择结果：

- sampled free、unknown-neighbor candidate、垂直 layer/span 通过与拒绝计数；
- envelope-v1 support 的 unknown/occupied/out-of-bounds 原因、首次失败深度/横向/高度分布；
- component 数量、大小分桶、最大尺寸，以及列数/面积/跨度/方向一致性拒绝计数；
- leaf scan、vertical、support、component 和总耗时。

随后新增仅在 `BUILD_TESTING` 下构建的 C++ bag analyzer，直接顺序读取 MCAP 中的
`/global_map`，逐帧调用 ROS-free `GlobalFrontierDetector` 并输出 CSV/JSON。这样不受 ROS timer、
QoS 和 freshness 抖动影响。生产模式保持首次失败即退出并只保留有界统计；envelope-v1 的完整
440 体素分布只在离线历史分析中计算。

算法修改按以下顺序一次只改变一个语义：

1. 支撑证据与本机运动安全的职责边界；
2. supported column 的 component 连通与区域形成；
3. 倾斜环垂直证据的相邻 column 聚合；
4. 最后才调整物理参数阈值。

每次修改都用同一 bag 比较，并将最终语义写成合成 OcTree gtest；bag 作为本地分析数据，不作为
仓库 CI 依赖，也不得把一次运行中的洞穴真值写入算法。

### Q：第一项 support 职责修复结果如何？

Detector 已改为上中位 Z、raw-key 字典序 anchor 和单体素 inward known-free ray；ray 外空间不参与
Detector support，本机 `KnownFreePathChecker` 行为未修改。旧 `support_width` 和 lateral/vertical
failure bins 已删除。

相同 68 帧 bag 的两次新 CSV SHA-256 均为
`a609d2821f0a28735b02ce83b85f13c187585cf033f014d0040b95a684ff93ea`：support pass 从 `1,199`
增至 `191,067`，36 帧为 `Accepted`，共得到 45 个 Region。下一瓶颈转移到 Component：9,195 个
component 中 8,690 个因 `min_columns` 拒绝、460 个因 direction consistency 拒绝。本轮没有降低
阈值或修改连通，后续按既定顺序独立评审。

### Q：第二次 headless 复测后，当前性能是否已经可以判定为完成？

不能。第二次同配置 Release 运行 `181.589 s`，181 个连续唯一 merger cycle（45～225）的 total
p50/p90 为 `0.702/0.801 s`，decode + normalize 仍占约 `60.4%/61.3%`；失败为 0，22,326 个 truth
状态全部 `Clear`。allocator freshness 为 `180/182`（98.9%），其中两次 applied header age 达
`5.02/5.64 s`。它证明 headless 三机中央式路径在该次运行中基本健康，也证明 5 s 阈值附近仍有波动，
不能外推为所有负载下均已解决。该 bag 的 SHA-256 为
`a1e5e83cc650612dbfca599c7920a4588a0d7f4a0886dd1d5189293c475b4a3c`。

带 RViz 的后续长时间现场给出了相反压力证据：15/15 个观察样本均 non-fresh，applied header age 约
`6.1～11.1 s`。因此收口结论是“可诊断、可复现的参考基线”，不是“性能问题完成”。

### Q：三架飞机为什么会先后停住，入口左侧飞机为什么会漂到右侧后最先停？

三机起始 `Y` 是不同初始位置，不是不同 yaw。它们使用同一类本机候选与短跳策略，但各自 OctoMap、
扫描历史、peer 排斥和候选时序不同，所以会先后选择不同的横向目标。现场中 `drone_1` 在启动后约
`50.9 s`、累计约 `6.50 m` 后停在 `(5.12,-1.40,1.50)`；另外两机分别运动约 `215.5/250.8 s`、
约 `23.5 m` 后才停。

三机最终状态都是 `ExplorationStalled/NoSafeCandidate`，body 为 `Safe`，truth audit 为 `Clear`，并非
碰撞或 allocator 错派。当前 `ExplorationStalled` 只会在收到 Assigned/Standby 或 OcTree size 变化时
恢复；仅有占用概率更新但 leaf 数不变时不会重新选路。横向候选、局部地图差异和这个恢复条件共同解释了
“曾经都靠右，但只有一架留在原处”。该行为是已知本机探索限制，本轮不修改。

### Q：当前完整 OctoMap bag 能否支持未来增量地图研究？

可以作为输入推导，但不等于已经支持增量协议。当前三路 `/drone_i/octomap` 都是 `fullMapToMsg()` 完整
快照。离线规范化相邻快照后，可以确定 added、removed、free/occupied flipped、dirty bounds 和 content
hash，用来验证 keyframe + delta 是否能重建同一状态。

完整快照无法恢复两次发布之间的逐 scan 更新顺序，也没有 session epoch、丢包/乱序或 Relay 路由信息。
因此 source-level bag 是未来算法对照资产；真正上线 delta 时仍需定义协议、关键帧、重同步和传输语义。

主资产改为容器内
`/workspaces/alien-scanner/ws/log/source_level_replay_loss_debug_seed42_20260720_131653`：seed 42、Release、
无 RViz，有效时长 `180.369575846 s`，18 个话题共 29,286 条消息，MCAP 为 `890,325,768` bytes。
三路 `/drone_i/octomap` 分别为 359/361/359 条，实际频率约 `1.99/2.00/1.99 Hz`；`/global_map`
为 150 条。MCAP SHA-256 为
`0f290620968453564b741a2b1aea3a548a385fbd2ca7c256fa1eb41a01175687`；`metadata.yaml` SHA-256 为
`2efb24146f9adc03490b46d08677384a5631836c09722445e95d25445bb736ef`。资产位于 Docker named volume，
不进入 Git。

当前 rosbag2 `0.26.10` 实际没有 `MessagesLostEvent` 消息或 event topic；上游按话题统计发布仍为 TODO。
新包使用 `--log-level debug` 保存 recorder 自身 DDS `QOSMessageLostInfo` 回调，从而在发生丢失时记录具体
topic。本轮没有任何 per-topic callback，也没有停止时的非零总数警告；同版本源码只在总数大于 0 时打印
该警告，因此可确定新包 transport loss 为 0。日志也没有 duplicate publisher、non-increasing stamp、
ERROR 或 FATAL。

旧包 `/workspaces/alien-scanner/ws/log/source_level_replay_seed42_20260720_123603` 保留不清理：三路 source
各 362 条、`/global_map` 180 条，但有 1 条无法归属话题的 transport loss。未来 snapshot/delta、边缘聚合
和拓扑研究默认使用无 transport loss 的新包；需要更密 `/global_map` 时间序列时，旧包作为第二份对照，
不能作为全话题逐条无损证据。

### Q：为什么现在收口，而不是继续 Detector B、merger C 或通信拓扑改造？

当前诊断已明确中央 merger 的主要成本是完整 source decode + normalize，Detector leaf scan 也曾在不同
构建/负载下成为主要成本；但 headless 与 GUI 现场差异表明，尚不能用一个局部优化宣称整体闭环。同时
当前没有 Relay 角色或增量协议，提前改 peer 拓扑不会自动降低中央 merger 对完整 source 的处理成本。

因此本轮冻结中央式三机参考基线：保留 revision/原子提交/分阶段诊断、Detector oracle、allocator epoch/
revision/lease 和安全契约；延期 B、C、Component 行为、本机 stalled 恢复、Relay/稀疏拓扑、delta/
keyframe、EdgeAggregator、角色/编队和 `N > 3`。3-9 保持未完成，3-10 不开始。

### 决策结论

- Step 3-9 support 职责修复已让真实 bag 产生 Region，但完整运行仍需验证 track、eligible edge、
  assignment 和真实运行状态，不能据此宣称验收完成。freshness 后台执行解耦和 revision 级时延诊断
  已通过自动验证，但真实负载端到端 freshness 尚未完成归因与闭环。
- Component 连通/方向语义仍未选定行为修复；allocator 时延诊断已补齐，当前不直接降低阈值、增大
  timeout 或修改调度。
- 真实负载证据已排除“local round-robin 是主要瓶颈”；A-only 与重复 Release 运行确认 merger 的完整
  source decode + normalize 是主要阶段。B/C 已延期，不在当前收口中继续实施。
- allocator 重型检测已与 ROS 回调解耦，但端到端性能修复不能替代 detector 语义校准。
- 任务区域证据可以重新设计；本机 first-hop、body 和 segment 安全检查不得放宽。
- 当前成果冻结为中央式三机参考基线；3-9 不勾选，3-10 不启动。另录 source-level replay bag 供未来
  delta/keyframe、边缘聚合和拓扑方案做同输入离线比较。

## 6. 当前阶段边界

当前文档记录的结论不意味着已经实现以下能力：

- 已通过真实单环地图人工验收的全局任务分配；
- 动态障碍物时间预测；
- 3D 全局路径规划；
- 长距离 A*/Theta* 或全局地图运动安全规划；
- 将 peer 可视化与自身 MarkerArray 完全拆分。

3-9 当前只提供全局区域所有权和本机 known-free 短跳引导；实际运行仍处于诊断与校准阶段，
不能与已完成验收的动态避碰、全局路径规划或复杂 3D 传感能力混淆。

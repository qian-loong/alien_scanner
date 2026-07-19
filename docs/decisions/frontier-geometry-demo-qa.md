# Frontier Geometry Demo QA

## 1. 范围与基线

本文定义 `frontier_geometry_demo` 当前版本的 RViz 验收规范和判读边界，只记录当前稳定行为、验收证据
和当前验收结论。V2 Stage 0–4 与 V3 Component Audit Stage 5–8 分开判定，V3 不改变 V2 算法链。

QA 重点区分：

- 标准参考 fixture 与真实观测 replay；
- 全局 Detector 结果与 selected endpoint 局部教学视图；
- 完整 OctoMap 数据与抽样 Marker；
- 算法状态与 RViz 教学可读性。

默认基线为 cave seed `42`、初始位置 `(1, 0, 0)`、`144` beams、`144` yaw steps、pitch
`20 deg`、max range `3.0 m`、OctoMap resolution `0.1 m` 和固定 `+X 0.8 m` hop。

五阶段旧版 support envelope 的可重放 Marker 基线位于
[`docs/bags/frontier-geometry-demo-support-envelope-v1`](../bags/frontier-geometry-demo-support-envelope-v1/README.md)。
修改后的 evidence ray 基线位于
[`docs/bags/frontier-geometry-demo-support-evidence-ray-v2`](../bags/frontier-geometry-demo-support-evidence-ray-v2/README.md)。
两份 MCAP 都固定 `selected_phi_degrees=0`，每个阶段话题各一条消息；它们用于相同相机下的前后
并排图，不作为 Detector 正确性或真实场景通过条件。

## 2. 阶段合成与 RViz 控制

默认启动：

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/alien-scanner/ws/install/setup.bash
ros2 launch swarm_controller frontier_geometry_demo.launch.py
```

一个 launch 启动 V2 Demo 节点和独立的 Component Audit Replay 节点。V2 只生成一次供 Stages 1–4
共享的真实 replay：

| RViz Display | MarkerArray 话题 | 默认状态 |
|---|---|---|
| `Stage 0: Standard Tunnel Geometry` | `/frontier_geometry_demo/stages/standard_tunnel_geometry/markers` | 关闭 |
| `Stage 1: Single Ring` | `/frontier_geometry_demo/stages/single_ring/markers` | 关闭 |
| `Stage 2: Bootstrap Yaw Sweep` | `/frontier_geometry_demo/stages/bootstrap_yaw_sweep/markers` | 关闭 |
| `Stage 3: Validated Observation Hop` | `/frontier_geometry_demo/stages/validated_observation_hop/markers` | 关闭 |
| `Stage 4: Accumulated Frontier` | `/frontier_geometry_demo/stages/accumulated_frontier/markers` | 打开 |
| `Stage 5: Component Audit Overview` | `/frontier_component_audit/stages/audit_overview/markers` | 关闭 |
| `Stage 6: Component Rejection` | `/frontier_component_audit/stages/component_rejection/markers` | 关闭 |
| `Stage 7: Direction Evidence` | `/frontier_component_audit/stages/direction_evidence/markers` | 关闭 |
| `Stage 8: Gap Counterfactual` | `/frontier_component_audit/stages/gap_counterfactual/markers` | 关闭 |

顶层 Display 控制整个阶段的显示和隐藏；展开 Display 后的 Namespace 控制阶段内部图层。允许同时打开
多个 V2 阶段，但 V2 五个场景使用同一地图坐标，叠加时会出现重合，这是比较模式的预期结果。Stage 0
是标准圆柱几何参考，不代表 ProceduralCaveField 的实际洞壁；Stages 1–4 才属于同一真实 replay。

Stages 5–8 读取仓库内 frame 3 的 component/membership 小快照。Stage 5 是统计图；Stages 6–8 是 XY
membership 审计图，各自把局部重心重定位到同一显示中心，不与 V2 或其他 V3 阶段的绝对地图坐标建立
语义对齐。重定位只为固定视野，不改变阶段内部的列间距、阈值、方向票数或反事实结果。

## 3. 当前判读规范

### QA-01：Stage 0 与真实 replay 的边界

`standard_tunnel_geometry` 是标准圆柱隧道参考 fixture，用于展示坐标系、法向/倾斜切面、pitch、扫描面
法线、标准环线和共面射线。它没有 `DemoPipelineSummary`，也不代表 ProceduralCaveField 的实际洞壁。

Stages 1–4 才来自同一次真实 replay：FakeLidar raycast、OctoMap 累积、known-free hop 和
`GlobalFrontierDetector::detectWithTrace()`。不能用 Stage 0 的标准圆解释真实 cave 的局部凹凸。

### QA-02：地图 Marker 与完整 OctoMap 的边界

- Stage 1 只显示固定 yaw 帧的一圈真实 returns，不显示体素地图；
- Stages 2–3 以初始传感器为中心显示 `max_range + resolution` 范围；
- Stage 4 围绕 selected endpoint 显示局部地图和 analysis window；
- OCCUPIED Marker 约抽样 `1/2`，FREE Marker 约抽样 `1/8`。

因此 Marker 之间的视觉空隙不等于 OctoMap unknown。Detector 始终读取未抽样的完整 OcTree，局部显示
窗口和抽样只影响 RViz。

### QA-03：默认 selected endpoint 位于正前方

`frontier_selected_endpoint` 默认位于无人机正前方，这是固定教学视角，不是 Detector 或探索策略选择的
目标。yaw sweep 固定捕获 `yaw=270 deg` 的扫描帧，`selected_phi_degrees=0` 在 map frame 中对应
`+X`。该端点可能是真实 hit，也可能是没有命中时的 max-range endpoint。

### QA-04：`selected_phi_degrees` 的角度含义

`selected_phi_degrees` 是固定 yaw 帧内 360 度扫描环的 beam 角，不是无人机 yaw、下一步飞行航向或
Frontier Detector 参数。忽略 pitch 时，它沿切面按以下顺序变化：

```text
0 deg 前方 -> 90 deg 顶部 -> 180 deg 后方 -> 270 deg 底部 -> 360 deg 前方
```

当前 pitch 为 `20 deg`，所以扫描切面略有倾斜。固定 `yaw=270 deg` 时，单位方向为：

```text
d_map(phi) = (cos(phi), -sin(phi) * sin(20 deg), sin(phi) * cos(20 deg))
```

因此 `90 deg` 主要向上并略偏 `-Y`，`270 deg` 主要向下并略偏 `+Y`。

### QA-05：`selected_phi_degrees:=72.0` 显示局部无 candidate

**现象：** 最终阶段显示 `LOCAL: NO FRONTIER CANDIDATE`。

**结论：** `72 deg` 的方向约为 `(0.309, -0.325, 0.894)`，主要朝向洞顶。该消息只表示以所选
endpoint 为中心、半径约 `1.5 m` 的局部窗口内没有 recorded candidate，不表示“无路可走”。

建图和全量 Detector 已在选择 `selected_phi` 前完成。修改该参数不会改变 OcTree、全局检测状态或
accepted Regions。Marker 同时显示：

```text
LOCAL: NO FRONTIER CANDIDATE
GLOBAL: N ACCEPTED REGION(S)
```

即使全局只剩一个 accepted Frontier Region，它也只是探索目标候选。能否飞往该 Region 仍需路径连通、
机体净空、碰撞检测和运动学约束验证；Frontier 本身不是可通行证明。当前 Demo 的固定 `+X 0.8 m`
known-free hop 也与 `selected_phi` 无关。

### QA-06：Component 颜色与两侧紫色柱

| 颜色 | Namespace | 含义 |
|---|---|---|
| 黄色 | `frontier_candidate` | selected endpoint 窗口内聚焦的 candidate column |
| 绿色 | `component_columns` | 通过 Component 接受条件的多列连通分量 |
| 橙色 | `component_columns` | 被 Component 条件拒绝的多列连通分量 |
| 紫色 | `component_singletons` | XY 八邻域中没有相邻 supported column 的单列分量 |

紫色竖柱虽然包含多个 Z 体素层，在算法中仍只算一个 XY column。默认生产阈值要求至少 `12` 列，因此
singleton 不会形成 accepted Region。它不代表障碍物、洞壁、可行通道或建议飞行方向。

### QA-07：Stage 3 的时间点

`validated_observation_hop` 位于 known-free 检查完成、第二轮扫描尚未开始的时间点。hop 安全时无人机
位置已经更新到终点，但 `observation_epochs` 仍为 `1`，且不显示第二轮体素或 Detector 结果；hop
不安全时位置保持在初始点。

### QA-08：Stage 4 的 support evidence ray

当前 Detector 不再检查 `support_depth × support_width × min_z_span` 三维包络。每个 direction attempt
使用上中位 Z、同 Z 下 `(x,y)` 字典序最小的 raw key 作为 anchor，并沿 unknown 反方向检查固定 Z、
单体素宽的连续 ray：

- `support_anchor` 以小球明确标出当前 attempt 的唯一 anchor，并与方向箭头和 evidence ray 起点重合；
- `support_inward_direction` 表示完整要求深度；
- `support_evidence_ray` 只连接 anchor 和 Trace 中最后一个已记录 sample；
- `support_known_samples` 为已访问的 free key；
- unknown/occupied/out-of-bounds sample 表示首个失败，失败后未访问的位置不显示。

ray 外 unknown 或 occupied 不属于全局 Detector support 判据，也不代表可安全通行。Stage 3 和运行期
allocator 仍通过独立 `KnownFreePathChecker` 检查机体与 first-hop segment。

`support_anchor` 不是 selected endpoint、candidate column 的几何中心、Region representative 或探索
目标；这些位置不同是预期行为。

### QA-09：为什么 v2 Demo 中看不到 `min_columns` 的位置？

`min_columns` 不是空间位置，而是 Component 的列数阈值。Detector 先将通过 vertical 和 support 的
体素按 XY 聚合为 column，再按 XY 八邻域连接成 Component，随后按固定顺序检查：

```text
column count >= min_columns
area >= min_area
horizontal span >= min_span
direction consistency >= min_direction_consistency
```

生产默认 `min_columns=12`，表示同一 Component 至少包含 `12` 个 supported XY columns；一个 column
可以包含多个 Z 层，因此它不是“至少 12 个体素”。阈值本身没有 Marker 坐标。`region_decision` 小球
位于 Component representative；绿色表示通过全部 Component 条件，橙色表示被某项条件拒绝，但小球
位置不是 `min_columns` 的位置。

evidence-ray-v2 的默认 combined 局部窗口只显示一个通过的 Component，所以画面中没有
`min_columns` 拒绝案例。真实 68 帧 `/global_map` analyzer 中 `8,690 / 9,195` 个 Component 在
`min_columns` 首判据被拒绝，但它们只承载 `17,458 / 191,067`（`9.137%`）的 supported columns；
direction reject 反而承载 `90.448%`。这是另一数据集上的全局统计，不能从这个固定合成局部窗口直接
读出，也不能只按被拒 Component 数量判断应降低 `min_columns`。
V3 Stage 5–6 现在直接提供真实 bag frame 3 的列数分布和代表性拒绝位置：

```bash
ros2 launch swarm_controller frontier_geometry_demo.launch.py \
  include_component_audit:=true
```

Stage 6 显示 `15` 列 accepted Component 与 `10`/`6` 列 `min_columns` rejects，并给出 `12` 列标尺；
这些列来自 normalized membership，不能恢复逐列 Z 层。原有的概念教学 fixture 仍可用于隔离说明：

```bash
ros2 launch swarm_controller frontier_geometry_demo.launch.py \
  mode:=component_fragmentation \
  min_component_columns:=12
```

该 fixture 显示 `17 COLUMNS -> 11 COMPONENTS`，但不宣称来自默认 combined 的 Detector Trace。

### QA-10：Allocator freshness 是否属于任务分配算法？

它属于 `global_task_allocator` 的运行期执行管线，但不属于 Region 匹配、评分或唯一 owner 选择算法。
当前重型 OctoMap decode 和全图 Frontier detect 已移到单独的有界后台 worker；timer 线程只应用完成
结果、调用 `GlobalTaskAllocator::update()` 并发布任务。global map 使用非递减 stamp，本机 map 使用严格
递增 stamp；每个来源最多保留一个 in-flight、一个 ready 和一个 latest-wins pending 快照，worker 在
global/local 来源间 round-robin，持续 global 输入不会饿死本机地图。

已开始处理的 revision 允许先于更新的 pending revision 完成并应用，否则持续输入会使旧计算永远被
丢弃、最新计算又永远追不上。完成结果必须严格晚于已消费 revision；pending 中间版本会被合并，后续
仍处理最新版本。invalid/resource-limit 结果只推进 consumed revision，不推进 applied revision、
`global_update_sequence` 或最后合法地图的 stamp/receive-time；旧合法地图按真实 age 继续变旧。较新的
无效 envelope 也不能被较旧的 in-flight 成功结果清除。

诊断公开 latest/consumed/applied/pending/in-flight/ready revision、pending coalescing、处理失败、资源
拒绝和最后合法地图 age。所有 map/diagnostics/odom 输入在构造 ROS 时间对象前校验正时间戳；未来 stamp
不进入 admission watermark。检测到 ROS clock 回拨时，节点清空旧时钟域地图和瞬态工作、重置 stamp
watermark，但保持 revision 单调，等待较低时间戳的新域输入恢复。

受控 launch test 已覆盖 global/local invalid -> 保留旧状态与真实 age -> 同 stamp 修正恢复、负/未来
时间戳、stale replay、持续高频输入下 coalescing/round-robin、ROS clock 回拨恢复，以及 worker 负载中
allocator 以退出码 `0` 清洁结束。该修复没有修改 Component 连通、Region 阈值、`KnownFreePathChecker`
或 `GlobalTaskAllocator` 的匹配/owner 逻辑。

### QA-11：Stage 5 审计统计的来源

Stage 5 的 `108` 个 Component、`2,519` 个 supported XY columns 和颜色分类来自
`swarm_3_9_20260716_091008` 的 analyzer frame 3，不是 V2 默认 `combined` 的现场 Detector replay。
它使用完整 detail/membership 快照，不按 RViz Marker 数量推导统计；`min_columns` reject 的数量很大，
但 column mass 只有 `242/2519`，不能据此直接修改生产阈值。

### QA-12：Stage 6/7 的平面审计边界

Stage 6 的列块代表 XY column footprint，绿色/橙色分别表示完整 Component 判定的 accepted 和
`Columns` primary rejection。Stage 7 的四支箭头是 C16 的 aggregate direction votes；箭头长度只
表达 vote 比例，不是无人机航向、support inward ray 或可飞行方向。membership 快照不含逐列 Z，因此
Stage 6/7 不展示垂直层数，也不替代 V2 Stage 4 的真实 Trace。

Stage 7 的计票来自生产 Detector 的水平 unknown 邻接判据：对每个已知 free voxel 检查同一 Z 层的
`+X / +Y / -X / -Y` 邻居，邻居在 OcTree 中不存在时给对应方向增加一票；已知 free 或 occupied 邻居
不增加该方向的票。票数先在 XY column 内按方向聚合。candidate 的 support attempt 按票数降序、
direction enum 升序尝试；第一个同时通过 vertical 和 inward evidence ray 的方向被选中，只有该方向
的票数保留到 supported column，其他方向票数清零。Component 再累加其 supported columns 的保留票数。

因此 Stage 7 的一致性是按 unknown 邻接证据数加权计算的：

```text
direction_consistency = max(+X, +Y, -X, -Y) / 四方向票数总和
```

C16 的票数为 `+X=89, +Y=34, -X=90, -Y=33`，总数 `246`，dominant 为 `-X`，一致性为
`90/246=0.366`，低于 `0.65`，所以在已通过列数、面积和跨度检查后被判为 `Direction reject`。
`-X` 只表示 unknown 邻接票数在四个方向中最多，不表示它占多数；对应的 inward evidence ray 方向是
相反的 `+X`，但该
ray 不是 Stage 7 箭头所显示的内容。箭头长度是相对比例示意，不是实际距离或飞行向量。

### QA-13：Stage 8 的 pair-only 与 radius-2 边界

Stage 8 选择 frame 3 中确定性排序的 C26/C37。一列离散间隔定义为两组 membership 的最小 Chebyshev
距离减一；pair-only 只合并已有 supported columns，C26+C37 可通过现有四项阈值。全图传递 radius-2
连通是独立反事实：frame 3 的 baseline accepted 为 `1`，传递合并后 accepted group 为 `0`，主要
原因是方向票数混合。该结果否决的是“无条件扩大连通半径或直接采用 gap bridge”这一具体候选方案，
不等于已经选定了另一种 Component 修复方案，也不等于 Component 行为问题已经解决。

### QA-14：Stage 5–8 是审计证据，不是修复结果

Stage 5–8 读取固定的 analyzer frame 3 component/membership CSV。V3 replay 节点不重新运行生产
`GlobalFrontierDetector`，不修改 `min_columns`、`min_area`、`min_span` 或
`min_direction_consistency`，不改变 Component 连通，也不把反事实合并结果送入 Region、track 或
allocator。因此这四个阶段用于回答“拒绝发生在哪里、候选修改可能造成什么回归”，不表示算法已经按
画面中的合并或分类运行。

当前修改应分开理解：

- `support envelope -> 单体素 inward evidence ray` 是此前已实施的真实 support 行为修复，改变了
  Detector 的 support 判定和 V2 Stage 4 结果；
- Component 精确列数、四方向票数、membership CSV 以及 Stage 5–8 是行为保持不变的审计和可视化增强；
- allocator freshness/后台 decode-detect 管线已作为独立执行层修复，不改变 Component 或分配算法行为。

### QA-15：Component 连通/方向语义的修复决策状态

截至当前审计，Component 连通或方向语义尚未选定最终修复方案，也没有对应的生产行为修改。Stage 7
证明了部分 supported columns 在同一空间 Component 内存在方向混合；Stage 8 进一步证明无条件扩大
连通半径会把更多 Component 合并为方向不一致的 group，并可能使已有 accepted Region 回归为 reject。

所以当前结论是：已排除一个过于宽松的修复方向，但还没有证明应当拆分 Component、采用方向感知连通、
选择性处理 gap，或调整任何阈值。任何行为修改都需要另立方案、通过审查，并用薄观测、成熟直廊、双分支、
occupied barrier、方向混合和缺口 fixture 做前后对比；在此之前保持现有 Component 规则不变。

## 4. 完成度与开放项

| 范围 | 状态 | 证据或剩余工作 |
|---|---|---|
| 真实观测与 Detector 数据链 | 完成 | 固定 replay、同一 OcTree、一次最终 Detector |
| Stage 0–4 场景合成 | 完成 | 五个独立 transient-local MarkerArray 话题 |
| Stage 5–8 Component Audit | 自动化完成，视觉待验收 | 只读 frame 3 快照、ROS-free 反事实和四个独立话题已接入，不改变生产行为 |
| support-v2 行为修复 | 完成 | 三维 support envelope 已替换为单体素 inward evidence ray；V2 前后 bag 可重放对比 |
| Component 连通/方向语义行为修改 | 未选定，暂缓实现 | Stage 8 否决无条件 radius-2/gap bridge；需要独立方案和行为审查 |
| allocator freshness/后台执行管线 | 实现与自动验证完成 | 有界 latest-wins slot、单 worker round-robin、revision/freshness 屏障、invalid/recovery/stale/shutdown 测试 |
| ROS 接线与晚加入缓存 | 完成 | launch integration test 覆盖九话题、QoS 和 Namespace |
| ROS-free 算法与 Marker 映射 | 完成 | V2/V3 与 allocator pipeline 的 `swarm_controller` CTest `28/28` 通过；共 238 项测试结果无失败 |
| 可重放视觉基线 | 完成 | v1/v2 五阶段各 1 条 MarkerArray；V3 使用 CSV 快照而非 bag |
| 代码审查 | 完成 | `gpt-5.6-sol` 最终窄范围复核无剩余 finding |
| Stage 4 默认视角的标签可读性 | 完成 | endpoint/scope 与 support 状态分置 analysis window 两侧，固定视角复核通过 |
| V2 五阶段最终 GUI 验收 | 完成 | envelope-v1 于 2026-07-17 通过；evidence-ray-v2 于 2026-07-18 通过 |
| V3 四阶段 GUI 验收 | 待进行 | 需固定相机逐阶段检查标签、footprint、vote arrows 和 gap 边界 |

当前 evidence-ray-v2 的功能、完整 CTest、Sol 复核、可重放 bag 和 RViz 目检均已完成；V3 已完成
实现和自动化验证，尚未宣称视觉验收完成。

## 5. 人工验收清单

验收结果：envelope-v1 于 2026-07-17 全部通过；evidence-ray-v2 于 2026-07-18 全部通过。v1/v2
均使用仓库内同一 Orbit 配置和同一窗口尺寸，分别单次播放 bag 后检查稳定帧；Stages 0–3 输出未受
support-v2 影响，Stage 4 完成前后对照和标签可读性复核。

1. 默认启动后出现五个 Display，且仅 Stage 4 默认打开。
2. Stage 0 显示标准圆柱、法向/倾斜切面、pitch、环线和共面射线，不包含 voxel 或 pipeline。
3. Stage 1 只显示一圈真实 scan returns，不显示 voxel、hop 或 Frontier。
4. Stage 2 显示以初始传感器为中心的完整 max-range 初始地图。
5. Stage 3 显示 fixed hop；不公开第二轮体素或 Detector 结果。
6. Stage 4 的 candidate、support anchor、support、Component、Region 均能追溯到同一次 Detector Trace。
7. 关闭 `component_singletons` 后，紫色单列分量消失，其他 Component 不受影响。
8. `selected_phi_degrees:=72.0` 局部无 candidate 时仍显示 GLOBAL accepted Region 数量。
9. 改变 `selected_phi_degrees` 不改变 region stable key、已知体素数或 hop 状态。
10. 默认相机下标签不互相遮挡主要几何，不遮挡 endpoint、candidate 和 Region 的对应关系。
11. Stage 5 的 component count / column mass 两条统计和来源标签可读，且颜色分类与文本一致。
12. Stage 6 能看到 accepted、`min_columns` rejects 和 `12` 列标尺；关闭某个 audit Namespace 不影响其他层。
13. Stage 7 的四个 vote arrows 与 C16 文本中的 `89/34/90/33` 一致，箭头没有被标签遮挡。
14. Stage 8 能看到 C26/C37 两组列、一列间隔虚线、pair-only 通过和 radius-2 回归结论。
15. Stage 6–8 的局部重定位边界和“XY footprint / 非完整 Z”标签可读，不被误认为 V2 的同一地图坐标。

## 6. 判读边界

- Marker 抽样空隙不等于 OctoMap unknown；Detector 使用未抽样的完整 OcTree。
- 局部无 candidate 不等于全局无 Frontier。
- 全局无 accepted Frontier 也不能单独证明物理上无路可走，还可能涉及观测覆盖和检测阈值。
- accepted Frontier 不等于路径可达，必须由规划与安全检查继续判定。
- Demo 不控制无人机，也不代表运行期探索策略已经选择该 endpoint 或 Region。

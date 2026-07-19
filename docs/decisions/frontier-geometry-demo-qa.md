# Frontier Geometry Demo QA

## 1. 范围与基线

本文定义 `frontier_geometry_demo` 当前版本的 RViz 验收规范和判读边界，只记录当前稳定行为、验收证据
和当前验收结论。

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

一个 Demo 节点发布一个标准参考 fixture，并只生成一次供 Stages 1–4 共享的真实 replay：

| RViz Display | MarkerArray 话题 | 默认状态 |
|---|---|---|
| `Stage 0: Standard Tunnel Geometry` | `/frontier_geometry_demo/stages/standard_tunnel_geometry/markers` | 关闭 |
| `Stage 1: Single Ring` | `/frontier_geometry_demo/stages/single_ring/markers` | 关闭 |
| `Stage 2: Bootstrap Yaw Sweep` | `/frontier_geometry_demo/stages/bootstrap_yaw_sweep/markers` | 关闭 |
| `Stage 3: Validated Observation Hop` | `/frontier_geometry_demo/stages/validated_observation_hop/markers` | 关闭 |
| `Stage 4: Accumulated Frontier` | `/frontier_geometry_demo/stages/accumulated_frontier/markers` | 打开 |

顶层 Display 控制整个阶段的显示和隐藏；展开 Display 后的 Namespace 控制阶段内部图层。允许同时打开
多个阶段，但五个场景使用同一地图坐标，叠加时会出现重合，这是比较模式的预期结果。Stage 0 是
标准圆柱几何参考，不代表 ProceduralCaveField 的实际洞壁；Stages 1–4 才属于同一真实 replay。

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
需要单独观察列数和碎裂关系时，可运行教学 fixture：

```bash
ros2 launch swarm_controller frontier_geometry_demo.launch.py \
  mode:=component_fragmentation \
  min_component_columns:=12
```

该 fixture 显示 `17 COLUMNS -> 11 COMPONENTS`，但不宣称来自默认 combined 的 Detector Trace。

### QA-10：Allocator freshness 是否属于任务分配算法？

它属于 `global_task_allocator` 的运行期执行管线，但不属于 Region 匹配、评分或唯一 owner 选择算法。
当前节点在 timer 中同步执行 OctoMap 反序列化和全图 Frontier 检测；处理百万级 leaf 时会阻塞同一执行
链上的订阅和 timer。检测结束后，地图或 diagnostics 可能已超过 freshness timeout，于是输入健康检查
失败并安全退回 `LocalFallback`，即使 ROS-free 匹配算法本身没有错误。

后续修复应把重型 decode/detect 移到有界后台 worker，合并过时 pending 快照，只发布与最新输入 revision
一致的完成结果，并保持 stale/replay/关闭时序和资源上限。该工作解决“任务分配算法能否持续获得新鲜
输入并及时运行”，不得与 Component 连通或 Region 阈值修改混在同一行为改动中。

## 4. 完成度与开放项

| 范围 | 状态 | 证据或剩余工作 |
|---|---|---|
| 真实观测与 Detector 数据链 | 完成 | 固定 replay、同一 OcTree、一次最终 Detector |
| Stage 0–4 场景合成 | 完成 | 五个独立 transient-local MarkerArray 话题 |
| ROS 接线与晚加入缓存 | 完成 | launch integration test 覆盖五话题、QoS 和 Namespace |
| ROS-free 算法与 Marker 映射 | 完成 | evidence-ray-v2 的 `swarm_controller` CTest `23/23` 通过；包级 199 tests 无失败 |
| 可重放视觉基线 | 完成 | v1/v2 均为五阶段各 1 条 MarkerArray |
| 代码审查 | 完成 | `gpt-5.6-sol` 最终窄范围复核无剩余 finding |
| Stage 4 默认视角的标签可读性 | 完成 | endpoint/scope 与 support 状态分置 analysis window 两侧，固定视角复核通过 |
| 五阶段最终 GUI 验收 | 完成 | envelope-v1 于 2026-07-17 通过；evidence-ray-v2 于 2026-07-18 通过 |

当前 evidence-ray-v2 的功能、完整 CTest、Sol 复核、可重放 bag 和 RViz 目检均已完成。

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

## 6. 判读边界

- Marker 抽样空隙不等于 OctoMap unknown；Detector 使用未抽样的完整 OcTree。
- 局部无 candidate 不等于全局无 Frontier。
- 全局无 accepted Frontier 也不能单独证明物理上无路可走，还可能涉及观测覆盖和检测阈值。
- accepted Frontier 不等于路径可达，必须由规划与安全检查继续判定。
- Demo 不控制无人机，也不代表运行期探索策略已经选择该 endpoint 或 Region。

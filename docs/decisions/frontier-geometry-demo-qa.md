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
该 MCAP 固定 `selected_phi_degrees=0`，每个阶段话题各一条消息；它用于相同相机下的前后并排图，
不作为 Detector 正确性或真实场景通过条件。

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

### QA-08：Stage 4 的 support anchor

`support_anchor` 是当前 selected direction attempt 的实际 anchor，以紫色小球显示。它必须与
`frontier_unknown_direction`、`support_inward_direction` 和 support envelope 前端面的起点一致，
并直接映射到 Detector Trace 的 `attempt.anchor`。它不是 selected endpoint、candidate column 的几何
中心、Region representative 或探索目标；这些位置不同是预期行为。

## 4. 完成度与开放项

| 范围 | 状态 | 证据或剩余工作 |
|---|---|---|
| 真实观测与 Detector 数据链 | 完成 | 固定 replay、同一 OcTree、一次最终 Detector |
| Stage 0–4 场景合成 | 完成 | 五个独立 transient-local MarkerArray 话题 |
| ROS 接线与晚加入缓存 | 完成 | launch integration test 覆盖五话题、QoS 和 Namespace |
| ROS-free 算法与 Marker 映射 | 完成 | `swarm_controller` CTest `23/23` 通过 |
| 可重放视觉基线 | 完成 | 五阶段各 1 条 MarkerArray，MCAP 约 156 KiB |
| 代码审查 | 完成 | Terra 修复后复核无剩余代码 finding |
| Stage 4 默认视角的标签可读性 | 通过 | 2026-07-17 人工目检确认 |
| 五阶段最终 GUI 验收 | 通过 | 2026-07-17 人工目检确认阶段、遮挡、颜色和 Namespace 开关可接受 |

因此当前 Demo 的功能、数据语义、自动化验证和 RViz 视觉验收均已完成。

## 5. 人工验收清单

验收结果：2026-07-17 全部通过。

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

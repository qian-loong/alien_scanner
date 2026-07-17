# Frontier Geometry Demo

## 1. 目的与边界

`frontier_geometry_demo` 是用于解释倾斜垂直二维雷达、OctoMap 体素状态和全局 Frontier 检测几何的
RViz 教学工具。它发布静态 `visualization_msgs/msg/MarkerArray`，不控制无人机，也不参与运行期任务
分配。

当前版本的完成度、人工验收规范和判读边界记录在
[`decisions/frontier-geometry-demo-qa.md`](decisions/frontier-geometry-demo-qa.md)。

该工具需要同时满足两项边界：

- 展示内容必须与真实 `GlobalFrontierDetector` 的数据语义一致；
- 教学视图可以选择和放大局部区域，但不得通过视觉偏移伪造算法关系。

目标场景是一架无人机的确定性静态回放，不运行飞控，不引入多机、地图融合或全局任务分配。洞穴
真值只允许供 LiDAR raycast 生成观测；短跳是否可执行只能读取已构建的 OctoMap，不能读取洞穴中轴、
实体查询或分叉真值。

默认启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/alien-scanner/ws/install/setup.bash
ros2 launch swarm_controller frontier_geometry_demo.launch.py
```

默认合成模式一次发布五个阶段话题，RViz Fixed Frame 为 `map`：

```text
/frontier_geometry_demo/stages/standard_tunnel_geometry/markers
/frontier_geometry_demo/stages/single_ring/markers
/frontier_geometry_demo/stages/bootstrap_yaw_sweep/markers
/frontier_geometry_demo/stages/validated_observation_hop/markers
/frontier_geometry_demo/stages/accumulated_frontier/markers
```

节点启动时立即发布五个静态场景，默认不周期重发。五个话题使用 transient-local QoS，RViz Display
晚加入或重新打开时会收到各自缓存的最后一帧。需要观察重复发布行为时可显式设置
`republish_rate_hz`；周期重发只重新序列化 Marker，不重新构造 OcTree 或运行 Detector。

## 2. 展示内容

### 2.1 观测几何

- `coordinate_axes_x/y/z`：`+X` 前进、`+Y` 左侧、`+Z` 向上的右手坐标系；
- `drone`：雷达原点和机体位置；
- `tunnel`、`tunnel_axis`：圆柱隧道及其中心轴；
- `normal_scan_plane`：未倾斜的垂直参考扫描面；
- `pitched_scan_plane`：绕 `+Y` 轴倾斜后的扫描面；
- `scan_plane_normal`、`pitch_reference`、`pitch_angle`：倾斜方向和角度参考；
- `scan_rays`、`scan_hits`、`pitched_scan_ring`：射线、洞壁命中点和扫描面与圆柱洞壁的交线。

### 2.2 OctoMap 状态

- `voxel_free`：射线穿过并确认无障碍的体素；
- `voxel_occupied`：射线末端命中的占据体素；
- `voxel_unknown`：尚未被观测的体素；
- `voxel_cut_diameter`：仅用于说明体素切片方位，不代表真实洞壁。

### 2.3 Frontier 检测阶段

- `frontier_selected_endpoint`：教学视图选中的扫描射线端点；它既可能是真实 hit，也可能是
  max-range free-ray endpoint；
- `frontier_candidate`：Detector 从局部地图中识别出的候选 column；
- `analysis_window`：围绕选中区域显示的局部检测窗口；
- `frontier_unknown_direction`：candidate 面向 unknown 的方向；
- `support_anchor`：当前显示的 direction attempt 使用的 anchor，以小球标出；
- `support_inward_direction`：从 candidate 指向已知空间内部的支撑检查方向；
- `support_pass_envelope`：支撑检测使用的三维包络；
- `support_known_samples`：包络内实际检查的 OctoMap 采样位置；
- `component_columns`：组成多列连通分量的 supported columns；
- `component_singletons`：未与其他列连通的单列 component；
- `component_links`：真实 XY 八邻域关系的可视化；
- `region_decision`：Component 接受或拒绝结果，位置使用生产 Detector 输出的
  `FrontierRegion::representative`，不是 selected candidate 或 Component 外框的几何中心。

## 3. 真实实现流程

完整实现以一架无人机的两次定点观测和一份持续累积的 OctoMap 为唯一数据源：

```text
固定初始 XYZ，执行离散 yaw sweep
  -> 每帧调用真实 ICaveField/FakeLidar raycast
  -> lidar-frame endpoint 转换到 map frame
  -> OctoMapBuilder 将射线路径写入 FREE、命中端写入 OCCUPIED
  -> KnownFreePathChecker 检查唯一固定 +X 短段
  -> Safe 才更新单机位置；否则停在入口并显示原因
  -> 新位置再次 yaw sweep，继续写入同一棵 OcTree
  -> GlobalFrontierDetector 扫描地图
  -> unknown-neighbor candidate voxels
  -> 按 XY 聚合为 columns
  -> vertical 检测
  -> support 检测
  -> supported columns
  -> XY 八邻域 Component 分组
  -> Component 条件检查
  -> Region 接受或拒绝
```

固定短段只用于验证第二观测位姿确实 locally known-free，不承担目标搜索：不调用
`FrontierExplorationStrategy`，不尝试其他方向，也不根据 Region 选择短跳。短段不安全时场景必须停在
第一观测 epoch。移动后必须完成第二次真实扫描再运行 Detector；单独改变位姿不会增加地图信息。

洞穴类型、seed、初始位姿、yaw 序列、beam 数、pitch、分辨率和 hop distance 都必须固定并进入测试。
默认使用生产 `GlobalFrontierDetectorConfig`；若教学 fixture 必须覆盖某项阈值，文档和 Marker 诊断必须
逐项公开，不能静默降低 Region 接受门槛。

Demo 不应独立复制一套 Frontier 算法。`GlobalFrontierDetector` 应提供可选的只读分析轨迹，返回候选
column、vertical 结果、support 采样位置、Component 成员、邻接关系和拒绝原因。关闭分析轨迹时不得
增加生产运行路径的持续开销，也不得改变 regions、状态、原因或稳定排序。

### 3.1 selected endpoint 的职责

建图和全量检测必须先完成，`frontier_selected_endpoint` 随后才用于选择教学视图聚焦的局部区域。默认选择角可使用
默认 `selected_phi_degrees = 0`。修改该参数只改变最终观测环中选中的射线和观察区域，不改变
两次 yaw sweep、累积 OcTree、Detector 的判定规则或全量检测输出。

```text
selected phi
  -> 选中扫描射线和端点
  -> 在端点附近建立观察窗口
  -> 从真实 Detector 输出中选择窗口内最近的 candidate/component
```

若选择半径内没有 candidate，Demo 应同时显示 LOCAL 无 candidate 和 GLOBAL accepted Region 数量，不能
人工生成 candidate。端点与 candidate 的关系是观测关联，不是固定距离变换。

### 3.2 Candidate 与 Support

Candidate 必须由 FREE/OCCUPIED/UNKNOWN 邻接和真实 Detector 规则产生。不得再使用
`selected_endpoint + inward * 0.38 m` 等展示偏移生成 candidate；`0.38 m` 没有算法意义。

Support 使用统一的局部正交坐标基：

```text
depth axis    = inward
vertical axis = global +Z
lateral axis  = vertical x inward
```

`support_inward_direction` 从 candidate 的几何中心出发，沿包络中心轴指向 inward，长度等于实际
`support_depth`，不进行避让偏转。包络线框和所有 sample 必须使用同一坐标基。采样点颜色来自对应
OctoMap 状态，而不是固定制造一块示意体素云。

### 3.3 Component 与 Region

Component 必须由同一 analysis window 中通过 vertical 和 support 的 columns 产生。不同颜色表示不同
连通分量，`component_links` 只连接真实满足 XY 八邻域关系的列，并统一放在列顶面高度。

Region 结果放在生产 Detector 输出的 `representative`：

- Component 被接受：显示接受状态和生产 representative；
- Component 被拒绝：在该 Component representative 显示拒绝状态和主要原因；
- 全部 Component 被拒绝：不得在初始 candidate 上显示类似已接受 Region 的结果球。

## 4. RViz 演示流程

同一个 Demo 节点同时发布一个标准几何参考阶段和四个真实 replay 阶段：

```text
standard_tunnel_geometry
single_ring
bootstrap_yaw_sweep
validated_observation_hop
accumulated_frontier
```

`standard_tunnel_geometry` 是 Stage 0，复用标准圆柱隧道 fixture，展示法向切面、pitch 后切面、扫描面
法线、pitch 角、标准环线和共面射线；它不包含 OctoMap、Detector 或 `DemoPipelineSummary`。

RViz 的五个顶层 MarkerArray Display 分别控制一个阶段，默认只打开
`Stage 4: Accumulated Frontier`。勾选或取消阶段 Display 即可切换，也可以同时打开多个阶段进行叠加比较；
每个 Display 内仍可通过 Namespace 检查该阶段的数据层。Stages 1–4 共享一次洞穴生成、两轮观测和
一次 Detector replay，不会因切换 Display 重新选择 seed 或运行 Detector。

`validated_observation_hop` 位于“路径检查完成、第二轮扫描尚未开始”的时间点：hop 安全时
`final_position` 已更新到 hop 终点，但 `observation_epochs` 仍为 `1`，且不公开第二轮体素或 Detector
结果；hop 不安全时位置保持在初始点。

### 4.1 坐标和观测

```text
coordinate_axes_x
coordinate_axes_y
coordinate_axes_z
drone
tunnel_axis
pitched_scan_plane
scan_plane_normal
pitch_reference
pitch_angle
scan_rays
scan_hits
pitched_scan_ring
```

`tunnel` 和 `normal_scan_plane` 是可选参考，局部分析时可关闭以减少遮挡。

### 4.2 OctoMap 状态

```text
voxel_free
voxel_occupied
voxel_unknown
```

可先只显示 `voxel_occupied` 确认命中环，再加入 `voxel_free` 和 `voxel_unknown`。

### 4.3 Candidate 形成

```text
frontier_selected_endpoint
analysis_window
frontier_candidate
pipeline_connector
frontier_unknown_direction
```

`pipeline_connector` 在 candidate 出现后再显示。它使用虚线或分段箭头表达“该观测局部产生了该
Detector 结果”，不得表达“端点沿固定距离移动后成为 candidate”。

### 4.4 Support 检测

```text
support_anchor
support_inward_direction
support_pass_envelope
support_known_samples
```

这一阶段应能直接观察 anchor 位于包络前端面、inward 箭头从 anchor 穿过包络中心轴，以及每个采样
位置的地图状态。

### 4.5 Component 与 Region

```text
component_columns
component_singletons
component_links
region_decision
```

这一阶段展示 supported columns 如何形成多个 Component，以及每个 Component 为什么被接受或拒绝。

## 5. 当前实现状态

当前默认 `combined` 已满足第 3 节的单机观测链契约：

- 使用固定 seed 的 `ProceduralCaveField` 和正式导出的 `drone_scanner::FakeLidar`；
- 初始位置执行固定 `144 x 144` beam/yaw sweep，lidar-frame endpoint 显式转换到 map frame；
- 每帧通过同一个 `OctoMapBuilder::insertScan()` 累积 FREE/OCCUPIED 状态；
- `KnownFreePathChecker` 对固定 `+X 0.8 m` hop 做门控，默认场景为 Safe 后执行第二观测 epoch；
- 最终快照只运行一次 `GlobalFrontierDetector::detectWithTrace()`，使用生产 Region 默认阈值；
- Trace 已按方向拆分 Support attempts，独立记录 direction、anchor、samples、failure、首次失败位置和
  selected attempt；
- `selected_phi_degrees` 在地图和全量检测完成后才选择局部视图，不改变 Detector 输出；
- 默认场景合成生成 Stage 0 `standard_tunnel_geometry` 参考 fixture，以及 `single_ring`、
  `bootstrap_yaw_sweep`、`validated_observation_hop`、`accumulated_frontier` 四个共享 replay 快照，
  并发布为五个独立 MarkerArray 话题；
- bootstrap/validated 地图以初始雷达位置为中心覆盖完整 max-range 观测范围；只有最终 Frontier 阶段
  围绕 selected endpoint 使用局部地图窗口；
- Marker 能展示无 candidate、vertical reject、support reject、Component reject、accepted Region 和
  Trace truncated；地图 Marker 使用确定性局部抽样，完整 OcTree 与 Detector 输入不抽样。

默认固定观测参数为：seed `42`、初始位置 `(1, 0, 0)`、beam `144`、yaw steps `144`、pitch `20 deg`、
max range `3.0 m`、分辨率 `0.1 m`、hop `0.8 m`。当前默认结果完成两个 observation epoch，并在生产
Detector 参数下产生 accepted Region。

当前 Demo 的数据链、阶段语义、ROS 接线、自动化测试和五阶段 RViz 人工验收均已完成，验收状态与
判读边界以 QA 文档第 4 节为准。

独立 `vertical_columns`、`support_envelope` 和 `component_fragmentation` 模式仍保留固定教学 fixture，
用于单独解释通过/拒绝状态和真实 bag 基线中的 17 列/11 Component 碎片比例。它们不宣称来自默认
combined 的同一次检测，也不能替代 Detector 正确性测试。

独立 fixture 模式继续只承担局部概念教学；真实链路验收以 `combined` 和 ROS-free pipeline 测试为准。

### 5.1 可重放视觉基线

当前三维 support envelope 版本的五阶段 MarkerArray 已录制为小型 MCAP：
[`frontier-geometry-demo-support-envelope-v1`](bags/frontier-geometry-demo-support-envelope-v1/README.md)。
它使用默认 `combined`、seed `42`、`selected_phi_degrees=0` 和 `republish_rate_hz=0`，五个阶段话题
各包含一条消息，当前 bag 约 `156 KiB`。

该 bag 用于后续 Detector/Trace 可视化变更时，在相同 RViz 相机和窗口尺寸下回放旧场景并生成并排图；
它只保存 Marker 结果，不重新运行 Detector，也不能替代真实 `/global_map` bag analyzer 的定量比较。
MCAP 的来源参数、话题清单、SHA-256 和回放方式记录在同目录 README 中。

## 6. Detector 复用与 Trace 设计

Demo 复用 ROS-free `GlobalFrontierDetector`，不得在 `FrontierGeometryDemo` 中复制 candidate、
vertical、support、Component 或 Region 判定算法。Demo 构造合成 `octomap::OcTree` 后直接调用
Detector，不依赖 allocator 节点、bag 或运行期 ROS 话题。

现有生产接口保持不变：

```cpp
FrontierDetectionResult detect(const octomap::OcTree & tree) const;
```

它继续只返回最终 regions、状态、原因和汇总诊断。教学和离线分析使用新增的详细入口：

```cpp
struct TracedFrontierDetectionResult {
    FrontierDetectionResult result;
    FrontierDetectionTrace trace;
};

TracedFrontierDetectionResult detectWithTrace(
    const octomap::OcTree & tree) const;
```

`detect()` 与 `detectWithTrace()` 必须进入同一个内部检测实现，不能形成两套算法。每个 candidate 的
Trace 必须按方向尝试分组，不能把多个 Support 尝试扁平合并。Trace 至少包含：

- candidate key、column points 和 vertical 通过/拒绝结果；
- 每次方向尝试的 unknown/inward 方向、实际 support anchor 和是否为最终选中尝试；
- 每次尝试独立的 Support samples、OctoMap 状态、pass/failure 类型和首次失败位置；
- supported columns；
- Component 成员、真实 XY 八邻域边、主要拒绝原因和生产 representative；
- 最终 Region 接受或拒绝状态。

普通 `detect()` 不保存逐 candidate、逐 sample 或逐 Component 的详细数据。Trace 分别限制 candidate
数量、support samples、Component 数量和 geometry elements；后者统一计入 candidate/attempt points、
component columns 与 edges，默认上限为 `500,000`。达到任一上限时允许将 Trace 标记为截断，但不得
改变 Detector 的 status、reason、regions、诊断计数或稳定排序。RViz 必须明确显示 Trace 已截断，
不能把不完整包络当作完整检测证据。

Demo 调用流程固定为：

```text
固定洞穴和单机观测序列生成完整 OcTree
  -> GlobalFrontierDetector::detectWithTrace()
  -> 全量 Detector 输出固定后，selected_phi 选择扫描端点和观察窗口
  -> 从 Trace 中选择窗口内最近的真实 candidate
  -> 绘制该 candidate 的 vertical/support/component/region 数据
  -> DemoScene 转换为 MarkerArray
```

若 selected endpoint 附近没有真实 candidate，Demo 只显示观测和局部地图，并明确区分 LOCAL 无
candidate 与 GLOBAL accepted Region 数量；不得人工补造后续检测结果。

## 7. 验收标准

- 所有 scan、voxel 和 Frontier Marker 来自同一单机 raycast/OctoMap 累积链和同一次 Detector 结果；
- 修改 `selected_phi_degrees` 只改变观察区域，不改变 Detector 全量输出；
- selected endpoint 附近没有 candidate 时不伪造后续元素；
- 每个方向的 Support attempt 独立，最终箭头、包络和 samples 必须来自同一个 selected attempt；
- Component 成员和 links 与真实八邻域结果一致；
- Region marker 位于生产 representative，接受/拒绝语义正确；
- 初始 yaw sweep 与第二观测 epoch 写入同一棵 OcTree；固定 +X hop 未通过 known-free 检查时不得移动；
- Detector 使用生产默认参数，任何 override 都必须公开并由测试固定；
- Namespace 按第 4 节顺序打开时，能逐步还原完整数据流；
- 关闭详细分析轨迹后，生产 Detector 行为和性能边界保持不变。

Demo 能验证确定性单机 raycast、OctoMap 插入、固定短段 known-free 门控、Detector 全管线、Trace
等价性和 Trace 到 Marker 映射。它不能证明一般三维 Frontier、本地目标搜索、飞控/TF/QoS/时间同步、
多机 merger/allocator、真实噪声下安全性、其他洞穴 seed 的泛化或生产性能上界。

## 8. 实施顺序

以下 1-8 项和第 9 项的自动化测试已在当前工作区实现；最终 RViz 人工目检仍需在可用 GUI 环境完成。

1. 先修正 Detector Trace：按方向保存 Support attempt、anchor、samples、failure 和 selected attempt，
   并证明 `detect()` 与 `detectWithTrace()` 结果、诊断和排序等价。
2. 删除旧人工 integrated 链路，清理 combined 中不生效或静默覆盖生产阈值的参数。
3. 为 `drone_scanner::FakeLidar` 建立正式可链接的 ROS-free 导出边界；不得在 Demo 内复制 beam 几何。
4. 固定洞穴 seed、初始位姿、yaw 序列、beam 数、pitch、分辨率和 hop distance，完成 lidar endpoint
   到 map endpoint 的显式转换。
5. 在初始 XYZ 完成 yaw sweep，每帧通过 `OctoMapBuilder::insertScan()` 写入同一棵树。
6. 使用 `KnownFreePathChecker` 检查唯一固定 `+X` 短段；不安全则停止并显示原因，安全才更新位姿。
7. 在新位置完成第二次 yaw sweep，继续更新同一棵树，并只对最终快照运行一次
   `detectWithTrace()`。
8. 从固定 Trace 派生 Marker；focus 参数只筛选已有结果，同时覆盖无 candidate、vertical reject、
   support reject、component reject、accepted region 和 trace truncated。
9. 补齐 Detector Trace 单测、raycast-to-OctoMap-to-Detector 的 ROS-free 集成测试、Marker/Trace 一一
   映射测试和 RViz 人工目检。

## 9. 验证记录

- `colcon build --packages-select drone_scanner swarm_controller --symlink-install`：通过；
- `swarm_controller` CTest：`23/23` 通过，其中包含五阶段话题、QoS、晚加入缓存和阶段 Namespace 的
  launch integration test；
- `drone_scanner` CTest：`10/10` 通过；包级 `flake8` 仍报告既有
  `fake_odom_launch.py` 4 处 `E501`，不属于本 Demo 改动；
- 默认节点提供五个 reliable + transient-local 阶段话题，每个话题一个 publisher，共发布
  `33,913` 个场景 Marker；bag 另含每阶段一个 `DELETEALL`；Stages 1–4 的初始构造只执行一次共享 replay；
- Terra 修复后窄范围代码复核：`no findings`；
- RViz 五个顶层 Display 的阶段切换、外观和叠加遮挡已于 2026-07-17 通过人工目检。
- 五阶段旧版 support envelope 视觉基线已录制为 5-message MCAP，`ros2 bag info` 校验通过。

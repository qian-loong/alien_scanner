# C1 可选射线证据调试视图 - 待审核方案

> 状态：仅供方案审核，尚未批准实施。
> 审核者：请只读检查本方案，优先报告高置信的语义错误、范围漂移、架构问题和缺失验收门；不得修改文件或实施代码。

## 0. 方案审核结论与修订

独立方案审核确认调试视图方向可行，同时发现三个必须先收口的 C1 契约问题：

1. `LaserScanAdapter::convert()` 必须自身执行完整验证并 fail closed，不能依赖
   调用方先调用 `validate()`；
2. `RayEvidenceCapability` 的底层整数比较必须先验证闭合枚举，未知值 `3/255`
   不得获得 `FullRay` 权限；
3. 冻结 descriptor 是 C1 发布前验证边界，不是 C2 必须另行获取的隐藏输入。
   C1 成功发布的 observation 是该批次的权威输入，消费者仍按 sensor/session
   做溯源，但不需要新增 descriptor topic。

审核还指出实施文档中的 ROS 2 Humble 是旧文本，必须统一为仓库实际 Jazzy 环境。

上述问题纳入本步前置收口。首版明确不增加旧地图静态快照；使用现有确定性
fixture 注入完整返回类型，等 C2 正式地图 consumer 完成后再由新接口生成可复现
地图快照。

## 1. 目标

为 C1 新增独立、可选的 RViz 调试入口，把正式
`perception_interfaces/LidarObservation` 中的射线证据直观显示为：

- occupied candidate：红色命中端点；
- hit free-space evidence：绿色 origin-to-hit 线段；
- full no-return evidence：青色 origin-to-range_max 线段；
- invalid measurement：灰色短方向标记和无效计数，不表示真实空间长度。

该视图用于核对 C1 证据契约，不写 occupancy，不成为 mapper，也不改变现有
`fixture_visualization.launch.py` 的几何验收入口。

## 2. 冻结边界

### 2.1 唯一运行时输入

调试节点只订阅新的 `perception_interfaces/msg/LidarObservation`，默认 topic 为
`perception/observations`。

明确禁止：

- 订阅、桥接或恢复 `/drone_0/scan_returns`；
- 启动旧 `FakeLidar` / `OctoMapBuilder` 作为本视图的数据依赖；
- 将旧带 `range/hit` 字段的 `PointCloud2` 提升为新通用
  `PointCloud2Adapter` 的 free-ray 输入；
- 在本节点中构建或发布 occupancy/OctoMap。

### 2.2 旧地图数据的允许用法

若需要更真实的背景，只允许使用从重构前数据离线导出的、仓库内冻结的降采样
静态参考快照。快照不得保留旧 topic/schema 运行时依赖，也不得被测试当成 C1
地图输出。

当前仓库没有现成 `.bt/.ot/.pcd/.bag` 资产。首版建议不新增地图快照，先用现有
确定性 fixture 完成证据可视化；是否增加快照应单独以资产大小、来源可复现性、
许可证和数值校验为验收门，不能把临时抓取数据直接提交。

## 3. 分层设计

### 3.1 ROS-free 几何构建器

在 `perception_fixtures` 的算法库中增加通用调试几何类型，例如：

```cpp
struct DebugPoint3 { float x, y, z; };

struct RayEvidenceDebugGeometry {
    std::vector<DebugPoint3> hit_endpoints;
    std::vector<DebugPoint3> hit_free_segments;       // LINE_LIST 成对点
    std::vector<DebugPoint3> no_return_free_segments; // LINE_LIST 成对点
    std::vector<DebugPoint3> invalid_indicators;      // LINE_LIST 成对点
    std::size_t invalid_count {0};
};
```

构建器直接接收 ROS-free `Scan2D`、`RayEvidenceCapability` 和抽样步长：

- 每个 `Hit` 始终生成命中端点；
- capability 至少为 `HitRay` 时，`Hit` 才生成 free 线段；
- 只有 `FullRay` 的 `NoReturn` 才生成到 `range_max` 的 free 线段；
- `Invalid` 只用固定短长度表示 beam 方向并计数，不使用原始无效 range 作为长度；
- 抽样只影响显示密度，不改变分类或能力判断；默认 `beam_stride` 应限制画面拥挤。

Cloud3D/HitOnly 首版只显示命中端点，不伪造 origin-to-point free 段。若消息字段
无法区分某点是否有效，则只使用有限 XYZ 点。

### 3.2 薄 ROS 节点

新增 `RayEvidenceDebugNode`：

- 订阅 `LidarObservation`；
- 在节点中完成 ROS message → ROS-free `Scan2D/Cloud3D` 转换；
- 调用几何构建器；
- 发布 `visualization_msgs/msg/MarkerArray`；
- 保留 observation 的 frame/stamp，Marker 设置有限 lifetime，防止历史射线无限累积；
- 参数仅包含 topic、beam stride、line/point 尺寸和 lifetime；颜色采用固定语义，
  不允许通过配置把不同证据类型改成不可区分的同色。

无效消息枚举值、data_type 与 payload 不一致、2D 元数据不足或数组越界必须
fail closed：不发布误导 Marker，并输出限频诊断。

## 4. Launch 与 RViz

新增独立 `ray_evidence_debug.launch.py`，默认：

- 启动确定性 2D/mixed fixture；
- 启动 `perception_input_node`，2D descriptor 显式为 `full_ray`，3D 为
  `hit_only`；
- 启动 `RayEvidenceDebugNode`；
- `show_rviz:=true` 时加载独立 RViz 配置。

为了同屏证明分类，调试 fixture 应在固定 beam 索引注入：正常 hit、`+inf`
no-return、NaN、负无穷、低于 range_min 和高于 range_max；注入只在新的调试
launch/参数下启用，不改变现有 fixture 几何验收默认输出。

RViz 配置显示 MarkerArray、传感器 TF 和可选原始 LaserScan/PointCloud2；不显示
或发布 occupancy，避免把证据视图误认成地图。

## 5. 验收与测试

### 5.1 gtest

- `HitOnly`：有红色命中端点，无 free 线段；
- `HitRay`：有 hit free 线段，无 no-return free 线段；
- `FullRay`：同时有 hit free 与 no-return free 线段；
- NaN、负无穷、量程外有限值均只进入 invalid 指示/计数；
- `beam_stride` 确定且不改变所选 beam 的分类；
- Cloud3D 只产生有限命中端点。

### 5.2 launch integration

- 只通过新 `LidarObservation` topic 驱动节点；
- Marker namespace、颜色、点数、frame、stamp、lifetime 符合契约；
- 非法 ray evidence/data_type/payload 被拒绝；
- launch 关闭 RViz 后仍能完成端到端 Marker 验证；
- 图中同时出现 hit endpoint、hit free、no-return free 和 invalid 指示；
- ROS graph 中不存在 `/drone_0/scan_returns` 发布者或订阅者。

### 5.3 手动 RViz

- 各 Marker display 可独立开关；
- 颜色与空间含义不混淆；
- 抽样后画面可读、无持续累积；
- 2D full-ray 与 3D hit-only 能直观看出能力差异。

## 6. 范围外

- occupancy/free/unknown 体素写入；
- OctoMap 构建或新旧地图数值一致性；
- 旧 session 消费拒绝、map epoch、窗口缓存；
- 旧 `/scan_returns` replay/adapter。

这些仍属于 C2 或独立的离线回归工作。

## 7. 待审核问题

1. `perception_fixtures` 是否仍是该 ROS-free 几何构建器和调试节点的合适归属，
   还是应建立独立 `perception_visualization` 包？首选前者以控制本步范围。
2. 首版不提交旧地图静态快照是否更符合 C1 边界？首选是；真实地图快照可在
   C2 map consumer 完成后用正式新接口生成。
3. invalid 使用固定短方向标记是否足够避免被误读为真实 free ray？

## 8. 用户确认的视觉场景修订（2026-07-26）

旧 debug launch 只有 7 条 180° scan beam 和 4 个 Cloud3D 点，虽覆盖返回分类，
但 RViz 呈现为散落点，不能直观表达隧道扫描。用户确认改为解析确定的环切面：

- map `+X` 为洞轴，`Ry(+pi/2)` 把 LaserScan local XY 平面映射到 map YZ；
- 360 条无重复方向 beam，椭圆半轴 local X=`3m`、local Y=`4m`，最大量程 `10m`；
- 索引 `[255,285]` 为朝 map `+Y` 的连续 `+inf/NoReturn` 岔口；
- 索引 `44/136/180/316` 分别为 NaN、`-inf`、below-min、above-max；
- debug launch 默认只发布 2D scan，Cloud3D hit-only 由直接 observation 注入测试，
  避免散点污染主环切面；
- 默认 fixture 路径保持不变，并用旧 181-beam golden 代表值验证，而非仅验证确定性。

有限值高于 `range_max` 始终属于 `Invalid`；只有 `+inf` 表示量程内未命中的
`NoReturn`。自动化测试固定 stride 1，RViz 默认 stride 2，确保四条偶数索引的
invalid beam 仍可见且绿色 hit-free 射线不过度遮挡红色洞壁环。

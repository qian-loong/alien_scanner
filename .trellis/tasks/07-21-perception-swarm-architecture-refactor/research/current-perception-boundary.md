# Research: 当前感知输入与本机建图边界

- Query: 当前 FakeLidar、AltitudeAdapter 和 OctoMap Builder 如何耦合，标准感知边界需要替换什么？
- Scope: internal
- Date: 2026-07-21

## Findings

### 当前数据路径

```text
FakeLidar raycast
  -> points PointCloud2（仅 hit）
  -> scan_returns PointCloud2（x/y/z/range/hit/intensity）
  -> OctomapBuilderNode
  -> /drone_i/octomap 完整 snapshot
```

- FakeLidar 同时发布 `points` 与 `scan_returns`：`ws/src/drone_scanner/src/FakeLidarNode.cpp:79`。
- `scan_returns` 使用普通 `PointCloud2` 承载项目自定义 `range`、`hit` 字段；OctoMap Builder 默认直接订阅该话题：`ws/src/swarm_controller/src/OctomapBuilderNode.cpp:55`、`ws/src/swarm_controller/src/OctomapBuilderNode.cpp:75`。
- Builder 对字段布局有硬要求，并发布完整 OctoMap：`ws/src/swarm_controller/src/OctomapBuilderNode.cpp:96`、`ws/src/swarm_controller/src/OctomapBuilderNode.cpp:271`。

### 扫描几何耦合

- FakeLidar 算法配置内含 `ring_pitch_rad`：`ws/src/drone_scanner/include/drone_scanner/FakeLidar.hpp:19`。
- ROS 节点参数再次暴露 `ring_pitch_rad`：`ws/src/drone_scanner/src/FakeLidarNode.cpp:51`。
- AltitudeAdapter 也读取相同概念来判断几何兼容性：`ws/src/drone_scanner/include/drone_scanner/AltitudeAdapter.hpp:19`。
- FakeLidar 查询最新 TF (`TimePointZero`) 后使用 TF stamp 或当前时间生成扫描 stamp：`ws/src/drone_scanner/src/FakeLidarNode.cpp:213`。新观测契约需要明确采样时刻与 TF 查询策略，不能把接收时刻当采样时刻。

### 已形成的候选接口

- 总览草案建议二维使用 `LaserScan`、三维使用 `PointCloud2`，不把 `/scan_returns` 保留为长期公共接口：`docs/decisions/perception-and-swarm-architecture-refactor.md:82`、`docs/decisions/perception-and-swarm-architecture-refactor.md:114`。
- 感知详解已定义每 sensor 独立 ID/frame/stamp/origin、`Hit/NoReturn/Invalid`、能力声明和多雷达窗口：`docs/decisions/perception-observation-interface.md:117`、`docs/decisions/perception-observation-interface.md:150`、`docs/decisions/perception-observation-interface.md:209`。
- 详解明确 hit-only 三维输入不能静默假设完整 free ray：`docs/decisions/perception-observation-interface.md:281`。
- RViz 使用稳定的聚合 `PointCloud2` 输出，raw 2D/3D 分别用对应 Display，RViz 不负责转换：`docs/decisions/perception-observation-interface.md:245`。

## Implications

- C1 不能只改话题名称；需要把 wire message、逻辑 ray 语义和当前 builder 输入分开。
- 当前倾斜环可以作为 `vertical_tilted_2d` fixture，但倾角应由安装姿态/TF 表达，避免 beam 与 TF 双重旋转。
- AltitudeAdapter 需要消费能力明确的观测视图，不能长期依赖 FakeLidar 配置字段。
- C1 与 C2 应分开验收：C1 固定 observation 契约，C2 再替换地图消费路径。

## Caveats / Not Found

- 尚未选择真实二维或三维设备，因此不能固定厂商字段、ring 布局或多回波编码。
- 尚未验证标准 `LaserScan` 表达当前垂直倾斜环时的具体 frame/angle 方向；该项属于 C1 的契约测试。


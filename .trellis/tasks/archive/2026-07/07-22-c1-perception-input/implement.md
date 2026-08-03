# C1：感知输入与观测数据模型 - 实施计划

> 子任务：07-22-c1-perception-input
> 父任务：07-21-perception-swarm-architecture-refactor

---

## 1. 实施顺序

### 阶段 1：核心类型与接口（1-2 天）

- [x] 定义 `SensorID`、`SessionID`、`SourceID`
- [x] 定义 `SensorDescriptor`（type, frame, FOV, mounting pose）
- [x] 定义 `LidarObservation`（Scan2D / Cloud3D）
- [x] 定义 `PoseEstimate`（position, orientation, quality, freshness, reset_epoch）
- [x] 定义 `HealthState`、`SensorHealth`
- [x] 定义 `MapperInputContract`、`MapperHealthGate`
- [x] 单元测试：类型构造、相等性比较、variant 访问

**验证门**：
- [x] 所有核心类型编译通过
- [x] 单元测试行覆盖率 > 90%（2026-07-26 独立 GCC/gcov 结果：209/210，99.52%）
- [x] 无 ROS 依赖（纯 C++17 + Eigen）

---

### 阶段 2：ROS Adapter（2-4 天）

- [x] 实现 `LaserScanAdapter::convert`
  - [x] 提取 ranges、intensities
  - [x] 转换为 `Scan2D`
  - [x] 保留 sensor_id、session_id、frame_id、clock_domain、stamp
- [x] 实现 `PointCloud2Adapter::convert`
  - [x] 解析 PointCloud2 字段（x, y, z, intensity）
  - [x] 转换为 `Cloud3D`
  - [x] 处理不同 point_step 和 field 顺序
- [x] 实现 `OdometryAdapter::convert` / `from_tf`
  - [x] 提取 position、orientation
  - [x] 计算 quality（基于协方差）
  - [x] 检测 reset epoch（frame 变化、时间回退、位置跳变）
  - [x] 从配置读取 clock_domain
  - [x] 从配置读取位置跳变阈值（默认 5m / 1s）
- [x] 实现验证逻辑（`validate` 方法）
- [x] 单元测试：LaserScan、PointCloud2、Odometry、TF 转换与异常输入

**验证门**：
- [x] LaserScan → Scan2D 转换正确（固定 fixture）
- [x] PointCloud2 → Cloud3D 转换正确（固定 fixture）
- [x] Odometry / TF → PoseEstimate 转换正确
- [x] Reset epoch 检测正确（时间回退、frame 变化、位置跳变）
- [x] Clock domain 正确附加到观测数据
- [x] 单元测试行覆盖率 > 85%（2026-07-26 独立 GCC/gcov 结果：249/277，89.89%）

---

### 阶段 3：健康门控（1-2 天）

- [x] 实现 `MapperHealthGate::evaluate`
  - [x] 检查 minimum viable input
  - [x] 检查 degraded combinations（按优先级）
  - [x] 检查位姿 freshness
  - [x] 检查位姿 expected frame
  - [x] 检查位姿 minimum quality
- [x] 实现 `current_capability` 方法
- [x] 实现 `degradation_reason` 方法（诊断信息）
- [x] 单元测试：各种健康状态转换路径

**评审优化建议（见 REVIEW-RESPONSE.md）**：
- [x] ✨ 优化 1：在传感器/位姿回调中立即检测健康状态变化，timer 负责周期发布
- [x] ✨ 优化 2：恢复采用连续 3 次稳定窗口；故障降级立即生效，避免继续产生伪 freshness

**验证门**：
- [x] Healthy → Degraded 转换正确
- [x] Degraded → Unavailable 转换正确
- [x] Unavailable → Degraded → Healthy 恢复路径正确
- [x] 位姿超时触发 Unavailable
- [x] 位姿 frame/quality 不满足契约时立即触发 Unavailable
- [x] 单元测试覆盖所有状态转换

---

### 阶段 4：ROS 节点（2-3 天）

- [x] 实现 `SensorSessionManager`
  - [x] `register_sensor`：启动时注册
  - [x] `freeze`：首次数据到达后冻结
  - [x] `can_reconnect`：验证重连合法性
- [x] 实现 `PerceptionInputNode`
  - [x] 从配置加载 sensor descriptors
  - [x] 为每个传感器创建订阅
  - [x] 在回调中调用 adapter 转换
  - [x] 发布 ROS-free 观测数据（封装在 ROS message）
- [x] 定时更新健康状态（1 Hz）
  - [x] 可配置选择 Odometry 或 TF pose 输入
- [x] 创建 `perception_interfaces` 包
  - [x] 定义 `LidarObservation.msg`
  - [x] 定义 `PoseEstimate.msg`
  - [x] 定义 `HealthState.msg`
- [x] Launch 文件（按用户要求使用 Python launch）
  - [x] `perception_input.launch.py`：主入口
  - [x] `fixture_2d.launch.py`：2D fixture
  - [x] `fixture_3d.launch.py`：3D fixture

**评审优化建议（见 REVIEW-RESPONSE.md）**：
- [x] ✨ 优化 4：Descriptor 冻结拒绝时的详细诊断消息（包括原因、旧 descriptor、新 descriptor）
- [x] ✨ 优化 5：不采纳 Humble 版本钉死；本仓库开发环境为 ROS 2 Jazzy，依赖由容器镜像统一

**验证门**：
- [x] 节点启动无错误
- [x] 从配置加载 descriptors 成功
- [x] 订阅传感器消息成功
- [x] 发布观测数据成功
- [x] Descriptor 冻结后拒绝新传感器
- [x] 健康状态定时发布

---

### 阶段 5：Fixture 与回归测试（2-3 天）

- [x] 创建 2D LaserScan fixture
  - [x] 走廊场景（180° 前向扫描）
  - 未采用 rosbag：确定性 C++ publisher 已取代该早期资产方案（非 C1 门禁）
  - [x] 配置 sensor descriptor YAML
- [x] 创建 3D PointCloud2 fixture
  - [x] 标准多线场景（默认 16 条仰角、每线 360 个方位点）
  - 未采用 rosbag：确定性 C++ publisher 已取代该早期资产方案（非 C1 门禁）
  - [x] 配置 sensor descriptor YAML
- [x] 创建混合 2D+3D fixture
  - [x] 2 个 2D + 1 个 3D 传感器
  - 未采用 rosbag：确定性 C++ publisher 已取代该早期资产方案（非 C1 门禁）
  - [x] 配置 sensor descriptor YAML
- [x] 标准多线 3D profile 回归
  - [x] 验证传感器 XY 平面、方位和正负仰角语义
  - C2 deferred：旧 FakeLidar 与 Phase 3 baseline 的地图对比（非 C1 门禁）
- [x] 创建 RViz 可视化配置
  - [x] 显示观测数据（LaserScan / PointCloud2）
  - 未采用 HealthState 颜色编码：主 fixture RViz 不承担该可视化增强（非 C1 门禁）
  - [x] 显示传感器 TF frames

**评审优化建议（见 REVIEW-RESPONSE.md）**：
- [x] ✨ 优化 3：补充详细的回归测试标准（解析公式、ROS round-trip、数值误差范围和旧路径边界）

**验证门**：
- [x] 2D fixture 可重复启动（Python launch smoke）
- [x] 3D fixture 可重复启动（Python launch smoke）
- [x] 混合 fixture 可重复启动（Python launch smoke）
- [x] 标准多线 3D profile 回归通过（FixtureScene gtest）
- [x] RViz 可目检观测数据

> 本阶段采用确定性 C++ fixture publisher 取代未提交的 rosbag 资产；rosbag 回放和地图数值对比留给后续 C2 回归门。

---

### 阶段 6：集成测试（2-3 天）

- [x] 多传感器并行输入
  - [x] 2 个 LaserScan + 1 个 PointCloud2
  - [x] 验证 sensor_id 区分正确
  - [x] 验证不被错误合并
- [x] 传感器掉线与重连
  - [x] 停止发布某个传感器
  - [x] 验证健康状态转换为 Degraded
  - [x] 恢复发布
  - [x] 验证健康状态恢复为 Healthy
- [x] Descriptor 冻结测试
  - [x] 启动后发布数据（触发冻结）并保持 session 身份稳定
  - [x] 单元测试：冻结后新增传感器 ID 被拒绝（`TestSensorSessionManager`）
  - [x] 集成测试：改变已有传感器 frame_id（descriptor 语义变化）被拒绝且不发布观测
  - [x] 集成测试：相同 descriptor 掉线后重连，session 不变
  - 不适用的测试形态：节点无运行时 register API；新增 sensor ID 必须经配置和新进程/session，inventory 拒绝语义已由单元测试覆盖
- [x] 位姿 reset epoch 测试
  - [x] 发布正常 Odometry
  - [x] 改变 frame_id
  - [x] 验证 reset_epoch 递增
  - [x] 时间回退
  - [x] 验证 reset_epoch 递增
- [x] 位姿超时测试
  - [x] 停止发布 Odometry
  - [x] 验证 freshness 超时
  - [x] 验证健康状态转换为 Unavailable
- [x] TF 位姿输入测试
  - [x] TFMessage 选择配置的 child frame 并转换 PoseEstimate
  - [x] parent frame 不匹配触发门控并递增 reset epoch
- [x] Producer session 生命周期测试
  - [x] 进程异常退出由 launch respawn
  - [x] 新进程 SessionID 与旧进程不同
  - [x] 新进程观测不复用旧 SessionID
- [x] Vehicle-local 自主性
  - [x] 不启动 fleet 组件
  - [x] 验证本机仍连续产出观测数据

**验证门**：
- [x] 所有集成测试通过
- [x] 无崩溃、无死锁（Session 测试中的 SIGKILL 为受控故障注入）
- [x] 日志清晰可追踪

---

### 阶段 7：AC-09 射线证据契约收口

- [x] 在 `perception_core` 定义闭合且有序的 `RayEvidenceCapability`：
  `HitOnly < HitRay < FullRay`
- [x] `SensorDescriptor` 和每批 `LidarObservation` 携带同一证据等级；该字段参与
  descriptor equality 和 inventory freeze
- [x] `Scan2D` 在不复制 ranges 的前提下确定性查询 `Hit/NoReturn/Invalid`
- [x] `SensorRequirement` 可声明最低射线证据，`MapperHealthGate` 只统计满足等级的
  active sensor，并分别暴露 hit free-space ray 与完整 no-return ray 能力
- [x] `LaserScanAdapter` 传播 descriptor 等级；当前 `PointCloud2Adapter` 对高于
  `HitOnly` 的声明在 `validate()` 和直接 `convert()` 两条路径都 fail closed
- [x] `HitRay`/`FullRay` LaserScan 只在消息 range/FOV/resolution 与冻结 descriptor
  一致时通过，基础 range/angle 元数据非法时所有等级都拒绝
- [x] `LidarObservation.msg` 和 `HealthState.msg` 完整映射上述能力，仓库 fixture
  配置显式声明 2D=`full_ray`、3D=`hit_only`
- [x] 节点解析 `minimum_lidar_ray_evidence`、`degraded_lidar_ray_evidence` 和
  `sensor.<id>.ray_evidence`；默认 `hit_only`，非法值启动失败
- [x] Core/adapter/session/launch tests 覆盖分类边界、能力冻结、健康门控和 ROS 字段
- [x] 同步 `docs/perception-input-testing.md` 与 `docs/perception-class-relations.md`

**验证门**：
- [x] `perception_core`、`perception_interfaces`、`perception_adapters`、
  `perception_input_node`、`perception_fixtures` 干净 Release 构建通过
- [x] 相关 gtest 与 launch tests 全部通过，`colcon test-result --verbose` 零失败
- [x] `git diff --check` 通过

**回滚点**：删除新增 capability 字段、参数和测试即可回到现有 C1 数据形态；
不得在回滚中修改 C2/OctoMap/Frontier 行为。

---

### 阶段 8：AC-09 安全加固与 AC-10 可选调试视图

**前置安全收口**：
- [x] 增加集中式 `is_valid_ray_evidence()`；`provides_at_least()` 对未知 actual/required fail closed
- [x] `LaserScanAdapter::convert()` 内部执行完整 validate，直接调用无法绕过元数据、frame、type 和数组检查
- [x] 明确成功发布的 observation 是权威批次；删除 C2 必须另取冻结 descriptor 的错误规范要求
- [x] 修正文档中的 ROS 2 Humble/Jazzy 漂移

**ROS-free 几何与薄节点**：
- [x] 在 `perception_fixtures` 算法库实现 `RayEvidenceDebugGeometryBuilder`
- [x] 2D 按能力生成 hit endpoint、hit free、full no-return free、invalid 短方向标记
- [x] Cloud3D hit-only 只生成有限命中端点，不生成 free ray
- [x] 实现薄 `RayEvidenceDebugNode`，只订阅 `LidarObservation` 并发布 `MarkerArray`
- [x] 未知 ROS 枚举、非法 payload/data_type/元数据整批 fail closed
- [x] Marker 保留 frame/stamp、有限 lifetime；支持确定性 `beam_stride`

**Launch、RViz 与 fixture**：
- [x] 新增独立 `ray_evidence_debug.launch.py` 和 RViz 配置，`show_rviz` 可关闭
- [x] 调试 fixture 专用参数注入 Hit/NoReturn/Invalid，不改变现有 fixture 默认输出
- [x] 红/绿/青/灰语义可独立开关且画面不持续累积
- [x] 不订阅或发布 `/drone_0/scan_returns`，不启动旧 FakeLidar/OctoMapBuilder
- [x] 首版不提交旧地图快照，真实参考地图留待 C2 用正式新接口生成

**视觉场景修订（用户确认，2026-07-26）**：
- [x] 扩展 `FixtureSceneConfig` 统一 scan count/angle/range；默认 181-beam fixture golden 不变
- [x] debug-only 生成 360° 椭圆隧道环，索引 `[255,285]` 为连续 `+inf` 岔口扇区
- [x] 索引 `44/136/180/316` 分别注入 NaN、`-inf`、below-min、above-max，且与岔口不相交
- [x] 使用 `map -> debug_scan_link` 的 `Ry(+pi/2)` 将 local XY 扫描面映射到 map YZ，并由 `/tf_static` 测试验证基向量
- [x] debug launch/RViz 默认 2D-only，移除 Cloud3D 散点和无来源 display；Cloud3D 契约改由直接 observation 注入测试
- [x] RViz 默认 stride 2、自动化 stride 1；沿 map X 观察完整环、青色岔口和四条灰色 invalid

**测试与验收**：
- [x] Core/adapter gtest 覆盖直接 convert 绕过与未知枚举 `3/255`
- [x] 几何 gtest 覆盖三档能力、所有返回分类、抽样和 Cloud3D hit-only
- [x] launch test 断言 Marker namespace/color/count/frame/stamp/lifetime 和非法批次拒绝
- [x] ROS graph 断言不存在 `/drone_0/scan_returns` publisher/subscriber
- [x] 独立 Release 构建、全量五包测试、`git diff --check` 通过
- [x] 更新测试与类关系文档
- [x] RViz 人工目检（用户于 2026-07-26 确认截图 `C:\Users\loong\AppData\Local\Temp\alien-c1-ac10-tunnel-rviz-20260726-11.png` 通过；map YZ 截面、红 X 法向、绿 Y/蓝 Z 面内，洞壁/岔口/invalid 可辨）

**回滚点**：删除调试几何、节点、独立 launch/RViz 和专用 fixture 参数；保留
前置安全修复。不得回滚到未知枚举可获得高权限或 adapter convert 可绕过验证。

---

## 2. 回滚策略

### 回滚点 1：Adapter 失败
**触发条件**：Adapter 转换结果与 Phase 3 基线不一致。

**回滚操作**：
1. 保留当前 `FakeLidar` + `OctoMap Builder` 入口
2. Adapter 路径只在对比路径启用（launch 参数控制）
3. 不删除旧代码，直到 C2 地图验收通过

---

### 回滚点 2：健康门控误判
**触发条件**：健康状态频繁抖动，或误判 Unavailable。

**回滚操作**：
1. 添加健康状态稳定窗口（连续 N 次才切换）
2. 调整 freshness 阈值
3. 提供诊断日志辅助调试

---

### 回滚点 3：性能问题
**触发条件**：CPU 占用 > 10%，或内存占用 > 100 MB。

**回滚操作**：
1. 优化 Adapter（减少拷贝）
2. 降低健康状态更新频率（1 Hz → 0.5 Hz）
3. 启用 ROS 2 zero-copy transport

---

## 3. 验收门（来自 PRD）

### AC-01：标准 ROS 消息输入
- [x] `sensor_msgs/LaserScan` 可转换为 `LidarObservation`
- [x] `sensor_msgs/PointCloud2` 可转换为 `LidarObservation`
- [x] 保留 frame_id、stamp、clock_domain、origin（sensor ID/session）

### AC-02：传感器身份与溯源
- [x] 每个 observation 携带 sensor ID、session、frame、stamp
- [x] 多个传感器的 observation 不被错误合并
- [x] producer 重启后 SessionID 变化且新数据不复用旧 SessionID；旧 session 拒绝属于 C2 消费边界

### AC-03：Sensor descriptor inventory 冻结
- [x] Vehicle session 启动后首次有效观测冻结 descriptor inventory（集成测试验证 session 稳定）
- [x] 相同 ID/descriptor 的 producer 掉线后可重连（集成测试）
- [x] 改变 descriptor 语义（frame_id 不匹配）时拒绝输入；新增传感器 ID 无运行时 API，由 `SensorSessionManager` 单元测试覆盖，需新 vehicle session

### AC-04：位姿输入契约
- [x] `nav_msgs/Odometry` 和 TF 可转换为 `PoseEstimate`
- [x] 保留 source ID/session、frame、clock_domain、quality、freshness、reset epoch
- [x] Pose stale（freshness 超时）时触发门控
- [x] Pose frame 与 expected frame 不匹配时触发门控
- [x] Pose quality 低于阈值时触发门控
- [x] Pose 时间回退时递增 reset epoch

**C2 deferred（非 C1 验收门）**：消费 reset epoch 并触发 map epoch 重建。

### AC-05：Mapper 输入声明
- [x] Mapper 启动时声明 minimum viable input
- [x] Mapper 启动时声明允许的 degraded 组合
- [x] 输入满足 minimum 时标记 Healthy
- [x] 输入满足 degraded 但不满足 minimum 时标记 Degraded
- [x] 输入不满足任何组合时标记 Unavailable
- [x] Unavailable 状态不产生伪 freshness（故障立即生效，恢复使用稳定窗口）

### AC-06：健康状态转换
- [x] Healthy → Degraded（传感器掉线但仍满足 degraded）
- [x] Degraded → Unavailable（不满足任何组合）
- [x] Unavailable → Degraded（传感器恢复但未全部恢复）
- [x] Degraded → Healthy（所有传感器恢复）
- [x] 恢复后重新通过健康门（不能直接从 Unavailable 跳到 Healthy）

### AC-07：Vehicle-local 自主性
- [x] 不启动 fleet 组件时，本机仍可产出 observation

**C2 deferred（非 C1 验收门）**：断开 fleet 链路后继续产出 local map。

### AC-08：确定性 fixture
- [x] 提供 2D LaserScan fixture
- [x] 提供 3D PointCloud2 fixture
- [x] 提供混合 2D+3D fixture
- [x] 标准 2D XY 与多线 3D 仰角方向约定（回归测试通过）
- [x] 固定 RViz 输出可目检

### AC-09：降级能力显式声明
- [x] descriptor/observation 显式声明 `HitOnly`、`HitRay`、`FullRay`
- [x] `Scan2D` 确定性区分 `Hit`、`NoReturn`、`Invalid`
- [x] 当前 hit-only `PointCloud2` 不能声明或伪造 free-ray/no-return 证据
- [x] HitRay/FullRay LaserScan 元数据漂移不能越过冻结 descriptor
- [x] mapper 最低能力及运行时配置、聚合健康消息和 observation ROS 消息均可读取射线证据等级

### AC-10：可选射线证据调试视图
- [x] LaserScan 直接 convert 不可绕过验证，未知 ray-evidence 数值 fail closed
- [x] 调试节点只消费 `LidarObservation`，不依赖 `/drone_0/scan_returns`
- [x] 三档能力和 Hit/NoReturn/Invalid 产生正确 Marker 语义
- [x] Cloud3D hit-only 只显示命中端点，不伪造 free ray
- [x] 独立 launch 和自动化测试通过，现有 fixture 默认行为不变
- [x] 360° YZ 隧道环、连续岔口 no-return 扇区、四条 invalid 和精确 TF 的自动化回归通过
- [x] RViz 人工目检（2026-07-26 用户确认上述截图通过）
- [x] 调试路径不发布 occupancy/OctoMap，首版不引入旧地图快照

---

## 4. 质量门（来自父任务）

### 代码质量
- [x] ROS-free 算法库（perception_core）
- [x] 薄 ROS 节点（perception_input_node）
- [x] 消息转换不渗入算法契约
- [x] 独立 interfaces 包（perception_interfaces）

### 测试覆盖
- [x] 纯算法 gtest（perception_core、perception_adapters、perception_fixtures）
- [x] ROS 接线 launch test（perception_fixtures 驱动 perception_input_node）
- [x] 可视化目检入口（RViz 配置）

### 性能基线
- [x] Release 构建
- [x] 固定 seed/参数/输入
- [x] 足够窗口（120 帧，0 丢帧）
- [x] 分别报告各阶段（Adapter 转换、ROS 发布链路、健康检查）

---

## 5. 风险与缓解

| 风险 | 缓解措施 | 责任人 |
|------|---------|--------|
| Adapter 转换错误 | 固定 fixture 单元测试；旧路径另做 Phase 3 replay 对比 | 实施者 |
| 健康状态抖动 | 稳定窗口；调试日志 | 实施者 |
| Descriptor 冻结限制灵活性 | 清晰错误诊断；文档说明重启边界 | 实施者 |
| 性能回归 | 基准测试；zero-copy 优化 | 实施者 |
| ROS 2 接口不稳定 | 固定仓库 ROS 2 Jazzy 容器；避免 beta 特性 | 实施者 |
| 调试视图误用旧接口 | 只订阅 `LidarObservation`；ROS graph 测试禁止 `/drone_0/scan_returns` | 实施者 |
| 未知枚举提升权限 | Core 比较和 ROS 消费边界双重闭合集校验 | 实施者 |

---

## 6. 依赖与接口

### 上游依赖
- 父任务架构决策（D-011、D-015、D-019、D-021、D-023、D-024、D-025）
- ROS 2 Jazzy
- sensor_msgs、nav_msgs
- Eigen 3

### 下游接口
- **C2 本机地图**：消费 `LidarObservation`、`PoseEstimate`、`HealthState`
- **C3 地图更新**：使用 sensor ID/session/epoch

---

## 7. 时间估算

| 阶段 | 工作量 | 依赖 |
|------|--------|------|
| 阶段 1：核心类型 | 1-2 天 | 无 |
| 阶段 2：ROS Adapter | 2-4 天 | 阶段 1 |
| 阶段 3：健康门控 | 1-2 天 | 阶段 1 |
| 阶段 4：ROS 节点 | 2-3 天 | 阶段 2, 3 |
| 阶段 5：Fixture 与回归 | 2-3 天 | 阶段 4 |
| 阶段 6：集成测试 | 2-3 天 | 阶段 5 |
| 阶段 7：射线证据契约 | 已完成 | 阶段 6 |
| 阶段 8：安全加固与可选调试视图 | 1-2 天 | 阶段 7 |
| **总计** | **11-19 天** | - |

**注**：阶段 2 增加 1 天缓冲（位置跳变检测和 clock_domain 集成）

---

## 8. 完成标志

- [x] 所有 C1 范围内验收门（AC-01 至 AC-10）通过；旧 session/map epoch/local map 明确 deferred 到 C2
- [x] 所有 C1 范围内质量门通过
- [x] 代码提交：用户于 2026-07-26 授权在当前 `phase/4-perception-swarm-refactor` 分支完成 C1 提交
- [x] C1 PRD、实施记录与测试文档已更新
- [x] 用户已验收 C1，并于 2026-07-26 批准提交、归档及进入后续子任务
- [x] Trellis 任务归档已获用户授权，与本次收口提交连续执行

---

## 9. 下一步

完成 C1 后：
1. 标记 C1 为 `completed`
2. 创建 C2 子任务（本机观测消费与局部地图）
3. C2 将消费 C1 的 `LidarObservation` 和 `PoseEstimate`

---

## 10. C1 收口补测记录（2026-07-24）

### 新增覆盖

- TFMessage → TransformStamped → PoseEstimate → health gate 全链路；
- Pose expected frame 与 minimum quality 门控；
- 受控 SIGKILL 后 launch respawn，验证新进程 SessionID 变化；
- 120 帧唯一观测端到端发布，验证 0 丢帧；
- Adapter 转换与 MapperHealthGate 分阶段性能基线。

### 干净质量门

使用独立目录，未复用 `ws/build` / `ws/install`：

```text
/tmp/alien-c1-final-release-20260724/build
/tmp/alien-c1-final-release-20260724/install
/tmp/alien-c1-final-release-20260724/log
```

结果：

```text
5 packages built
55 tests
0 errors
0 failures
0 skipped
```

性能记录：

```text
laser_scan_conversion    1000 samples average 0.107 µs
mapper_health_evaluation 5000 samples average 0.056 µs
ros_publish_pipeline     120 frames, lost 0
  average 1.092 ms, P95 0.477 ms
  max 106.014 ms (first-frame DDS discovery)
```

### 剩余边界

- RViz 可视化目检已完成：统一场景中的水平 2D、向 +Z 抬起的倾斜 2D、标准多线 3D、TF 和独立勾选行为均通过；
- CLion 手写头文件跳转已由用户人工确认通过；
- 旧 SessionID 的消费侧拒绝、map epoch 重建和 local map 属于 C2；
- free-ray capability 由本轮阶段 7 收口，具体 occupancy 消费仍属于 C2。

---

## 11. AC-09 独立检查记录（2026-07-26）

`trellis-check` 对 Core → Adapter → Node/YAML → ROSIDL → fixture/docs 做了全量
跨层审查，并补齐以下覆盖：

- `HitOnly` 不满足 `HitRay`；`HitRay` 满足 degraded `HitRay` 但不满足 minimum
  `FullRay`；ROS health 区分 `(free_hit=true, full_no_return=false)` 与两者都为真；
- minimum、degraded、sensor 三个射线证据配置入口的非法值均启动失败；
- LaserScan range/FOV/resolution 相对冻结 descriptor 的双向漂移均拒绝；
- 默认 descriptor 与 observation 证据等级固定为保守的 `HitOnly`。

独立 Release 目录：

```text
/tmp/alien-c1-ac09-check-20260726-02/build
/tmp/alien-c1-ac09-check-20260726-02/install
/tmp/alien-c1-ac09-check-20260726-02/log
```

结果：

```text
5 packages built
70 tests
0 errors
0 failures
0 skipped
git diff --check passed
```

未发现 AC-09 范围内的高置信遗留问题；occupancy 写入、观测窗口、map epoch
和旧 session 消费拒绝仍保持为 C2 边界。该历史检查点之后的 AC-10 实施与
用户验收结果见 §12-§13；本检查点未自动归档或提交。

---

## 12. AC-10 实施验证记录（2026-07-26）

独立 Release 目录：

```text
/tmp/alien-c1-ac10-release-20260726-01
```

结果：

```text
5 packages built
80 tests
0 errors
0 failures
0 skipped
git diff --check passed
```

自动化已覆盖 Marker 语义、非法批次拒绝和 ROS graph 中不存在
`/drone_0/scan_returns` publisher/subscriber。此历史检查点记录自动化结果；
RViz 人工验收随后已在 §13 的最终收口中由用户确认通过。本轮未自动
归档或提交。

---

## 13. AC-10 最终全范围检查记录（2026-07-26）

最终 `trellis-check` 复核五包与跨层契约后，补齐三项安全/验收证据：

- `PointCloud2Adapter::convert()` 对 hit-only 输入也执行完整字段、步长和 data
  长度验证，畸形空存储不会进入点读取；未知 `3/255` 继续 fail closed；
- 输入节点直接调用一次 adapter `convert()` 并在 callback 内捕获、限频记录输入
  异常，消除 LaserScan/PointCloud2 双校验且不让异常逃出 callback；
- 调试消费者拒绝 Cloud3D 携带 2D 标量元数据、空 clock domain 和零 boot
  session；launch test 证明稳定 marker ID、空 `ADD` 覆盖能力降级后的旧线段、
  有限 lifetime，以及 graph 无旧节点和 OctoMap/occupancy topic。

最终独立 Release 目录：

```text
/tmp/alien-c1-ac10-tunnel-final-20260726-01
```

结果：

```text
5 packages built
74 logical test methods + 14 CTest aggregate entries = 88 tests
0 errors
0 failures
0 skipped
git diff --check passed
```

独立证据调试 RViz 已由用户于 2026-07-26 对截图
`C:\Users\loong\AppData\Local\Temp\alien-c1-ac10-tunnel-rviz-20260726-11.png`
确认通过。默认视图是 map YZ 截面，红 X 轴为截面法向，绿 Y/蓝 Z 轴位于
截面内；洞壁 hit、岔口 no-return 扇区和四类 invalid 均可辨。既有性能报告
保持不变，本轮没有自动提交或归档。

---

## 14. C1 覆盖率收口记录（2026-07-26）

早期阶段 1/2 的 `>90%` / `>85%` 仍作为正式单元测试行覆盖率门处理，未以
后续测试总数替代。容器没有 `gcovr`/`lcov`，使用 GCC 13.3 自带 `gcov` 的
JSON 中间格式在全新目录中实测：

```text
/tmp/alien-c1-closeout-coverage-20260726-02
```

口径：只运行 `perception_core` 和 `perception_adapters` 的 gtest；分别纳入包内
生产 `include/**` 与 `src/**` 的 gcov 可执行行，排除 `test/**`、生成代码、ROS
节点和外部依赖。按规范化源码路径与行号合并 JSON 记录；同一模板/inline 行被
多个翻译单元实例化时，任一实例执行即视为该源码行覆盖。

构建与测试命令：

```bash
cd /workspaces/alien-scanner/ws
source /opt/ros/jazzy/setup.bash

colcon --log-base /tmp/alien-c1-closeout-coverage-20260726-02/log build \
  --build-base /tmp/alien-c1-closeout-coverage-20260726-02/build \
  --install-base /tmp/alien-c1-closeout-coverage-20260726-02/install \
  --symlink-install --packages-up-to perception_adapters \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
    '-DCMAKE_CXX_FLAGS=-O0 -g --coverage' \
    '-DCMAKE_EXE_LINKER_FLAGS=--coverage' \
    '-DCMAKE_SHARED_LINKER_FLAGS=--coverage'

source /tmp/alien-c1-closeout-coverage-20260726-02/install/setup.bash
ROS_DOMAIN_ID=98 colcon \
  --log-base /tmp/alien-c1-closeout-coverage-20260726-02/test-log test \
  --build-base /tmp/alien-c1-closeout-coverage-20260726-02/build \
  --install-base /tmp/alien-c1-closeout-coverage-20260726-02/install \
  --packages-select perception_core perception_adapters

coverage_root=/tmp/alien-c1-closeout-coverage-20260726-02
mkdir -p "${coverage_root}/gcov-json/core" \
         "${coverage_root}/gcov-json/adapters"
for spec in core:perception_core adapters:perception_adapters; do
  label=${spec%%:*}
  package=${spec##*:}
  (
    cd "${coverage_root}/gcov-json/${label}"
    find "${coverage_root}/build/${package}" -name '*.gcda' \
      -exec gcov --json-format '{}' +
  )
done

cd /workspaces/alien-scanner
python3 -m unittest scripts/test_summarize_perception_coverage.py
python3 scripts/summarize-perception-coverage.py \
  --source-root /workspaces/alien-scanner/ws/src/alien_perception \
  --gcov-root "${coverage_root}/gcov-json" \
  --package perception_core:core:90 \
  --package perception_adapters:adapters:85
```

`summarize-perception-coverage.py` 按规范化 realpath 与行号对生产源码做并集；
同一行在多个 JSON/翻译单元中出现时，任一实例 `count > 0` 即视为覆盖。
脚本仅接受包内 `include/**` 和 `src/**` 的 gcov 可执行行，排除测试、外部/
生成源码及 `*Node.cpp`，并对严格 `>` 门限返回非零失败码。汇总结果：

```text
perception_core      209 / 210 lines = 99.52% (required > 90%) PASS
perception_adapters  249 / 277 lines = 89.89% (required > 85%) PASS
39 logical gtest methods + 2 CTest aggregate entries = 41 tests
0 errors, 0 failures, 0 skipped
```

两个覆盖率门均有独立、可重复证据并已通过。

---

## 15. 未完成流程与下游边界

- C2 deferred：旧 SessionID 的消费侧拒绝、pose reset 触发 map epoch 重建、断开
  fleet 后的 local map、旧 FakeLidar 地图数值对比。
- 历史非门禁：rosbag 资产已被确定性 C++ fixture 取代；HealthState RViz 颜色编码
  未纳入 C1；节点没有运行时动态注册 API，该 inventory 语义由单元测试覆盖。
- C1 的代码提交、进入后续子任务和 Trellis 归档均已于 2026-07-26 获用户授权，
  由本次收口连续执行。

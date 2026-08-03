# C1：感知输入与观测数据模型

> 子任务状态：in_progress（C1 功能与验收已收口，仅剩提交、批准进入 C2 与归档流程）
> 父任务：07-21-perception-swarm-architecture-refactor
> 创建时间：2026-07-22

---

## 1. 目标

建立**后端无关的观测数据模型**，使多种传感器输入（2D LiDAR、3D LiDAR、混合配置）和位姿估计可以通过统一契约进入本机地图构建流程，同时保留来源追溯、健康状态和降级能力声明。

### 核心交付物

1. **ROS-free 观测数据契约**：`LidarObservation` 抽象和标准 ROS 消息的转换 adapter
2. **传感器身份与健康模型**：sensor ID/descriptor/session、health state、capability 声明
3. **位姿输入契约**：标准 Odometry/TF 转为 ROS-free `PoseEstimate`，保留质量、freshness、reset epoch
4. **Mapper 输入声明**：minimum viable input、degraded 组合、健康门控
5. **确定性 fixture 与回归**：标准 2D/多线 3D/混合 fixture；旧 FakeLidar 兼容性由专属回放负责

---

## 2. 需求（Requirements）

### R-01：标准传感器输入支持

**内容**：支持标准 ROS `sensor_msgs/LaserScan` 和 `sensor_msgs/PointCloud2` 作为原始输入。

**理由**：
- 保持与现有仿真/回放工具的兼容性
- 降低真实硬件接入门槛
- 不强制用户切换到自定义消息格式

**依赖决策**：D-011（独立 interfaces 包，原始 LiDAR 保持标准消息）

---

### R-02：传感器身份与溯源

**内容**：每个 sensor 必须有独立的：
- **Stable sensor ID**：用户配置的稳定标识符（不随重启改变）
- **Frame ID**：TF 树中的坐标系名称
- **Origin stamp**：观测产生的本地时间戳
- **Session ID**：当前 producer 启动生成的 session 标识
- **Health state**：Healthy / Degraded / Unavailable

**理由**：
- 支持多传感器融合时区分来源
- 断线重连后能识别是同一物理传感器
- 健康状态变化可触发 mapper 降级策略

**依赖决策**：D-015（stable ID + boot session）、D-019（sensor inventory 冻结）

---

### R-03：传感器 descriptor inventory 冻结

**内容**：
- 同一 **vehicle session** 内，传感器 descriptor inventory **冻结**
- Descriptor 包括：geometry（安装位置/朝向）、capability（视场角/分辨率与射线证据等级）、type（2D/3D）
- 相同 ID 和 descriptor 的 producer 可以掉线后重连
- 增删传感器或改变 descriptor 必须建立**新 vehicle session** 并重新 registration

**理由**：
- 避免运行中突然出现新传感器导致地图坐标系混乱
- 简化 mapper 的输入假设（不需要处理动态增删）
- 重连场景明确：同一硬件可恢复，换硬件必须重启

**依赖决策**：D-019（sensor inventory 冻结）

---

### R-04：位姿输入边界

**内容**：
- **本轮不实现定位算法**（VIO/LIO/SLAM/IMU 融合）
- 以标准 `nav_msgs/Odometry` 和 TF 作为输入
- 转换为 ROS-free `PoseEstimate`，保留：
  - **Source ID / session**：位姿来源的身份
  - **Frame ID**：参考坐标系
  - **Quality / covariance**：估计质量
  - **Freshness**：最后更新时间，用于超时检测
  - **Reset epoch**：位姿源重置计数器（重定位/坐标系变化）

**理由**：
- 本轮聚焦感知与地图架构，不分散精力实现定位
- 保留足够元数据以便后续替换真实定位算法
- Freshness 和 reset epoch 用于触发安全门控

**依赖决策**：D-023（定位算法外部化）、D-024（pose reset 处理）

---

### R-05：Mapper 输入契约声明

**内容**：Active mapper 必须在启动时声明：
- **Minimum viable input**：最低运行要求（如"至少 1 个 3D LiDAR + 位姿"）
- **Degraded combinations**：允许的降级配置（如"单个 2D LiDAR 时标记 Degraded，但仍可产出地图"）

**行为**：
- 满足 minimum 时正常运行（Healthy）
- 满足 degraded 但不满足 minimum 时标记 **Degraded**，仍产出地图但标注能力受限
- 不满足任何组合时标记 **Unavailable**，停止地图 revision，触发 freshness 超时

**理由**：
- 明确降级边界，避免"半死不活"的状态
- 健康状态可驱动上游任务分配决策
- 恢复后能重新通过健康门

**依赖决策**：D-021（mapper 输入契约声明）、D-022（分层健康门控）

---

### R-06：Vehicle-local 自主性

**内容**：
- **断开 fleet 链路后，本机仍可产出 observation 和 local map**
- 观测处理不依赖中央协调器或其他节点
- 本机地图更新只依赖本机传感器和位姿

**理由**：
- 保证通信分区时的本机安全（known-free 判断）
- 降低对网络延迟的敏感度
- 符合"本地优先"的架构原则

**依赖决策**：D-025（mapper 位于 vehicle-local domain）

---

### R-07：确定性 fixture 与回归

**内容**：
- 提供 **2D、3D、混合配置的 fixture**（固定输入数据）
- 2D `LaserScan` 保持消息 frame 的 XY 平面约定；3D 使用传感器自身 XY 平面的多线仰角通道
- 雷达安装倾角由 TF 表达，不把旧 Phase 3 的特殊单环俯仰公式复制到通用 fixture
- 旧 `FakeLidar` 的方向等价性由旧路径专属 replay/adapter 验证
- 固定 RViz 输出，便于目检

**理由**：
- 确保新契约不破坏现有感知行为
- 回归测试可重复
- 便于后续子任务（C2 地图）建立基线对照

**依赖决策**：D-031（分级仿真验收，Level 0 使用确定性 perception fixture；旧 FakeLidar 仅用于旧路径专属回放）

---

### R-08：降级能力显式声明

**内容**：
- 每个 sensor descriptor 和每批 `LidarObservation` 必须声明射线证据等级：
  - `HitOnly`：只证明命中端点，不得产生 free-space 证据；
  - `HitRay`：可使用从可靠 origin 到命中端点的 free-space 证据，但没有完整 no-return 射线；
  - `FullRay`：可区分 `Hit`、`NoReturn`、`Invalid`，并对合法 hit/no-return 射线提供完整 free-space 证据。
- `LaserScan` 保留原生 ranges，并按标准数值语义确定性分类：量程内有限值为 `Hit`，正无穷为 `NoReturn`，NaN、负无穷和量程外有限值为 `Invalid`。
- 当前 `PointCloud2` adapter 只保留 XYZ 命中点，因此只允许声明 `HitOnly`；不得根据点序、安装 TF 或量程参数静默补造 no-return 射线。
- `PointCloud2Adapter::convert()` 自身必须 fail closed，不能依赖调用方先执行 `validate()`；直接传入 `HitRay`/`FullRay` descriptor 也不得返回 observation。
- `HitRay`/`FullRay` LaserScan 必须在转换前验证消息量程、角边界和角增量均有限有效，并与冻结 descriptor 在明确容差内一致；元数据漂移时拒绝整批 observation。
- Active mapper 的输入要求可以声明最低射线证据等级；当前有效能力和每批 observation 的证据等级必须可由 ROS-free 消费者及 ROS 消息读取。
- 节点运行时配置必须分别声明 minimum/degraded input 的最低射线等级；非法枚举值启动失败，未配置时保守使用 `HitOnly`。
- Descriptor 的射线证据等级属于冻结 inventory；运行中改变该等级必须建立新 vehicle session。

**理由**：
- C2 必须知道哪些观测可以清理 free space，不能从消息类型猜测证据语义
- hit-only 点云若被当作完整射线会制造不存在的 known-free 区域，破坏本机安全门
- 保留原生 payload 并按需查询分类，避免为统一 ray array 强制复制大型点云

**依赖决策**：D-008（后端无关 occupancy 契约）、D-021（mapper 输入契约）、D-025（vehicle-local mapper）

---

### R-09：可选射线证据调试视图

**内容**：
- 提供独立可选 RViz 调试入口，只消费正式
  `perception_interfaces/LidarObservation`，把命中端点、hit free-space、完整
  no-return free-space 和 invalid measurement 显式区分；
- 调试视图不得写 occupancy、发布地图或成为 mapper，不得改变现有 fixture
  几何验收入口；
- 2D 视图严格按 `HitOnly < HitRay < FullRay` 控制可显示的空间证据；3D
  hit-only 点云只显示命中端点，不得画 origin-to-point free ray；
- 确定性调试 fixture 在固定 beam 注入 Hit、正无穷、NaN、负无穷及量程外有限
  值，且不改变现有 fixture 的默认几何数据；
- 默认 RViz 调试场景使用洞轴沿 map `+X` 的 360° `YZ` 椭圆隧道截面：洞壁为
  有限 `Hit`，固定岔口扇区为 `+inf/NoReturn`，四条独立 beam 分别表达 NaN、
  `-inf`、低于最小量程和高于最大量程；默认画面不混入 Cloud3D 散点；
- 运行时不得订阅、桥接或恢复旧 `/drone_0/scan_returns`；首版不引入旧地图
  静态快照，旧地图只能在未来通过正式新接口离线生成可复现参考资产；
- `LaserScanAdapter::convert()` 的完整验证不可绕过，未知 ray-evidence 数值必须
  fail closed，防止调试消费者或未来 C2 把非法枚举提升为 `FullRay`。

**理由**：
- 让 C1 证据语义可以目检，同时不混入 C2 occupancy 行为；
- 只验证正式新数据流，避免调试工具反向延长已废弃接口寿命；
- 在进入 C2 前封闭两个可绕过的 free-space 权限边界。

---

## 3. 验收标准（Acceptance Criteria）

### AC-01：标准 ROS 消息输入
- [x] `sensor_msgs/LaserScan` 可转换为 `LidarObservation`
- [x] `sensor_msgs/PointCloud2` 可转换为 `LidarObservation`
- [x] 保留 frame_id、stamp、clock_domain、origin（sensor ID/session）

### AC-02：传感器身份与溯源
- [x] 每个 observation 携带 sensor ID、session、frame、stamp
- [x] 多个传感器的 observation 不被错误合并
- [x] producer 进程重启后 SessionID 变化，重启后的数据不复用旧 SessionID

**C2 deferred（非 C1 验收门）**：旧 session 由 observation/map 消费者拒绝。

### AC-03：Sensor descriptor inventory 冻结
- [x] Vehicle session 启动时冻结 descriptor inventory
- [x] 相同 ID/descriptor 的 producer 掉线后可重连
- [x] 增删传感器或改变 descriptor 时，输入节点拒绝并要求新 vehicle session

### AC-04：位姿输入契约
- [x] `nav_msgs/Odometry` 和 TF 可转换为 `PoseEstimate`
- [x] 保留 source ID/session、frame、clock_domain、quality、freshness、reset epoch
- [x] Pose stale（freshness 超时）时触发门控
- [x] Pose frame 与配置的 expected frame 不匹配时触发门控
- [x] Pose quality 低于配置阈值时触发门控
- [x] Pose 时间回退时递增 reset epoch 并触发相应门控状态
- [x] Pose 位置跳变（> 5m in < 1s）时递增 reset epoch

**C2 deferred（非 C1 验收门）**：消费 reset epoch 并触发 map epoch 重建。

### AC-05：Mapper 输入声明
- [x] Mapper 启动时声明 minimum viable input
- [x] Mapper 启动时声明允许的 degraded 组合
- [x] 输入满足 minimum 时标记 Healthy
- [x] 输入满足 degraded 但不满足 minimum 时标记 Degraded
- [x] 输入不满足任何组合时标记 Unavailable
- [x] Unavailable 状态不产生伪 freshness（故障立即生效，恢复需稳定窗口）

### AC-06：健康状态转换
- [x] Healthy → Degraded（传感器掉线但仍满足 degraded）
- [x] Degraded → Unavailable（不满足任何组合）
- [x] Unavailable → Degraded（传感器恢复但未全部恢复）
- [x] Degraded → Healthy（所有传感器恢复）
- [x] 恢复后重新通过健康门（不能直接从 Unavailable 跳到 Healthy）

### AC-07：Vehicle-local 自主性
- [x] 不启动任何 fleet 组件时，本机仍可连续产出 observation

**C2 deferred（非 C1 验收门）**：断开 fleet 链路后继续产出 local map。

### AC-08：确定性 fixture
- [x] 提供 2D LaserScan fixture
- [x] 提供 3D PointCloud2 fixture
- [x] 提供混合 2D+3D fixture
- [x] 标准 2D XY 与多线 3D 仰角方向约定（回归测试通过）
- [x] 固定 RViz 输出可目检

### AC-09：降级能力显式声明
- [x] ROS-free descriptor/observation 使用闭合枚举声明 `HitOnly`、`HitRay`、`FullRay`，且 capability 变化参与 descriptor equality/freeze 判断
- [x] `Scan2D` 对 `Hit`、`NoReturn`、`Invalid` 提供无额外 payload 复制的确定性查询；边界值、NaN 和正负无穷均有 contract tests
- [x] 当前 `PointCloud2` adapter 对高于 `HitOnly` 的声明 fail closed，并有测试证明不会伪造 free-space/no-return 证据
- [x] `HitRay`/`FullRay` LaserScan 的 range/FOV/resolution 与冻结 descriptor 不一致时整批拒绝；量程边界和元数据漂移均有测试
- [x] `MapperInputContract` 及运行时 minimum/degraded 配置可声明最低射线证据；非法值启动失败，聚合健康状态和 `LidarObservation.msg` 可被上游读取并区分 free-hit-ray 与 full no-return-ray 能力

### AC-10：可选射线证据调试视图
- [x] `LaserScanAdapter::convert()` 直接调用也执行完整验证，非法元数据不产生 observation
- [x] 未知 `RayEvidenceCapability` 数值在能力比较和 ROS 消费边界均 fail closed
- [x] 调试节点只消费 `LidarObservation`，ROS graph 中没有 `/drone_0/scan_returns` 依赖
- [x] MarkerArray 分别编码红色 hit endpoint、绿色 hit free、青色 full no-return free 和灰色 invalid 指示
- [x] `HitOnly`/`HitRay`/`FullRay` 的 Marker 输出严格遵守能力上限；Cloud3D hit-only 不画 free ray
- [x] 独立 launch 可关闭 RViz 做自动化验证，现有 fixture visualization 默认行为不变
- [x] 默认 RViz 以 map `+X` 方向观察 360° `YZ` 隧道环切面，洞壁、岔口 no-return 扇区和四类 invalid 清晰可辨；map 红色 X 轴为截面法向，绿色 Y/蓝色 Z 轴位于截面内。用户于 2026-07-26 确认截图 `C:\Users\loong\AppData\Local\Temp\alien-c1-ac10-tunnel-rviz-20260726-11.png` 目检通过
- [x] 调试视图不发布 occupancy/OctoMap，首版不提交旧地图静态快照

---

## 4. 依赖

### 上游依赖
- 父任务架构决策（D-011、D-015、D-019、D-021、D-023、D-024、D-025）

### 下游依赖
- **C2 本机地图**：消费 `LidarObservation` 和 `PoseEstimate`
- **C3 地图更新**：使用 sensor ID/session/epoch 进行溯源

---

## 5. 风险与缓解

| 风险 | 缓解措施 |
|------|---------|
| 新契约破坏现有感知行为 | 固定 fixture 回归测试；并行运行新旧路径对比 |
| 健康状态转换逻辑复杂 | 明确状态机；单元测试覆盖所有转换路径 |
| Descriptor 冻结限制灵活性 | 明确重启边界；提供清晰的错误诊断 |
| Pose reset 触发逻辑不明确 | 接口定义在 C1，实际触发逻辑留给 C2 地图子任务 |
| hit-only 点云被错误提升为 free-ray | PointCloud2 adapter 只接受 `HitOnly`；能力提升必须由未来具有可靠 ray schema 的独立 adapter 实现 |
| 调试工具延长旧接口寿命 | 只订阅 `LidarObservation`；集成测试断言 ROS graph 中不存在 `/drone_0/scan_returns` |
| 未知枚举获得高权限 | 所有比较和 ROS 解析先验证闭合枚举，未知值统一 fail closed |

---

## 6. 相关文档

- 父任务：`.trellis/tasks/07-21-perception-swarm-architecture-refactor/`
- 决策总览：`docs/decisions/perception-and-swarm-architecture-refactor.md`
- Phase 3 基线：`docs/decisions/phase-03-swarm-qa.md`
- 旧草案迁移记录：`research/superseded-c1-draft-merge.md`

---

## 7. Notes

- 任务已进入实施；本轮收口 AC-09 安全边界并增加 AC-10 可选调试视图，不把旧草案中未采用的 `ObservationWindow`、缓存和 replay 扩展重新塞入 C1
- PRD 聚焦需求、约束和验收标准，不包含实现细节
- 实施顺序和回滚策略见 `implement.md`

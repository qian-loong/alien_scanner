# C1：感知输入与观测数据模型 - 实施计划

> 子任务：07-22-c1-perception-input
> 父任务：07-21-perception-swarm-architecture-refactor

---

## 1. 实施顺序

### 阶段 1：核心类型与接口（1-2 天）

- [ ] 定义 `SensorID`、`SessionID`、`SourceID`
- [ ] 定义 `SensorDescriptor`（type, frame, FOV, mounting pose）
- [ ] 定义 `LidarObservation`（Scan2D / Cloud3D）
- [ ] 定义 `PoseEstimate`（position, orientation, quality, freshness, reset_epoch）
- [ ] 定义 `HealthState`、`SensorHealth`
- [ ] 定义 `MapperInputContract`、`MapperHealthGate`
- [ ] 单元测试：类型构造、相等性比较、variant 访问

**验证门**：
- [ ] 所有核心类型编译通过
- [ ] 单元测试覆盖率 > 90%
- [ ] 无 ROS 依赖（纯 C++17 + Eigen）

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
- [ ] 单元测试覆盖率 > 85%

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
  - [ ] 录制 rosbag
  - [x] 配置 sensor descriptor YAML
- [x] 创建 3D PointCloud2 fixture
  - [x] 标准多线场景（默认 16 条仰角、每线 360 个方位点）
  - [ ] 录制 rosbag
  - [x] 配置 sensor descriptor YAML
- [x] 创建混合 2D+3D fixture
  - [x] 2 个 2D + 1 个 3D 传感器
  - [ ] 录制 rosbag
  - [x] 配置 sensor descriptor YAML
- [x] 标准多线 3D profile 回归
  - [x] 验证传感器 XY 平面、方位和正负仰角语义
  - [ ] 旧 FakeLidar 与 Phase 3 baseline 的地图对比（旧路径专属，由 C2 子任务验证）
- [x] 创建 RViz 可视化配置
  - [x] 显示观测数据（LaserScan / PointCloud2）
  - [ ] 显示健康状态（颜色编码）
  - [ ] 显示传感器 TF frames

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
  - [ ] 运行时动态注册新传感器 ID：当前节点无运行时 register API；新增仅能经配置 + 新进程/session，由单元测试覆盖 inventory 拒绝语义
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
- [ ] Pose reset epoch 变化时触发 map epoch 重建（接口定义，逻辑留给 C2）

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
- [ ] 断开 fleet 链路后，本机仍可产出 local map（接口定义，逻辑留给 C2）

### AC-08：确定性 fixture
- [x] 提供 2D LaserScan fixture
- [x] 提供 3D PointCloud2 fixture
- [x] 提供混合 2D+3D fixture
- [x] 标准 2D XY 与多线 3D 仰角方向约定（回归测试通过）
- [x] 固定 RViz 输出可目检

### AC-09：降级能力显式声明
- [ ] Mapper 缺失 free-ray 能力时显式声明（不静默失败）
- [ ] 缺失能力的声明可被上游读取（接口定义，使用留给后续子任务）

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
| ROS 2 接口不稳定 | 固定 ROS 2 Humble；避免 beta 特性 | 实施者 |

---

## 6. 依赖与接口

### 上游依赖
- 父任务架构决策（D-011、D-015、D-019、D-021、D-023、D-024、D-025）
- ROS 2 Humble
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
| **总计** | **10-17 天** | - |

**注**：阶段 2 增加 1 天缓冲（位置跳变检测和 clock_domain 集成）

---

## 8. 完成标志

- [ ] 所有验收门（AC-01 至 AC-09）通过
- [ ] 所有质量门通过
- [ ] 代码已提交到 `feature/c1-perception-input` 分支
- [ ] 文档已更新（README、API 文档）
- [ ] 用户已验收并批准进入 C2 子任务

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
- free-ray capability 接口仍按父任务后续设计处理。

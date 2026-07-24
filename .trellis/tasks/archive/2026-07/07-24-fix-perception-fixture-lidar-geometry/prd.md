# 修正感知测试雷达几何

## Goal

纠正 `perception_fixtures` 将旧 `drone_scanner::FakeLidar` 的可俯仰 YZ 垂直环误作通用 3D LiDAR fixture 的问题，使测试数据分别遵守标准 ROS 2D `LaserScan` 坐标约定和标准多线 3D LiDAR 束线约定，并提供不含旧项目特殊几何的确定性感知链路输入和安装姿态可视化场景。

## Background

- `FixtureScene::cloud_points()` 当前使用
  `Ry(ring_pitch_rad) * (0, cos(theta), sin(theta))`，因此 `0 rad` 是 YZ 垂直环；证据位于 `ws/src/alien_perception/perception_fixtures/src/FixtureScene.cpp:32`。
- 该约定复制自旧 `drone_scanner::FakeLidar` 的特殊前倾垂直环，不是标准多线 3D LiDAR 的固定仰角通道模型。
- `perception_fixtures` 没有代码或链接依赖 `drone_scanner`；本次可以独立修正新 fixture，不需要修改旧 `FakeLidar`。
- 当前 `LaserScan` 数据由 `scan_ranges()` 独立生成，`ring_pitch_rad` 只影响 3D `PointCloud2`。
- 当前集成测试只验证 3D 点数、类型与 frame，没有验证点云的多线仰角结构。

## Requirements

### R1：保持标准 2D LaserScan 约定

- 2D 扫描数据继续位于消息 `frame_id` 的 XY 平面，角度绕 +Z 轴定义。
- 雷达相对 `base_link` 的水平、倾斜或垂直安装姿态由 TF 表达，不通过修改 `LaserScan` 内部点几何表达。
- 本次不把旧 YZ 垂直环迁移为新的 2D fixture 数据格式，也不修改旧 `FakeLidar`。

### R2：使用标准多线 3D LiDAR fixture

- 从通用 `FixtureSceneConfig` 和 fixture publisher 参数中移除 `ring_pitch_rad`。
- 3D fixture 使用一组固定垂直仰角通道；每个通道按完整方位角采样并生成确定性 XYZ/I 点云。
- 默认 profile 为厂商无关的 16 线模型：仰角 `-15°、-13°、...、+15°`，每线 360 个方位采样，共 5760 点。
- 配置接口允许显式传入 `elevation_angles_rad`，以便单元测试使用包含 `0 rad` 的小型通道集合。
- 垂直仰角以传感器自身 XY 平面为 `0 rad`，正值朝 +Z，负值朝 -Z。
- 3D 雷达整体安装姿态属于 TF，不编码进束线仰角或点云坐标。
- 点云生成保持固定参数下完全确定。

### R3：同步 ROS 接线与测试

- `fixture_3d`、`fixture_mixed` launch 和 mixed 集成测试不得再传入 `ring_pitch_rad`。
- Publisher 暴露与标准 3D fixture 对应的清晰参数；点数和距离沿用现有最小值规范化，空、非有限或超出 `[-pi/2,+pi/2]` 的仰角配置必须拒绝。
- gtest 必须逐项验证默认 16 线角表、点数、径向距离公式、垂直通道数量、正负仰角、intensity 和确定性，而不是仅验证存在非零 X。
- 集成测试必须继续验证 2D、3D、mixed 数据能够通过 `perception_input_node`；mixed 用紧凑的 3 通道 × 4 方位 profile 锁定 Publisher/Adapter 后的实际 XYZ、intensity 和顺序。
- ROS 层必须断言 `LaserScan` 的 `angle_min/max/increment`，并证明中间样本为零角；根据 ROS 消息约定，零角沿 +X、正角绕 +Z 朝 +Y。
- 更新 `docs/perception-input-testing.md` 与 `docs/perception-class-relations.md`，明确 fixture 是确定性链路输入，不是旧 `FakeLidar` 的兼容回放。
- 纠正 C1 任务文档中将通用 fixture 绑定旧 Phase 3 `ring_pitch_rad` 公式的条目；旧 FakeLidar 迁移等价性继续由旧路径专属 replay/adapter 承担。

### R4：提供统一的安装姿态可视化场景

- 单个可视化 launch 同时发布水平 2D、倾斜 2D 和标准多线 3D fixture，不新建独立 examples/demo 包。
- 两个 `LaserScan` 的原始数据都保持传感器 frame 的 XY 平面；倾斜 2D 只通过 `base_link -> sensor frame` 静态 TF 表达安装 pitch。
- 倾斜 2D 默认绕 Y 轴使用 `-30°` pitch，使传感器本地 +X 朝 `base_link` 的 +Z 方向抬起。
- RViz 使用 `base_link` 作为 Fixed Frame，并为两组 2D 扫描、3D 点云和 TF 提供可独立勾选的 display；任意组合即表示相应的混装观察方式。
- 可视化 launch 支持关闭 RViz 做无界面 smoke，既有自动化 `fixture_2d/3d/mixed` launch 保持不变。

### R5：保护旧系统边界

- 不修改 `ws/src/drone_scanner`、`ws/src/swarm_controller` 及其 Phase 2/3 几何、测试和 launch。
- 不改变 `perception_core`、`perception_adapters` 或 `perception_interfaces` 的公共观测契约。
- 不把新旧扫描路径同时接入同一个 mapper。

## Acceptance Criteria

- [x] AC1：仓库的 `perception_fixtures` 生产代码、launch 和测试中不再出现 `ring_pitch_rad`。
- [x] AC2：ROS-free 与 ROS round-trip 都证明零仰角通道位于传感器 XY 平面，正/负仰角通道分别产生正/负 Z 分量，零方位沿 +X，正方位朝 +Y。
- [x] AC3：默认角表逐项等于 `-15°、-13°、...、+15°`；每个配置通道覆盖完整方位角，总点数等于方位采样数乘以垂直通道数。
- [x] AC4：相同配置重复生成的 ranges、XYZ 和 intensity 完全一致；径向距离与 `(seed + linear_index) % 256` 逐点符合设计公式。
- [x] AC5：2D `LaserScan` 的 `angle_min=-pi/2`、`angle_max=+pi/2`、增量、181 点、frame 和零角中间样本契约保持不变。
- [x] AC6：`perception_fixtures` 的 gtest 与全部 launch integration test 通过，`colcon test-result --verbose` 无失败。
- [x] AC7：实施前后的受保护目录文件清单、工作区状态和 SHA-256 清单一致，证明旧 `FakeLidar`、`drone_scanner` 与 `swarm_controller` 没有因本任务产生改动。
- [x] AC8：统一 RViz 场景能在 `base_link` 下同时显示水平 2D、倾斜 2D、标准多线 3D 和对应 TF；三个传感器 display 可独立勾选，倾斜 2D 的原始 `LaserScan` 几何不变，默认安装使传感器本地 +X 朝 `base_link` 的 +Z 方向抬起。
- [x] AC9：`docs/perception-class-relations.md` 的 `FixtureSceneConfig` 说明与实现一致，不再描述通用 3D fixture 为单环俯仰。
- [x] AC10：C1 任务文档不再要求通用 fixture 复现旧 `ring_pitch_rad`，同时没有删除旧路径专属迁移验证要求。

## Out Of Scope

- 旧 `FakeLidar` 的重构、删除或兼容迁移。
- 基于洞穴或真实环境的 raycast、遮挡、运动畸变及厂商雷达模型。
- C2 mapper、OctoMap 或 Phase 3 replay 等价性修改。
- 本任务内新建面向二次开发者的独立 examples/demo 包。

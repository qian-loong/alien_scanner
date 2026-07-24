# 修正感知测试雷达几何 - 技术设计

## 1. 边界与结论

本任务只修正新 `perception_fixtures` 的确定性输入模型。旧
`drone_scanner::FakeLidar` 的 YZ 垂直环是 Phase 2/3 已冻结的特殊传感器模型，继续由旧回放和迁移验证负责；新 fixture 不再复制其几何，也不链接该包。

2D 与 3D 的本体几何和安装外参分开：

```text
2D raw data: LaserScan in sensor-frame XY
3D raw data: multi-channel PointCloud2 in sensor frame
world/base orientation = TF(base_link -> sensor frame) * raw direction
```

安装姿态通过一个统一的可视化场景验收。该场景只组合 fixture publisher、
静态 TF 和 RViz，不向 `FixtureScene` 写入安装倾角，也不改变既有自动化集成
测试入口。

## 2. ROS-free 配置契约

`FixtureSceneConfig` 调整为：

```cpp
struct FixtureSceneConfig {
    std::size_t scan_point_count = 181;
    std::size_t cloud_azimuth_sample_count = 360;
    float cloud_range_m = 5.0F;
    std::vector<float> elevation_angles_rad = default_16_channel_angles;
    std::uint32_t seed = 17U;
};
```

默认垂直角为 `-15°、-13°、...、+15°`，共 16 条，厂商无关且关于水平面对称。显式数组允许测试使用 `{-0.2F, 0.0F, 0.2F}` 等小 profile。

旧字段迁移：

| 旧字段/参数 | 新字段/参数 | 处理 |
| --- | --- | --- |
| `cloud_point_count` | `cloud_azimuth_sample_count` | 旧值只表示单环点数，改名后明确为每通道方位采样数 |
| `ring_radius_m` | `cloud_range_m` | 明确表示传感器到返回点的径向距离基值 |
| `ring_pitch_rad` | `elevation_angles_rad` | 删除旧特殊平面旋转语义，替换为多线束仰角数组 |

该包尚未形成稳定外部 API，launch 和测试都在当前工作区新增阶段，因此不保留已废弃参数别名，避免继续传播错误语义。

## 3. 3D 点云生成

对每个垂直通道 `elevation` 和每个方位样本 `azimuth`：

```text
azimuth = 2*pi*sample_index/azimuth_sample_count
range = cloud_range_m * (1 + 0.05*cos(2*azimuth))
horizontal_range = range*cos(elevation)

x = horizontal_range*cos(azimuth)
y = horizontal_range*sin(azimuth)
z = range*sin(elevation)
```

约定：

- `azimuth=0` 沿传感器 +X；正方位角绕 +Z 朝 +Y。
- `elevation=0` 位于传感器 XY 平面。
- 正仰角产生 `z>0`，负仰角产生 `z<0`。
- 输出顺序固定为通道优先、通道内方位递增。
- intensity 使用 `(seed + linear_point_index) % 256`，保持确定性。
- 点数严格等于 `elevation_angles_rad.size() * cloud_azimuth_sample_count`。

保留轻微确定性距离变化，以便可视化中识别方位，同时所有通道仍共享相同方位距离函数；它不模拟遮挡或真实场景。

## 4. 配置校验

沿用现有 fixture 的宽容数值下限：

- `scan_point_count` 最少为 2；
- `cloud_azimuth_sample_count` 最少为 1；
- `cloud_range_m` 最少为 0.1 m。

仰角数组承担结构语义，采用严格校验：

- 数组不得为空；
- 每个角必须是有限数；
- 每个角必须位于闭区间 `[-pi/2, +pi/2]`；
- 不强制排序，以保留调用者指定的通道顺序；重复角不影响正确性，暂不增加额外限制。

无效仰角由 `FixtureScene` 构造函数抛出 `std::invalid_argument`，节点启动失败并给出参数问题，而不是静默生成错误点云。

## 5. ROS Publisher 与 Launch

`FixturePublisher` 继续作为薄节点：读取 ROS 参数、转换 `double[]` 到 ROS-free `float` 配置、封装标准消息并发布。

参数调整为：

- `cloud_azimuth_sample_count`，默认 360；
- `cloud_range_m`，默认 5.0；
- `elevation_angles_rad`，默认 16 线数组。

`fixture_3d.launch.py` 与 `fixture_mixed.launch.py` 删除 `ring_pitch_rad`。默认 profile 由 publisher 单一来源提供，launch 不复制角度表。`fixture_2d.launch.py` 和 2D 消息契约保持不变。

`PointCloud2` 字段仍为 `x/y/z/intensity` 四个 `FLOAT32`，frame、topic、QoS 和 Adapter 接口不变。

### 5.1 统一可视化 Launch

新增 `fixture_visualization.launch.py`，同时启动三个独立 publisher：

| 类型 | Topic | Frame | 相对 `base_link` 的安装 |
| --- | --- | --- | --- |
| 水平 2D | `/fixture/scan/flat` | `fixture_scan_flat_link` | 平移 `(0, 0.35, 0.15)`，零旋转 |
| 倾斜 2D | `/fixture/scan/tilted` | `fixture_scan_tilted_link` | 平移 `(0, -0.35, 0.15)`，绕 Y 轴默认 `-30°` |
| 标准 3D | `/fixture/points` | `fixture_lidar_link` | 平移 `(0, 0, 0.25)`，零旋转 |

负 pitch 按 ROS 右手系使传感器 +X 方向朝 `base_link` 的 +Z 抬起。默认平移只用于
在同一机器人坐标系中区分安装位置，不改变各传感器消息中的原始点几何。
`tilted_scan_pitch_rad` 作为 launch 参数暴露，便于目检其他安装角。

RViz 只维护 `perception_fixtures.rviz` 一份配置：Fixed Frame 为 `base_link`，
`Flat2DScan`、`Tilted2DScan`、`MultiLineCloud` 和 `SensorTF` 可独立勾选。
不增加专门的 mixed RViz 配置，勾选任意多个传感器 display 即构成混装视图。
`show_rviz:=false` 用于无 GUI launch smoke。

## 6. 测试设计

### 6.1 gtest

- 保留 2D ranges 确定性测试。
- 默认 profile：逐项断言 16 个角为 `-15°、-13°、...、+15°`，断言 16 条乘 360 点，并验证重复生成完全一致。
- 零仰角：使用单通道小配置断言所有点 `z≈0`。
- 正负仰角：用 `{-0.2, 0, +0.2}` 验证每个通道的实际 `atan2(z, hypot(x,y))`。
- 方位覆盖：用 4 个样本验证 +X、+Y、-X、-Y 顺序和闭环不重复端点。
- 径向与 intensity：逐点验证 `hypot(x,y,z)` 的确定性距离公式以及 `(seed+linear_index)%256`。
- 无效仰角：空数组、非有限值和越界值必须拒绝。

### 6.2 launch integration

- mixed 测试显式配置 `elevation_angles_rad={-0.2,0,+0.2}` 和 4 个方位样本，预期 12 点；默认 5760 点及默认角表由 gtest 锁定，public 3D launch smoke 验证默认参数可启动。
- 对 2D observation 断言 `angle_min=-pi/2`、`angle_max=+pi/2`、`angle_increment=pi/180`、181 点，并计算中间索引对应零角。
- 对 3D observation 按固定通道/方位顺序断言 12 个实际 XYZ 和 intensity，覆盖 +X、+Y、-X、-Y 以及正/零/负 Z。
- 继续验证 2D 两个 sensor ID、3D sensor ID、类型、frame 和 payload 不被混淆。
- 运行 `perception_fixtures` 全部 launch tests，确认更大的默认点云不破坏健康、session、TF 与性能用例。
- 对可视化 launch 使用 `show_rviz:=false` 做 smoke，确认三个 publisher 和三条静态 TF 持续运行且无 ERROR/FATAL。
- RViz 外观不纳入自动单元测试；人工确认水平扫描位于 XY 网格附近、倾斜扫描通过 TF 形成斜面、3D 点云呈多仰角通道，并验证 display 可独立开关。

### 6.3 静态检查

- `perception_fixtures` 生产代码、launch 和测试中不得残留 `ring_pitch_rad`、`cloud_point_count`、`ring_radius_m` 或 `PitchedCloud`。
- 实施前记录 `drone_scanner` 与 `swarm_controller` 的文件清单、`git status --short` 和逐文件 SHA-256；结束时逐项比较，而不是依赖无法归因的 HEAD diff。

## 7. 文档与既有规划协调

- `docs/perception-input-testing.md` 更新 fixture 层描述与测试数量/点数口径。
- `docs/perception-class-relations.md` 更新 `FixtureSceneConfig` 和多线 3D 几何说明。
- C1 旧规划中“使用 Phase 3 pitch 公式验证倾斜 profile 等价”的 fixture 条目由本任务纠正；通用输入 fixture 不再承担旧传感器兼容回放。
- `.trellis/tasks/07-22-perception-observation-model` 中针对旧 FakeLidar 到新 observation 边界的受控迁移验证仍然有效，但应使用旧路径专属 replay/adapter，不依赖通用 `FixtureScene`。

## 8. 风险与回滚

| 风险 | 缓解 |
| --- | --- |
| 默认点数从 360 增至 5760，暴露性能或超时问题 | 运行全部现有 launch tests；必要时只调整测试超时，不降低默认模型语义 |
| 参数重命名遗漏 | 对 `perception_fixtures` 做定向 `rg`，并运行 public launch smoke |
| 可视化 launch 在无显示环境阻塞验证 | 提供 `show_rviz:=false`，接线 smoke 与 GUI 目检分开执行 |
| 误改旧 Phase 2/3 几何 | 实施前后比较受保护目录的文件清单、工作区状态和 SHA-256；禁止修改 `drone_scanner`、`swarm_controller` |
| 文档继续宣称 pitch 等价 | 同步两个用户文档和 C1 fixture 规划记录 |

回滚单位是 `perception_fixtures` 的配置、生成器、launch、测试与对应文档；可视化组合层可独立回滚，不通过修改旧 `FakeLidar` 来兼容新 fixture。

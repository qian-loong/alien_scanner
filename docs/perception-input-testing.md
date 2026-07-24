# C1 感知输入测试与性能基线

本文记录 `perception_core`、`perception_adapters`、`perception_input_node` 和
`perception_fixtures` 的自动测试边界、运行方法及 2026-07-25 最终收口结果。

## 测试分层

| 层次 | 内容 | 逻辑用例数 |
| --- | --- | ---: |
| Core gtest | 数据契约、健康状态、freshness/frame/quality 门控、性能 | 22 |
| Adapter gtest | LaserScan、PointCloud2、Odometry、TF 转换与性能 | 8 |
| Session gtest | descriptor inventory 冻结与重连 | 3 |
| Fixture gtest | 确定性 2D/标准多线 3D 场景、仰角校验和方向 | 6 |
| launch integration | 2D、mixed、health、descriptor、TF、可视化安装方向、Session restart、120 帧性能 | 9 |

合计 48 个逻辑测试方法。`colcon test-result` 还会统计 12 个 CTest 汇总项，
因此最终报告为 `60 tests`。

## 位姿输入参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `pose_input_type` | `odometry` | `odometry` 或 `tf` |
| `odom_topic` | `odom` | Odometry 输入 topic |
| `tf_topic` | `/tf` | TFMessage 输入 topic |
| `tf_child_frame` | `base_link` | TF 模式选择的 child frame |
| `expected_pose_frame` | 空 | 非空时要求 PoseEstimate frame 完全匹配 |
| `minimum_pose_quality` | `0.0` | `[0, 1]`，低于阈值或非有限值时门控 |
| `pose_timeout_s` | `1.0` | pose freshness 阈值 |

TF 和 Odometry 最终都转换为 ROS-free `PoseEstimate`，再统一经过 freshness、
frame 和 quality 健康门。

## Fixture 几何契约

`perception_fixtures` 是感知链路的确定性输入源，不是旧
`drone_scanner::FakeLidar` 的兼容回放，也不承担真实环境 raycast。

- 2D fixture 发布标准 `LaserScan`：181 个点，`angle_min=-π/2`、
  `angle_max=+π/2`、`angle_increment=π/180`，原始数据位于消息 frame 的
  XY 平面。雷达水平、倾斜或垂直安装姿态由 TF 表达，不改写这组扫描角。
- 3D fixture 发布标准多线 `PointCloud2`：默认 16 条仰角通道
  `-15°,-13°,...,+15°`，每条 360 个方位样本，共 5760 点。零仰角在传感器
  XY 平面，零方位沿 +X，正方位朝 +Y，正仰角朝 +Z。单帧几何是多条完整
  方位环，不是螺旋；点按仰角通道、再按方位角排列。
- `FixtureSceneConfig` 可传入小型仰角数组；mixed 集成测试使用
  `[-0.2,0,+0.2]` 和 4 个方位样本，验证 ROS round-trip 后的 XYZ、顺序和
  intensity。

默认点云只描述传感器自身坐标系的束线，安装外参不编码进点云或
`LaserScan`。`fixture_visualization.launch.py` 通过 `base_link` 下的静态 TF
组合水平 2D、倾斜 2D 和标准多线 3D 三种安装方式。当前 3D 雷达使用零旋转
安装作为标准基线，但它也可以通过 TF 整体倾斜，不需要修改点云生成公式。

## RViz 可视化验收

容器中构建并 source 工作区后运行：

```bash
DISPLAY=host.docker.internal:0.0 \
  ros2 launch perception_fixtures fixture_visualization.launch.py
```

唯一的 RViz 配置默认同时显示：

| Display | 数据 | 颜色/形式 |
| --- | --- | --- |
| `Flat2DScan` | `/fixture/scan/flat` | 绿色水平 2D 扫描 |
| `Tilted2DScan` | `/fixture/scan/tilted` | 橙色向 +Z 抬起的 2D 扫描 |
| `MultiLineCloud` | `/fixture/points` | 按 intensity 着色的 16 线 3D 点云 |
| `SensorTF` | `base_link` 到三个传感器 frame | 坐标轴、箭头和 frame 名称 |

在 Displays 面板中独立勾选以上项目即可查看单雷达或任意混装组合，不另设
mixed RViz。倾斜 2D 默认绕 Y 轴 `-30°`，使传感器本地 +X 朝
`base_link` 的 +Z 方向抬起；可用
`tilted_scan_pitch_rad:=<弧度>` 改变安装角，而两组 `LaserScan` 的消息内几何
始终保持自身 XY 平面。

无 GUI 环境可只验证节点与 TF 接线：

```bash
timeout 12s ros2 launch perception_fixtures fixture_visualization.launch.py \
  show_rviz:=false
```

## 干净质量门

本次验证使用新的临时目录，不复用 `ws/build` 和 `ws/install`：

```bash
cd /workspaces/alien-scanner/ws

colcon --log-base /tmp/alien-fixture-tilt-up-20260725-a/log build \
  --build-base /tmp/alien-fixture-tilt-up-20260725-a/build \
  --install-base /tmp/alien-fixture-tilt-up-20260725-a/install \
  --symlink-install \
  --packages-up-to perception_fixtures \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source /tmp/alien-fixture-tilt-up-20260725-a/install/setup.bash

ROS_DOMAIN_ID=87 colcon --log-base /tmp/alien-fixture-tilt-up-20260725-a/test-log test \
  --build-base /tmp/alien-fixture-tilt-up-20260725-a/build \
  --install-base /tmp/alien-fixture-tilt-up-20260725-a/install \
  --packages-select perception_core perception_interfaces \
                    perception_adapters perception_input_node \
                    perception_fixtures

colcon test-result \
  --test-result-base /tmp/alien-fixture-tilt-up-20260725-a/build \
  --all --verbose
```

结果：

```text
5 packages built
60 tests
0 errors
0 failures
0 skipped
```

## 性能基线

以下数字是 2026-07-24 在 ROS 2 Jazzy 开发容器和干净 colcon
build/install 中记录的固定比较基线，不是每次完整测试运行的即时输出。

| 阶段 | 样本 | 结果 | 门限 |
| --- | ---: | ---: | ---: |
| LaserScan Adapter 转换 | 1000 | 平均 0.107 µs | 平均 < 5000 µs |
| MapperHealthGate evaluate | 5000 | 平均 0.056 µs | 平均 < 1000 µs |
| ROS publish-to-observation | 120 帧 | 0 丢帧，P95 0.477 ms | P95 < 500 ms |

ROS 链路详细结果：

```text
samples=120, lost=0, average_ms=1.092, p95_ms=0.477, max_ms=106.014
```

最大值来自首帧 DDS 发现等待；稳态 P95 为 0.477 ms，因此同时保留启动
离群值和稳态分位数，不只报告平均值。

## Session 生命周期测试

测试通过 launch 向 `perception_input_node` 发送 `SIGKILL`，由
`respawn=True` 拉起新进程，并断言：

1. 新进程 PID 不同；
2. 新进程 SessionID 不同；
3. 新进程发布的观测不复用旧 SessionID。

C1 负责生成和传播 SessionID。旧 session 数据是否可以进入 local map，由 C2
消费者负责拒绝。

## 人工验收状态

- [x] RViz 中水平 2D、向 +Z 抬起的倾斜 2D、标准多线 3D、TF 及任意混装组合已通过最终人工目检。

自动化质量门和本次雷达可视化人工验收均已通过。当前 RViz 配置不显示
`HealthState`；该能力属于后续可视化增强，不是本次雷达几何验收的未完成项。

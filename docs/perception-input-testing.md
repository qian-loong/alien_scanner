# C1 感知输入测试与性能基线

本文记录 `perception_core`、`perception_adapters`、`perception_input_node` 和
`perception_fixtures` 的自动测试边界、运行方法及 2026-07-26 AC-09/AC-10
收口结果。

## 测试分层

| 层次 | 内容 | 逻辑用例数 |
| --- | --- | ---: |
| Core gtest | 数据契约、射线分类、闭合集、健康状态、能力门控、性能 | 26 |
| Adapter gtest | LaserScan、PointCloud2、不可绕过验证、Odometry、TF 转换与性能 | 13 |
| Session gtest | descriptor inventory 冻结、能力变化与重连 | 4 |
| Fixture gtest | 确定性 2D/标准多线 3D、调试注入与证据几何 | 14 |
| launch integration | 输入/health/session、非法配置、fixture/RViz、证据 Marker、非法批次和 legacy graph | 17 |

合计 74 个逻辑测试方法。`colcon test-result` 还会统计 14 个 CTest 汇总项，
因此最终报告为 `88 tests`。

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

## 射线证据能力

射线证据等级单调有序：

```text
HitOnly < HitRay < FullRay
```

| 等级 | 可用证据 | 禁止推断 |
| --- | --- | --- |
| `hit_only` | occupied hit endpoint | free-space ray、no-return ray |
| `hit_ray` | hit endpoint 及 origin 到 hit 的 free space | no-return 到最大量程 |
| `full_ray` | hit ray 及明确 no-return 的完整 free space | 无 descriptor 授权的额外证据 |

`Scan2D::return_kind(index)` 直接查询原生 `ranges`，不会建立第二份 ray payload：
量程内有限值（含最小/最大边界）为 `Hit`，正无穷为 `NoReturn`，NaN、负无穷
和量程外有限值为 `Invalid`。payload 分类与使用权限分离，例如 `HitRay` 扫描
中的正无穷仍不能被 mapper 当作最大量程 free-space 证据。

当前 `PointCloud2Adapter` 只保留 XYZ/intensity 命中点，所以只接受
`hit_only`。直接 `convert()` 也执行完整 type、frame、FLOAT32 字段、point/row
step 和 data 长度验证，畸形存储在读取点字节前抛出 `std::invalid_argument`；
`hit_ray/full_ray` 和未知 `3/255` 同样 fail closed，不能由点序或量程配置补造
射线。`LaserScanAdapter` 对所有等级检查基础 range/angle 元数据；
`hit_ray/full_ray` 还要求消息的 range、水平 FOV 和角分辨率与冻结 descriptor
在 `1e-5` 绝对容差内一致，漂移时拒绝整批。

`is_valid_ray_evidence()` 是 C++ 闭合集的集中检查；`provides_at_least()` 会先
验证 actual 和 required，因此底层值 `3/255` 不会因整数较大而获得权限。
`LaserScanAdapter::convert()` 内部调用同一完整 `validate()`，直接库调用也无法
绕过 sensor type、frame、元数据、ranges 或可选 intensities 长度检查。输入节点
只调用一次 `convert()`，在 subscription callback 内捕获并限频记录输入异常，
不会重复验证同一批消息或让异常逃出 callback。

冻结 descriptor 是 C1 的发布前边界。成功发布的 `LidarObservation` 是权威
批次，已经携带消费所需的能力、frame、stamp、session 和 payload 元数据；
下游保留 sensor/session 溯源，但不需要第二个 descriptor topic 重新推断能力。

运行时参数如下，值域均为 `hit_only`、`hit_ray`、`full_ray`；未配置时默认
`hit_only`，其他值使节点以明确诊断启动失败。

| 参数 | 作用 |
| --- | --- |
| `minimum_lidar_ray_evidence` | Healthy 最低射线证据等级 |
| `degraded_lidar_ray_evidence` | Degraded 组合最低射线证据等级 |
| `sensor.<id>.ray_evidence` | 冻结到 descriptor 的传感器能力 |

仓库 fixture 参数文件显式声明 2D=`full_ray`、3D=`hit_only`。ROS
`LidarObservation.ray_evidence` 保留每批等级，`HealthState` 通过
`has_free_space_hit_rays` 和 `has_full_no_return_rays` 分别暴露当前有效能力。
集成测试分别锁定 `HitOnly` 不满足 `HitRay`、`HitRay` 不满足 `FullRay`，并
验证健康消息可区分 `(true, false)` 的 hit-ray 与 `(true, true)` 的 full-ray。

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
- `FixtureSceneConfig` 是 2D scan 点数、角边界和量程边界的算法与 Publisher
  单一来源，角增量派生为 `(angle_max-angle_min)/(point_count-1)`。点数小于 2、
  非有限角度、`angle_min>=angle_max`，以及不满足
  `0<=range_min<range_max` 的量程都会拒绝。默认仍为 181 点、
  `[-pi/2,+pi/2]`、`[0.1,30]`，并由代表 beam 数值 golden 锁定。
- 配置也可传入小型仰角数组；mixed 集成测试使用 `[-0.2,0,+0.2]` 和 4 个
  方位样本，验证 ROS round-trip 后的 XYZ、顺序和 intensity。

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

## 可选射线证据调试视图

独立入口只消费正式 `LidarObservation`，不会启动旧 FakeLidar、OctoMapBuilder，
也不会订阅或发布 `/drone_0/scan_returns`：

```bash
ros2 launch perception_fixtures ray_evidence_debug.launch.py
```

无 GUI 的自动化/接线验证：

```bash
timeout 12s ros2 launch perception_fixtures ray_evidence_debug.launch.py \
  show_rviz:=false beam_stride:=1
```

| Marker namespace | 颜色 | 含义 |
| --- | --- | --- |
| `ray_evidence/hit_endpoints` | 红 | 2D hit 或有限 3D hit endpoint |
| `ray_evidence/hit_free` | 绿 | 至少 `HitRay` 的 origin-to-hit 证据 |
| `ray_evidence/no_return_free` | 青 | 仅 `FullRay` 的 no-return 到 `range_max` 证据 |
| `ray_evidence/invalid` | 灰 | 固定短方向段，不表示真实空间长度 |

RViz 的 MarkerArray display 可按 namespace 独立开关。Marker 使用 observation
原始 frame/stamp、稳定的 sensor marker ID 和有限 lifetime。每批都会为四个
namespace 发布 `ADD`，空 points 会覆盖上一批已不再允许显示的证据；输入中断或
非法批次后由 0.5 秒 lifetime 清理，因此不会持续累积。

默认入口是 2D-only 的解析隧道截面：360 条 beam 覆盖 `[-pi,pi)` 且首尾方向
不重复，local X/Y 半轴分别为 3 m/4 m，普通 hit 满足：

```text
r(theta) = 1 / sqrt(cos(theta)^2 / 3^2 + sin(theta)^2 / 4^2)
```

场景先生成全部椭圆 hit，再将 `[255,285]` 覆盖为连续 `+inf/NoReturn` 岔口，
最后把 `44/136/180/316` 覆盖为 NaN、`-inf`、由配置量程派生的 below-min
finite 和 above-max finite。固定量程为 `[0.1,10]`；高于最大量程的有限值仍是
Invalid。launch 中同一个 `DEBUG_SCAN` 常量组同时派生 fixture 参数和 input
descriptor，避免 count/FOV/increment/range 跨进程漂移。

`map -> debug_scan_link` 是零平移 `Ry(+pi/2)`：local `+Z -> map +X`、local
`+X -> map -Z`、local `+Y -> map +Y`，因此 scan 平面落在 map YZ，岔口朝
map `+Y`。RViz Fixed Frame 为 `map`；`YZCrossSectionGrid` 明确使用 YZ plane，
不是默认 XY grid；`MapRightHandAxes` 用标准 X 红/Y 绿/Z 蓝显示 map 右手轴，
相机沿 map X 略微偏转，使 X 轴仍可辨而截面保持近似正视。默认
`beam_stride=2`，自动化入口显式使用 1。`inject_debug_returns` 默认关闭，现有
181-beam fixture 不变。

debug launch 不发布 Cloud3D，也没有 cloud descriptor、cloud TF 或 RViz cloud
display。Cloud3D hit-only 契约由 gtest 和 launch test 直接向正式
`LidarObservation` topic 注入验证，只产生红色 endpoint。高等级、未知 ROS
枚举、非法 data type、交叉 payload 或非法元数据会整批拒绝，不发布 Marker。

## 干净质量门

本次验证使用新的临时目录，不复用 `ws/build` 和 `ws/install`：

```bash
cd /workspaces/alien-scanner/ws

colcon --log-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/log build \
  --build-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/build \
  --install-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/install \
  --symlink-install \
  --packages-up-to perception_fixtures \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source /tmp/alien-c1-ac10-tunnel-final-20260726-01/install/setup.bash

ROS_DOMAIN_ID=97 colcon --log-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/test-log test \
  --build-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/build \
  --install-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/install \
  --packages-select perception_core perception_interfaces \
                    perception_adapters perception_input_node \
                    perception_fixtures

colcon test-result \
  --test-result-base /tmp/alien-c1-ac10-tunnel-final-20260726-01/build \
  --all --verbose
```

结果：

```text
5 packages built
74 logical test methods + 14 CTest aggregate entries = 88 tests
0 errors
0 failures
0 skipped
```

### 单元测试行覆盖率

阶段 1/2 的覆盖率阈值继续作为正式门禁，并于 2026-07-26 在全新目录
`/tmp/alien-c1-closeout-coverage-20260726-02` 使用 GCC 13.3 `--coverage` 与
`gcov --json-format` 实测。只运行 `perception_core`、`perception_adapters` 的
gtest；口径纳入各包生产 `include/**`、`src/**` 的 gcov 可执行行，排除
`test/**`、生成代码、ROS 节点和外部依赖。JSON 记录按规范化源码路径与行号
合并；同一模板/inline 行存在多个翻译单元实例时，任一实例执行即计为覆盖。

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
生成源码及 `*Node.cpp`，并对严格 `>` 门限返回非零失败码。结果：

```text
perception_core      209 / 210 lines = 99.52% (required > 90%) PASS
perception_adapters  249 / 277 lines = 89.89% (required > 85%) PASS
39 logical gtest methods + 2 CTest aggregate entries = 41 tests
0 errors, 0 failures, 0 skipped
```

## 性能基线

外部工具的 CPU、ROS tracing、Heaptrack、ASan/LSan、Memcheck、Massif 及三次
RSS/PSS 基线见 [`perception-resource-profiling.md`](perception-resource-profiling.md)。
该报告没有修改感知业务行为，并明确记录 LinuxKit PMU 限制和第三方动态加载器
的 64 B Memcheck finding。

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
- [x] 独立证据调试 RViz 的红色 hit、绿色 hit free、青色 no-return free、灰色 invalid、TF、namespace 独立开关和无累积行为已由用户于 2026-07-26 确认通过。验收截图：`C:\Users\loong\AppData\Local\Temp\alien-c1-ac10-tunnel-rviz-20260726-11.png`。

截图确认默认画面为 map YZ 截面：红色 X 轴是截面法向，绿色 Y 轴和蓝色 Z 轴
位于截面内；洞壁 hit 环、岔口 no-return 扇区和四类 invalid 均清晰可辨。
自动化质量门与两套 RViz 人工验收均已通过。主 fixture RViz 配置不显示
`HealthState`；该能力属于后续可视化增强，不是本次雷达几何验收的未完成项。

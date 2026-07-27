# C2 本机观测地图

`perception_local_map` 在单个 vehicle session 内消费 C1 的
`LidarObservation`、`PoseEstimate` 和 `HealthState`，并维护唯一一条 source-local
occupancy revision 链。领域层是 ROS-free C++；默认后端是 OctoMap，轻量
deterministic voxel backend 只用于同一套 conformance 测试。

## 契约边界

- `HitOnly` 只写 occupied endpoint。
- `HitRay` 额外写 hit endpoint 之前的 free cells。
- `FullRay` 才消费明确 no-return；最大量程 boundary cell 保持 unknown。
- map revision 只随 backend `Applied` 推进。重放、拒绝、空 evidence 和全 Invalid
  batch 不推进；更高 origin stamp 的相同 evidence 仍是新 commit。
- canonical lattice 使用
  `floor((coordinate - lattice_origin) / resolution)`；cell 和 region 均为
  `[min,max)`，负坐标不向零截断。
- mapper 自行运行冻结的 C1 `MapperHealthGate`。上游 health 必须携带同一 producer
  session 和 canonical contract fingerprint，且只作为不可抬高的 ceiling。
- pose source/session、reset epoch 或确认的空间跳变会关闭旧 revision 链、撤销旧
  alignment、推进 map epoch，并从空图恢复。旧 lineage replay 不重复清图。
- fleet、alignment Ready 和 shared/global map 不参与本机地图启动或 health。

## ROS 接口

输入：

- `perception/observations`
- `perception/pose`
- `perception/health`

输出：

- `local_map/state`：`Reliable + Volatile + KeepLast(1)`；heartbeat 只推进
  `state_sequence`，不推进 map revision，也不刷新 commit provenance。
- `local_map/octomap`：`Reliable + Transient Local` 的 RViz/兼容快照；它不携带
  revision，不能和 state topic 猜配成 C3 authoritative update。
- `/diagnostics`：wire decode、TF、contract、pose 和 backend 拒绝原因。

`LocalMapState` 只含 identity、frame、canonical geometry、epoch/revision、remaining
validity budget、known bounds 和 capability 摘要。它不含 cells、content hash、dirty
region、delta 或 update kind；这些属于后续 C3。

## 构建与自动验证

在 Windows 仓库根目录运行：

```powershell
docker exec alien-scanner-dev bash -lc "
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  cd /workspaces/alien-scanner/ws
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to perception_local_map perception_fixtures
  source install/setup.bash
  colcon test --event-handlers console_direct+ \
    --packages-select perception_core perception_interfaces perception_adapters \
      perception_input_node perception_local_map perception_fixtures
  colcon test-result --verbose
"
```

无 GUI smoke：

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map local_map_debug.launch.xml show_rviz:=false
"
```

debug launch 复用 `perception_fixtures` 的固定 360-beam 岔口/no-return/Invalid
注入、C1 input node 和 ray-evidence markers。它不启动 `FakeLidar`、旧
`OctoMapBuilder` 或 `scan_returns`。

## 连续 FullRay 洞穴场景

`cave_full_ray_scene.launch.py` 是独立的 20 秒连续验证入口。它从
`cave_full_ray_scene.yaml` 一次读取并 fan-out tree cave、10 m 直线轨迹、标准
`LaserScan`、pose lead gate、C1/C2 descriptor/contract 和 map geometry。真值
`/cave/points` 只供 RViz；C1/C2 仅消费 odometry 与 released scan。

无 GUI 运行：

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map cave_full_ray_scene.launch.py show_rviz:=false
"
```

正式 launch test 会完整运行轨迹并断言至少 100 个唯一 observation/revision、同一
map epoch 内单调推进、known bounds 沿 X 至少扩展 9 m、C1/C2 fingerprint 一致、
无 pose missing/stale 拒绝，以及终点 drain 后 revision/provenance 停止刷新而
heartbeat/freshness 继续按契约变化。ROS-free scene gtest 通过 mapper
`MapReadTransaction` 直接验证 occupied/free/unknown，不解析 visualization-only
OctoMap 消息。

场景 RViz：

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map cave_full_ray_scene.launch.py show_rviz:=true
"
```

场景目检应同时确认 `CaveTruthOracleOnly`、`FrozenLineTrajectory`、
`CurrentReleasedFullRayLaserScan`、`SceneTF` 和
`C2AccumulatedAuthoritativeOctoMap`。scan 平面经唯一静态 `body <- scan` TF 映射为
body YZ 垂直环；descriptor mounting quaternion 只作为 fingerprint/外参期望元数据，
不再次旋转射线。该人工目检需单独验收，真实 RViz 进程 smoke 不能替代。

2026-07-27 用户确认连续 FullRay 洞穴场景 RViz 显示正常，洞穴真值、直线路径、
扫描/TF 与累计 C2 OctoMap 的人工外观验收通过。

## RViz 目检

ROS 2 Jazzy 当前镜像中的 `liboctomap_rviz_plugins.so` 未声明
`liboctomap.so` 动态依赖。debug launch 仅对 `local_map_debug_rviz` 子进程设置
`LD_PRELOAD=liboctomap.so`，避免 `OcTreeStamped` 符号加载失败；mapper、fixture
及调用该 launch 的其他进程不会继承此 workaround。

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_local_map local_map_debug.launch.xml show_rviz:=true
"
```

在 RViz 中检查：

- 启动时 RViz 可能在约 10 秒后报告一次首帧 LaserScan 早于其本地 TF cache；这是
  RViz 晚加入动态 TF 后丢弃已排队首帧的显示层瞬态。后续 scan 持续显示，且该告警
  不代表 C1/C2 pose 配对或 authoritative revision 被拒绝。

1. Fixed Frame 为 `map`，右手坐标轴存在。
2. `C1RayEvidenceByAuthority` 同时区分 hit endpoint、hit free、no-return free 和
   Invalid；Invalid 不产生 C2 voxel。
3. `C2AuthoritativeOccupiedVoxels` 非空，环切面垂直 map `+X`。
4. no-return 最大量程 boundary 不出现 occupied voxel；范围外仍 unknown。
5. `CanonicalYZLatticeCrossSection` 仅表示 canonical lattice，不把 bounds 内缺失
   voxel 推断为 free。

GUI 目检是自动 gtest/launch test 之后的单独验收项。

2026-07-27 用户目检通过：C1 射线分类、C2 OctoMap、垂直于 map `+X` 的
YZ 环切面、右手坐标轴、岔口 no-return 最大量程边界及范围外 unknown 均符合预期。

## Legacy 边界

旧 `swarm_controller::OctoMapBuilder` 及其 launch 保留为隔离的查询行为 oracle。
C2 debug/public launch 不 include legacy builder，新旧链不会发布同一个 authoritative
topic。本任务不迁移 global merger、allocator、fleet topology，也不定义 C3 地图更新
协议。

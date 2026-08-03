# Research: legacy 500k limit

- Query: 调查重构前“50 万节点上限”的准确来源、所属模块、实际限制对象、当前 C2 是否继承，以及 bounded/expanding 300-600 s workload 是否会触发；给出对 profiling 方案的最小修订。
- Scope: internal
- Date: 2026-07-27

## Findings

### 1. 结论

仓库中与旧扫描地图直接对应的“50 万上限”不是 ROS node 数、voxel 数或 OctoMap node 数，而是 Phase 2 `drone_scanner/scan_accumulator` 保存并发布的 **hit-only XYZ 点数**：

- 总计划明确写“累积默认 50 万点上限，超出丢最早点”（`docs/xenomorph-scanner-plan.md:179-183`）。
- Phase 2 细化文档明确归属 `PointCloudAccumulator`，参数名 `max_points`，FIFO 丢弃最早写入点（`docs/phases/phase-02-drone-scanner.md:280-286`）；launch 参数名为 `max_cloud_map_points`，默认 `500000`（同文件 `:318-325`）。
- C++ library 本身默认 `max_points=0`，含义是 unlimited（`ws/src/drone_scanner/include/drone_scanner/PointCloudAccumulator.hpp:12-17,26-30`）；**500000 是 ROS node/launch 的默认配置**：`ScanAccumulatorNode` 把 `max_points` 参数交给 accumulator，并声明默认 500000（`ws/src/drone_scanner/src/ScanAccumulatorNode.cpp:9-17,32-39`），两个 legacy sensing launch 也声明 `max_cloud_map_points=500000`（`ws/src/drone_scanner/launch/fake_lidar_launch.py:105`；`drone_sensing_stack.launch.py:125,204-218`）。

实际 cap 行为是一个 `std::vector<Point3f>` 的数量上限：未达到 cap 时追加；单批超过 cap 时只保留该批最后 `max_points` 个；累计超过 cap 时保留旧 vector 的尾部再追加新批，即 FIFO 丢最老点（`PointCloudAccumulator.cpp:10-42`）。gtest 用 cap=3 明确证明第 4 个点到来后留下 2/3/4（`ws/src/drone_scanner/test/TestPointCloudAccumulator.cpp:40-50`）。

该 vector 的输入是 `/points` 中的 hit endpoint：legacy `FakeLidarNode` 遍历 ray returns，只把 `ret.hit` 放入 `hits` 并发布 point cloud（`ws/src/drone_scanner/src/FakeLidarNode.cpp:230-239`）；`ScanAccumulatorNode` 从 `PointCloud2` 提取 XYZ、变换到 map frame 并 append（`ScanAccumulatorNode.cpp:70-82,124-155`）。因此它：

- 不保存 no-return；
- 不沿 ray 生成 free voxels；
- 不做 voxel dedup；
- 不保存 occupancy probability；
- cap 同时限制 accumulator 内的 XYZ point vector 和随后发布的 `cloud_map PointCloud2` 点数。

“50 万节点”是对“50 万累计点”的口头误称。

### 2. 不是 legacy OctoMapBuilder 的上限

旧 `swarm_controller::OctoMapBuilder` 的 public state 只有 `octomap::OcTree tree_`，没有 max-points/max-voxels 字段（`ws/src/swarm_controller/include/swarm_controller/OctoMapBuilder.hpp:19-34`）。它对每批 ray 构造 free/occupied key set 并更新 OcTree（`ws/src/swarm_controller/src/OctoMapBuilder.cpp:31-70`），`knownCount()` 只是遍历 leaf nodes 计数（同文件 `:82-104`）。对应 node/launch 的数量相关参数搜索也只有 `max_range=30`，没有 500000 cap（`ws/src/swarm_controller/src/OctoMapBuilderNode.cpp:32-35,52-59,186-197`）。

仓库另有数值恰为 500000 的 `frontier.max_trace_geometry_elements`（`ws/src/swarm_controller/include/swarm_controller/GlobalFrontierDetector.hpp:49`；`ws/src/swarm_controller/launch/frontier_geometry_demo.launch.py:143`），它限制 frontier trace geometry 展开量，不是 local map 容量，也不在 C2 profiling 路径。当前 global merger 的 map 数量门则是每 source 5,000,000 voxels、global 10,000,000 voxels（`ws/src/swarm_controller/launch/multi_drone_exploration.launch.py:407-409`；`GlobalMapMergerNode.cpp:231-232`），同样不是本问题中的旧 500k。

### 3. 当前 C2 没有继承 500k cap

当前 C2 production target 只链接 `perception_local_map_core`、`perception_local_map_octomap`、OctoMap 与 ROS dependencies，不链接 `drone_accumulator` 或 `PointCloudAccumulator`（`ws/src/alien_perception/perception_local_map/CMakeLists.txt:35-90`）。canonical C2 requirements 明确禁止 scene 启动 `/scan_returns`、`scan_accumulator`、`cloud_map` 和 legacy `OctoMapBuilder`（`.trellis/tasks/archive/2026-07/07-27-c2-local-observation-map/prd.md:171-184`；design `:675-684`）；现有 launch test 也断言 graph 中没有 `/scan_returns`、任何 `cloud_map` topic、`scan_accumulator` 或 `octomap_builder` node（`ws/src/alien_perception/perception_local_map/test/test_cave_full_ray_scene_integration.py:315-332`）。

C2 `OctoMapBackend` 持有 `octomap::OcTree` 和 `std::set<VoxelIndex> known_cells`（`ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp:13-22`）。apply 对 known cell 只有 `insert`，无数量检查/FIFO erase（同文件 `:160-201`）；reset 才整体重建 backend（同文件 `:152-158`）。全 C2 source 中没有 `max_points`、`max_voxels`、`max_nodes` 或 500000 参数。唯一相近的固定容量是 pose history 默认 128（`LocalObservationMapper.hpp:30-38`；`PerceptionLocalMapNode.cpp:450-455`），与地图容量无关。

所以当前 C2 不会在 exact known voxel 达到 500000 时裁剪、停止 commit 或保持常量。它的真实边界是：

- OctoMap coordinate representability，越界会 whole-batch reject（`OctoMapBackend.cpp:32-38,172-180`）；
- 进程/cgroup 可用内存；
- 随 map 增长而上升的 apply、known-bounds 和 snapshot 成本。

没有内建 count cap 也意味着 expanding run 可以因正常 map 扩张耗尽资源；这正是 C2 baseline 要测的对象，不能把系统 OOM 与 legacy FIFO cap 混为一谈。

### 4. 300-600 s workload 是否触发

#### 若误启动 legacy scan_accumulator

canonical 参数是 10 Hz、360 beams（`ws/src/alien_perception/perception_local_map/config/cave_full_ray_scene.yaml:51-55`）。每 scan 的 legacy `/points` 数等于 hit beam 数 `H`，所以达到旧 cap 的时间是：

```text
t_cap = 500000 / (10 * average_H) seconds
```

在当前 360-beam 上，所有 beam 都 hit 时最早约 `138.9 s`；300 s 内达到 cap 只要求平均 `H > 166.7`，600 s 内只要求 `H > 83.3`。因此若 profiling graph 错误包含 legacy accumulator，300/600 s 很可能达到 cap，随后 RSS/point count 人为平台化并 FIFO 丢历史点，可能被误判成“bounded memory 收敛”。Phase 2 历史文档也描述停扫是为了避免 cap 裁掉起点，并给出旧配置下“约 80 s”的经验数（`docs/phases/phase-02-drone-scanner.md:345-353`）；该 80 s 不是当前 360-beam 的严格下界，正式方案应使用实际 sink hit count 和上式。

#### 对当前 C2

不会“触发旧 cap”，因为该条件不存在。bounded workload 的 exact known voxel 应因空间重复而自然 plateau；legacy point accumulator 则即使重复观察相同点也继续累计直到 500k，两者语义相反。expanding workload 可能在 300-600 s 内跨过 **数值上的** 500000 exact known voxels，但跨线后应继续增长，而不是裁剪或停止；具体 crossing revision 必须由 deterministic oracle 测得，不能从 360 beams 直接推断，因为 C2 会做 ray traversal、batch dedup 和跨 revision voxel reuse。

### 5. 对性能方案的最小修订

建议在昂贵 formal matrix 前增加一次 production `500k capacity ramp`，复用已规划的 expanding fixture/oracle，不改 beam/range/resolution/rate：

1. 同一 `10 Hz / 360 beams / 0.2 m / FullRay / fixed seed` expanding sequence 先由 ROS-free oracle 计算 `observation_stamp -> revision -> exact known/free/occupied`，确定 500k crossing 是否落在 600 s 内。
2. production preflight 使用安装后的 `perception_local_map_node`，graph gate 明确拒绝 `scan_accumulator`、`cloud_map`、`/scan_returns` 和 `octomap_builder`；目标进程 PID/RSS 仍只归属 C2。
3. 若 oracle 预计会 crossing，则跑到第一个 `exact_known > 500000` 后至少再保留两个 100-revision checkpoint；要求 epoch/fingerprint 不变、revision/last stamp 连续、known 与 X bounds 继续增长、无 reject/backend diagnostic、RSS/PSS 样本可解析。建议以 `>550000` 作为方便的完成目标，但判定依据是“跨线后连续增长”，不是精确命中 550k。
4. 若同条件 600 s oracle 仍不到 500k，记录 `legacy_500k_crossing=not_reached`；不要通过提高 beam、降低 resolution、重复 stamp 或扩大 range 人工制造 crossing。可选择只延长这个 preflight 的时长，不能把改变业务负载后的结果混入 formal baseline。
5. capacity ramp 是可行性/非继承 gate，不替代三次 plain、perf、heap 或 sanitizer；其 raw 表至少保存 revision、last stamp、known/free/occupied、bounds、RSS/PSS 和 crossing marker。

对既有 `c2-measurement-boundaries.md` 不需要改变 bounded/expanding 定义。只需新增三条验收：

- bounded plateau 必须由 authoritative exact known voxel 证明，不能用 RSS、OctoMap bytes 或任何 raw point count 平台代替；
- expanding 报告显式标记 first `known > 500000` checkpoint；crossing 后 plateau/bounds 回退/revision gap 为 finding，而不是正常 legacy 行为；
- 每次 formal run 的 ROS graph/provenance 保存“legacy accumulator absent”证据，防止旧 500k FIFO 产生假收敛。

## Files Found

- `docs/xenomorph-scanner-plan.md` - 旧 Phase 2 总览明确称“50 万点上限”。
- `docs/phases/phase-02-drone-scanner.md` - cap 所属组件、FIFO 语义、launch 参数和停扫动机。
- `ws/src/drone_scanner/include/drone_scanner/PointCloudAccumulator.hpp` - library 默认 0=unlimited，内部保存 `vector<Point3f>`。
- `ws/src/drone_scanner/src/PointCloudAccumulator.cpp` - cap/FIFO 的实际实现。
- `ws/src/drone_scanner/src/ScanAccumulatorNode.cpp` - node 默认 `max_points=500000` 和 `PointCloud2` 输入/输出。
- `ws/src/drone_scanner/src/FakeLidarNode.cpp` - `/points` 只发布 hit endpoints。
- `ws/src/swarm_controller/src/OctoMapBuilder.cpp` - 旧 OctoMap builder 无 500k cap 的反证。
- `ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp` - 当前 C2 persistent storage 及无 count cap 的实现。
- `ws/src/alien_perception/perception_local_map/test/test_cave_full_ray_scene_integration.py` - 当前 C2 graph 排除 legacy 路径的自动断言。

## Related Specs

- `.trellis/spec/backend/local-observation-map-contract.md` - C2 authoritative map 与 visualization-only snapshot/read transaction 边界。
- `.trellis/spec/backend/perception-ray-evidence-contract.md` - C2 FullRay free/occupied evidence 语义；不能与 legacy hit-only cloud point accumulation 等同。
- `.trellis/tasks/07-27-c2-performance-memory-baseline/research/c2-measurement-boundaries.md` - bounded/expanding、exact voxel oracle 和 production RSS 对齐方案。

## External References

- 无。本结论由仓库当前源码、历史 Phase 文档和 C2 归档设计交叉确认。

## Caveats / Not Found

- `python ./.trellis/scripts/task.py current --source` 本次仍返回 `Current task: (none)`；写入路径来自 dispatch 明确指定的 `.trellis/tasks/07-27-c2-performance-memory-baseline`。
- 没有发现任何证据表明 legacy `OctoMapBuilder` 或当前 C2 曾使用 500000 作为 OctoMap leaf/node/voxel cap；发现的直接证据都指向 `scan_accumulator` 的 point vector。
- Phase 2 文档的“约 80 s”属于历史经验说明，与当前 10 Hz/360-beam 的理论最早 138.9 s 不一致；正式判断必须使用实际每 scan hit count，不复用该估算。
- 当前 C2 exact known count 不在 ROS message 中；capacity ramp 仍依赖已推荐的 deterministic ROS-free oracle 与 production last-observation-stamp 对齐。

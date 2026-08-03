# C2 capacity ramp bounds phase lag

## Goal

归因 2026-07-29 §7 capacity ramp（expanding，600 s 上限，v8 production 构建）中
analyzer 拒绝原因 "revision 700: production/oracle bounds mismatch"：production
known-bounds 的 `max_x` 在扩张前沿滞后 oracle 精确预测约 4 个 revision（rev 700
时 36.0 vs 36.2，rev 704 才达 36.2；rev 600 及更早所有对账点精确一致）。判定该
一 voxel 相位滞后属于 production mapper 缺陷、oracle 模型缺陷，还是 join/对账
时点语义问题；不修改 C2 occupancy 语义、不放宽 analyzer 精确相等门、不调整
workload。

## Background

- Raw（保留，valid=false）：`/tmp/alien-c2-capacity-ramp-20260729-v8`；
  `capacity_status=covered`，`required_end_revision=740`，实跑至 rev 757 正常收尾。
- 分歧模式：7 个 oracle checkpoint（每 100 revision）中仅 rev 700 失配，且为
  production 滞后（非缺格）；扩张速度 0.05 m/revision，滞后 4 rev = 0.2 m = 1 voxel。
- 容量信息性数据（不作为通过证据）：500k crossing rev 440；750k 时 RSS≈126 MiB、
  PSS≈110 MiB、target CPU mean 70.9%（max 101%），10 Hz 维持。

## Requirements

- 离线对比 `perception_profiling` oracle 的 expanding 场景 bounds 语义与
  `LocalObservationMapper`（及其 backend）的 known-bounds 更新语义，定位首次分歧
  的确切 revision 与触发几何（端点是否落在 voxel 边界、max-range endpoint 处理、
  pose watermark 时序）。
- 用 raw 中的 observations/states/oracle checkpoints 复算验证假设；不重跑 workload
  即可完成归因的优先离线完成。
- 结论三选一：production 缺陷（→ 独立修复任务，冻结本证据）；oracle 缺陷
  （→ 修 oracle 并加回归测试，属测量资产可在本 finding 修）；join 语义问题
  （→ 修 analyzer join 并加回归测试，同上）。修复后 fresh 重跑 capacity ramp。

## Acceptance Criteria

- [x] 分歧的触发条件可从 raw 数据或代码语义精确复现/解释。
- [x] 按分类完成对应动作，fresh capacity ramp 以 `valid=true` 且
  `capacity_status` 机器判定收尾（或记录量化 blocker 交评审）。
- [x] 未修改 C2 occupancy/health/epoch/revision 语义，未放宽 bounds 精确相等门。

## 2026-07-29 归因结论（research/root-cause.md 全文）

- 机制：`OctoMapBackend::known_bounds()` 对 known-cell `std::set` 的 O(N) 扫描在
  每次 `publish_state()`（~55 Hz）与 read transaction 触发；~500k cells 后目标核
  饱和（pidstat 99–101%，rev ~570 起），单线程 executor 饿死 20 Hz pose 订阅，
  mapper 使用 0.35 s 旧 pose（在 1 s 新鲜度预算内，无诊断），扩张前沿滞后
  7 revisions ≈ 1 voxel。rev 1–600 滞后 k=0（与 oracle 位级一致），
  rev 701–750 k=7（12/12 精确拟合）；首次分歧 rev 612/613。
- 分类：**production 性能缺陷**（观测吞吐 10 Hz 保持、无 backlog，但 500k 后
  无法维持全保真 pose 消费）；oracle 与 analyzer join 均无缺陷。
- 触发 §7 预案："covered 且 500k 前后不能维持安全 headroom → 独立 finding +
  暂停长时矩阵"。本 raw（valid=false）冻结保留；capacity 数字仅信息性。
- 修复点（不在本 finding 实施）：`OctoMapBackend.cpp` 增量维护 min/max
  VoxelIndex（O(1) bounds）；次要：`publish_state`/`publish_octomap` 触发频率。
  修复属 `perception_local_map` 产品代码，需独立修复任务与用户批准。
- 影响面评估：§9 expanding 矩阵与 ramp 重跑被阻塞；§8 bounded 矩阵的 plateau
  地图较小（120 s bounded 校准 CPU ≈35%，远未饱和），技术上不受影响，但按
  §7 预案暂停长时矩阵直至用户决策。

## 2026-07-29 验收记录（finding 关闭）

- 修复由 `07-29-octomap-backend-o1-known-bounds` 完成（O(1) known_bounds，
  0 Blocking / 0 High，包 48/0/0、全区 267/0/0）。
- v9 前缀（`/tmp/alien-c2-prod-20260729-v9`，修复后源码，隔离测试 267/0/0）重跑
  ramp：`/tmp/alien-c2-capacity-ramp-20260729-v9`，`valid=true`、
  `normal_completion=true`、`capacity_status=covered`、end rev 760 ≥ required 740。
- 修复效果：bounds 对账 7/7 精确一致（v8 为 6/7）；目标核 CPU mean
  70.9%→25.1%，尾段（~750k cells）99–101%→37.9%（max 41.6%）；RSS/PSS 不变
  （rev 700 时 ≈125.8/109.5 MiB）。pose 饥饿消除，oracle 位级等价恢复。

---

## 收口（2026-07-31）：已解决

根因确认为 `OctoMapBackend::known_bounds()` 的 O(N) 全扫描——地图增长到约 500k
cells 后饱和目标核、饿死 pose 订阅，导致对外报告的 bounds 前沿滞后 1 voxel。

修复由任务 `07-29-octomap-backend-o1-known-bounds` 完成（改为增量维护
min/max，提交 `eb78d46`），验收通过：

- capacity ramp 重跑 `valid=true` / `covered` / end rev 760，**bounds 7/7 精确一致**
- 尾段 CPU 由 99–101% 降至 37.9%；同负载稳态 CPU 由约 35% 降至约 13.3%

**注意**：expanding 场景在约 2.6M cells 处仍会出现 bounds 滞后，但那是回调预算
被吃满导致的，与 `known_bounds` 无关，属另一问题（见下方 capacity-knee 记录）。

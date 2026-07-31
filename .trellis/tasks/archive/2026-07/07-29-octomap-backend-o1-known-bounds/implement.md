# OctoMapBackend O(1) known bounds - 实施计划

- [x] `OctoMapBackend.cpp`：`Impl` 增加 `std::optional<VoxelIndex> minimum_known /
  maximum_known`；`apply_cell` 内 insert 后按分量更新；`known_bounds()` 改读缓存
  （空集仍返回 `std::nullopt`；`voxel_bounds(min, max+1, geometry)` 语义不变）。
- [x] 测试：在 perception_local_map 测试中新增 bounds 等价性用例（增量 vs 全扫描
  重算；含 reset 后清空、负索引、重复 insert 幂等）。
- [x] 构建并运行 `perception_local_map` 全部测试（沿用仓库既定构建方式），零失败。
- [x] trellis-check 复核（0 Blocking / 0 High）。
- [x] 记录 diff 与测试结果；ramp 重跑归 finding 任务执行。（2026-07-31 用户授权提交）

## 2026-07-29 实施记录

- diff：`OctoMapBackend.cpp` +20/−11（Impl 缓存 + apply_cell 更新 + O(1)
  known_bounds）；`TestLocalMap.cpp` +84（BackendConformance.
  OctoMapKnownBoundsMatchFullScan：全扫描独立复算等价、负象限、重复 insert、
  Rejected 批不动 bounds、reset 清空重建）。负向对照（扰动缓存更新）确认测试
  非空洞。
- 主 ws Release 构建 0 警告；包内 48/0/0/0，全工作区 267/0/0。
- trellis-check 静态复核 `0 Blocking / 0 High`：insert-only 不变量、Rejected
  前置校验、reset 经 Impl 重建、锁纪律（写在 unique_lock 路径、读在
  shared_lock）逐点确认。
- AC2 已验收：v9 ramp valid=true/covered，尾段 CPU 37.9%；详见 finding 记录。未提交。

## 2026-07-31 收口

- 用户授权提交；本任务四项全部完成。
- **修复收益已由 C2 正式基线独立量化**（见
  `docs/local-map-resource-profiling.md`）：同负载稳态 target CPU 由约 35% 降至
  约 13.3%（**2.6 倍**）。该效应远超测量环境 ±30% 的争用污染幅度
  （见 playbook §2.8），是可下结论的效应。
- capacity ramp 验收：`/tmp/alien-c2-capacity-ramp-20260729-v9`
  `valid=true` / `covered` / end rev 760 / bounds **7/7 精确一致**，
  尾段 CPU 由 99–101% 降至 37.9%。
- **遗留（不属本任务）**：expanding 场景在约 2.6M cells 处仍有容量拐点，机制为
  回调预算被 `snapshot_serialization` 与 `mapper_apply` 吃满，与 `known_bounds`
  无关；由 finding `07-31-c2-expanding-capacity-knee` 跟踪。
- 提交范围：`OctoMapBackend.cpp`、`TestLocalMap.cpp`（stage 探针与 profiling
  资产属另一提交）。

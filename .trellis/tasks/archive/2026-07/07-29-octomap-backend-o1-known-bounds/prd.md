# OctoMapBackend O(1) known bounds

## Goal

消除 `OctoMapBackend::known_bounds()` 的 O(N) 全集扫描：地图 ~500k known cells
后该扫描（每次 `publish_state()` 约 55 Hz + read transaction）使被测核饱和、
饿死 pose 订阅、扩张前沿滞后 1 voxel（capacity ramp 被 oracle 对账门正确拒绝）。
改为增量维护的 O(1) bounds，语义与现全扫描位级等价。

## Background

- 归因证据：`.trellis/tasks/07-29-c2-capacity-bounds-phase-lag/research/root-cause.md`
  （pidstat 99–101% 饱和、pose 滞后 k=0→7 逐窗拟合、rev 704 追平精确复现）。
- 前提事实：`Impl::known_cells` 只增不删（`apply_cell` insert；`reset()` 重建
  Impl），故 min/max 增量维护精确等价，无需处理删除。

## Requirements

- `known_bounds()` 返回值与改动前对任意 apply/reset 序列**位级相同**；
  公共 API、ROS 接口、occupancy/health/epoch/revision 语义零变化。
- 仅改 `OctoMapBackend.cpp`（Impl 内加缓存 + `apply_cell` 更新 + `known_bounds()`
  读缓存）；不改 `query_region`/`for_each_known_cell`（非本缺陷热路径）。
- 回归测试：新增等价性测试（多批 apply 后 bounds == 按 known cells 全扫描重算；
  覆盖 reset、单 cell、跨象限负索引、重复 insert）。
- 既有 `perception_local_map` 全部测试零失败。

## Acceptance Criteria

- [x] 等价性与既有测试全绿（Release 或既定构建型别）。
- [x] 修复后 fresh capacity ramp（新 pair 构建）`valid=true` 且 bounds 对账全过，
      目标核不再于 500k 处饱和（pidstat 尾段 < 100%）——v9 尾段 37.9%（max 41.6%）。
- [x] 未触碰任何公共接口与业务语义；diff 限于 OctoMapBackend.cpp 与测试。

## Notes

- 触发 finding：`07-29-c2-capacity-bounds-phase-lag`（保持独立，本任务完成后其
  ramp 重跑验收在 finding 侧记录）。
- 提醒：本修复改变 ws/src → v8 pair 身份失效；§8 stage-latency 正式 run 前需按
  标准新建 pair 并复跑校准（决策在父性能任务 §8 时点做）。

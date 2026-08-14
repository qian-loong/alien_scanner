# C4.1 分块地图布局估算与 COW 优化：实施计划

> 当前已进入实施并完成 Gate B 短筛选。结论为 no-go：保留可选 chunked COW 实现，
> 默认 storage 继续使用 vector；详见 `validation/gate-b-short-ab.md`。

## 1. 规划与基线冻结

- [x] 创建父任务下的 C4.1 子任务。
- [x] 读取 C3/C4 contract、C4 正式质量门、分块后续决策和当前 applier/workload。
- [x] 明确当前一维 C4 workload 对 `32` 有偏，不能独立选型。
- [x] 明确 canonical SHA-256 仍要求完整 cell traversal，分块不等于端到端 `O(K)`。
- [x] 完成 PRD convergence pass 和技术设计。
- [x] 用户审核 `prd.md`、`design.md`、`implement.md` 并确认启动。
- [ ] 按当前可用的已约定审核方式完成方案评审，无高置信待修项。
- [x] 运行 `task.py start`，在独立分支 `phase/4-chunked-map-cow` 实施。

## 2. 阶段 A：只读布局估算

- [x] 在 `perception_map_update` ROS-free core 中加入共享的 chunk coordinate/floor
  division 逻辑及 gtest，避免 estimator 与正式实现各写一套坐标规则。
- [x] 在 profiling/test 边界加入布局估算器，输出结构化 JSON/CSV 摘要。
- [x] 接入现有一维 C4 bounded/expanding 数据。
- [x] 复用 `MapUpdateReplayOracle` + `SnapshotDiffer` 生成三维 exact-revision
  snapshot/delta；不新增第二套地图生成算法。
- [x] 构造负坐标、跨边界和空间分散最坏场景。
- [x] 对 `8/16/32` 输出 chunk、bucket、copied-cell、写放大、共享率、填充率和元数据。
- [x] 按 PRD R2 生成选择报告；edge 8 的元数据代价超过上限，因此选择 `16` 作为折中默认。
- [x] **决策门 A**：三档均明显降低了局部更新复制量，允许进入实现验证。

验证：

```bash
cd /workspaces/alien-scanner/ws
colcon build --symlink-install --packages-up-to perception_profiling
source install/setup.bash
colcon test --packages-select perception_map_update perception_profiling \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 3. 阶段 B：存储无关读取边界

- [x] 定义 ROS-free canonical cell view/cursor：`size`、确定性 traversal、显式
  materialization。
- [x] 把 ContentHasher、OctoMap adapter、replay compare、receiver diagnostics 和 C4
  消费点迁移到 view；保留 vector view 实现作为基线。
- [x] 搜索并处理所有 `reconstructed_map()->cells`/`map.cells` 直接依赖。
- [x] golden hash 证明 vector view 与 canonical vector 字节流完全一致。
- [x] 跨负坐标/交错空间块的 traversal 测试证明全局 `(x,y,z)` 顺序正确。

回滚点：本阶段只引入读取边界，默认仍使用 vector storage；任何回归可在不改变 wire 的
情况下退回原消费路径。

## 4. 阶段 C：不可变空间块与 COW

- [x] 实现所选 edge 的 `ImmutableCellChunk`、固定分片目录和 bucket COW。
- [x] keyframe 构建 chunked snapshot；summary/remove/revision-only 保持最小分配。
- [x] delta 按 chunk 分组，只复制 touched bucket/chunk；未变化 chunk identity 共享。
- [x] 实现 candidate canonical cursor 和 v1 content hash 重算，不物化完整 candidate。
- [x] 用 checked arithmetic 重写 peak admission，覆盖 bucket/chunk/cursor scratch。
- [x] `MapUpdateApplier` 支持 vector/chunked 两种独立 storage；Gate B 未通过，默认保持 vector。
- [x] 增加内部观测指标：total/touched/shared chunks、copied cells/bytes、bucket entries、
  candidate/hash/commit duration。
- [x] **决策门 B**：conformance 通过，但 R5 和短 A/B 端到端收益未通过；判定 no-go，
  chunked 不设为生产默认。

## 5. 语义与集成验证

- [x] 增加 vector/chunked 并排 conformance，覆盖全部 `ApplyUpdateStatus`，并保留
  `TestMapUpdateCore` 的完整 vector 基线场景。
- [x] 覆盖 keyframe、delta、revision-only、summary、remove、gap、conflict、corruption、
  resource limit、epoch replacement、tombstone 和 resync recovery。
- [x] 每个三维 replay checkpoint 对比 source/geometry/revision/cells/content/update hash。
- [x] 运行受影响包：`perception_map_update`、`perception_local_map`、
  `perception_profiling`、`swarm_data_plane`。
- [x] 运行 C4 cave visual launch test；界面外观不变，不要求重新设计 RViz。
- [ ] 运行 ASan/LSan 和严格 Memcheck，确认无业务内存错误/泄漏。

## 6. 性能与内存 A/B

- [x] 在同一构建身份、输入和采样窗口中重跑 `vector + flat SHA-256` 基线；不能只用
  历史 C4 数字替代同期对照。
- [x] 同进程短筛选比较 `vector + flat SHA-256` 与使用同一 flat hash 的 `8/16/32`
  chunked COW（实现 edge 可配置），验证 estimator 与实际 copied-cell/bucket 指标一致。
- [x] 对胜出 edge 运行 C4 smoke。
- [ ] 正式运行 `2 x 100k`、`8 x 100k`、`2 x 500k` bounded，三轮 300 秒。
- [ ] 运行 expanding、keyframe replacement、Heaptrack、ASan/LSan、Memcheck。
- [x] 报告短 A/B 的 decode/candidate/hash/commit/callback、PSS/USS、复制量和消息守恒；
  正式矩阵的 peak heap/allocations 未采集。
- [x] 对照同期 vector 基线，明确哪些改善来自 delta COW、哪些仍受完整 hash/keyframe
  限制。
- [x] 把 Merkle/hash v2 留给独立后续任务；其基线固定为本任务验收后的
  `chunked COW + flat SHA-256`，不得在当前 COW 正式 A/B 中启用。
- [x] 原始证据位于独立 profiling archive，任务 `validation/` 保留可审计摘要和 hash。

Gate B 已在短筛选阶段判定 no-go。上面保留的正式矩阵、Heaptrack、Sanitizer 和 Memcheck
条目不是“通过”，而是因不切换默认而停止投入；若用户决定继续研究可选 chunked 实现，
再恢复这些采集。

## 7. 完成流程

- [x] 加载 `trellis-check`，按任务产物、backend spec 和四个受影响包做全量检查并修复。
- [x] 执行最终 validation，记录命令、构建身份、测试结果和剩余风险。
- [x] 加载 `trellis-update-spec`，把稳定的 chunk/view/resource contract 更新到
  `.trellis/spec/backend/`。
- [x] 更新父任务：C4.1 完成，C5 拓扑与角色成为下一关键路径。
- [ ] 按 Trellis Phase 3.4 给出仅包含本任务文件的提交计划，获得用户一次确认后提交；
  不 push。
- [ ] 用户验收后运行 finish-work，归档任务并记录 session。

## 8. 预计受影响文件

主要范围：

- `ws/src/alien_perception/perception_map_update/include/`
- `ws/src/alien_perception/perception_map_update/src/`
- `ws/src/alien_perception/perception_map_update/test/`
- `ws/src/alien_perception/perception_profiling/`
- `ws/src/swarm_data_plane/test/`（workload/runner/指标，不改 wire）
- `.trellis/spec/backend/map-state-update-contract.md`
- `.trellis/spec/backend/performance-measurement-boundaries.md`

除非读取边界编译需要，不修改 `perception_interfaces` 或 `swarm_data_interfaces`；任何
消息定义变更都视为超出本任务范围并退回规划。

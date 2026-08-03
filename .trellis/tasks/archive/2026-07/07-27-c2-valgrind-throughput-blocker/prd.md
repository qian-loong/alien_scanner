# C2 Valgrind throughput blocker

## Goal

Determine why Valgrind Massif and Memcheck cannot maintain the frozen C2
workload, then produce valid evidence at the original load or document a
reproducible tool-capability blocker and an explicit evidence strategy.

## Requirements

- Preserve the frozen workload: 10 Hz unique observations, 360 beams,
  FullRay range `[0.1, 30] m`, and 0.2 m map resolution. Do not lower the
  publisher rate, reduce beam count, change geometry, relax the warmup/revision
  gates, or replay timestamps to make Valgrind pass.
- Reproduce the Massif and Memcheck backlog with exact target ELF provenance,
  target-only process identity, observation/revision lag, and normal tool/role
  finalization. Keep the existing invalid raw directories unchanged.
- Separate target throughput from Valgrind overhead with an equivalent
  uninstrumented control, and evaluate a strategy that can preserve the
  workload and required formal windows. An alternative tool may be proposed
  only with an explicit statement of which Massif/Memcheck contract it can and
  cannot replace.
- Do not modify C2 occupancy, health, pose, epoch/revision, FullRay, or
  visualization snapshot semantics as part of this task.

## Acceptance Criteria

- [x] Massif and Memcheck perturbation is reproducible from preserved raw
  evidence, including input rate, applied revision rate, backlog, target/tool
  exit status, and profiler report completeness.
- [x] Either both required Valgrind modes complete valid runs at the frozen
  load and formal duration, or the task records a quantified blocker and a
  reviewable evidence plan without claiming the invalid runs as baseline
  evidence.
- [x] No workload reduction, acceptance-gate relaxation, or C2 business
  semantic change is used to close the finding.

## Notes

- Parent evidence:
  `/tmp/alien-c2-smoke-valgrind-massif-20260727` and
  `/tmp/alien-c2-smoke-valgrind-memcheck-20260727`.

## 2026-07-30 复测与量化结论（走 AC 路径 B：量化 blocker + 证据策略）

- 复测条件：O(1) known_bounds 修复后的 v9 前缀（无 Valgrind 时稳态 CPU 由 ~35%
  降至 13.3%），冻结负载不变（10 Hz、360 beam、0.2 m、bounded）。
- 结果：Memcheck 60 s 与 Massif 180 s 均 `valid=false`、
  `normal_completion=false`，失败于 "workload did not reach its required warmup
  revision"。Memcheck 实测 applied 速率 **3.24 rev/s vs 合同 10 rev/s**
  （572 revisions / 175.8 s；observation 收到 1824 帧，消费侧积压）——较修复前
  （无法过 warmup 且更慢）改善约 3 倍，但仍差 ~3.1 倍。
- 判定：**结构性工具能力阻断**（Valgrind 虚拟机放大 ×，帧耗时 ~310 ms >
  100 ms 预算），非被测代码缺陷。invalid raw 保留：
  `/tmp/alien-c2-retest-{memcheck,massif}-20260730-v9` 及 2026-07-27 原始证据。
- **证据策略（待用户评审）**——明确各替代项能/不能覆盖的 Memcheck/Massif 合同：
  1. 内存错误（越界/UAF）：ASan 60 s 已通过（2026-07-30，零报告）——**可替代**
     Memcheck 的错误检测；**不能替代**未初始化读检测（MSan 需全依赖重编译，
     成本过高，记为已知证据缺口）。
  2. 运行期泄漏：LSan 60 s 已通过（无 leak summary）——覆盖 definite 类运行期
     泄漏；**不能替代** Memcheck 的四级分级（definite/indirect/possible/
     still-reachable）。
  3. 堆时间线：Heaptrack 300 s 已通过（peak 28.96 MB，时间线完整）——**可替代**
     Massif 的形状证据（同为堆采集，且扰动小）。
  4. 补偿证据：对既有 gtest/集成测试套件（无实时合同）跑 Memcheck，覆盖同一
     mapper/backend 代码路径的未初始化读与泄漏分级——不受 10 Hz 门约束，
     属合法补偿控制，不冒充长时 workload 证据。
- 不降频、不减 beam、未放宽任何门；invalid run 不计入基线。

## 2026-07-30 补偿证据（缺口②④闭合，finding 关闭）

- 用户批准证据策略后，对确定性测试套件（无实时合同）执行 Memcheck
  `--leak-check=full --track-origins=yes`：TestLocalMap（27 用例）、
  TestCaveFullRayScene（1）、TestPerceptionProfiling（10，Valgrind 下 648 s
  仍全过）。raw：`/tmp/alien-c2-memcheck-testsuite-20260730`（含 sha256sum.txt）。
- 结果：三个二进制 `ERROR SUMMARY: 0 errors from 0 contexts`（无未初始化读、
  无非法访问）；泄漏分级无 definite/indirect/possible，仅 still-reachable
  小块（72–96 B，运行时全局）。与 LSan（无摘要）、Heaptrack（exit 期 455 KiB）
  三方互证：无 C2 业务泄漏。
- 残余缺口（如实记录）：②④ 的证据来自测试负载路径，非 300 s 正式 10 Hz 长跑
  负载；该残余在当前工具能力下不可消除。
- 本 finding 关闭：结构性吞吐阻断已量化（3.24 vs 10 rev/s），替代/补偿证据链
  完整且逐子能力声明了可/不可替代范围。

---

## 收口（2026-07-31）：已解决

Valgrind（Memcheck/Massif）10–50× 扰动导致无法维持冻结的 10 Hz 负载，实测
3.24 vs 10 rev/s，确认为**结构性阻断**，非配置问题。禁止降频/减 beam/放宽
warmup 门的纪律全程未破。

补偿方案已落实并验证：

- **Massif 的堆形状证据由 Heaptrack 替代**（bounded 300 s、expanding 100 s 均
  有效，含 Massif 兼容时间线 10 082 snapshots）。
- **Memcheck 的未初始化读与泄漏分级改到无实时约束的测试套件负载上取得**，
  结果 0 errors、仅 still-reachable。
- 残余缺口"证据来自测试负载而非长跑负载"已如实记录，未假装等价。

现状见 `docs/local-map-resource-profiling.md` §4.5 与
`docs/performance-memory-testing-playbook.md` §2.7。

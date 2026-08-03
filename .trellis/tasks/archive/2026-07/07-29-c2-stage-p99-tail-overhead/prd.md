# C2 stage calibration p99 tail overhead

## Goal

归因 2026-07-29 v8 正式 120 秒 stage 校准中 callback p99 门失败的尾延迟差异
（callback vs full：2 815 445 ns > 阈值 2 308 828 ns，约 12.2%，门为
max(10% relative, 50 us)），判定其属于 full nested probe 真实 overhead、
单次运行的 order/variance 噪声，还是其他系统性原因；不放宽阈值、不修改
C2 业务语义、不调整 workload。

## Background

- 配对构建等价性已由 v8 pair 机器证明（closure 字节相同、身份五检查点零漂移）：
  `/tmp/alien-c2-stage-pair-20260729-v8`，
  `paired_source_identity_sha256=53e54c67ad8e206dc6fa93cb34a6b33963df4daf09ddbe1696c249ac2c7dff6f`。
- 正式三组 raw（全部 `valid=true`、`normal_completion=true`）：
  `/tmp/alien-c2-formal-unprobed-20260729-v8`、
  `/tmp/alien-c2-formal-callback-20260729-v8`、
  `/tmp/alien-c2-formal-full-20260729-v8`。
- aggregator schema-2 输出：`/tmp/alien-c2-stage-calibration-20260729-v8.json`
  （SHA-256 `b26accdbe07a3a8312304d06636fa6f22cb6af7b9bb237107f2f8eaf45ca2d70`）、
  quality `c620e94a7ef4215959a6433b207e9c6b64312dd24d5353ddb2a946fa0673555e`；
  `AGGREGATOR_EXIT=1`。
- 通过项：CPU 门（delta 0.067 pp，阈值 1.7617 pp）、callback p50（59 901 ns）、
  p95（935 077 ns）；仅 p99 失败。CPU 几乎零差说明整体 overhead 极小，失败集中在
  尾部，指向 tail variance 或 probe 对长尾样本的选择性放大。

## Requirements

- 保留三组 v8 formal raw 与 aggregate 不变；任何复跑写入全新目录。
- 归因手段可包括：同构建多次重复 run 的 p99 分布（order/variance 检验）、
  callback 与 full 的逐样本延迟分布对比、长尾样本与 GC/page fault/调度事件的
  相关性；全部在 v8 pair（或按同标准新建 pair）上进行。
- 结论三选一并给证据：真实 probe overhead（则评估门径设定是否合理，交用户决策）、
  运行间方差（则给出所需重复次数与统计判据）、环境扰动（则定位并消除后复跑）。
- 不降频、不减 beam、不放宽 warmup/revision 门、不改阈值公式。

## Acceptance Criteria

- [ ] p99 失败的原因有可复现证据支撑，且分类明确（overhead / variance / 环境）。
- [ ] 依据结论给出下一步：按原门复跑通过，或提交带量化 blocker 的证据策略评审。
- [ ] 未修改 C2 业务语义、workload 或阈值。

## Notes

- 父任务：`07-27-c2-performance-memory-baseline`（§6 阶段延迟证据）。
- 构建等价性子任务：`07-28-c2-stage-calibration-build-equivalence`（v8 evidence）。

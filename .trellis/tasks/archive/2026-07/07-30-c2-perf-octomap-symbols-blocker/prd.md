# C2 perf record octomap symbolization blocker

## Goal

解除 §8 perf-record 行的符号化门阻塞：2026-07-29 v9 正式 120 s perf-record
（`/tmp/alien-c2-formal-perfrecord-20260729-v9`，valid=false 保留）样本充足
（15120，0 lost）但 `unknown_percent=44.25% > 20%` 门。unknown 几乎全部落在
`/opt/ros/jazzy/lib/x86_64-linux-gnu/liboctomap.so.1.10`（ROS 官方二进制，已
strip，无 `.symtab`；无 dbgsym 包，debuginfod 不覆盖 packages.ros.org）。
不放宽 20% 门。

## Requirements

- 首选证据策略：将 octomap 1.10.0 以带符号 + frame-pointer 的源码构建纳入
  profiling closure（vendored 进 workspace 或独立 underlay），由 closure
  provenance 声明并验证；目标业务二进制链接语义不变（同版本、同 API）。
- 该策略改动 ws/src / .devcontainer → source identity 变化；必须在当前 §8/§9
  批次全部收尾后实施，并新建前缀重跑 perf-record（120 s）。
- 备选（需用户评审）：记录量化限制，perf-record 行以"top-N 可解析热点 +
  unknown 结构性说明"形式交付，不计 gate_pass。
- 已可解析的热点证据（信息性，冻结在 invalid raw 中）：
  `octomap::updateInnerOccupancyRecurs` 29.96%、liboctomap 内部地址合计 ≈37%、
  node 自身符号全部可解析。该分布同时指向 `OctoMapBackend::apply()` 每 revision
  全树 `updateInnerOccupancy()` 的热点问题——由 §10 阶段单独立优化 finding，
  本任务只解决符号化。

## Acceptance Criteria

- [x] perf-record 以原门（≥1000 samples、unknown ≤20%、0 lost）在新前缀通过；
      或备选策略经用户批准并记录。（2026-07-30 用户批准方案 C：受限交付）
- [x] 符号化改动不改变目标二进制的依赖闭包语义（closure validator 通过）。（方案 C 未引入符号化，无 closure 变更）
- [x] 不放宽任何 gate。

## 2026-07-30 方案 C 批准记录（finding 关闭）

- 用户批准受限交付：perf-record 行以"可解析 top-N 热点 + unknown 44% 结构性
  说明"写入基线文档；20% 门不放宽、该行不计通过；invalid raw
  （`/tmp/alien-c2-formal-perfrecord-20260729-v9`）冻结保留。
- 可交付热点结论（closure=ROS 官方 octomap，与矩阵其余行一致）：
  `octomap::updateInnerOccupancyRecurs` 29.96%、liboctomap 内部未符号化地址
  合计 ≈37%、node 自身符号全部可解析（样本 15120、0 lost、频控正常）。
- octomap 带符号 underlay 的引入推迟至将来 "octomap 热点优化" finding
  （届时作为其第一步，重建前缀并按原门重跑 perf-record）。

---

## 收口（2026-07-31）：**未解决，不再跟进**

perf record 的 unknown 比例 **44.25%**，远超 `<=20%` 门。门**未放宽**，该项在
C2 基线中**未计为通过**。

已确认的事实：

- 根因是 ROS 侧 octomap 库无符号表，属打包层面的结构性限制。
- 本机无硬件 PMU，perf record 只能用软件 `cpu-clock` 采样，仅支持热点排序。
- 可解析部分的热点信号：`updateInnerOccupancyRecurs` 约 30%、`liboctomap` 约 37%。

**为什么不再跟进**：C2 的阶段延迟探针已提供更精确的归因
（`snapshot_serialization` 55% / `mapper_apply` 43%，见基线报告 §5.3），
perf record 的符号化价值被覆盖。要真正解决需重新打包带符号的 octomap 或换有
PMU 的原生 Linux 机器，成本高于收益。

现状记录于 `docs/local-map-resource-profiling.md` §4.6 与 §8。

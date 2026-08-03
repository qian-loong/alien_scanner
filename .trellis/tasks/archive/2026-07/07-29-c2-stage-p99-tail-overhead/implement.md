# C2 stage calibration p99 tail overhead - 实施计划

## 1. 运行间方差归因（首选假设）

- [x] 在 v8 pair 上（运行前身份复算必须与盖章一致）各追加 3 组 120 秒 callback 与
  3 组 120 秒 full run，全新输出目录，driver 位于 /tmp，不触碰身份域；
  workload/affinity/event set/阈值零改动。
- [x] 从每组 run 的 stage 分位数输出提取 p50/p95/p99；计算同 mode 跨 run 的 p99
  离散度（含首轮 formal run 共 4 个样本/模式），以及全部 callback×full 配对的
  p99 差分布。
- [x] 判据：若同 mode 跨 run p99 极差与首轮 callback↔full 差（2.815 ms）同量级或
  更大，或配对差分布横跨阈值（2.309 ms）双侧 → 分类为 run-to-run variance，
  给出通过原门所需的重复次数/统计判据建议；若所有配对差稳定超阈且同 mode 离散度
  小 → 分类为真实 probe overhead；若个别 run 出现明显环境扰动证据
  （调度/负载尖峰、Hz 偏离）→ 分类为环境扰动并定位来源。

## 2. 证据与边界

- [ ] 保留首轮 formal raw 与 aggregate 不变；新 run 全部独立目录 + sha256sum。
- [x] 每组 run 要求 `valid=true`、`normal_completion=true`，无效 run 保留但不计入
  统计并记录原因。
- [x] 结论与建议写回本文件及父任务 §6；不改阈值、不降载、不修改 C2 语义。

## 2026-07-29 方差复测结果与归因结论

- 6 组追加 run（callback/full 各 3，交替顺序）全部 `valid=true`、`normal_completion=true`，
  每组约 1200 个 callback 样本；raw 位于 `/tmp/alien-c2-var-{callback,full}-r{2,3,4}-20260729-v8`
  （各含 sha256sum.txt），首轮 formal raw 未动。运行前身份复算与 v8 盖章一致。
- **p99 数据**（ns，r1 = 首轮 formal）：
  - callback: r1 23 088 283, r2 34 402 705, r3 23 209 621, r4 23 399 014（极差 11.31 ms）
  - full:     r1 25 903 728, r2 33 060 185, r3 21 825 989, r4 23 353 307（极差 11.23 ms）
- **归因判据全部指向 run-to-run variance**：
  1. 同 mode 跨 run p99 极差（≈11.3 ms）是首轮 callback↔full 差（2.82 ms）的 4 倍；
  2. 4×4 配对差双向横跨阈值（9/16 FAIL，但 full 有时反而低于全部 callback，如
     fu_r3 21.83 ms < 所有 cb），无系统性方向；
  3. 时间相邻配对 r2/r3/r4 均 PASS（r4 差仅 45 707 ns），只有 r1 FAIL；
  4. 两 mode 的 4-run 均值几乎相同（26.025 ms vs 26.036 ms，差 0.04%）；
  5. r2 窗口两个 mode 的 p50/p95/p99 同步抬升（p99≈33–34 ms），为一次影响双方的
     瞬时环境扰动，非 probe 相关。
- **分类：run-to-run variance（含一次双侧环境扰动窗口）**。首轮 formal 的 p99 FAIL
  是尾部采样噪声；CPU 门（0.067 pp）与 p50/p95 的稳定通过支持"无系统性 overhead"。
- **建议的统计判据（需用户评审，不改阈值公式本身）**：p99 门改为对 >=3 组有效
  callback/full run 取各 mode p99 中位数后按原公式比较。用现有 4 组数据演算：
  callback 中位数 23 304 318 ns、full 中位数 24 628 518 ns，差 1 324 200 ns <
  阈值 2 330 432 ns → PASS。CPU 与 p50/p95 门维持单三组原样（它们方差小且已通过）。
- 未改 workload、affinity、event set、阈值公式或 C2 语义。

## 2026-07-29 方案 A 落地：中位数判据实现与最终聚合

- 用户批准方案 A（p99 门改为 >=3 组有效 run 各取中位数后按原公式比较；阈值公式
  max(10%, 50 us) 与 CPU/p50/p95 门不变）。
- 代码实现：`scripts/lib/stage_latency_calibration.py` 的 `analyze_calibration` 新增
  可选 callback/full replicate 目录（要求两侧数量相等且 >=3，全部 replicate 过
  与主 run 相同的 role/closure/provenance/target/1200-callback 门，身份必须独立）；
  有 replicate 时 schema 升 3，p99 用中位数比较并输出 `p99_evaluation` 明细；无
  replicate 时行为与 schema 2 完全不变。CLI 新增可重复 `--callback-replicate` /
  `--full-replicate`。
- 测试：`scripts/test_analyze_local_map_profile.py` 新增 3 个用例（中位数吸收单点
  outlier 后 pass、多数 run 偏高时 fail、非法 replicate 集合拒绝：数量不足/不等、
  target provenance 不一致、身份重复）。C2 39 tests、C1 19 tests 全通过；
  preserved mixed-build 旧 raw 仍 exit 2 且无 stale 输出；旧 schema-2 v8 aggregate
  SHA（`b26accdb…`/`c620e94a…`）磁盘文件未被触碰。注意：这两个 digest 是**存档值**——
  当前代码对同一 raw 复算会额外输出 `p99_evaluation` 段而产生不同 digest；可复算的
  权威结论以 schema-3 中位数 aggregate（`ff93bd44…`）为准。
- **最终聚合（schema 3，1 unprobed + 4 callback + 4 full）exit 0，`gate_pass=true`**：
  - `/tmp/alien-c2-stage-calibration-median-20260729-v8.json`
    `ff93bd44c433421955fd8f91df97dd7b60d3e692fbcd7e766788aff110d4dbfc`；
  - quality `/tmp/alien-c2-stage-calibration-median-quality-20260729-v8.txt`
    `e0599bd2c7ab45c4ee45359453446c88caa7d484d762aef9251eff10fc0785c9`；
  - CPU 0.067 pp / 阈 1.7617 pp ✔；p50 59 901 ns ✔；p95 935 077 ns ✔；
    p99 中位数差 1 324 200 ns / 阈 2 330 431.75 ns ✔（callback 中位数 23 304 317.5，
    full 中位数 24 628 517.5）。
- 结论：finding 关闭条件满足——p99 失败归因为 run-to-run variance（AC1），按批准的
  中位数判据在原阈值公式下复算通过（AC2），未修改 workload/阈值公式/C2 语义（AC3）。
- 注意：本次修改了 `scripts/`，canonical source identity 已再次变化；v8 raw 是在
  运行时身份匹配下产生的、记录完整，作为测量证据仍有效；但**后续任何新的
  stage-latency 测量**需要按标准新建 pair。§7–§9 的 plain/perf/Heaptrack 等非
  stage 模式不受 stage 身份门影响。

---

## 收口（2026-07-31）：结论成立，但**本任务提出的判据已被后续证据推翻**

### 仍然成立的部分

首轮 p99 失败归因为 **run-to-run variance** 的判定成立：同 mode 跨 run p99 极差
约 11.3 ms，是首轮 callback↔full 差（2.82 ms）的 4 倍；配对差双向横跨阈值、
无系统性方向；两 mode 的 4-run 均值几乎相同。

### 已被推翻的部分（重要，勿照搬）

本任务建议并经批准落地的 **"多 run 中位数判据"（schema-3，后扩展到 CPU 的
schema-4）不成立**。2026-07-31 补跑严格同序轮次后发现：

- 在宿主单调漂移下，各 mode 独立取中位数会**退化成"只评估时间居中的那一轮"**——
  a3 的聚合值恰好等于 r8、a4 的恰好等于 r9，一位不差。
- 它**破坏了 schema-2 原有的轮内配对设计**（同一轮内背靠背比较，漂移自动抵消）。
- 同一批 12 组数据，选 r7/r8/r9 得 0.1415 pp（通过）、选 r7/r9/r10 得
  0.9501 pp（失败）。**两个结论都不能单独成立。**

根因不在统计量而在指标本身：CPU 时间在本机被争用污染约 30%，待分辨的探针开销约
3%，低一个数量级（见 `docs/performance-memory-testing-playbook.md` §2.8）。

### 最终处置

CPU 侧以**受限交付**收口：不称通过原门，只给出边界"未观察到系统性探针 CPU
开销，上界约 1 pp、无系统性方向"。延迟侧四轮按原门逐轮通过，表述为"本轮条件下
通过"。阈值公式、workload、affinity、event set 全程未改。

**方法论教训**：判据不能在看过数据之后设计、再用同一批数据验证——那是后验循环
论证。指标被污染时应承认不可测并给出边界，而不是换一个恰好能通过的统计量。

现状记录于 `docs/local-map-resource-profiling.md` §7。

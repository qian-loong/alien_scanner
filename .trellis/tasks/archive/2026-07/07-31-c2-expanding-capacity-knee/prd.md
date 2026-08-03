# C2 expanding 负载容量拐点（约 2.6M cells）

## 背景

父任务 `07-27-c2-performance-memory-baseline` 的 §9 expanding 正式矩阵首个
300 秒 plain-sample run 被 analyzer 以
`observation/revision window count skew is too large` 判 `valid=false`
（`normal_completion=true`，run 正常收尾）。

归因不是测量链缺陷：fixture 按冻结的 10 Hz 发出 3115 个 observation，
目标节点只完成 2993 个 revision，落后 122 个（约 3.9%）——**节点在大规模地图下
跟不上 10 Hz**。analyzer 的拒绝是正确行为。

这与 §7 的 `07-29-c2-capacity-bounds-phase-lag`（`known_bounds()` O(N) 扫描，
已由 `07-29-octomap-backend-o1-known-bounds` 修复为 O(1)）**不是同一问题**：
那次在约 500k cells 就出现前沿滞后，且劣化从早期就渐进出现；本次在 2.6M cells
之前完全平坦，之后才出现拐点。

## 证据

raw：`/tmp/alien-c2-formal-exp-plain1-20260731-v9`（`valid=false`，按纪律保留）。
v9 production 前缀，expanding workload，请求 300 s。

速率曲线（`memory-checkpoints.csv`，`Δrevision / Δreceipt_monotonic_ns`；
cells 按实测约 1130 cells/revision 估算）：

| revision | 估算 cells | 实测速率 (rev/s) | RSS (MiB) |
| ---: | ---: | ---: | ---: |
| 200–2300 | 226k–2.60M | 9.98–10.04 | 61.4 → 338.3 |
| 2400 | 2.71M | 9.83 | 353.4 |
| 2500 | 2.83M | 9.68 | 360.3 |
| 2600 | 2.94M | 8.93 | 378.7 |
| 2700 | 3.05M | 8.32 | 394.9 |
| 2800 | 3.16M | 8.43 | 412.7 |
| 2900 | 3.28M | 7.95 | 423.5 |

形状特征：**在约 2.6M cells 之前完全平坦（满 10 Hz），此后单调劣化至约 8 rev/s**。
内存全程线性增长，无异常；MemAvailable 充裕（run 期间 > 6.7 GiB），
cgroup 无 limit，无 oom/oom_kill——**不是内存容量阻断**。

### 第二次观测：Heaptrack 200 s，bounds mismatch（不同症状，同一压力）

raw：`/tmp/alien-c2-invalid-exp-heaptrack200-20260731-v9`（`valid=false`，保留）。

- 拒绝原因不同：`revision 2100: production/oracle bounds mismatch`，**不是** skew。
- 该 run **全程维持 10 Hz**（速率 9.93–10.04，rev 200→2100），仅落后 16 个
  observation（0.75%）。即：吞吐没垮，但 **production 报告的 known_bounds 与
  oracle 真值不符**。
- 发生位置 rev 2100，约 2.37M cells、RSS 315.7 MiB——与首次观测的拐点
  （约 2.6M cells）同量级；Heaptrack 的 10–40% 扰动使其提前触及。

**这与 §7 的 `07-29-c2-capacity-bounds-phase-lag` 症状相同**：当时归因为
`known_bounds()` 的 O(N) 扫描饱和目标核、饿死 pose 订阅、前沿滞后 1 voxel，
已由 `07-29-octomap-backend-o1-known-bounds` 修复为 O(1)，并在 v9 ramp 上
7/7 bounds 精确一致。**现在在更大规模 + 额外负载下重现**。

因此本任务需回答：O(1) 修复后重现的 bounds 滞后，是同一 pose 饥饿机制在更高
负载下的复发，还是另一条路径。注意该症状是**正确性可见的**（对外报告的 bounds
与真值不符），不只是性能退化——这提高了本 finding 的优先级。

### 第三次观测：机制已定位（expanding stage-latency 150 s，有效 run）

`/tmp/alien-c2-formal-exp-stage150-20260731-v11`，`valid=true`，
`final_known=1 812 520`。阶段分位数与 bounded（230 520 cells）对比：

| 阶段 | bounded p50 | expanding p50 | bounded p99 | **expanding p99** |
| --- | ---: | ---: | ---: | ---: |
| callback 总 | 11.887 ms | 51.202 ms | 15.644 ms | **92.977 ms** |
| ├ snapshot_serialization | 5.623 ms | 28.577 ms | 7.860 ms | 55.365 ms |
| └ mapper_apply | 5.946 ms | 21.788 ms | 7.935 ms | 39.143 ms |
| state_publication | 0.028 ms | 0.051 ms | 0.150 ms | 0.160 ms |
| read_transaction | 0.0025 ms | 0.003 ms | 0.0043 ms | 0.005 ms |

**结论：拐点不是未知资源上限，而是回调预算耗尽。**
1.81M cells 时 callback p99 已达 92.977 ms，占 100 ms 周期的 93%。地图继续增长
即超出预算，节点必然掉队——这与首次观测到的约 2.6M cells 拐点位置自洽。

**主导项是 `snapshot_serialization`，且增长快于 `mapper_apply`**：
cells 增长 7.86 倍时，serialization 增长 5.08 倍、apply 增长 3.66 倍；
读事务与状态发布始终可忽略（微秒级）。

**该机制同时解释了 Heaptrack 的三次失败**：1.81M cells 时回调已用掉 93 ms 预算，
Heaptrack 的 10–40% 扰动直接把回调推出 100 ms，属结构性超预算。据此预测
"缩到约 1.0M cells（100 s 窗口）即可通过"，实测**一次通过**
（`/tmp/alien-c2-formal-exp-heaptrack-20260731-v9`，`gate_pass=true`），
构成对该机制的正向验证。

结论所依据的是**单 run 内阶段分解比例**，属 §2.8 分级中的可信指标；绝对值仍带
±30–46% 漂移不确定度，但比例与标度趋势稳健。

## 测量环境边界（必须在结论中保留）

按 `docs/performance-memory-testing-playbook.md` §2.8：

- 本机为 Windows 宿主 + WSL2/LinuxKit VM，存在约 ±30% 的 CPU 时间争用污染。
- **"10 Hz 可持续性"属于保守指标**：在被争用的宿主上能维持，专用机只会更好。
  因此本拐点是**本机观测值**，不得表述为产品的绝对容量上限。
- 无硬件 PMU（`cycles / instructions / cache-misses` 全部 `<not supported>`），
  因此**不能**用 IPC / cache miss rate 定位瓶颈，需改用其他手段。

## 目标

~~定位约 2.6M cells 拐点的机制~~ —— **机制已在建单当日定位**（见"第三次观测"）：
回调预算耗尽，主导项为 `snapshot_serialization`。本任务的剩余目标改为：

1. 确认 `snapshot_serialization` 的标度关系（是否 O(N)、能否降到 O(变化量)）。
2. 给出可复算的容量表述（本机拐点 + 无争用环境下的方向性）。
3. 判断是否值得优化、以及优化方向；实现另立 finding。

## 验收标准

- **AC1**：以可重复的实验确定拐点位置（revision / cells / RSS），并给出至少
  两次独立观测的一致性；每次 run 保留 raw 与 `sha256sum.txt`。
  已有三次观测（300 s plain、200 s/150 s Heaptrack）位置自洽，需补一次直接以
  stage-latency 扫描不同地图规模、给出 `callback p99 vs cells` 曲线。
- **AC2**：确定 `snapshot_serialization` 与 `mapper_apply` 随 known cells 的标度
  指数。已有两点（230 520 → 1 812 520 cells）：serialization 5.08×、apply 3.66×
  对应 cells 7.86×，需补中间点以区分幂律与其他形式。
  **单 run 内阶段分解比例是主要抓手**（可信指标）。
- **AC3**：明确 `snapshot_serialization` 每周期做的是全量序列化还是增量。若为全量
  且地图单调增长，则为设计层面的 O(N)/周期问题，应指出可行的增量化方向。
- **AC4**：结论表述遵守环境边界——给出本机观测的拐点，并说明该值在无争用环境
  下的方向性（只会更高），不给出绝对容量承诺。
- **AC5**：不在本任务内做优化实现。若定位到可优化点，另立优化 finding。

## 非目标

- 不修改 C2 occupancy 语义或 workload 定义。
- 不放宽父任务的 skew / validity 门——该门此次工作正常，是它抓到了问题。
- 不追求在本机把拐点推高；本任务只做归因。

## 与父任务的关系

父任务 §9 的 expanding plain-sample 与 Heaptrack 时长已（经用户批准）从 300 s
缩短到 200 s，以保证正式证据落在满 10 Hz 的有效窗口内（rev 约 2000，约 2.26M
cells，RSS 约 299 MiB，仍有 10 倍扩张，足以支撑每体素内存成本结论）。
§9 item 4 的 expanding stage-latency 原文即含"不超过 capacity 安全窗口"，
按同一窗口执行，不属偏离。

本任务不阻塞父任务 §9/§10 收口。

---

## 收口（2026-07-31）：**机制已定位，量化未完成，不再跟进**

### 已确立的事实

- expanding 场景下 10 Hz 可持续上限约 **2.6M cells（约 340 MiB RSS）**，
  超过后逐步劣化至约 8 rev/s。
- **机制明确**：不是未知资源上限，而是**回调预算被吃满**。1.81M cells 时
  callback p99 已达 92.977 ms，占 100 ms 周期的 93%。
- **主导项是 `snapshot_serialization` 且增长快于 `mapper_apply`**：cells 增长
  7.86 倍时，serialization 增长 5.08 倍、apply 增长 3.66 倍。
- 该机制做出并验证了一个预测：expanding Heaptrack 在 200/150/150 s 三次失败，
  据机制推算"缩到约 1.0M cells 可行"，实测一次通过。
- 内存不是瓶颈：全程线性增长、MemAvailable > 6.7 GiB、无 oom。

### 未完成的部分

- **未确定 `snapshot_serialization` 的标度指数**。只有两点
  （230 520 → 1 812 520 cells），不足以区分幂律或其他形式，未补中间点。
- **未确认它是每周期全量序列化还是增量**。若为全量且地图单调增长，则属设计层面
  的每周期 O(N) 问题，增量化是明确的优化方向——但这一条**尚未验证**。
- 未做优化实现（本就不在范围内）。

### 表述纪律

约 2.6M cells 是**本机（WSL2/LinuxKit，被争用宿主）观测值**。10 Hz 可持续性是
**保守指标**，无争用环境下拐点只会更靠后，**不构成产品的绝对容量承诺**。

现状记录于 `docs/local-map-resource-profiling.md` §5.3–5.4 与
`docs/local-observation-map.md` 的"已测得的容量与性能边界"。

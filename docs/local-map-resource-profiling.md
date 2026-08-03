# C2 局部观测地图性能与内存基线

本报告是 C2（`perception_local_map`）的冻结资源基线，供 C3 比较使用。测量对象是
目标节点进程本身；fixture、sink、oracle、采样器与 profiler 均为独立进程，不计入
目标资源数字。本轮没有修改 C2 业务源码语义。

姊妹报告：C1 见 `docs/perception-resource-profiling.md`；方法论与新手指引见
`docs/performance-memory-testing-cookbook.md`。

---

## 0. 阅读本报告前必须知道的事

> **测量环境有结构性限制，部分指标不可信。**
> 完整论证见 `docs/performance-memory-testing-cookbook.md` §6，
> 可执行约束见 `.trellis/spec/backend/performance-measurement-boundaries.md`。

环境为 **Windows 宿主 + WSL2 / Docker Desktop 的 LinuxKit VM**（内核
`6.10.14-linuxkit`）。两条实测事实：

1. **无硬件 PMU**。`/sys/bus/event_source/devices/` 无 `cpu` 项；
   `cycles / instructions / branches / cache-*` 一律 `<not supported>`，root 亦不可得
   （与 `perf_event_paranoid` 无关）。容器**确实持有** `cap_perfmon` 与
   `cap_sys_ptrace`（与 `docker-compose.yml` 的 `cap_add` 一致）——capability 到位
   并不能补上虚拟机层面缺失的 PMU。
2. **CPU 时间指标被争用污染约 ±30%**。实测：四轮 workload 逐位相同
   （各 1201 observations / 1202 revisions / 120.08–120.10 s），目标进程实耗 CPU
   时间为 12.966 / 13.986 / 14.806 / 16.897 秒。

**由此得出的分辨率下限：本机可分辨效应量级约 ≥ 30–50%；≤10% 的效应不可分辨。**

| 本报告中的指标 | 可信度 |
| --- | --- |
| 精确计数（known/free/occupied、revision、observation、bounds） | **完全可信**（确定性 + oracle 逐点校验） |
| RSS / PSS / USS 及每体素内存成本 | **完全可信**（页面计数，与调度无关） |
| Heaptrack 分配次数 / peak heap | **完全可信**（由代码路径决定） |
| 泄漏与错误有无（ASan / LSan / Memcheck） | **完全可信**（确定性事实） |
| **单 run 内**的阶段分解比例 | **可信**（同进程同时刻测量，争用等比影响） |
| 延迟绝对分位数 | 带 ±30–46% 漂移，只能当量级用 |
| CPU 绝对百分比 | 带 ±30%，仅当效应远超此幅度时结论才成立 |
| 10 Hz 可持续性 | 可信且**保守**（被争用宿主上成立，专用机只会更好） |
| IPC / cache miss rate / 探针 CPU 开销门 | **不可用**，本报告不提供 |

---

## 1. 结论

**吞吐与实时性**

- bounded 冻结场景下，目标节点稳定维持 10 Hz：600 秒窗口 6001 个 observation，
  `final_known=230520`，无 backlog。
- expanding 场景下，节点在**约 2.6M cells（约 340 MiB RSS）以内维持满 10 Hz**，
  超过后逐步劣化至约 8 rev/s。该拐点为**本机观测值**，无争用环境下只会更靠后。

**内存**

- bounded 稳态平坦：三轮 300 秒 plain sample 的 RSS/PSS/USS 为
  `62 484 / 44 586 / 39 884 KiB`，三轮斜率 RSS `0/0/0`、PSS `0/0/0.097 KiB/min`
  （阈值 1024），`suspected_sustained_growth=false`，**无疑似泄漏**。
- expanding 的**每新增 known 体素成本约 121–126 B**，由两条独立路径交叉验证
  （smaps 采样与 Heaptrack），离散度约 1%。地图正常扩张**未**被判为泄漏。
- 退出期残留 `455.54 K` 在 bounded 与 expanding 下**完全相同**，说明与地图规模
  无关，属固定的退出期行为，非随扩张增长的泄漏。

**延迟与热点**

- bounded（230 520 cells）：callback p50 `11.887 ms`、p99 `15.644 ms`，占 100 ms
  周期约 12–16%。
- expanding（1 812 520 cells）：callback p50 `51.202 ms`、p99 `92.977 ms`，
  **已占用周期预算的 93%**。
- **主导项是 `snapshot_serialization`，且增长快于 `mapper_apply`**：cells 增长
  7.86 倍时，serialization 增长 5.08 倍、apply 增长 3.66 倍。读事务与状态发布
  始终可忽略（微秒级）。
- **这解释了容量拐点**：拐点不是未知资源上限，而是回调预算被序列化与 apply 吃满。

**关键修复的收益**

- `OctoMapBackend::known_bounds()` 由 O(N) 改为 O(1)（任务
  `07-29-octomap-backend-o1-known-bounds`）后，同负载稳态 CPU 由约 35% 降至约
  13%（2.6 倍），并消除了大图下的 bounds 前沿滞后。该效应远超 ±30% 污染，结论成立。

**本报告不提供的结论**

- stage 探针的 CPU 开销**在本机结构性不可判定**，仅能给出上界（见 §7）。
- IPC、cache miss rate 及任何依赖硬件 PMU 的分析。
- perf record 的完整符号化热点（44.25% unknown，见 §8 限制）。

---

## 2. 环境与构建

| 项目 | 值 |
| --- | --- |
| OS / ROS | Ubuntu 24.04 / ROS 2 Jazzy |
| 内核 | `6.10.14-linuxkit`（Docker Desktop LinuxKit VM on WSL2） |
| CPU / cgroup | 22 logical CPUs；无 CPU quota；无 memory limit |
| capability | `cap_perfmon`、`cap_sys_ptrace`（与 compose `cap_add` 一致） |
| PMU | **不可用**（无 `cpu` event source） |
| Affinity | target CPU 0；辅助角色分离 |
| image id | `sha256:6eb20770ab231c3a9e270b63c469fe12356d1d99f51b991f34d6ba65d88f0d52` |

**构建前缀**（均为独立 prefix，不继承主 `ws/install`）：

| 前缀 | 用途 | 构建类型 | 功能测试 |
| --- | --- | --- | --- |
| `/tmp/alien-c2-prod-20260729-v9` | bounded/expanding 正式矩阵 production | RelWithDebInfo | 九包 `266/0/0/0` |
| `/tmp/alien-c2-stage-pair-20260730-v11` | stage 探针配对（production + stage） | RelWithDebInfo | production `266/0/0/0`、stage `47/0/0/0` |
| ASan/LSan prefix | sanitizer 证据 | `-O1 -g -DNDEBUG` + frame pointer | 九包 `266/0/0/0`（`detect_leaks=0`） |

**v9 与 v11 的可比性**（机器比对确认）：两者的 `ws/src` **完全相同**——tracked diff
逐字节一致（`40b4e1bd…`），untracked `ws/src`（23 文件）内容 `diff -r` 无差异。
身份差异**仅来自 `scripts/`** 的分析侧代码，不进入被测 ELF。因此 v9 与 v11 测的是
同一份被测源码，数字可比；ELF SHA 不同仅因构建非比特级可复现。

`scripts/profile-local-map.sh:97` 确认 paired-identity 门只作用于 `stage-latency`
模式，`plain-sample` / `heaptrack` 不要求——这是两个前缀分工的依据。

---

## 3. 固定 workload 与有效性门

workload 由 `perception_profiling` 包冻结：sequence、pose、stamp、隧道横截面、
beam/range/resolution 全部固定，10 Hz。bounded 收敛到 plateau，expanding 持续扩张
（实测约 1130 cells/revision）。

每个 run 必须同时满足（任一不满足即 `valid=false`，raw 保留但不计入基线）：

- 参数、graph、endpoint 门；**明确拒绝** `scan_accumulator`、`cloud_map`、
  `/scan_returns`、`octomap_builder`（避免旧 500k FIFO 制造假平台）
- 正式窗按 unique stamp 与 revision delta 计数，不把 heartbeat 当 commit
- observation/revision 计数偏斜门
- 每 100 revisions 与 oracle 的 counts/bounds 精确一致
- epoch / fingerprint / stamp / revision 连续性
- checkpoint sampler 与目标 smaps/pidstat 的时间对齐（skew 门）
- 角色 PID/starttime/affinity、退出码、`normal_completion`
- workspace closure provenance 与（stage 模式）paired source identity

**这些门在本轮真实拒绝过坏证据**，记录见 §9。

---

## 4. bounded 正式基线

前缀 `/tmp/alien-c2-prod-20260729-v9`（stage-latency 除外，见下）。

### 4.1 吞吐与 CPU

| 项目 | 值 |
| --- | --- |
| 正式窗口 | 600.0 s（perf-stat run） |
| observation | 6001（10 Hz 精确） |
| `final_known` | 230 520 |
| pidstat 600 s CPU 均值 | **约 13.3%**（±30%，见 §0） |

O(1) `known_bounds` 修复前的同负载校准值约 35%，修复后约 13.3%——2.6 倍差异远超
污染幅度，是可下结论的效应。

### 4.2 内存（三轮独立 300 秒 plain sample）

| Run | RSS | PSS | USS | RSS 斜率 | PSS 斜率 |
| --- | ---: | ---: | ---: | ---: | ---: |
| plain1 | 62 484 KiB | 44 586 KiB | 39 884 KiB | 0 | 0 |
| plain2 | 62 484 KiB | 44 586 KiB | 39 884 KiB | 0 | 0 |
| plain3 | 62 484 KiB | 44 586 KiB | 39 884 KiB | 0 | 0.097 KiB/min |

三轮全程平坦，斜率远低于 `1024 KiB/min` 阈值，
`suspected_sustained_growth=false`。三轮趋势聚合
`/tmp/alien-c2-plain3-trend-20260729-v9.json`（exit 0）。

### 4.3 阶段延迟（300 秒，v11 stage 前缀）

run `/tmp/alien-c2-formal-stage300-20260731-v11`，`valid=true`，窗口
300.092 s，observation 3001 / revision 3002（rev 805→3806），10 Hz 精确，
`final_known=230520` 与本节一致，`map_epoch=1`。full event set，各阶段 3000 样本。

| 阶段 | p50 | p95 | p99 | max |
| --- | ---: | ---: | ---: | ---: |
| callback 总 | 11.887 ms | 13.833 ms | 15.644 ms | 20.231 ms |
| ├ mapper_apply | 5.946 ms | 7.125 ms | 7.935 ms | 10.065 ms |
| └ snapshot_total | 5.819 ms | 6.970 ms | 8.121 ms | 11.018 ms |
| &nbsp;&nbsp;└ snapshot_serialization | 5.623 ms | 6.749 ms | 7.860 ms | 10.726 ms |
| state_publication | 27.99 µs | 51.30 µs | 150.17 µs | 480.82 µs |
| read_transaction | 2.53 µs | 3.32 µs | 4.27 µs | 67.56 µs |

分解自洽：`mapper_apply.p50 + snapshot_total.p50 = 11.765 ms ≈ callback.p50
11.887 ms`。callback p99 占 100 ms 周期 15.6%，无 backlog。
`snapshot_serialization` 占 `snapshot_total` 的 96.6%。

> 绝对值带 ±30–46% 漂移；**阶段之间的比例可信**（同 run 同时刻测量）。

### 4.4 堆（Heaptrack 300 秒）

| 项目 | 值 |
| --- | --- |
| peak heap | 28.96 MB |
| 分配调用 | 120 836 次（temporary 42.95%） |
| exit 期 "leaked" | 455.54 KiB |

### 4.5 错误与泄漏

- **ASan 60 s** / **LSan 60 s**：均正常退出、报告可解析，无 AddressSanitizer /
  LeakSanitizer error 或 leak summary。
- **Memcheck**：无法在冻结的 10 Hz 负载下维持吞吐（实测 3.24 vs 10 rev/s），
  改用测试套件负载取其独有能力，结果为 **0 errors、仅 still-reachable**。
  残余缺口"证据来自测试负载而非长跑负载"如实记录。
- **Massif**：同为 Valgrind 吞吐阻断；堆形状证据由 Heaptrack 替代。
- 退出期 455.54 KiB 与 LSan/Memcheck 交叉后**不定性为 C2 业务泄漏**；
  rcutils/glibc dlopen 与 LTTng TLS 的归因由独立 finding 跟踪。

### 4.6 perf record 热点（受限交付）

`>=120 s`、`>=1000` samples、0 lost 均达标，但 **unknown 44.25%**（门为 `<=20%`），
系 ROS 侧 octomap 无符号表的结构性限制。**门未放宽，本项不计为通过。**
可解析部分的热点信号：`updateInnerOccupancyRecurs` 约 30%、`liboctomap` 约 37%。
符号化由独立 finding `07-30-c2-perf-octomap-symbols-blocker` 跟踪。

注：本机无 PMU，perf record 使用软件 `cpu-clock` 采样，仅支持热点**排序**，
不支持周期级归因。§5.3 的阶段分解给出了更精确的归因。

---

## 5. expanding 正式基线

### 5.1 三轮独立 200 秒 plain sample

`/tmp/alien-c2-formal-exp-plain{1,2,3}-20260731-v9`，均 `valid=true`，
`capacity=covered`，`segmented_evidence_available=true`，三个预注册互斥桶
pre `[141,340]` / crossing `[341,540]` / post `[541,740]` 齐备，
`crossing_revision=440`（与 oracle 预测一致）。

| 指标 | plain1 | plain2 | plain3 |
| --- | ---: | ---: | ---: |
| final_known / free / occupied | 2 377 520 / 2 334 388 / 43 132 | 同左 | 同左 |
| known_delta | 2 147 000 | 2 147 000 | 2 147 000 |
| **RSS / 每新增 known 体素** | **121.53 B** | **121.95 B** | **121.41 B** |
| PSS / 每新增 known 体素 | 121.43 B | 121.85 B | 121.30 B |
| RSS / revision | 134.11 KiB | 134.57 KiB | 133.98 KiB |
| RSS peak | 307.3 MiB | 307.2 MiB | 306.9 MiB |

三次独立 run 的最终计数**逐位相同**，证明扩张过程确定性且与 oracle 对齐。

### 5.2 Heaptrack 100 秒与 heap 成本

`/tmp/alien-c2-formal-exp-heaptrack-20260731-v9`，`valid=true`，
heaptrack 与 massif 时间线两个 gate 均 `gate_pass=true`、`target_verified=true`。

| 项目 | 值 |
| --- | --- |
| runtime | 114.05 s |
| final_known / known_delta | 1 247 520 / 1 017 000 |
| peak heap | **122.08 MB** |
| peak RSS with overhead | 194.83 MB |
| 分配调用 | **8 339 973** 次 |
| temporary allocations | 42 290（**0.5%**） |
| Massif 兼容时间线 | 10 082 snapshots，peak 于 112.945 s |
| exit 期 leaked | 455.54 K |

**每新增 known 体素成本，跨独立工具交叉验证：**

| 来源 | 每体素 |
| --- | ---: |
| plain1/2/3 RSS | 121.53 / 121.95 / 121.41 B |
| plain1/2/3 PSS | 121.43 / 121.85 / 121.30 B |
| plain1 USS | 121.34 B |
| Heaptrack run RSS | 122.55 B |
| **Heaptrack peak heap** | **125.87 B**（含基线堆，故略高） |
| 分配调用 / 新增体素 | 8.20 |

smaps 采样与 Heaptrack 两条独立路径落在 **121–126 B**，离散度约 1%。

> 对照：CPU 指标的跨轮离散度是 30%。两者相差两个数量级，实证了 §0 的指标分级。

**正常扩张未被判为泄漏**：expanding run 不输出 `suspected_sustained_growth`
（该启发式只对 bounded 生效）；泄漏结论仅取 exit 期固定的 455.54 K，且该值与
bounded 完全相同、与地图规模无关。

### 5.3 阶段延迟（150 秒）与容量拐点机制

`/tmp/alien-c2-formal-exp-stage150-20260731-v11`，`valid=true`，
`final_known=1 812 520`，`capacity=covered`，三桶齐备。

| 阶段 | bounded p50 | expanding p50 | bounded p99 | **expanding p99** |
| --- | ---: | ---: | ---: | ---: |
| callback 总 | 11.887 ms | 51.202 ms | 15.644 ms | **92.977 ms** |
| ├ snapshot_serialization | 5.623 ms | 28.577 ms | 7.860 ms | 55.365 ms |
| └ mapper_apply | 5.946 ms | 21.788 ms | 7.935 ms | 39.143 ms |
| snapshot_total | 5.819 ms | 28.966 ms | 8.121 ms | 55.825 ms |
| state_publication | 27.99 µs | 51 µs | 150 µs | 160 µs |
| read_transaction | 2.53 µs | 3 µs | 4.27 µs | 5 µs |

（bounded = 230 520 cells，expanding = 1 812 520 cells）

**结论：1.81M cells 时 callback p99 已占用 100 ms 周期的 93%。**

标度关系（两点）：cells 增长 7.86 倍时，`snapshot_serialization` 增长 **5.08 倍**、
`mapper_apply` 增长 **3.66 倍**——**序列化是主导项且增长更快**。

### 5.4 容量拐点

首次 300 秒 expanding run 观测到的速率曲线（该 run 因掉队被正确判 invalid，
raw 保留为 `/tmp/alien-c2-invalid-exp-plain300-20260731-v9`）：

| revision | 估算 cells | 速率 (rev/s) | RSS |
| ---: | ---: | ---: | ---: |
| 200–2300 | 226k–2.60M | 9.98–10.04 | 61.4 → 338.3 MiB |
| 2400 | 2.71M | 9.83 | 353.4 MiB |
| 2600 | 2.94M | 8.93 | 378.7 MiB |
| 2900 | 3.28M | 7.95 | 423.5 MiB |

内存全程线性增长、MemAvailable > 6.7 GiB、无 oom——**不是内存阻断**。
结合 §5.3，机制清楚：**回调预算被序列化与 apply 吃满**。

该机制做出并验证了一个预测：expanding Heaptrack 在 200 s / 150 s 下三次失败，
是因为 1.81M cells 时回调已用掉 93 ms，Heaptrack 的 10–40% 扰动直接把回调推出
100 ms 预算；据此预测"缩到约 1.0M cells 即可通过"，实测**一次通过**。

> **容量表述纪律**：约 2.6M cells 是**本机观测值**。10 Hz 可持续性是保守指标，
> 无争用环境下拐点只会更靠后。**不构成产品的绝对容量承诺。**

归因与优化方向由独立 finding `07-31-c2-expanding-capacity-knee` 跟踪。

---

## 6. capacity ramp

`/tmp/alien-c2-capacity-ramp-20260729-v9`：`valid=true`、`capacity_status=covered`、
end revision 760，bounds **7/7 精确一致**。

首次 ramp（O(1) 修复前）曾在 revision 700 出现 production/oracle bounds mismatch，
归因为 `known_bounds()` 的 O(N) 扫描饱和目标核、饿死 pose 订阅、前沿滞后 1 voxel
（finding `07-29-c2-capacity-bounds-phase-lag`）。修复任务
`07-29-octomap-backend-o1-known-bounds` 完成后重跑验收通过，尾段 CPU 由 99–101%
降至 37.9%。

---

## 7. stage 探针扰动校准（受限交付）

**延迟侧（`callback` vs `full`，同一 ELF `d5be6ef1…`，仅 tracepoint 集合不同）**

四轮 120 秒证据，按原判据逐轮评估，全部通过：

| 轮次 | Δp50 | 阈值 | Δp95 | 阈值 | Δp99 | 阈值 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| r7 | +0.370 ms | 0.988 | +0.347 ms | 1.186 | +0.520 ms | 1.257 |
| r8 | +0.203 ms | 1.065 | +0.119 ms | 1.226 | +0.625 ms | 1.298 |
| r9 | −0.194 ms | 1.211 | +0.032 ms | 1.603 | +0.986 ms | 1.832 |
| r10 | −0.192 ms | 1.269 | +0.131 ms | 1.510 | +0.012 ms | 1.742 |

**表述纪律**：可陈述为"**本轮条件下通过原门**"，不可陈述为无条件成立——该门被同一
漂移机制污染（callback p99 基线跨轮由 12.569 漂到 18.323 ms，+46%），仅因阈值为
10% 相对（CPU 门为 5%）而容差加倍；更早的 v8 首轮正式测量的 p99 即曾以
2.815 ms > 2.309 ms 失败。

**CPU 侧（`unprobed` vs `full`）——本机结构性不可判定**

判定依据（三条）：

1. **无对争用不变的替代指标**：硬件 PMU 不可用。
2. **指标被污染**：四轮 workload 逐位相同，实耗 CPU 时间却相差 30%；待分辨的
   探针开销约 3% 相对量，比污染低一个数量级。
3. **无法交错采样**：`unprobed` 用 ELF `976d6af6…`（stage option OFF），
   `callback`/`full` 用 `d5be6ef1…`（ON）——不同二进制，两侧必然相隔约 5 分钟、
   必然采样到不同宿主状态。

任何统计量修正（中位数、配对、增大轮数、更严规则）都无法解决——**问题在指标不在
统计**。同一批 12 组数据，选 r7/r8/r9 聚合得 0.1415 pp（通过），选严格同序的
r7/r9/r10 得 0.9501 pp（失败）；两个结论都不成立。

**可支持的结论边界**：轮内配对差为 `−0.2999 ~ +0.9501 pp`（基数 11–14 pp），
**符号会翻转、无系统性方向**。可陈述为

> 未观察到系统性探针 CPU 开销，上界约 1 pp

**不可**陈述为"通过 `max(5% 相对, 0.5 pp)` 门"。阈值公式、workload、affinity、
event set 全程未修改。

要真正判定，需要有 PMU 的原生 Linux 专用机；不在本任务范围。

---

## 8. 当前限制与已知未解决问题

> **本轮所有相关 finding 任务已于 2026-07-31 关闭并归档**，其中部分**未解决**。
> 下表的"状态"列如实区分"已解决"与"未解决且不再跟进"——**未解决项不会有人接着
> 做**，将来若需要，请按"若要继续"一列重新立项。

| 限制 | 性质 | 状态 | 若要继续 |
| --- | --- | --- | --- |
| 探针 CPU 开销门不可判定 | 环境结构性（无 PMU + 30% 污染） | **未解决**，受限交付（见 §7） | 需换有 PMU 的原生 Linux 专用机 |
| perf record unknown 44.25% | ROS octomap 无符号表 | **未解决**，本项未计为通过 | 需重打包带符号的 octomap；但阶段分解已提供更精确归因，价值有限 |
| expanding 约 2.6M cells 容量拐点 | **production 性能特性** | **机制已定位，量化未完成** | 补中间点定标度指数；确认序列化是全量还是增量 |
| Memcheck 退出期 definite/possible loss 归因 | rcutils/glibc dlopen、LTTng TLS | **未解决**，不定性为业务泄漏 | 量级固定且与业务无关，优先级低 |
| Massif / Memcheck 无法维持 10 Hz | Valgrind 10–50× 扰动 | **已解决**（补偿方案见 §4.5） | — |
| bounds 前沿滞后 | O(N) known_bounds | **已修复**（提交 `eb78d46`） | — |
| stage 配对构建等价性 | 传递依赖 provenance | **已解决**（v11 pair 交付） | 注意 CRLF 规范化会作废 v11 身份，需重建 pair |
| p99 尾差归因 | run-to-run variance | **归因成立，但其建议的中位数判据已被推翻**（见 §7） | 勿照搬该判据 |

**未闭合的测量链缺口（如实记录）**

- **sanitizer 报告门无故障注入覆盖**。门实现于
  `scripts/profile-local-map.sh:1411-1414`（grep
  `ERROR:/SUMMARY: (AddressSanitizer|LeakSanitizer)` → `invalidate`），
  另有 `:459-461` 要求目标链接 `libasan`。C1 的 19 个与 C2 的 41 个合成测试覆盖了
  perf-control / perf-stat / perf-record / heaptrack / massif / memcheck /
  massif-timeline 的报告门，**唯独没有 sanitizer**。ASan 此前唯一的真实拒绝来自
  checkpoint sampler 门而非 sanitizer 报告门；LSan 无拒绝记录。
  **影响**：§4.5 的"无 ASan/LSan 错误"结论依赖一个未被反例验证的门。
  建议后续以合成测试闭合（与其他工具门同一模式），本轮未做。
- **未运行代码审核 subagent 复核**（原计划要求 `0 Blocking / 0 High`），本轮未做。

**优化方向（本轮未做，也无人接手）**

`snapshot_serialization` 是延迟主导项且随地图规模增长最快（cells 增长 7.86 倍时
它增长 5.08 倍，而 `mapper_apply` 只增长 3.66 倍），是 C3 之前最值得优化的目标。

**尚未验证的关键问题**：它每周期做的是**全量序列化还是增量**？若为全量且地图
单调增长，则属设计层面的每周期 O(N)，增量化是明确方向。这一条**没有被验证过**，
相关 finding 已关闭，**当前无人跟进**。

若要推进，建议顺序：先补中间规模的 stage-latency 点位确定标度指数 → 读
`OctoMapSnapshotBridge` 确认序列化粒度 → 再决定是否值得增量化。

---

## 9. 有效性门的真实 fail-closed 记录

本任务执行期间，门在真实条件下正确拒绝坏证据的记录（均非人为构造）：

| 模式 | 事件 | 触发的门 |
| --- | --- | --- |
| `plain-sample` | 300 s expanding 节点掉队 3.9%（obs 3115 / rev 2993） | observation/revision skew |
| `plain-sample` | 首个 60 s ASan workload 把 warmup 前历史 state 与当前内存绑定 | checkpoint sampler 绑定 |
| `heaptrack` | 200 s expanding，rev 2100 bounds 不符 | production/oracle bounds mismatch |
| `heaptrack` | 150 s expanding try1，skew 2.5% | observation/revision skew |
| `heaptrack` | 150 s expanding try2，rev 1600 bounds 不符 | production/oracle bounds mismatch |
| `stage-latency` | v10 分析侧代码在 pair 盖章后变更 | workspace closure provenance |
| `capacity-ramp` | 首次 ramp rev 700 bounds 不符（O(N) known_bounds） | production/oracle bounds mismatch |
| 构建 | 首轮 colcon 测试继承主 `ws/install` | install base 污染诊断 |

全部 invalid raw 按纪律保留，**无一放宽门后重收**。

---

## 10. 验收标准对照

| AC | 状态 | 依据 |
| --- | --- | --- |
| **AC1** 独立 profiling 与 sanitizer 构建可复现，Release 与 ASan prefix 全量功能测试通过 | **通过** | §2：production `266/0/0/0`、stage `47/0/0/0`、ASan `266/0/0/0` |
| **AC2** bounded 与 expanding 通过全部有效性门；graph 排除旧 accumulator/cloud_map；capacity 跨越 500k 后继续增长 | **通过** | §3、§5.1（`covered`、三桶齐备、`crossing_revision=440`）、§6（bounds 7/7） |
| **AC3** 延迟报告含各阶段分位数与样本数；正式窗口 ≥300 s；探针事件完整、扰动有界、无隐瞒积压 | **部分通过** | §4.3 满足 300 s + 各阶段 p50/p95/p99/max + 3000 样本；expanding 为 150 s（受容量安全窗口约束，计划原文允许）；**"扰动有界"仅延迟侧成立，CPU 侧不可判定（§7）** |
| **AC4** 同窗 perf stat/record 有完整 provenance、计数、符号化质量和热点报告 | **未通过** | §4.6：unknown 44.25% > 20% 门，未放宽，独立 finding 跟踪 |
| **AC5** 重复 plain sample 给出 CPU、RSS/PSS/USS、增长斜率与每新增 known voxel 成本；`covered` 时形成分段结论 | **通过** | §4.2（三轮斜率）、§5.1–5.2（121–126 B/体素，双工具交叉验证）、§5.1（三桶分段） |
| **AC6** Heaptrack/Massif/Memcheck/ASan/LSan 均有正常退出和可解析证据；finding 区分业务缺陷/still-reachable/profiler 开销 | **部分通过** | §4.4、§4.5、§5.2 有效；Massif/Memcheck 为结构性阻断并有补偿证据与残余缺口记录；退出期 455.54 K 明确不定性为业务泄漏 |
| **AC7** 生成面向 C3 的冻结 baseline 报告，明确可比较条件、限制与独立 finding；业务源码无未授权语义 diff | **通过** | 本报告；§0 可比较条件、§8 限制与 finding；C2 业务源码本轮未改 |
| **AC8** bounded 完整矩阵与 expanding 定向矩阵达到正式窗口/workload/退出完整性门；无空跑、强杀或不完整产物计为通过 | **部分通过** | bounded：perf-record 未达门（AC4）、Massif/Memcheck 阻断；expanding：plain 200 s、Heaptrack 100 s、stage 150 s，**均短于计划的 300 s**，原因是容量安全窗口（§5.4），偏离经用户批准并记录在案；**无任何空跑或强杀产物被计为通过** |

**总体**：AC1 / AC2 / AC5 / AC7 通过；AC3 / AC6 / AC8 部分通过并有明确记录；
AC4 未通过且有独立 finding。**没有任何未达门的项被记为通过。**

---

## 11. raw 证据位置与复算

正式 raw 写在容器 `/tmp`（仓库外），每个 run 含 `sha256sum.txt`。
关键摘要（各 run 的 `run-manifest.txt`、`sha256sum.txt`、`analysis-summary.json`、
checkpoint CSV、各聚合 JSON/quality）已导出到宿主机
`log/evidence-archive/`（151 份），容器丢失时仍可验证证据链锚点。

**bounded 正式 run**

`/tmp/alien-c2-formal-{perfstat,perfrecord,rostrace,heaptrack,plain1,plain2,plain3}-20260729-v9`、
`/tmp/alien-c2-formal-{asan,lsan}-20260730-v9`、
`/tmp/alien-c2-formal-stage300-20260731-v11`

**expanding 正式 run**

`/tmp/alien-c2-formal-exp-{plain1,plain2,plain3}-20260731-v9`、
`/tmp/alien-c2-formal-exp-heaptrack-20260731-v9`、
`/tmp/alien-c2-formal-exp-stage150-20260731-v11`

**capacity ramp**：`/tmp/alien-c2-capacity-ramp-20260729-v9`

**保留的 invalid 证据**（按纪律不删除，不计入基线）

`/tmp/alien-c2-invalid-exp-plain300-20260731-v9`、
`/tmp/alien-c2-invalid-exp-heaptrack200-20260731-v9`、
`/tmp/alien-c2-invalid-exp-heaptrack150-try{1,2}-20260731-v9`、
`/tmp/alien-c2-formal-perfrecord-20260729-v9`（unknown 超门）

**校准证据**（受限交付，双份保留不择优）

`/tmp/alien-c2-v11-cal-{unprobed,callback,full}-r{7,8,9,10}`、
`/tmp/alien-c2-stage-calibration-20260731-v11a3.json`（r7/r8/r9，`gate_pass=true`）、
`/tmp/alien-c2-stage-calibration-20260731-v11a4.json`（r7/r9/r10，`gate_pass=false`）、
逐轮 schema-2 诊断 `/tmp/diag-schema2-r{7,8,9,10}.json`

**复算方式**

```bash
# 单 run / 三轮趋势
python3 scripts/analyze-local-map-profile.py <run_dir> [<run_dir> ...] --output <json>
# stage 校准聚合
python3 scripts/analyze-local-map-stage-calibration.py \
    <unprobed> <callback> <full> [--callback-replicate ...] [--full-replicate ...] \
    [--unprobed-replicate ...] --output <json> --quality <txt>
```

报告中的每个数字均可由上述 raw 与命令重算。

---

## 12. 验收状态

**本基线可作为 C3 的比较基准**，但比较时必须携带 §0 的指标可信度分级：
内存与计数类结论可直接比较；CPU 与延迟绝对值只能在 30% 以上的效应量级上比较；
不要基于本报告做 IPC / cache 层面的推断。

冻结时间：2026-07-31。被测源码 revision `06a5d7fe`（含记录在案的工作区 diff）。

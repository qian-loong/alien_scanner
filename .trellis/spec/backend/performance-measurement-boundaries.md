# 性能测量能力边界（本仓库环境）

> **规则文档**：完整论证与实测证据见
> `docs/performance-memory-testing-cookbook.md` §6。本文件只列可执行的约束，
> 供后续会话直接遵守，避免重新论证。

---

## 环境事实

测量环境为 **Windows 宿主 + WSL2 / Docker Desktop 的 LinuxKit VM**
（内核 `6.10.14-linuxkit`）。两条结构性限制已实测确认：

1. **无硬件 PMU**。`/sys/bus/event_source/devices/` 无 `cpu` 项；
   `cycles / instructions / branches / cache-references / cache-misses`
   在 `perf stat` 中一律 `<not supported>`，root 亦不可得（与
   `perf_event_paranoid` 无关）。
2. **CPU 时间指标被争用污染约 ±30%**。实测：四轮 workload 逐位相同
   （各 1201 observations / 1202 revisions / 120.08–120.10 s），实耗 CPU 时间为
   12.966 / 13.986 / 14.806 / 16.897 秒。

## 分辨率下限

> **可分辨效应量级约 ≥ 30–50%；≤10% 的效应在本机不可分辨。**

## 禁止事项

- **禁止**新增预期效应 <10% 的 CPU 时间型 A/B 判定门。此类门在本机等价于掷硬币，
  无论用中位数、配对、增大轮数还是更严规则都无法挽救——问题在指标不在统计。
- **禁止**采集或分析 IPC、cache miss rate 及任何依赖硬件 PMU 的派生指标。
- **禁止**删除已有的 `<not supported>` 采集记录。它们是"检查过"的证据，
  删除后无法与"从未检查"区分。
- **禁止**在报告中给 CPU 或延迟绝对值写三位有效数字的精度暗示。

## 指标可信度分级

| 级别 | 指标 | 说明 |
| --- | --- | --- |
| 可信 | RSS / PSS / USS | 页面计数，与调度和争用无关 |
| 可信 | Heaptrack 分配次数、peak heap | 由代码路径决定，与时间无关 |
| 可信 | 精确功能计数（known/free/occupied、revision、observation、bounds） | 确定性 + oracle 逐点校验 |
| 可信 | 泄漏有无（ASan / LSan / Memcheck） | 确定性事实 |
| 可信 | 正确性与等价门（oracle join、fingerprint、epoch） | 确定性事实 |
| 可信 | **单 run 内**的阶段分解比例 | 同进程同时刻测量，争用等比影响，比值稳定 |
| 需标注 | 延迟绝对分位数 | 跨 run 漂移 ±30–46%，只能当量级用 |
| 需标注 | CPU 绝对值 | ±30%；仅当效应远超此幅度时结论才成立 |
| 需标注 | 10 Hz 可持续性 / backlog | **保守指标**：被争用宿主上成立，专用机只会更好 |
| 不可用 | PMU 硬件计数器、IPC、cache miss rate | 取不到数 |
| 不可用 | <10% 效应的 CPU 时间 A/B | 污染高于信号一个数量级 |

## 措辞纪律

- 大效应可下结论：`known_bounds` O(1) 改造（CPU 35% → 13.3%，2.6 倍）、
  Valgrind 吞吐阻断（3.24 vs 10 rev/s）均远超噪声，结论成立。
- 小效应只能给边界：如 stage 探针 CPU 开销，只可陈述
  "未观察到系统性开销，上界约 1 pp"，**不可**陈述为"通过 `max(5%, 0.5 pp)` 门"。
- 容量类结论必须标注为**本机观测值**并说明方向性（无争用环境下只会更高），
  不给绝对容量承诺。

## 需要突破时

需要 IPC / cache 分析或 <10% 效应判定时，唯一出路是**换有 PMU 的原生 Linux
专用机**。不要在本环境继续投入时间。

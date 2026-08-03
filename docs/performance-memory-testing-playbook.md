# 性能与内存测试实践手册（从零起步）

> ## ⚠️ 本文已冻结，不再更新
>
> 本文是 2026-07 那轮 C1/C2 性能与内存分析的**过程复盘**，含当时的试错、
> 被推翻的判据与版本教训。
>
> **现行的测量方法见 [`performance-memory-testing-cookbook.md`](performance-memory-testing-cookbook.md)**
> ——那是自包含的操作手册，不依赖本文。
>
> 保留本文有两个用途：一是让 `.trellis/tasks/archive/2026-07/` 下的任务记录中
> 引用的章节可追溯；二是保留"为什么某些做法行不通"的推导过程。
> **不要按本文的判据设计新的测量**（尤其是 §2.3 中已被证伪的中位数判据）。

> 本文从 2026-07-26 ~ 2026-07-28 的 C1/C2 性能与内存验证会话中倒推整理。
> 目标读者：没有系统做过性能剖析 / 内存测试的人。
> 目标产物：知道**为什么很多“测过了”其实无效**，以及**怎样开始测才算有效基线**。
>
> 相关已有材料：
> - C1 结果报告：[`docs/perception-resource-profiling.md`](perception-resource-profiling.md)
> - C2 结果报告：[`docs/local-map-resource-profiling.md`](local-map-resource-profiling.md)
> - 脚本入口：`scripts/profile-perception.sh`、`scripts/profile-local-map.sh`
>
> **任务状态（2026-07-31）**：C1/C2 性能与内存基线相关任务及其全部 finding 已
> 关闭归档至 `.trellis/tasks/archive/2026-07/`，其中部分**未解决**（见 C2 报告
> §8 的"已知未解决问题"表）。本手册保留为方法论沉淀，**不代表仍有人在跟进**。

---

## 0. 当前项目进度对照（2026-07-31 更新）

### 0.1 推进思路

性能/内存基线不是“直接跑几条 perf 看数字”，而是按这条链推进：

1. **先完成并归档 C2 功能**（authoritative local observation map，提交 `8dbdba5`）。
2. **独立开性能/内存任务**，只测不优化；发现热点/泄漏另立修复任务。
3. **复用 C1 已验证的 runner 合同**（PID 身份、workload 门、产物 SHA、正常退出）。
4. 为 C2 补齐确定性 fixture/oracle/sink、独立构建前缀和九种 runner mode。
5. 阶段延迟校准发现 **production 与 stage 依赖构建不一致** → 拆出 build
   equivalence 子任务，机器证明配对构建可比后才采信校准。
6. 正式校准 p99 单点失败 → 归因 finding 证明为运行间方差 → 经批准改用
   **多 run p99 中位数判据**（阈值公式不变）。**但该判据随后被证伪**：补跑严格
   同序轮次后，同一批数据换轮次即翻盘，归因为宿主 CPU 时间指标被争用污染
   （见 §2.8）。**CPU 侧最终以受限交付收口，延迟侧按原门逐轮通过。**
7. §9 expanding 矩阵首个 run 暴露容量拐点 → 立独立 finding，并用 stage 阶段
   分解定位机制（回调预算被序列化吃满），再据机制预测并验证 Heaptrack 可行窗口。

### 0.2 当前状态（§ 编号对应父任务 implement.md）

| 里程碑 | 状态 |
| --- | --- |
| §1–§4 测量基础设施（runner/analyzer/oracle/合成测试） | ✅ 完成 |
| §5 构建与短程质量门 | 🟡 实质收口；范围经批准收窄（弃用指标不再活体 smoke）；**剩 sanitizer 报告门无故障注入覆盖**一项未闭合 |
| §6 阶段延迟校准 | ✅ **受限交付收口**：延迟侧（callback vs full）四轮按原门全部通过；**CPU 侧本机结构性不可判定**，仅给出上界约 1 pp、无系统性方向 |
| §7 capacity ramp | ✅ 收口（v9）：`covered`、bounds 7/7 精确、尾段 CPU 37.9% |
| §8 bounded 正式矩阵 | ✅ 收口：perf-stat 600 s、ros-trace 60 s、Heaptrack 300 s、plain×3、ASan/LSan、stage-latency 300 s 全部有效；perf-record 受限交付（unknown 44.25%，未计通过）；Massif/Memcheck 结构性阻断并有补偿证据 |
| §9 expanding 矩阵 | ✅ 收口：plain×3（200 s）、Heaptrack（100 s）、stage-latency（150 s）全部有效；时长因容量安全窗口经批准缩短 |
| §10 报告 | ✅ `docs/local-map-resource-profiling.md`；**未运行代码审核 subagent**一项未闭合 |

**C2 bounded 基线核心数字**（v9/v11 前缀，均已过 fail-closed 门）：plateau
230 520 known voxels、10 Hz 精确、target CPU 均值 **约 13.3%**（±30%）、
RSS/PSS/USS 62 484/44 586/39 884 KiB 全程平坦（三轮斜率 0/0/0.097 KiB/min →
无疑似泄漏）、heap 峰值 28.96 MB、callback p50/p99 = 11.887/15.644 ms。

**C2 expanding 基线核心数字**：每新增 known 体素 **121–126 B**（smaps 与 Heaptrack
双路交叉验证，离散 1%）、10 Hz 可持续上限**约 2.6M cells**（本机观测值）、
1.81M cells 时 callback p99 达 92.977 ms（占周期 93%）、主导项
`snapshot_serialization` 增长快于 `mapper_apply`。

历史教训归档：v3（review 改 scripts 后身份失效）、v4/v5（fixed-domain 误伤、
untracked 收集过宽）、v6（首轮测试失败被覆盖、身份事后重盖、算法中途切换——
经独立审计降级为历史证据）、v7（驱动脚本 unbound variable）、**v10（分析侧
判据代码在 pair 盖章后才改 → stage 300 s 被身份门拒绝）**、**v11 尝试 1/2
（宿主噪声窗口内窄阈值门不可靠）**。最终 pair 为 **v11**，production 前缀为 **v9**。
两者 `ws/src` 经机器比对完全相同，数字可比。

**最重要的方法论教训**：判据不能在看过数据之后设计、再用同一批数据验证——那是
后验循环论证。发现指标本身被污染时，正确的做法是承认不可测并给出边界，而不是
换一个恰好能通过的统计量。

---

## 1. 新手先建立的三个原则

### 原则 A：测的是“被测进程 + 正式窗口”，不是整台机器热闹一阵子

常见错误：

- 看 `top` 里某个节点一会儿 20%，就当基线；
- 把 fixture（造数据）、sink（收数据）、profiler（测量工具）的 CPU/RSS 算进业务；
- 把启动、预热、退出清理也算进稳态。

正确做法：

- **明确 target PID**（本项目 C2 是 `perception_local_map_node`）；
- 其它角色单独记账，不并入 target 数字；
- 只统计 **t0–t1 正式窗口**内的样本；预热（warmup）与收尾另记。

### 原则 B：没有 provenance 的数字，默认不可信

一次有效 run 至少要能回答：

| 问题 | 为什么重要 |
| --- | --- |
| 测的是哪一版源码？ | dirty tree / 改脚本后旧 pair 全部失效 |
| 二进制 flags 是什么？ | Debug / `-O3` / 漏 frame pointer 会让热点和延迟不可比 |
| 依赖库来自哪？ | stage 用 overlay，core 却链到主 workspace `Release -O3` → 假对比 |
| 输入是否达标？ | 10 Hz 变成 3 Hz，延迟/内存都会“看起来更好” |
| 工具是否完整退出？ | SIGKILL、截断报告、lost samples → 不能当通过 |
| 产物能否复算？ | 没有 SHA / manifest，下次无法证明同一证据 |

本项目把这些写成 **fail-closed 门**：任一条件不满足，整次 run `invalid`，而不是“差不多能看”。

### 原则 C：先保证“测得对”，再谈“数字好不好”

性能任务的目标是 **可复现基线 + 可归因**，不是本轮把 CPU 压下去。  
发现热点/泄漏 → 另开 fix 任务；**不要为了更好看的数字改业务负载或阈值**。

---

## 2. 你需要分清的几类“测量”

| 类型 | 工具例子 | 回答什么 | 不回答什么 |
| --- | --- | --- | --- |
| 资源采样 | `pidstat` / `smem` / `smaps_rollup` | 正式窗 RSS/PSS/USS、斜率 | 谁分配了堆、是否业务泄漏 |
| CPU 统计 | `perf stat` | task-clock、占用率 | 函数级热点（硬件计数器在 LinuxKit 上常不可用） |
| CPU 热点 | `perf record` + 调用栈 | 哪类函数吃 CPU | 若采样不足/符号化差则无结论 |
| ROS 调度 | `ros2` LTTng trace | callback/take/publish 是否在跑 | 业务内部阶段耗时 |
| 阶段延迟 | UST begin/end 或 uprobe | callback / apply / publish 分位数 | 若与 unprobed 扰动过大则不可采信 |
| 堆分配 | Heaptrack | 分配次数、峰值、热点栈 | 退出期 “leaked” 汇总≠运行期泄漏 |
| 堆时间线 | Massif | 堆随时间变化 | 极慢，可能改变吞吐 |
| 内存错误 | ASan | 越界/UAF 等 | 不是性能数字；构建必须独立 |
| 泄漏检测 | LSan / Memcheck | 是否有 definite leak | 动态加载器/退出期噪声要交叉验证 |

**一句话**：没有一种工具能同时给出“又快又准又全”的结论；正式基线是 **工具矩阵 + 共同 workload 合同**。

下面按类别逐工具展开：探测什么、关键指标、指标能/不能说明什么、本项目的有效性门。

### 2.1 资源采样：pidstat / smaps_rollup / smem

**探测什么**：以固定间隔（本项目 1 秒）从 `/proc` 读取**目标 PID** 的 CPU 占用与内存占用快照。完全外部观察，对被测进程零侵入。

| 指标 | 含义 | 能说明 | 不能说明 |
| --- | --- | --- | --- |
| CPU%（pidstat） | 该进程在采样间隔内占用的 CPU 时间比例（单核=100%） | 稳态负载水平、探针/改动前后的整体 CPU 差异 | 时间花在哪个函数/阶段 |
| RSS | 进程驻留物理内存总量，**含共享库整页计入** | 粗量级、增长趋势 | 多进程共享时会高估“这个进程自己用了多少” |
| PSS | 按共享比例摊派后的物理内存（共享页 ÷ 共享进程数） | 多进程系统里最接近“该进程真实占用”的数字 | 谁分配的、堆内还是映射 |
| USS | 该进程独占、杀掉即释放的内存 | 泄漏判断的下界参考 | 同上 |
| 斜率（KiB/min） | 对正式窗内 RSS/PSS 样本做线性回归 | bounded 场景 plateau 后仍正斜率且三轮重复 → 疑似泄漏 | 单轮短窗的斜率无意义 |

**本项目门**：pidstat 采样必须紧贴正式窗（前后余量 ≤2 s）、样本只属于 target PID、样本数与时长匹配；checkpoint sampler 必须把内存快照与**时间合法的** map state 对齐（错配即 invalid）。

**C1 实测示例**：稳态 RSS ≈32 MiB 但 PSS ≈13 MiB、USS ≈11 MiB——差值主要是 ROS 共享库，说明只看 RSS 会高估单进程成本约 3 倍。

### 2.2 CPU 剖析：perf stat / perf record

**探测什么**：`perf stat` 用内核计数器统计整个正式窗的累计量；`perf record` 按频率采样 PC/调用栈，统计“哪些函数被抓到的次数多”。

| 指标 | 含义 | 能说明 | 不能说明 |
| --- | --- | --- | --- |
| task-clock | 进程实际获得的 CPU 时间 | 换算平均占用率（C1：600 s 窗 ⇒ 0.608%），与 pidstat 互为交叉验证 | 峰值/抖动 |
| 采样热点（top symbols） | 各函数占样本的百分比 | CPU 时间的**相对**去向（C1：3D 点搬运与 DDS/CDR 序列化最多） | 精确耗时；采样不足或符号缺失时无结论 |
| unknown ratio | 无法符号化的样本比例 | ≤20% 才可采信热点排名 | — |
| lost samples | 内核丢弃的样本数 | 必须为 0 或在门内，否则热点分布有偏 | — |

**注意**：本容器（LinuxKit）硬件 PMU 不可用，IPC/cache-miss 等硬件指标**不测**；热点可信的前提是 RelWithDebInfo + frame pointer（见 §3.2）。

### 2.3 阶段延迟：LTTng-UST 编译期 tracepoint（“探针”）

**探测什么**：编译进 `perception_local_map_node` 的静态打点（CMake 开关
`PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS`，production 默认
OFF）。每个处理阶段入口/出口各打一个纳秒时间戳事件，由独立 consumerd 异步
收集；entry/return 相减得到该阶段单次耗时。**不读扫描内容、不改业务逻辑。**

覆盖阶段：`callback`（整帧处理，最外层）、`apply`（射线证据写入占据地图）、
`state`（revision 状态发布）、`acquire`/`materialize`/`snapshot-total`
（revision-locked 读与可视化快照序列化）。两种 event set：**callback**（只开
最外层）与 **full**（全部嵌套探针）。

| 指标 | 含义 | 能说明 |
| --- | --- | --- |
| 每阶段 count/mean/p50/p95/p99/max | 该阶段耗时分布；p50=典型帧，p99=最慢 1% 帧（尾延迟） | 10 Hz 预算（100 ms/帧）的消耗与余量；C3 改动后哪个阶段变慢 |
| lost/unmatched/nesting/incomplete 等计数 | 事件配对完整性 | 全 0 才证明测量自身没丢数据 |

**校准合同（§6）**：探针数字可采信的前提是先证明探针自身无扰动。方法是三组
对比——unprobed（production OFF）/ callback-only / full——要求：

- CPU 门：full vs unprobed ≤ max(5% relative, 0.5 pp)
- 分位数门：full vs callback-only 的 p50/p95/p99 ≤ max(10% relative, 50 µs)

> ### ⚠️ 中位数判据已被证伪，勿照搬
>
> 曾经引入过"多 run 各取中位数"的判据（p99 用 schema-3、CPU 用 schema-4，
> 阈值公式不变），当时看起来解决了单点噪声问题。**2026-07-31 补跑严格同序轮次后
> 证明该判据不成立**：
>
> - 在宿主单调漂移下，各 mode 独立取中位数会**退化成"只评估时间居中的那一轮"**
>   （实测聚合值与中间轮的值逐位相同）。
> - 它**破坏了单轮内背靠背比较的配对设计**——而配对正是抵消漂移的机制。
> - 同一批 12 组数据，选 r7/r8/r9 得 0.1415 pp（通过）、选 r7/r9/r10 得
>   0.9501 pp（失败）。**两个结论都不能单独成立。**
>
> **根因不在统计量而在指标本身**：CPU 时间在本机被争用污染约 30%，待分辨的探针
> 开销约 3%，低一个数量级（见 §2.8）。加样本量、换统计量都救不了。
>
> **最终处置**：CPU 侧以受限交付收口——不称通过原门，只给出边界"未观察到系统性
> 探针 CPU 开销，上界约 1 pp、无系统性方向"。延迟侧四轮按原门逐轮通过，表述为
> "本轮条件下通过"，而非无条件成立。
>
> **通用教训**：
> 1. 阈值随被测程序变快而收窄，可能跌破环境噪声底。此时**先判断指标本身还能不能
>    分辨这个量级**，再谈统计方法。
> 2. **判据不能在看过数据之后设计、再用同一批数据验证**——那是后验循环论证。
> 3. 窄阈值门必须在宿主空闲时测（实测同代码同负载下，白天负载可使 p99 由 14 ms
>    涨到 25 ms、CPU 抬升 2 pp，足以让任何单点比较翻盘）。

  driver 应在开跑前静置等待（如 `load1 < 0.5`），并考虑轮内顺序对照平衡以消除
  "最后跑的 mode 吃最多漂移"的时序偏置。

### 2.4 堆分析：Heaptrack / Massif

**探测什么**：拦截 malloc/free（Heaptrack 用 LD_PRELOAD，低开销；Massif 用
Valgrind 虚拟机，极慢），回答“堆内存**是谁在哪个调用栈上分配的**”——这是
smaps 类采样永远回答不了的。

| 指标 | 含义 | 能说明 | 不能说明 |
| --- | --- | --- | --- |
| peak heap | 堆峰值 | 与 RSS 峰值互证；扩张场景的每 voxel 堆成本 | — |
| total / temporary allocations | 总分配次数、“分配后很快释放”次数 | temporary 高 → 分配热点（潜在优化点，另立任务） | 不是泄漏 |
| 分配热点栈 | 哪个调用栈分配最多 | 内存归因到代码 | — |
| “leaked”（退出时未释放） | 进程退出时仍持有的分配 | **≠运行期泄漏**：含全局单例、退出期未析构、工具自身；必须与 LSan/Memcheck 交叉 | — |
| Massif 时间线 | 堆随时间的快照序列 | bounded 应 plateau、expanding 应线性——形状对不对 | Massif 慢 10–100×，若压垮 10 Hz 吞吐则整 run 无效（见 throughput blocker） |

### 2.5 内存错误与泄漏：ASan / LSan / Memcheck

**探测什么**：内存**正确性**，不是性能。ASan 是编译期插桩（独立 `-O1 -g`
前缀，绝不与性能前缀混用），Memcheck 是 Valgrind 虚拟机逐指令检查。

| 工具/指标 | 含义 | 判读纪律 |
| --- | --- | --- |
| ASan 报告 | 越界、UAF、double-free 等 | 任何一条都是缺陷 → 立 fix 任务；无报告 + 正常退出才算通过 |
| LSan leak summary | 退出时不可达且未释放的分配 | C++ target 无摘要=通过；Python launch 宿主噪声按窄栈 suppress |
| Memcheck definitely lost | 指针全丢的分配 | **最强泄漏信号**，但必须看栈：本项目两例根在 glibc loader / `rcutils_load_shared_library` / LTTng TLS，无业务栈 → 归因 finding，不改 C2（`07-27-c2-memcheck-runtime-leak`） |
| indirectly lost | 随 definite 一起丢的子结构 | 跟随 definite 归因 |
| possibly lost | 只剩内部指针 | 弱信号，单独不定罪 |
| still reachable | 退出时仍有指针（全局/单例） | 通常不是缺陷 |

**吞吐前提**：Valgrind 类工具慢 10–100×，若 target 无法维持冻结的 10 Hz
workload，该 run 的一切结论（含泄漏报告）都不可采信——只能记录 blocker
（`07-27-c2-valgrind-throughput-blocker`），禁止降频造通过。

### 2.6 有效性体系：让上面所有数字可信的“元工具”

不产生性能数字，但决定数字是否可采信（原则 B 的机器化）：

| 机制 | 回答什么 |
| --- | --- |
| source identity（revision + diff + untracked 归一化归档的 SHA） | 测的是哪一版源码；scripts/ 变了旧 pair 即作废 |
| workspace closure digest（依赖/编译/链接/ELF/helper 的规范化清单） | 依赖真的来自声明的前缀；A/B 对比的两个构建除探针开关外完全等价 |
| run manifest（PID/starttime/affinity/窗口/工具版本/退出码） | 这次 run 的完整身份，任何字段缺失即 fail-closed |
| `valid` + `normal_completion` + sha256sum.txt | 单 run 是否达标、产物能否复算 |
| aggregator exit 0/1/2 | 0=过门，1=有效但阈值未过，2=证据结构/provenance 无效（数字禁止引用） |

**核心教训**（§4.2 详述）：三个单 run 各自 `valid=true` 不代表跨 run 对比
可采信——2026-07-28 的 gate_pass=false 就是 stage 目标误链主 workspace
`Release -O3` 依赖造成的假“探针 overhead”，closure digest 门补上后才暴露。

---

### 2.7 工具扰动量级与适用场景对照（观察者效应速查）

所有工具都有观察者效应，差别只在量级；某场景能不能用某工具，由"扰动是否破坏
负载合同"决定，并由 run 的机器门自动裁决（破坏即 invalid → 立 blocker）。

| 工具 | 扰动量级 | 10 Hz 实时管线下 | 备注 |
| --- | --- | --- | --- |
| pidstat / smaps 采样 | ≈0（外部读 /proc） | ✅ 无条件 | 只有粗粒度；**正式 CPU/内存基线只用它产出** |
| perf 采样 | 1–5% | ✅ | — |
| LTTng-UST 探针 | 亚微秒/事件 | ✅ | 不默认可信；CPU 侧开销在本机**不可判定**（见 §2.8），延迟侧可校准 |
| ASan | ~2× | ✅（bounded 实测保 10 Hz） | 独立前缀 |
| Heaptrack | ~10–40% | ⚠️ **取决于剩余预算**（见下） | bounded 300 s 通过；expanding 1.81M cells 处三次失败 |
| LSan | 仅退出期 | ✅ | — |
| Valgrind（Memcheck/Massif） | 10–50× | ❌（实测 3.24 rev/s vs 10） | 换场地取独有能力（见下） |

#### 能不能用，不是工具的固有属性

**判据是"工具开销 × 剩余预算"，不是"这个工具够不够轻"。**
同一个 Heaptrack，在两个场景下结论相反：

| 场景 | callback p99 | 占 100 ms 预算 | 剩余余量 | Heaptrack(+10~40%) |
| --- | ---: | ---: | ---: | --- |
| bounded（230k cells） | 15.644 ms | 16% | **84%** | 通过 ✅ |
| expanding（约 1.25M cells） | 约 70 ms | 70% | 30% | 勉强通过 ✅ |
| expanding（1.81M cells） | 92.977 ms | **93%** | **7%** | 必然超预算 ❌ |

C2 实测印证：expanding Heaptrack 在 200 s / 150 s / 150 s 三次失败
（分别触发 bounds mismatch、skew、bounds mismatch），**失败位置跟随窗口末尾而非
固定地图规模**；据此把窗口缩到约 1.0M cells 后**一次通过**。

**用法**：动手前先查当前规模下的 callback p99，算出剩余余量，再决定能上哪种工具。

#### 拖慢破坏真实性的三种方式

1. **数字直接失真**——测的是"代码 + 工具"，不是代码。
2. **代码路径变形，造成漏报**（最隐蔽）：走的是超时/丢弃/积压分支而非正常路径；
   并发交错被串行化，竞态类问题藏起来；跑不到正常稳态，规模相关问题不暴露。
   **不会制造误报**（错误判定与速度无关），但"clean 报告只对实际执行过的路径负责"。
3. **触发本不该触发的门**：C2 三次 Heaptrack 失败并非 C2 有缺陷，而是工具拖慢
   导致节点掉队、前沿滞后。**没有 fail-closed 门的话，这三次会产出看似正常、
   实则失真的数字。**

#### 但有一类量不受拖慢影响

关键区分：**被测量本身是否依赖时间**。

| 类型 | 例子 | 工具拖慢的影响 |
| --- | --- | --- |
| **时间无关量** | 分配次数、peak heap、known/free/occupied 计数、bounds、泄漏有无 | **不影响，数字仍然为真** |
| **时间相关量** | 延迟分位数、CPU%、吞吐 | **直接污染，结论作废** |

所以 Heaptrack 虽然慢 40%，它测出的 peak heap 与分配次数**仍然可信**——分配行为
由代码路径决定，不由执行速度决定。但拿 Heaptrack 去测延迟就完全无效。

C2 的交叉验证印证了这一点：每体素成本从 smaps（扰动≈0）得 121.5 B、从
Heaptrack（扰动 10–40%）得 125.87 B，**差 1%**——两者扰动差几十倍，但测的都是
时间无关量。

**拖慢对"正确性扫描"本身的影响方向**：不会制造误报（错误判定与速度无关），
但会造成**漏报**——路径覆盖变形（超时/丢弃/积压分支替代正常路径）、并发交错
被串行化（竞态类隐藏）、跑不到正常稳态（规模相关错误无从暴露）。因此
"clean 报告只对实际执行过的路径负责"。

**高实时场景下 Valgrind 的正确用法**：不硬塞实时合同，而是换到无实时约束的
确定性负载（如既有测试套件，放宽超时）上取其独有能力（未初始化读检测、
泄漏四级分级），并如实记录"证据来自测试负载而非长跑负载"的残余缺口。
每个替代/补偿方案必须逐子能力声明"能替代什么、不能替代什么"——工具是能力
捆，主能力被覆盖不等于全部被覆盖。

### 2.8 本机环境能力边界（哪些指标不可测，别浪费时间）

本仓库的测量环境是 **Windows 宿主 + WSL2/Docker Desktop 的 LinuxKit VM**
（内核 `6.10.14-linuxkit`）。该环境对指标可信度有**结构性**限制，下面两条是
实测证据，不是推测。

**证据 1：没有硬件 PMU。**
`/sys/bus/event_source/devices/` 只有
`breakpoint / kprobe / msr / power / software / tracepoint / uprobe`，**没有 `cpu`**。
`perf stat` 的 `cycles:u / instructions:u / branches:u / cache-references:u /
cache-misses:u` 一律返回 `<not supported>`。root 也取不到——这不是
`perf_event_paranoid` 权限问题，是虚拟机没有虚拟化 PMU。

**证据 2：相同工作量的 CPU 时间波动 30%。**
2026-07-31 的四轮 120 秒校准中，每轮 workload 逐位相同
（1201 observations / 1202 revisions / 120.08–120.10 秒窗口、同一 ELF、同一
bounded plateau），目标进程实耗 CPU 时间却是：

| 轮次 | 起始 | observations | 实耗 CPU 秒 |
| --- | --- | --- | --- |
| r7 | 00:45 | 1201 | 12.966 |
| r8 | 01:00 | 1201 | 13.986 |
| r9 | 01:14 | 1201 | 14.806 |
| r10 | 03:00 | 1201 | 16.897 |

同样的工作多花 30% 的 CPU 时间——这是宿主争用（缓存/内存带宽）与频率变化造成的
**指标污染**，不是随机噪声，加样本量无法消除。

#### 由此得出的分辨率下限

> **本机可分辨的效应量级约 ≥ 30–50%；≤10% 的效应不可分辨。**

大效应仍然可靠：`known_bounds` 的 O(1) 改造（CPU 35% → 13.3%，2.6 倍）、
Valgrind 吞吐阻断（3.24 vs 10 rev/s）都远超噪声，结论成立。小效应无解：
stage 探针开销约 3% 相对量，落在污染之下。

#### 指标三分类

| 分类 | 指标 | 处理方式 |
| --- | --- | --- |
| **放弃** | PMU 硬件计数器（cycles / instructions / branches / cache-misses）、IPC、cache miss rate | 取不到数，不要再尝试 |
| **放弃** | 预期效应 <10% 的 CPU 时间 A/B 对比（如探针开销门） | 再多轮次也是掷硬币，不要新增此类门 |
| **放弃** | `perf stat` 的 `task-clock` 绝对值 | 就是 CPU 时间，同样被污染 |
| **可信** | RSS / PSS / USS | 页面计数，与调度和争用无关 |
| **可信** | Heaptrack 分配次数、peak heap | 由代码路径决定，与时间无关 |
| **可信** | 精确功能计数（known/free/occupied、revision、observation、bounds） | 确定性 + oracle 逐点校验 |
| **可信** | 泄漏有无（ASan / LSan / Memcheck） | 确定性事实，非时序事实 |
| **可信** | 正确性与等价门（oracle join、fingerprint、epoch） | 同上 |
| **需标注** | 延迟绝对分位数 | 跨 run 漂移 ±30–46%，只能当量级用 |
| **需标注** | **单 run 内**的阶段分解比例 | **可信**——同进程同时刻测量，争用等比影响，比值稳定 |
| **需标注** | CPU 绝对值 | ±30%；仅当效应远超此幅度时结论才成立 |
| **需标注** | 10 Hz 可持续性 / backlog | 可信且**保守**：被争用的宿主都跟得上，专用机只会更好 |

#### 纪律

1. 不再新增预期效应 <10% 的 CPU 时间型 A/B 门。
2. 已采集的 `<not supported>` 记录**保留不删**——它们是"我们检查过"的证据，
   删掉之后无法与"从未检查"区分。
3. 报告里的 CPU 与延迟绝对值必须带不确定度，不写三位有效数字的精度暗示。
4. 需要 IPC / cache 分析或 <10% 效应判定时，唯一出路是**换有 PMU 的原生 Linux
   专用机**，不在本仓库当前范围内。

### 2.9 指标含义速查：每个数字到底回答什么问题

前面几节讲的是"用什么工具测"，这一节讲"测出来的数字是什么意思"。
表格里的示例值取自 C2 冻结基线（`docs/local-map-resource-profiling.md`）。

#### 内存类

| 指标 | 全称 / 含义 | 回答什么问题 | C2 实测 |
| --- | --- | --- | --- |
| **RSS** | Resident Set Size，进程当前驻留物理内存总量。**共享库等共享页会被完整计入**，多个进程各自都算一遍 | "这一个进程现在占了多少物理内存" | bounded 62 484 KiB |
| **PSS** | Proportional Set Size，独占页全算 + 共享页按共享进程数摊分 | "多个节点共存时，总内存该怎么加"——**唯一能相加不重复计数的口径** | bounded 44 586 KiB |
| **USS** | Unique Set Size，仅该进程独占的页 | "杀掉这个进程能立刻回收多少内存" | bounded 39 884 KiB |
| **peak heap** | 堆分配峰值（Heaptrack 视角，只看 `malloc/new` 一类） | "堆需求的上限"。与 RSS 的差额是代码段、栈、共享库、以及已释放但未还给 OS 的部分 | expanding 122.08 MB |
| **分配调用次数** | 窗口内 `malloc/new` 的总调用次数 | "分配器被敲了多少次"——次数高意味着分配器本身成为开销 | expanding 8 339 973 |
| **temporary 比例** | 分配后很快就释放的比例 | "有没有短命对象 churn"。比例高说明存在可以复用缓冲区来消除的浪费 | expanding 0.5%（很好）；bounded 42.95% |
| **RSS/PSS 斜率** | 单位时间的内存增长速率（KiB/min） | "有没有持续泄漏"。**只对 bounded（地图不再增长）场景有意义**，expanding 的增长是正常扩张 | bounded 0/0/0.097，阈值 1024 |
| **每新增 known 体素成本** | 窗口内内存增量 ÷ 新增已知体素数 | "地图每长大一格要多少内存"——**做容量规划的唯一依据** | 121–126 B |

三者关系恒为 `RSS >= PSS >= USS`。C2 的 `RSS − PSS ≈ 17.9 MB` 是与其他进程
共享的库；`PSS − USS ≈ 4.7 MB` 是摊到自己头上的那份共享。

#### 延迟类（分位数）

| 指标 | 含义 | 回答什么问题 |
| --- | --- | --- |
| **p50** | 中位数，一半的调用比它快 | "典型情况有多快" |
| **p95** | 20 次里有 1 次比它慢 | "常见的慢情况" |
| **p99** | 100 次里有 1 次比它慢 | **"接近最坏的情况"——实时系统真正该盯的数** |
| **max** | 窗口内观测到的最慢一次 | "最坏情况"，但单点易受偶发干扰，不单独下结论 |
| **mean** | 均值 | 参考价值最低：**会被尾部掩盖**。均值 12 ms 的系统完全可能每 100 次卡 90 ms |

**为什么实时系统看 p99 不看均值**：10 Hz 意味着每 100 ms 必须处理完一次。
p99 = 93 ms 的含义是"每 100 次回调里有 1 次要花 93 ms"——虽然均值可能只有
51 ms，但那 1 次已经吃掉 93% 的预算，再有任何扰动就会掉帧。

#### C2 的阶段分解（回调内部各段在干什么）

一次 observation 到达后，回调内部按顺序做这些事：

| 阶段 | 具体在做什么 | 这个数大意味着什么 | bounded p50 | expanding p50 |
| --- | --- | --- | --- | --- |
| **callback 总** | 从收到 observation 到本次处理全部结束 | 端到端成本；**必须 < 100 ms** 否则掉帧 | 11.887 ms | 51.202 ms |
| **mapper_apply** | 把这批 ray evidence 写进 OctoMap（真正的建图） | **建图本身的成本**，随地图规模增长 | 5.946 ms | 21.788 ms |
| **snapshot_total** | 生成对外快照的总耗时 | **"给别人用"的成本**，不是建图成本 | 5.819 ms | 28.966 ms |
| &nbsp;&nbsp;**snapshot_serialization** | 把地图序列化成 ROS 消息 | 占 snapshot 的 96.6%，**C2 的头号热点** | 5.623 ms | 28.577 ms |
| **state_publication** | 发布状态消息（revision、bounds、health 等） | 微秒级即正常 | 27.99 µs | 51 µs |
| **read_transaction** | revision-locked 读事务加锁读取 | 微秒级说明**并发锁设计没问题**，读者不会阻塞写者 | 2.53 µs | 3 µs |

关系：`callback ≈ mapper_apply + snapshot_total`（C2 实测 5.946 + 5.819 =
11.765 ≈ 11.887，差额是未插桩的胶水代码）。

**这个分解的价值**：它把"节点变慢了"拆成可行动的信息。C2 的结论是
**序列化（对外发布）比建图本身还贵，且增长更快**——所以优化方向是增量序列化，
而不是优化 OctoMap 写入。

#### CPU 与吞吐类

| 指标 | 含义 | 回答什么问题 | C2 实测 |
| --- | --- | --- | --- |
| **单核 CPU %** | 占用一个逻辑核的百分比（pidstat 口径） | "吃掉多少算力"。13.3% = 一个核的 13.3%，**不是整机的** | bounded 约 13.3% |
| **task-clock** | 进程实际消耗的 CPU 时间（ms） | 与上面同源，换算关系是 `task-clock ÷ 窗口时长` | — |
| **rev/s** | 每秒完成的地图修订次数 | **是否跟得上输入**。应恒等于 10 | 10.0（掉队时降到 7.95） |
| **observation vs revision 偏斜** | 收到多少条 vs 真正处理完多少条 | **积压量**。差值持续扩大 = 节点掉队 | 掉队时 3115 收 / 2993 处理 |
| **known / free / occupied** | 地图中已知 / 已知自由 / 已知占据的体素数 | 地图规模；`known = free + occupied` | bounded 230 520 |
| **known_delta** | 窗口内新增的已知体素数 | 扩张速度，用于算每体素成本 | expanding 2 147 000 |

#### 判定门类（不是性能数字，是"这次测量算不算数"）

| 门 | 在检查什么 | 不通过意味着 |
| --- | --- | --- |
| `valid` | 全部有效性检查的总结果 | 这次 run 的数字**不能用** |
| `normal_completion` | 是否正常收尾（无强杀、无角色提前退出） | 产物可能被截断 |
| **skew 门** | observation 与 revision 的计数偏斜 | 节点掉队了，负载合同已被破坏 |
| **bounds mismatch** | production 报告的地图边界 vs oracle 真值 | 对外报告的数据与真实地图不符（**正确性问题**） |
| **checkpoint skew** | 内存采样时刻与状态时刻是否对齐 | 内存数字可能绑到了错误的地图状态 |
| **closure provenance** | 被测二进制及其全部依赖的构建来源 | 测的可能不是你以为的那个构建 |
| **paired source identity** | 配对比较两侧的源码是否同一份 | 跨构建比较无效（v10 就栽在这） |

---

## 3. 从零开始：怎样启动一次“有效”测试

把下面当作检查清单。任一勾不上，就不要把结果写进基线文档。

### 3.1 环境

1. 在约定容器内测（本仓库：`alien-scanner-dev`）。
2. 镜像要 **真的带工具**，不要依赖“某次临时 apt 装过”的脏容器。  
   本会话教训：旧 image 文档写了工具，实际没装进 image；重建后才稳定。
3. 需要的能力（按工具）：
   - `SYS_PTRACE`（多数 attach 类工具）
   - `PERFMON` 或可用的 `perf` fallback 路径
   - 非交互的 LTTng / babeltrace
4. 记录：镜像 SHA、内核、RMW、CPU/cgroup、`perf_event_paranoid`、工具版本。

### 3.2 构建（最容易踩坑）

| 用途 | 推荐 flags | 说明 |
| --- | --- | --- |
| 性能/热点 | `RelWithDebInfo`：`-O2 -g -DNDEBUG -fno-omit-frame-pointer` | 有符号、可采样、接近发布优化 |
| 内存错误 | 独立 ASan 前缀：`-O1 -g -fsanitize=address ...` | **绝不能**和性能前缀混用 |
| 产品/日常 | 主 workspace 的 Release 等 | **不要**拿主 `ws/install` 冒充 profiling closure |

必须机器检查：

- 目标 ELF 的 compile 命令真有上述 flags；
- 有 `.debug_info` / 符号；
- build-id + SHA-256 写入 manifest；
- `ldd` 不链到未声明的主工作区 install。

本会话真实事故：

> stage target 自己是 stage 构建，但 `perception_core` 仍从主 workspace 链到 **Release -O3**；  
> production 则是独立 `/tmp` 的 **RelWithDebInfo -O2 + frame pointer**。  
> 三组 raw 单 run 都 `valid=true`，跨组校准却 **不可采纳**。

### 3.3 负载（workload）必须冻结

至少冻结：

- 频率（如 10 Hz）、不得用重复 stamp 注水；
- 传感器几何 / beam / range / resolution / seed；
- 场景是 **bounded（空间收敛）** 还是 **expanding（地图扩张）**；
- 正式窗时长与最低消息/revision 数。

本项目 C2 还要求：

- 权威输入由 **单一 deterministic fixture** 发布；
- scanner / C1 全链只做短程等价，不计入正式资源；
- graph 拒绝误接入旧 `scan_accumulator` / `/scan_returns` 等。

### 3.4 角色与 PID 合同

正式窗内持续检查：

- target / fixture / sink / oracle / profiler 的 **PID、starttime、是否 zombie**；
- 角色有独立 PGID；信号前重新核对身份，防止 PID 复用误杀；
- affinity：例如 target CPU0，辅助与采样 CPU1（共享 daemon 要披露，不强改全局）。

任一角色提前退出、身份漂移 → **整 run invalid**。

### 3.5 工具生命周期

好的 runner 不是“后台挂着再 pkill”：

1. 启动并确认身份；
2. warmup 到稳定（地图 plateau 或固定 revision）；
3. **同步打开** 正式窗（perf enable ACK 等）；
4. 到点 **同步关闭**；
5. 正常退出（接受约定退出码，如 perf 的 0/130）；
6. 解析报告完整性；
7. 清理残留 session/PID；
8. 写 manifest + 全产物 SHA。

本会话修过的典型故障：

- **perf controlled-stop 死锁**：只等 stop ACK 却不发计划内 SIGINT；
- **Heaptrack wrapper/target PID 模型错误**：把 wrapper 当业务进程；
- **sampler 错配**：把 warmup 前历史 state 接到当前 RSS → 假 invalid / 假结论；
- **LTTng `/dev/shm` 配置**导致 stage short smoke 失败；
- **`ros2 trace --list` 进入交互模式**阻塞自动化。

### 3.6 分析结果前先过 gate

最低机器门（示例）：

- `valid=true` 且 `normal_completion=true`
- 输入频率 / revision 达标
- 无 backlog、无角色崩溃
- profiler 报告段完整（perf enabled/running、Heaptrack 明细、Massif snapshot…）
- lost samples / unmatched pair / nesting 异常为 0（按模式）
- checksum 可复算

**短 smoke（10–60 s）只证明接线与合同**，不产生正式基线数字。  
**正式窗**（本项目设计：perf stat ≥600 s、Heaptrack ≥300 s、plain ×3 ≥300 s 等）才写入基线。

---

## 4. 会话中真实踩过的坑 → 怎么解决

下面按“现象 → 根因 → 处理”汇总，方便对照。

### 4.1 容器/工具层

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| 文档说有 perf/valgrind，跑起来没有 | 工具只在旧容器临时安装，没进 image | Dockerfile 固化工具；重建 image；记录 image SHA |
| `perf` 报 kernel tool mismatch | 容器内核与工具包版本不一致 | 使用 fallback：`/usr/lib/linux-tools/.../perf` |
| 硬件 PMU `not supported` | LinuxKit 限制 | 只用 `task-clock` 等软件事件；不计算 IPC/cache miss |
| `perf record` 样本极少 | 内核节流 / 采样策略不对 | 保留 frame pointer；改 fp call-graph 与频率；设最少 sample 门 |
| 外部 userspace probe 不可用 | 内核/权限不支持可靠 uprobe | 改用 **默认 OFF** 的 LTTng-UST 编译期 tracepoint fallback，不把容器改成 privileged |

### 4.2 构建与可比性

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| 测试“莫名”链到主 workspace | colcon 默认继承 `ws/install` | `env -i` + 只 source `/opt/ros` 与 pair install；显式 `--install-base` |
| stage vs production 延迟差不可信 | 传递依赖 flags/路径不一致 | **paired closure**：production underlay + stage overlay；dependency/helper digest 三组必须相同 |
| 审核改了 scripts 后旧 pair 作废 | source identity 覆盖 scripts | fail-closed；重建 v4 pair + 重跑短 smoke |
| symlink 逃出 closure | 证据路径指向声明根之外 | provenance 拒绝越界路径 |
| 干净环境 launch test 全挂 | 未设 `ROS_LOG_DIR` | pair-local 日志目录；与业务回归区分 |
| 空 output 目录被创建 | 参数校验失败仍 mkdir | 校验失败不得留半成品目录 |
| 旧 `gate_pass=true` 被复用 | analyzer 写到已有路径 | 禁止复用 aggregate 路径；exit 2 清理部分输出 |

### 4.3 Workload 与语义

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| 单帧 gtest 的 1.7 ms / 20 MB 当长期基线 | 混合测试进程启动历史 | 独立进程 + 长窗 + 重复 plain run |
| 20 s 真场景当内存增长结论 | 窗口太短 | 设计 ≥300 s plain，且 bounded 需 plateau 后拟合斜率 |
| 地图变大被当成泄漏 | 把 expanding 正斜率叫 leak | **拆两类负载**：bounded 看疑似泄漏；expanding 看每 voxel 成本 |
| 误接旧 50 万点上限 | legacy `max_points=500000` FIFO | graph 拒绝 accumulator；500k 只作 C2 known-voxel **里程碑**，不是 cap |
| `capacity=covered` 被误解为“这次已跑满三桶” | 字段表示计划可覆盖 | 与 `segmented_evidence_available` 分开 |

### 4.4 采样与对齐

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| RSS 对不上地图规模 | 用 topic 到达顺序或 OctoMap leaf 冒充 authoritative count | 用 `last_observation_stamp` join oracle exact counts |
| checkpoint 错配 | 历史 state + 当前 smaps | sampler 只绑定时间合法的 state；错配则 invalid |
| 可视化 snapshot 当权威地图 | OctoMap wire 无 epoch/revision | snapshot 仅 transport/可视化；权威在 local map state |

### 4.5 工具语义误读

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| Heaptrack “total leaked” 很大 | 含退出期/工具自身 | 与 LSan/Memcheck 交叉；不单凭汇总改业务 |
| Memcheck 64 B definite lost | 动态加载器 / `rcutils_load_shared_library` | 标为 ROS/loader 退出期 finding；无业务栈则不改业务 |
| Python launch_testing LSan 噪声 | 宿主 Python 对象 | 仅 suppress 含 `python3.x` 的宿主栈；C++ target 不吃这条 |
| Valgrind 下 revision 上不去 | 工具放大 10–100×，吞吐崩溃 | 单独 **throughput blocker** 任务；不降低业务 10 Hz 造通过 |
| stage probe 让 CPU 变差 | 探针扰动 | 先 unprobed vs full 的 CPU/频率门；callback-only vs full 的分位数门 |

### 4.6 自动化与残留

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| 二次 run 被旧 LTTng session 污染 | 未清理 | smoke 后检查无残留 PID/session |
| 并行构建让 flags/顺序难复现 | 多 worker 竞态 | 正式 pair：**单 worker**（`colcon sequential` + `gmake -j1`） |
| “Samples: 1K” 被解析成失败 | perf 缩写 | 用明细 sample 列精确求和 |

---

## 5. 推荐的新手上手顺序（本仓库可执行）

不要一上来跑 2 小时正式矩阵。按成本递增：

### Step 1：读合同，不改业务

- C1 报告：`docs/perception-resource-profiling.md`
- C2 PRD/设计：`.trellis/tasks/07-27-c2-performance-memory-baseline/{prd,design,implement}.md`
- 明确：**只测 `perception_local_map_node`，只测不优化**

### Step 2：静态门（秒–分钟级）

```text
# 在容器 /workspaces/alien-scanner 下（示意）
bash -n scripts/profile-local-map.sh
python -m py_compile scripts/analyze-local-map-profile.py
# 跑 C1/C2 合成测试（具体命令以当前 implement.md 为准）
```

目的：解析器、负例、provenance 拒绝逻辑是活的。

### Step 3：独立构建前缀（十分钟–数十分钟）

- production-like RelWithDebInfo closure（`/tmp/...`，不要主 `ws/install`）
- 需要时再 stage overlay（只重建 `perception_local_map` + instrumentation）
- ASan 另一前缀

检查 compile flags、ELF SHA/build-id、测试 `N/N` 全绿。

### Step 4：10–60 秒 smoke（只验证有效性）

每种 mode 至少一次短跑：

- plain
- perf-stat / perf-record
- ros-trace
- heaptrack
- 如启用：stage callback / full

通过标准：`valid=true`、`normal_completion=true`、频率门、无残留、checksum 过。  
**此时数字可以记在笔记里，但不要当正式基线。**

### Step 5：stage 可比性（已按此流程完成，留作范式）

1. 生成 paired production/stage，写入 closure digest；
2. 三组 10 s smoke：unprobed / callback / full；
3. dependency digest、helper digest、source identity 完全一致；
4. **用户授权后**再跑 120 s 正式校准 + aggregator；
5. p99 用 ≥3 组 run 的中位数判据（schema-3，见 §2.3）；单点 p99 在本环境
   受宿主机噪声主导，不可采信；
6. 只有 `gate_pass` 且无 mixed-build 时，才采信探针开销结论。
   （2026-07-29 实际结果：CPU 0.067 pp，p99 中位数差 1.32 ms < 2.33 ms，通过。）

### Step 6：正式矩阵（1.5–2 小时量级，需授权）

按设计单工具顺序跑，例如：

- bounded：plain×3、perf stat、perf record、ROS trace、Heaptrack、Massif、Memcheck、ASan/LSan、stage-latency
- expanding：plain×3 + 定向堆，并做 500k capacity ramp 前置门

任一次 invalid → 修 runner/环境后重跑该 mode，**不要放宽阈值**。

### Step 7：写结论时的措辞纪律

- 写清：构建、镜像、workload、窗口、valid 门、产物路径与 SHA
- bounded 正斜率且三轮重复 → “疑似泄漏（待 fix 任务）”
- expanding 正斜率 → “地图扩张成本”，**不叫泄漏**
- loader 退出期 64 B → “工具链 finding”，不写“C2 泄漏已确认”
- 探针 run 与 unprobed 超门 → “阶段延迟未完成/不可采信”

---

## 6. 最小心智模型：一次 run 的数据流

```text
冻结源码 identity
    ↓
独立构建 prefix（flags + 依赖闭包证明）
    ↓
启动 roles：fixture / target / sink / oracle / profiler
    ↓
warmup（地图收敛或固定预热 revision）
    ↓
正式窗 t0 ── 同步 enable ── 采样/计数 ── 同步 disable ── t1
    ↓
正常停止 + 报告完整 + 无残留
    ↓
analyzer 复算 gate → valid ?
    ├─ no  → 丢弃数字，修合同或环境
    └─ yes → 写入基线（仍带 provenance）
```

---

## 7. 常见问法速查

**Q: 我本地 `Release` 跑一下 top 能当基线吗？**  
A: 不能。缺独立 prefix、正式窗、角色分离和产物哈希。

**Q: 短 smoke 都绿了，是不是性能合格了？**  
A: 只说明 **测量系统可工作**。合格基线要靠正式时长与重复 run。

**Q: Valgrind 太慢，能不能降到 2 Hz 测泄漏？**  
A: 不能当本项目正式结论。降频改变了被测系统；应记 throughput blocker，或接受“该工具下无法维持合同频率”。

**Q: 为什么要单 worker 构建 pair？**  
A: 降低并行编译引入的难复现差异；正式证据优先可重复，不优先速度。

**Q: 旧 raw `valid=true` 为啥还要作废？**  
A: 单 run 有效 ≠ 跨构建可比。缺 closure provenance 或 source identity 已变，只能当历史，不能当新 gate 的通过证据。

---

## 8. 与本仓库脚本的对应关系

| 目的 | 入口 |
| --- | --- |
| C1 单节点资源剖析 | `scripts/profile-perception.sh` + `analyze-perception-profile.py` |
| C2 local map 剖析 | `scripts/profile-local-map.sh` + `analyze-local-map-profile.py` |
| C2 stage 三组校准汇总 | `scripts/analyze-local-map-stage-calibration.py` |
| 构建/依赖 provenance | `scripts/lib/local_map_build_provenance.py` |
| 公共 PID/报告解析 | `scripts/lib/profile-runner-common.sh`、`profile_report_parsers.py` 等 |
| 确定性负载包 | `ws/src/alien_perception/perception_profiling/` |

具体 CLI 参数以脚本 `--help` 与任务 `implement.md` 为准；本文强调 **合同与失效模式**，避免把过期命令抄成教条。

---

## 9. 一页纸：有效基线的充要条件

一次（或一组）结果可以写入“正式性能/内存基线”，当且仅当：

1. **功能已冻结**（或明确声明测的是哪一功能提交），性能任务不顺手改语义；
2. **独立构建** + flags/依赖/helper **机器可证明**；
3. **workload 冻结**且正式窗计数达标（频率、revision、epoch/fingerprint 等）；
4. **角色/PID/退出/产物 SHA** 全过；
5. **工具报告完整**，无 lost/unmatched/截断（按该 mode 合同）；
6. 需要对比的 A/B（如 unprobed vs full）共享同一 source/dependency/helper identity；
7. 结论区分：稳态泄漏 vs 扩张成本 vs 工具噪声 vs 探针开销；
8. 大块 raw 在仓库外，文档只留摘要 + SHA 引用。

不满足任一条：可以做实验笔记，**不能**当回归基线。

---

## 10. 文档维护

- 来源会话（只读参考，勿改）：  
  `C:\Users\loong\.codex\sessions\2026\07\26\rollout-2026-07-26T11-44-05-019f9c85-d8da-7882-a5cc-2066cb6e4061.jsonl`
- 整理日期：2026-07-28；2026-07-29 更新：§0 进度快照重写（v8 收口）、§2 扩为
  逐工具指标参考（2.1–2.6，含 p99 中位数判据）、§5 Step 5 标记完成。
- 若 build equivalence 或正式矩阵合同有变，应同步改 §0 / §5，并在任务 `implement.md` 留证据链接。

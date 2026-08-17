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

## 更新类型混杂检查

V1/V2 或不同 content-identity 实现的端到端 A/B，除了固定地图规模、操作数、频率、
warmup、窗口和 ROS domain，还必须固定 `keyframe`/`delta` 的组成。生产 producer 的
`max_delta_chain_length` 或周期 keyframe 参数可能在同一时间窗口内插入完整基线；如果
两组的 keyframe 数不同，payload、candidate、内存和 apply 成本就不再只反映算法差异。

- profiling fixture 必须在摘要中记录明确的 `keyframe_policy`，runner 对该字段做校验；
- bounded V1/V2 筛选若采用“每来源仅初始 keyframe”，应将 fixture 专用 chain limit 设为
  `0`，不改变生产默认 chain limit；
- 若两组总消息数相同但 keyframe 数不同，不能把差异解释为处理速度造成的 cadence；应先
  停止归因并统一触发策略；
- A/B 摘要必须同时报告 `messages_*`、`keyframes_*`、`deltas_*` 和最终 revision，
  keyframe 计数不一致的样本不得进入性能 Gate 聚合。

## 内存工具归因与严格门

### 1. 适用范围

当 ASan/LSan、Memcheck 或 Heaptrack 参与 rollout Gate 时，工具结论和归因必须分开记录。
第三方 runtime 的 retained allocation 可以解释来源，但不能静默覆盖工具的严格门结果。

### 2. 摘要字段

Memcheck 摘要至少保留：`target_verified`、`invalid_access`、`definite_lost_bytes`、
`indirect_lost_bytes`、`possibly_lost_bytes`、`still_reachable_bytes` 和 `gate_pass`。Sanitizer
摘要至少保留 `configured`、`enabled`、`report_files` 和 `gate_pass`。Heaptrack 必须同时记录
primary artifact、parse/target verification、peak heap 和工具自己的 leak 口径。

### 3. 契约

- `possibly_lost_bytes > 0` 时，严格 Memcheck `gate_pass` 必须保持 false，即使调用栈落在
  `liblttng-ust`、DDS 或其他第三方库；
- 严格工具门与业务资源门分列：只有在同一栈已被至少两次历史基线复现、调用栈不经过业务
  代码、业务 definite/indirect/invalid 均为零且 retained bytes 不随 workload 增长时，
  才能显式记录 `business_memory_gate=pass` 和固定的第三方例外 ID；该记录不修改原始
  Memcheck `gate_pass=false`；
- 报告必须另写 attribution，明确是业务代码、第三方 runtime 还是无法归因；
- Heaptrack 的 `total_memory_leaked` 不等同于 Memcheck 的 definitely lost，不能跨工具替换口径；
- 工具开销导致消息、revision 或最终 cell 不守恒时，该样本只能用于工具诊断，不能进入业务 Gate；
- “未发现业务 leak”与“严格内存门通过”是两个独立陈述，不得互换。

### 4. 验证与错误矩阵

| 条件 | 必须结果 |
| --- | --- |
| definite/indirect lost 或 invalid access 非零 | 业务 Gate 失败，修复后重跑 |
| only possibly lost，且栈在第三方 runtime，并满足历史复现/非业务栈/不增长条件 | 严格工具门仍失败；可显式记录第三方例外并通过 business memory gate |
| sanitizer report 为空但 workload 未完成 | 样本无效，不能写成 sanitizer Gate 通过 |
| Heaptrack parse 通过但 target 未验证 | 只保留 allocation 诊断，不纳入正式 Gate |
| full dependency closure 有非目标包错误 | 精确重跑受影响包并并列记录；不能伪称整个闭包通过 |

### 5. Good / Base / Bad

- Good：业务链完整，Memcheck definite/indirect 为 0、possible 为 768 B，调用栈归因第三方，
  且 C1/C2/C4 历史基线复现；报告同时写 `strict_memcheck_gate=false`、
  `business_memory_gate=pass` 和例外 ID。
- Base：第三方 possible-lost 尚未完成历史复现或 workload 相关性检查，business memory gate
  保持 pending，不得依赖猜测放行。
- Bad：看到第三方调用栈后手工把 `gate_pass` 改为 true，或用 Heaptrack leak 数替代 Memcheck。

### 6. 必需测试

- runner/parser 单测断言每个工具字段存在且数值解析正确；
- positive control 必须能让 invalid access 或 definite leak 使 Gate 失败；
- workload completion 断言消息、revision、最终 cell 和 target verification 守恒；
- C4/C5 等跨包链路必须把目标包精确结果与 dependency-closure 结果分列。

### 7. Wrong vs Correct

Wrong：`possible lost 来自第三方，因此 Memcheck passed`。

Correct：`strict_memcheck_gate=false；definite/indirect/invalid=0，possible=768 B，第三方 runtime 归因且历史可复现；business_memory_gate=pass，例外 ID 已记录`。

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

## Profiling 证据保留与迁移契约

### 1. 适用范围

当 task-local validation 同时包含 Git 可审计摘要和不应进入仓库的大体积 raw 时，或已归档
任务需要把本地 raw 移到 `profiling-archive/` 时，必须使用本节契约。

### 2. 路径与清单签名

- Git 摘要根：`.trellis/tasks/archive/<month>/<task>/validation/`。
- 本地 raw 根：`profiling-archive/<task-and-capture-date>/raw/`。
- raw 选择集合：从源 validation 根执行
  `git ls-files --others --exclude-standard -- <source-root>`，不得靠扩展名或目录名猜测。
- 树摘要行：`<lowercase sha256><two spaces><forward-slash relative path>\n`；按整行 ordinal
  排序、UTF-8 无 BOM 拼接后再计算 SHA-256。
- 可重放 analyzer：`analyzer --raw-root <archive-raw-root> --output-dir <git-summary-root>`；
  raw 输入根与 aggregate 输出根必须可独立指定。

### 3. 契约

- 任务目录只保留 README、aggregate、质量门和 provenance 等小型可审计摘要；完整 raw 保留
  在本地 archive，`raw/` 必须由 `/profiling-archive/*/raw/` 忽略。
- 移动前后必须分别计算文件数、总字节数和树摘要，三者全部相等才可声明迁移完成。
- 逐文件保持相对路径，拒绝目标覆盖，并验证解析后的源/目标路径仍位于各自固定根内。
- 不改写 aggregate、run manifest、`sha256sum.txt` 或 raw 中的采集期绝对路径；新增
  `relocation-provenance.txt` 映射 capture root、当前 raw root 和 Git summary root。
- 迁移后的 README/provenance 记录迁移日期、三项完整性指标、清单格式、路径保留声明和
  raw 未修改声明。
- 同一 raw 树和同一 analyzer 必须产生字节级一致的 aggregate。不得把 analyzer 执行时的
  当前时间写入 aggregate；确需时间身份时，使用 raw 中已冻结的 capture 时间或独立
  postprocessing provenance。迁移后必须直接从 archive raw 重跑，不能先复制 raw 回任务目录。
- 提交前必须证明 raw 的 tracked 数为 0，且 staged 路径不包含任何 `raw/` 或无关未跟踪文件。

### 4. 验证与错误矩阵

| 条件 | 必须结果 |
| --- | --- |
| 目标 raw 根已存在且非空 | 停止，不覆盖也不合并目录 |
| raw 数量、总字节数或树摘要任一不一致 | 迁移失败；按冻结清单回滚，不删除异常文件 |
| 源集合包含 tracked 摘要 | 停止；修正为 `git ls-files --others --exclude-standard` 的结果 |
| 目标文件未命中 ignore 规则或被 Git 跟踪 | 停止提交并修复布局/规则 |
| raw 内仍记录旧 task 绝对路径 | 保留原值；通过 relocation provenance 解释当前位置 |
| analyzer 只从脚本同级目录读取 raw，或每次重跑改变 aggregate SHA | 停止归档收尾；分离 raw/output 根并移除非确定性执行时间 |
| Windows PowerShell 的 `check-ignore --stdin` 首行受 BOM 影响 | 使用占位首行吸收 BOM，或逐条/无 BOM 输入复核；不得据此误判规则缺口 |
| 归档脚本把其他未跟踪 raw 纳入自动提交 | 立即从索引移除并核对文件仍在磁盘，修正提交后再继续 |

### 5. Good / Base / Bad

- Good：冻结 Git 未跟踪集合，迁移后从目标文件系统独立复算三项指标，并提交 README 与
  relocation provenance。
- Base：新的 profiling run 从一开始就直接写入被忽略的 archive raw 根，同时把摘要复制到
  task validation 并做哈希验证。
- Bad：整体移动 validation 目录、重写 JSON 中旧路径、只比较文件数，或使用 `git add -A`
  把邻近未跟踪证据一并提交。

### 6. 必需检查

- 比较迁移前后 raw file count、total bytes、tree manifest SHA-256。
- 比较迁移前后既有 tracked 摘要的 blob/hash，确认内容身份未变化。
- 从 archive raw 直接运行 analyzer 两次，断言 aggregate 字节/SHA 一致，并验证报告引用该
  SHA；镜像到 Git 的 run manifest 与 summary 逐文件对比 archive SHA。
- 执行 `git check-ignore` 覆盖每个目标 raw，并执行
  `git ls-files -- <archive-raw-root>` 断言输出为空。
- 执行 `git diff --check`，逐项审查 `git status --short`、`git diff --name-only` 和最终 staged
  路径；归档/自动提交后再次检查，不能假定脚本只提交当前任务目录。

### 7. Wrong vs Correct

Wrong：

```powershell
Move-Item validation profiling-archive/c3/raw
python validation/analyze.py  # 假定 raw 仍与脚本同目录，且写入当前时间
git add -A
```

Correct：

```powershell
$raw = @(git ls-files --others --exclude-standard -- $sourceRoot)
# 冻结 count/bytes/tree hash，逐文件拒绝覆盖地移动并保持相对路径。
# 从目标根独立复算三项指标后，只暂存 README、摘要和 relocation provenance。
python validation/analyze.py --raw-root profiling-archive/c3/raw --output-dir validation
# 固定 raw/analyzer 重跑时，aggregate SHA 必须不变。
git ls-files -- $archiveRawRoot  # 必须为空
```

## 存储布局 A/B 归因契约

### 1. 适用范围

当接收端在不改变 wire/content identity 的前提下比较 vector、chunked COW 或其他地图
存储布局时，使用本节。该对照只能衡量存储复制、索引、分配和遍历方式，不能把未来
Merkle/hash 改造的收益提前归入布局优化。

### 2. 命令与身份签名

```text
run_c4_resource_profile.py --source <absolute-elf> --receiver <absolute-elf>
  --storage-mode vector|chunked --chunk-edge 8|16|32
  --chunk-bucket-count <positive-int> --output-dir <new-dir> ...
perception_chunk_layout_estimator <sequence-count> <new-output-dir>
```

报告至少保存 receiver、source、runner 和 estimator 的 SHA-256，以及 build type、git
revision/tree state、workload 参数、ROS domain、PID/starttime 和每个摘要文件的 SHA-256。

### 3. 契约

- 归因基线固定为 `vector + flat SHA-256`，候选为
  `chunked COW + 同一 flat SHA-256`；两组必须使用同一 receiver/runner 二进制身份、
  wire 输入、构建类型、地图规模、operation 数、频率、warmup 和采样窗口。
- 阶段指标至少拆分 payload decode、candidate build、canonical hash、commit 和 callback；
  同时记录 copied cells、touched/shared chunks、copied bucket entries、candidate owned bytes、
  PSS/USS 与业务守恒。只报总 apply 不能证明 COW 优化了哪一阶段。
- edge 选择先用代表性三维 replay 的复制量与元数据，再看人工分散最坏场景，最后看一维
  兼容 workload。一维 `(x,0,0)` 连续更新会系统性偏向较大块，不能独立决定边长。
- 估算器必须让每行 workload 通过真实 `CellSnapshotStore::apply()` 做 actual-metric
  conformance；预测值不一致时不得继续用该报告选型。
- copied cells、candidate bytes、业务计数和页面内存用于确定性/容量结论；跨 run CPU 与
  延迟绝对值继续遵守本文件约 +/-30% 噪声边界。短 A/B 是筛选证据，不替代正式矩阵。
- 决策门失败时保留候选实现和原始证据，但生产默认不切换，也不得把停止投入的正式
  矩阵、Heaptrack、Sanitizer 或 Memcheck 写成“通过”。

### 4. 验证与错误矩阵

| 条件 | 必须结果 |
| --- | --- |
| 任一组 receiver/runner SHA、build type、wire/hash 或 workload 不同 | 停止比较，证据不可归因 |
| 报告 `valid=false`、消息/revision/hash/cell 不守恒或存在 reject/backlog | 不纳入 A/B |
| estimator 预测指标与实际 store apply 不一致 | 修复估算/观测定义并重新生成全部候选 |
| 只有一维 workload 支持某个 edge | 不能选型；补三维 replay 与人工分散场景 |
| flat hash 仍占主要时间且总 apply 未改善 | 如实判为布局端到端 no-go；Merkle 另建任务 |
| Gate 失败后未运行正式内存工具 | 明确标记未执行，不得标记通过或无泄漏 |

### 5. Good / Base / Bad

- Good：同一二进制连续跑 vector 与 `8/16/32`，逐组校验守恒，报告候选构建下降但 flat
  hash 上升，并据既定门给出 no-go。
- Base：短 A/B 只用于筛掉无收益候选；只有通过筛选后才投入 3 x 300 秒正式矩阵。
- Bad：拿历史 vector 数字与新 chunked 构建比较，或把 edge 8 的最低 copied cells 单独
  写成“最优”而忽略其 bucket/chunk 元数据。

### 6. 必需检查

- 对布局 estimator 的负坐标、块边界、revision-only、三维 replay 和分散最坏场景做
  确定性测试，并逐行对比实际 store metrics。
- 对所有 storage 运行相同业务守恒检查，并验证 `analysis-summary.json` 的身份与 SHA。
- 从 raw 重新计算阶段 mean、copied cells/delta、candidate bytes 和 PSS/USS；报告不得只
  复制终端文本。
- Gate 结论同时列出通过项、失败项和未执行项，生产默认只在明确通过证据后修改。

### 7. Wrong vs Correct

Wrong：

```text
历史 vector + 新 chunked/Merkle -> “COW 节省了全部时间”
edge 8 copied cells 最少 -> “edge 8 最优”
未运行 ASan/Memcheck -> “内存门通过”
```

Correct：

```text
同一 build：vector + flat SHA-256 -> chunked + flat SHA-256
复制量 + 元数据 + 三维/最坏场景 -> edge 折中候选
Gate no-go -> 默认仍为 Vector，正式工具明确记为未执行
```

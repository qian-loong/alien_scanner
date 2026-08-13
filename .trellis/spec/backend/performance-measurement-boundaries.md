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

### 3. 契约

- 任务目录只保留 README、aggregate、质量门和 provenance 等小型可审计摘要；完整 raw 保留
  在本地 archive，`raw/` 必须由 `/profiling-archive/*/raw/` 忽略。
- 移动前后必须分别计算文件数、总字节数和树摘要，三者全部相等才可声明迁移完成。
- 逐文件保持相对路径，拒绝目标覆盖，并验证解析后的源/目标路径仍位于各自固定根内。
- 不改写 aggregate、run manifest、`sha256sum.txt` 或 raw 中的采集期绝对路径；新增
  `relocation-provenance.txt` 映射 capture root、当前 raw root 和 Git summary root。
- 迁移后的 README/provenance 记录迁移日期、三项完整性指标、清单格式、路径保留声明和
  raw 未修改声明。
- 提交前必须证明 raw 的 tracked 数为 0，且 staged 路径不包含任何 `raw/` 或无关未跟踪文件。

### 4. 验证与错误矩阵

| 条件 | 必须结果 |
| --- | --- |
| 目标 raw 根已存在且非空 | 停止，不覆盖也不合并目录 |
| raw 数量、总字节数或树摘要任一不一致 | 迁移失败；按冻结清单回滚，不删除异常文件 |
| 源集合包含 tracked 摘要 | 停止；修正为 `git ls-files --others --exclude-standard` 的结果 |
| 目标文件未命中 ignore 规则或被 Git 跟踪 | 停止提交并修复布局/规则 |
| raw 内仍记录旧 task 绝对路径 | 保留原值；通过 relocation provenance 解释当前位置 |
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
- 执行 `git check-ignore` 覆盖每个目标 raw，并执行
  `git ls-files -- <archive-raw-root>` 断言输出为空。
- 执行 `git diff --check`，逐项审查 `git status --short`、`git diff --name-only` 和最终 staged
  路径；归档/自动提交后再次检查，不能假定脚本只提交当前任务目录。

### 7. Wrong vs Correct

Wrong：

```powershell
Move-Item validation profiling-archive/c3/raw
git add -A
```

Correct：

```powershell
$raw = @(git ls-files --others --exclude-standard -- $sourceRoot)
# 冻结 count/bytes/tree hash，逐文件拒绝覆盖地移动并保持相对路径。
# 从目标根独立复算三项指标后，只暂存 README、摘要和 relocation provenance。
git ls-files -- $archiveRawRoot  # 必须为空
```

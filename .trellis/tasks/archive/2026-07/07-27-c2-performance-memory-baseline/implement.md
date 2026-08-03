# C2 性能与内存基线 - 实施计划

## 1. 前置与兼容保护

- [x] 保存当前工作区、目标提交、容器/image/capability 和旧 C1 raw 清单；重建容器前
  按现有 SHA 将 C1 raw 导出到仓库外并回读校验。
- [x] 为 C1 runner/analyzer 建立 CLI、合成 parser 和旧 raw 复算兼容基线。
- [x] smoke LTTng/perf userspace probe；记录可用性、entry/return 配对和权限。失败时
  启用设计中默认 OFF 的 profiling-only UST fallback，不扩大容器为 privileged。

## 2. 抽取公共 profiling 基础

- [x] 抽取 shell common 的 PID/role/provenance/perf/tool lifecycle/report gate，保持
  `scripts/profile-perception.sh` 四参数 CLI 和 C1 行为不变。
- [x] 抽取可导入 Python parser/statistics helper；保持
  `analyze-perception-profile.py` 对 C1 raw 的输出和拒绝门兼容。
- [x] 扩充合成测试：role child crash、duplicate evidence、低 workload、perf ACK/lost/
  symbols、Heaptrack/Massif/Memcheck 缺段、正常退出和 source provenance。
- [x] 运行 `bash -n`、Python tests 和三份 C1 300 秒旧 raw 复算回归。

## 3. 实现确定性 workload 与 oracle

- [x] 新增 `perception_profiling` C++ 包及 ROS-free scenario core，冻结 bounded/
  expanding 的 sequence、pose、stamp、解析隧道横截面、beam/range/resolution。
- [x] 新增薄 authoritative fixture 节点和可关闭 target 的 profile launch；以同一
  SessionID/fingerprint 直接发布 health/pose/FullRay observation，并保证 pose watermark；
  同节点发布冻结的 `base_link <- profile_scan_link` static TF，oracle 复用同一外参。
- [x] 新增 C++ sink，保存完整 observation 或固定 schema digest、state、diagnostics、checkpoint CSV 与 transport
  完整性；不把 profiling 字段加入公共 ROS 接口。
- [x] 新增 ROS-free oracle，消费同一 scenario exact sequence，驱动真实 mapper/backend 并按 exact receipt 输出每 100
  revisions 的 counts/bounds/stamp，并输出 100k/250k/500k/750k milestone；覆盖
  bounded plateau 与 expanding growth。
- [x] gtest scenario 的无跳变/unique stamp/确定性、椭圆有限 hit、固定 `+inf` 岔口、
  origin-near free 与 max-range endpoint unknown；集成测试 10 Hz、TF 三轴/quaternion、
  graph、epoch、fingerprint、revision 和
  production-oracle join 正反例。
- [x] 保留现有 TreeCave/scanner/gate/C1/C2 完整链为短程等价门，验证 authoritative
  payload/contract/map oracle；不把该链资源写入正式 C2 baseline。

## 4. 实现 C2 runner 与 analyzer

- [x] 新增薄 `scripts/profile-local-map.sh`，支持 plain/perf/trace/stage-latency/
  Heaptrack/Massif/Memcheck/ASan/LSan，拒绝已存在目录和未知 mode。
- [x] 参数/graph/workload gate 验证所有 roles 和 endpoints；正式窗按 unique stamp 与
  revision delta 计数，不把 heartbeat/state_sequence 当 commit。
- [x] checkpoint sampler 将 state monotonic time 与 target smaps/pidstat/smem 对齐，
  manifest 记录 skew、valid 和 normal_completion。
- [x] 新增 C2 analyzer，机器重算 plateau、三轮 slope、oracle join、每新增 known voxel
  成本、阶段分位数和工具 report 完整性。
- [x] 合成正反例覆盖未 plateau、expanding 不增长、epoch/fingerprint 变化、stamp/
  revision gap、skew 超限、probe 丢失/unmatched、提前退出与重复三轮证据。

## 5. 构建与短程质量门

- [x] 独立构建 production-like RelWithDebInfo 和 latency（如需要）package closure；
  检查 compile flags、debug info、ELF SHA/build-id/ldd/nm。
  - 当前 production-like 九包前缀为
    `/tmp/alien-c2-relwithdebinfo-current-20260727`；provenance 固化在其
    `provenance/`，目标 flags/ELF/stage-default-OFF 门均通过。
- [x] 独立构建 ASan/LSan package closure；检查 `-O1 -g -DNDEBUG`、frame pointer、
  sanitizer linkage、debug info、ELF SHA/build-id/ldd/nm，且 stage trace option 默认 OFF。
- [x] 在独立 production-like prefix 运行既有 C2 全量功能测试，要求零
  error/failure/skip。
  - 首轮 test 因 colcon 默认 install base 继承主 `ws/install`，只保留为污染诊断，
    不计入 closure。最终使用 `env -i`、仅 source `/opt/ros/jazzy` 与当前 `/tmp`
    install、显式 `--install-base` 的九包复跑为 `266/0/0/0`。
- [x] 在 ASan prefix 以 `detect_leaks=0` 对 C2 九包 closure 跑全量功能测试，再运行
  独立 60 秒 bounded ASan workload，最后独立运行 60 秒 bounded LSan。
  - 2026-07-27：ASan 九包 closure 通过 `266/0/0/0`。首个 60 秒 workload 因 sampler
    将 warmup 前历史 state 与当前内存绑定而被正确标记 invalid；该历史保留。
  - 修复后 `/tmp/alien-c2-asan-smoke-checkpointfix2-20260727` 与
    `/tmp/alien-c2-lsan-smoke-checkpointfix-20260727` 均为 `valid=true`、
    `normal_completion=true`，正式窗口分别为 60.092310587 秒和 60.092093042 秒，
    无 AddressSanitizer/LeakSanitizer error 或 leak summary。
- [ ] 每个 runner mode 做短程有效 smoke 和至少一个故障注入；不得把 smoke 数字写入
  正式 baseline。

  #### 2026-07-31 范围收窄（经用户批准）

  收窄原则：**门保护的是指标；指标若已按
  `docs/performance-memory-testing-playbook.md` §2.8 弃用，则不再为其做活体
  smoke**。被排除的模式其解析门仍由合成测试覆盖，并非无保护。

  | mode | 产出指标 | 活体 smoke | 理由 |
  | --- | --- | --- | --- |
  | `valgrind-massif` | 堆形状 | **不做** | 10 Hz 下结构性跑不起来；已由 Heaptrack 替代 |
  | `valgrind-memcheck` | 未初始化读/泄漏分级 | **不做** | 同上；已由测试负载 Memcheck 补偿 |
  | `perf-stat` | task-clock（已弃用）+ PMU（不可用）+ 调度计数 | **不做** | 唯一剩余的 migrations=0 已由 runner 经 `/proc` affinity 独立验证，无增量信息 |
  | `ros-trace` | 回调时序 | **不做** | 时序被争用污染；结构信息与 stage-latency 重叠且后者更精确 |
  | `perf-record` | 热点排序 | **不做** | 已被 stage 阶段分解取代（精确归因 serialization 55% / apply 43%）；自身有 44% 未符号化 blocker |
  | `plain-sample` | RSS/PSS/USS、精确计数 | **保留** | §2.8 可信指标 |
  | `heaptrack` | 分配次数、peak heap | **保留** | §2.8 可信指标 |
  | `stage-latency` | 单 run 内阶段比例 | **保留** | §2.8 可信指标 |
  | `asan-smoke` / `lsan-smoke` | 错误/泄漏有无 | **保留** | 确定性事实 |
  | `capacity-ramp` | 容量分段 | **保留** | 计数类，确定性 |

  被排除模式的门覆盖来源（合成正反例，已在 §2/§4 勾选）：
  `test_perf_control_window_must_be_nested`、
  `test_perf_control_rejects_duplicate_ack_evidence`、
  `test_perf_stat_requires_every_declared_event`、
  `test_perf_record_rejects_lost_samples`、
  `test_massif_requires_stack_aware_peak`、
  `test_memcheck_requires_target_command`（C1 19 测试）。

  #### 保留模式的真实 fail-closed 证据登记

  本任务执行期间，门在真实条件下正确拒绝坏证据的记录（均非人为构造）：

  | mode | 真实拒绝事件 | 触发的门 |
  | --- | --- | --- |
  | `plain-sample` | 300 s expanding run 节点掉队 3.9%（obs 3115 / rev 2993） | observation/revision skew |
  | `plain-sample` | 首个 60 s ASan workload 把 warmup 前历史 state 与当前内存绑定 | checkpoint sampler 绑定门 |
  | `heaptrack` | 200 s expanding，rev 2100 bounds 不符 | production/oracle bounds mismatch |
  | `heaptrack` | 150 s expanding try1，skew 2.5% | observation/revision skew |
  | `heaptrack` | 150 s expanding try2，rev 1600 bounds 不符 | production/oracle bounds mismatch |
  | `stage-latency` | v10 分析侧代码在 pair 盖章后变更 | workspace closure provenance |
  | `capacity-ramp` | 首次 ramp rev 700 bounds 不符（O(N) known_bounds） | production/oracle bounds mismatch |

  全部 invalid raw 按纪律保留，无一放宽门后重收。

  #### 已识别的真实缺口（待处理）

  **`asan-smoke` / `lsan-smoke` 的 sanitizer 报告门在两层均无故障注入覆盖。**
  - 门实现为 `scripts/profile-local-map.sh:1411-1414`（grep
    `ERROR:/SUMMARY: (AddressSanitizer|LeakSanitizer)` → `invalidate`），
    另有 `:459-461` 要求目标链接 `libasan`。
  - C1 的 19 个与 C2 的 41 个合成测试覆盖了 perf/heaptrack/massif/memcheck/
    massif-timeline 的报告门，**唯独没有 sanitizer 报告门**。
  - ASan 此前唯一的真实拒绝来自 checkpoint sampler 门，**不是** sanitizer 报告门；
    LSan 无任何拒绝记录。
  - 处置见下方"待办"。
- [x] 重建/核验容器 capability 后，重新通过 perf control、ROS trace、Heaptrack、
  Massif、Memcheck 和 sanitizer report gate。
  - **2026-07-31 capability 复核完成**：容器实测
    `CapEff=00000040a80c25fb`，`capsh --decode` 含 **`cap_perfmon`** 与
    **`cap_sys_ptrace`**，与 `.devcontainer/docker-compose.yml:31-33` 的
    `cap_add: PERFMON, SYS_PTRACE` 一致。design.md 第 200 行"当前 live container
    缺 PERFMON"的记录**已过时**，据此更新。
    注意：`CAP_PERFMON` 到位**不改变 PMU 不可用的结论**——PMU 缺失在虚拟机层面
    （无 `cpu` event source），capability 给不了，两者不冲突。
  - 各工具 report gate 复核状态：perf control FIFO/ACK（批次 A perf-stat 600 s /
    perf-record）✓、ROS trace 60 s ✓、Heaptrack（bounded 300 s + expanding 100 s）✓、
    ASan/LSan 各 60 s ✓；Massif/Memcheck 为结构性阻断并已有补偿证据（见 §8）。
  - sanitizer **报告门**的故障注入缺口归属上一条（item 1），不阻塞本条。
  - Valgrind Massif 与 Memcheck 均因无法维持冻结的 10 Hz workload 而未形成有效
    smoke；吞吐阻断由
    `.trellis/tasks/07-27-c2-valgrind-throughput-blocker` 跟踪，禁止降频、减 beam 或
    放宽 warmup/revision 门。
  - Memcheck 的 rcutils/glibc definite loss 与 LTTng TLS possible loss 归因由
    `.trellis/tasks/07-27-c2-memcheck-runtime-leak` 跟踪；当前不分类为 C2 业务泄漏。

## 6. 阶段延迟证据

- [x] 在正式 RelWithDebInfo ELF 上验证符号/build-id 并配置外部 entry/return probe；
  如环境不支持，构建独立 latency prefix 的默认 OFF UST tracepoint fallback。
- [x] 短跑验证 callback/apply/state/acquire/materialize/snapshot-total 嵌套配对和 applied
  计数一致，且不记录 per-cell 事件。
- [x] 各运行至少 120 秒 unprobed、callback-only 与 full nested probe：前后 CPU 差异
  `<= max(5% relative, 0.5 percentage point)`，callback p50/p95/p99 差异
  `<= max(10% relative, 50 us)`，并保持 10 Hz 无 backlog；失败不放宽阈值。
  （2026-07-31 按方案 D 受限交付收口，经用户批准：延迟侧 p50/p95/p99 四轮通过
  原门（本轮条件下）；**CPU 侧本机结构性不可判定**——无硬件 PMU、相同工作量
  CPU 时间波动 30%、两侧不同 ELF 无法交错采样，待分辨效应比污染低一个数量级。
  可支持结论为"未观察到系统性探针 CPU 开销，上界约 1 pp"，不得称通过 CPU 门。
  阈值公式未变。详见下方"r10 严格同序补跑 → 收口决定"与
  `docs/performance-memory-testing-playbook.md` §2.8。历史：2026-07-29 曾以
  schema-3 中位数判定 `gate_pass=true`，该结论已被 r10 证明依赖轮次选择，
  不再单独成立。）
  - 三角色聚合器与 28 个 C2 合成测试已完成；CPU 和 callback 门均按双向绝对差计算。
    runner 已用对应 build tree 的 CMake cache/compile database 加 installed/build ELF
    build-id 等价门写入 build provenance，从 `/proc` 写入规范化 target affinity，并把
    pidstat 身份确认后的 start 与 T1 后、停止前的 stop 紧贴正式窗口记录和复核；真实
    10 秒 smoke 已通过且两侧 margin 均约 14 ms。
  - 2026-07-28 三组 120 秒证据已运行，旧 analyzer 均写出 `valid=true`、
    `normal_completion=true`，聚合器对 unprobed/callback/full target CPU mean
    `35.6230% / 37.7041% / 37.7203%`，full 相对 unprobed 增加
    `2.097333 pp / 5.887582%`，超过 `1.781150 pp` 门并输出 `gate_pass=false`。
    callback 与 full 的
    p50/p95/p99 差异 `2.2294% / 2.2228% / 4.5638%` 均通过，且 4020 对 42326 个
    target events 几乎没有均值差，因此 full 额外内部事件不是失败来源。
  - 离线归因确认比较构建不是等价 closure：production 使用独立
    `RelWithDebInfo -O2` 依赖，stage 目标却链接工作区 `Release -O3` 的
    `perception_core`。旧 runner/analyzer 没有校验传递依赖，故三个单 run 的旧 validity
    不能使跨 build 校准可采纳，`gate_pass=false` 也不能归因为真实探针 overhead。
    详细证据在 `research/c1-profiling-assets.md`；独立 profiling evidence finding 为
    `.trellis/tasks/07-28-c2-stage-calibration-build-equivalence`。该 finding 必须先补
    transitive dependency provenance/rejection 门，并以 production prefix 为 underlay
    重建 paired latency overlay，之后才可按原门重跑。复选框保持未勾选，5%/0.5 pp 门
    不变，当前范围不重跑。
  - child finding 的独立代码审核已达到 `0 Blocking / 0 High`，C2 `36 tests`、C1
    `19 tests` 通过；但审核修复改变了 canonical identity 覆盖的 `scripts/`。review 前
    v3 pair/smoke 只保留为历史证据。
  - 2026-07-28 后续曾完成 **v6** fresh pair 与三组 10 秒 smoke；2026-07-29 独立审计
    发现 v6 存在首轮测试失败被覆盖、身份事后重盖、算法中途切换三项过程瑕疵，v6 降级
    为历史证据。当前有效证据为 **v8** fresh pair（详情见 child
    `07-28-c2-stage-calibration-build-equivalence/implement.md` 的 v8 evidence）：
    pair root `/tmp/alien-c2-stage-pair-20260729-v8`；
    `paired_source_identity_sha256=53e54c67ad8e206dc6fa93cb34a6b33963df4daf09ddbe1696c249ac2c7dff6f`；
    production/stage 隔离测试均首轮 `266/0/0/0` 与 `47/0/0/0`；closure/helper digests
    跨 production/stage/三组 smoke 一致；unprobed target `bda41357…86bc1`（OFF），
    callback/full 共用 stage target `9308ee2b…41821`（ON）；五个检查点 source identity
    零漂移。v3/v4/v5/v6/v7 均作废或仅作历史证据。
  - **正式 120 秒 unprobed/callback/full 与 schema-2 aggregator 已于 2026-07-29 在 v8
    pair 上获授权运行**：CPU 门通过（full−unprobed 0.067 pp，阈值 1.7617 pp，证实旧
    2.097 pp 超标系构建不等价所致）、callback p50/p95 通过，**仅 p99 失败**
    （2 815 445 ns > 2 308 828 ns），`AGGREGATOR_EXIT=1`。raw 与 aggregate 已保留
    （`/tmp/alien-c2-formal-{unprobed,callback,full}-20260729-v8`、
    `/tmp/alien-c2-stage-calibration-20260729-v8.json`）；p99 尾差归因由独立 finding
    `.trellis/tasks/07-29-c2-stage-p99-tail-overhead` 跟踪。父任务本复选框在该 finding
    得出结论并按原门通过（或用户评审 blocker 证据）前保持未勾选；5%/0.5 pp 与
    p50/p95/p99 门均不变；short smoke 数字不得写入 baseline。

## 7. 大图 capacity ramp

- [x] oracle 先预测冻结 expanding 场景最长 600 秒的 milestone，计算
  `crossing_revision` 与 `required_end_revision=crossing+300`；机器验证固定互斥的
  `[c-299,c-100]`、`[c-99,c+100]`、`[c+101,c+300]` 三个 200-sample 桶，并据此分类
  `covered`/`not_reached`/`crossing_without_post_window`。
- [x] 使用后续正式矩阵同一 RelWithDebInfo install、ELF SHA/build-id、参数和 affinity
  运行无重量级 profiler ramp；`covered` 运行到 required-end 加 drain，其余最长 600 秒。
- [x] graph/provenance 明确拒绝 `scan_accumulator`、`cloud_map`、`/scan_returns` 和
  `octomap_builder`，避免旧 500k FIFO 制造 RSS/point-count 假平台。
- [x] 同窗采样 target CPU/RSS/PSS/USS、callback backlog、cgroup memory.current/max/
  events 和 host headroom；`memory.current >= 80% memory.max`、host MemAvailable
  `<=2 GiB` 或 oom/oom_kill 增加时正常停止，不接受 OOM/SIGKILL 产物。
- [x] `covered` 时在 500k 前后不能维持 10 Hz/连续增长或安全 headroom，立即建立独立
  finding 并暂停长时矩阵；其他 capacity 状态本身不是失败，但不能形成 500k 分段结论。

### 2026-07-29 §7 首次 ramp 结果与暂停记录

- 首次 capacity ramp（`/tmp/alien-c2-capacity-ramp-20260729-v8`，expanding，
  v8 production 构建）：oracle 预测 500k crossing=rev 440、required_end=740，
  实跑至 rev 757 正常收尾，`capacity_status=covered`；但 analyzer 以
  "revision 700: production/oracle bounds mismatch" 判 `valid=false`，raw 冻结。
- 归因（finding `07-29-c2-capacity-bounds-phase-lag`）：`OctoMapBackend::known_bounds()`
  O(N) 扫描致 ~500k cells 后目标核饱和、pose 订阅被饿、前沿滞后 1 voxel——
  production 性能缺陷，非测量链缺陷。信息性数据：750k cells 时 RSS≈126 MiB、
  PSS≈110 MiB；内存与 observation 吞吐无阻断，瓶颈是 bounds 扫描 CPU。
- 按 §7 预案曾暂停长时矩阵；修复任务 `07-29-octomap-backend-o1-known-bounds`
  （O(1) known_bounds）完成并经 v9 前缀 ramp 重跑验收：
  `/tmp/alien-c2-capacity-ramp-20260729-v9` `valid=true`/`covered`/end rev 760，
  bounds 7/7 精确一致，尾段 CPU 99–101%→37.9%。**§7 于 2026-07-29 收口**；
  长时矩阵解除暂停。注意：ws/src 已变，§8 stage-latency 行前需新建 pair 并按
  schema-3 中位数判据复跑校准；v9 为当前正式矩阵 production 前缀。

## 8. 正式 bounded 矩阵

- [x] oracle 先证明 600 revisions plateau；运行 600 秒 perf stat。（2026-07-30 v9 通过）
- [x] 运行 perf record 至 `>=120 s`、`>=1000` samples、unknown `<=20%` 且 0 lost
  或明确门内；保存 build-id 和 top symbols。
  （2026-07-30 经用户批准按方案 C 受限交付：样本/lost 达标，unknown 44.25% 系
  ROS octomap 无符号表的结构性限制，门未放宽、不计通过；可解析热点
  updateInnerOccupancyRecurs≈30% + liboctomap≈37% 入文档；符号化推迟到
  octomap 优化 finding。详见 `07-30-c2-perf-octomap-symbols-blocker`。）
- [x] 运行 60 秒 ROS trace和 300 秒 stage-latency，生成完整分位数与扰动校准。
  - ROS trace 60 s：2026-07-30 随批次 A 通过。
  - stage-latency 300 s：2026-07-31 在 v11 pair 上运行完成且 `valid=true`，
    完整分位数见"§8 批次 B"。
  - 扰动校准：**按方案 D 受限交付收口**（用户批准）。延迟侧四轮通过原门
    （本轮条件下）；CPU 侧本机结构性不可判定，上界约 1 pp、无系统性方向。
    依据见"§6 r10 严格同序补跑 → 收口决定"与
    `docs/performance-memory-testing-playbook.md` §2.8。
- [x] 顺序运行 300 秒 Heaptrack、180 秒 Massif、60 秒 Memcheck、60 秒 ASan、60 秒
  LSan；全部要求正常退出和可解析报告。
  （2026-07-30 收口：Heaptrack 300 s / ASan 60 s / LSan 60 s 在 v9/新 ASan 前缀
  有效通过。Massif/Memcheck 经复测确认结构性吞吐阻断（3.24 vs 10 rev/s，
  finding `07-27-c2-valgrind-throughput-blocker` 已关闭）：Massif 形状证据由
  Heaptrack 替代；Memcheck 的未初始化读与泄漏分级由测试套件补偿 Memcheck 覆盖
  （0 errors、仅 still-reachable），残余缺口"证据来自测试负载"已记录。）
- [x] 独立运行三次 300 秒 plain sample，保存 target-only CPU/RSS/PSS/USS、revision、
  exact counts 和 raw samples；heap 结论来自独立 Heaptrack/Massif，不用 smaps 冒充。
  （2026-07-30 v9 通过，三轮斜率 0/0/0.097 KiB/min，无疑似泄漏）

### 2026-07-30 §8 批次 A 结果（v9 前缀，全部 fail-closed 门通过）

- 有效正式 run 六个（各含 sha256sum.txt）：perf-stat 600 s、ros-trace 60 s、
  Heaptrack 300 s、plain ×3 各 300 s（`/tmp/alien-c2-formal-{perfstat,rostrace,
  heaptrack,plain1,plain2,plain3}-20260729-v9`）。
- bounded plateau：`final_known=230520`，6001 observations/600 s（10 Hz 精确）。
- CPU：pidstat 600 s 均值 **13.32%**（修复 O(1) bounds 前同负载校准值 ≈35%，
  修复的附带收益）。
- 内存：三轮 plain RSS/PSS/USS = 62 484/44 586/39 884 KiB 全程平坦；三轮
  斜率 RSS 0/0/0、PSS 0/0/0.097 KiB/min（阈 1024）→ `suspected_sustained_growth=false`。
- Heaptrack：peak heap 28.96 MB，120 836 次分配（temporary 42.95%），
  exit 期 "leaked" 455.54 KiB（按纪律与 LSan/Memcheck 交叉后再定性，不单独定罪）。
- 三轮趋势 aggregate：`/tmp/alien-c2-plain3-trend-20260729-v9.json`（exit 0）。
- 未决行：perf-record（octomap 符号化 blocker `07-30-c2-perf-octomap-symbols-blocker`，
  invalid raw 保留，热点信号 updateInnerOccupancyRecurs≈30%+liboctomap≈37% 留待
  §10 立优化 finding）；stage-latency 300 s（需 v10 pair+schema-3 校准复跑）；
  Massif/Memcheck（Valgrind 吞吐 blocker）；ASan/LSan（需新源码重建 ASan 前缀）。

## 9. 正式 expanding 矩阵

- [x] 三次独立 300 秒 plain sample；每 100 revisions 对齐 oracle counts/bounds 和
  target memory，证明 map 实际扩展。（2026-07-31 以 200 s 完成，见下方时长调整）
- [x] 运行至少一次 300 秒 Heaptrack，以 `heaptrack_print -M` 生成 heap 时间线；验证
  snapshot 数、elapsed-time 覆盖和 checkpoint skew 后，关联 persistent/temporary
  allocation hotspot 与 exact new-known delta。
- [x] 机器计算每 revision、每新增 known voxel 和 bounds 增量的 RSS/PSS 成本，并从
  Heaptrack Massif-compatible 时间线计算 heap 成本；
  不把正常地图扩张标记为泄漏。
- [x] expanding stage-latency 在 `covered` 时运行
  `max(300 s, required_end_stamp + drain)`、上限 600 秒且不超过 capacity 安全窗口，并按
  三个预注册互斥 200-sample 桶输出 pre/crossing/post；其他状态运行 300 秒、标记
  segmented evidence unavailable；始终保留连续 map-size 回归。

### 2026-07-31 §9 时长调整（经用户批准）与容量拐点 finding

**首个 300 秒 expanding plain-sample 被正确判 invalid。**
raw `/tmp/alien-c2-invalid-exp-plain300-20260731-v9`（原名
`...formal-exp-plain1...`，改名保留，按纪律不删除）；log
`/tmp/alien-c2-v9-expanding-matrix-300s-invalid.log`。
`normal_completion=true` 但 analyzer 报
`observation/revision window count skew is too large`：fixture 发出 3115 个
observation，节点只完成 2993 个 revision（落后 3.9%）。**门工作正常，是它抓到了
节点掉队**，不放宽。

**实测容量曲线**（`Δrevision / Δreceipt_monotonic_ns`，cells 按约 1130/rev 估算）：

| revision | 估算 cells | 速率 (rev/s) | RSS (MiB) |
| ---: | ---: | ---: | ---: |
| 200–2300 | 226k–2.60M | 9.98–10.04 | 61.4 → 338.3 |
| 2400 | 2.71M | 9.83 | 353.4 |
| 2600 | 2.94M | 8.93 | 378.7 |
| 2900 | 3.28M | 7.95 | 423.5 |

约 2.6M cells 前完全平坦，此后单调劣化。内存线性增长、MemAvailable > 6.7 GiB、
无 oom——**不是内存阻断**。归因与定位由独立 finding
`.trellis/tasks/07-31-c2-expanding-capacity-knee` 跟踪；本任务不顺手优化。
按 `docs/performance-memory-testing-playbook.md` §2.8，"10 Hz 可持续性"是
**保守指标**，该拐点为本机观测值，专用机上只会更靠后，不得表述为绝对容量上限。

**时长调整（偏离，经用户批准）**：

- 上两项（plain ×3、Heaptrack）由 **300 s → 200 s**，使正式证据落在满 10 Hz 的
  有效窗口内。200 s 到 rev 约 2000（约 2.26M cells、RSS 约 299 MiB），仍是 10 倍
  扩张，足以支撑"map 实际扩展"与每体素内存成本结论。
- 第四项 expanding stage-latency 原文即含"**不超过 capacity 安全窗口**"，按同一
  200 s 窗口执行，**不属偏离**；输出目录相应改为
  `/tmp/alien-c2-formal-exp-stage200-20260731-v11`。
- workload 定义、affinity、10 Hz 频率、skew/validity 门、C2 语义**均未修改**。
- Heaptrack 有 10–40% 扰动，200 s 仍可能触及拐点；若其 skew 门失败，按实测速率
  曲线再缩短并记录，不放宽门。

#### 执行结果

- **plain ×3 @ 200 s 全部有效**：`/tmp/alien-c2-formal-exp-plain{1,2,3}-20260731-v9`，
  三个 run 均 `gate_ok`（`valid=true` / `normal_completion=true`），各含
  `sha256sum.txt`。200 s 的判断成立。
- **Heaptrack @ 200 s 被拒，但原因不是掉队**：
  `revision 2100: production/oracle bounds mismatch`。该 run 全程维持 10 Hz
  （9.93–10.04），仅落后 16 个 observation（0.75%），却在 rev 2100（约 2.37M
  cells、RSS 315.7 MiB）出现 production 报告 bounds 与 oracle 真值不符。
  raw 保留为 `/tmp/alien-c2-invalid-exp-heaptrack200-20260731-v9`。
  **这是 §7 `07-29-c2-capacity-bounds-phase-lag` 症状在 O(1) 修复后于更大规模的
  重现**，且属正确性可见问题；归因移交 finding
  `07-31-c2-expanding-capacity-knee`，本任务不顺手优化。
- 按预注册处置（不放宽门，按速率曲线缩短）：Heaptrack 与 expanding stage-latency
  改为 **150 s**（rev 约 1500、约 1.7M cells、RSS 约 240 MiB，明显在窗口内），
  输出为 `/tmp/alien-c2-formal-exp-heaptrack-20260731-v9` 与
  `/tmp/alien-c2-formal-exp-stage150-20260731-v11`。已有效的三个 200 s plain run
  不重跑；每体素成本为速率量，两侧窗口不同不影响该结论。

#### §9 正式结果（2026-07-31）

**三次独立 200 秒 expanding plain sample（item 1 / item 3 交付）**

`/tmp/alien-c2-formal-exp-plain{1,2,3}-20260731-v9`，均 `valid=true` /
`normal_completion=true`，`capacity=covered`，`segmented_evidence_available=true`，
三个预注册互斥桶 pre `[141,340]` / crossing `[341,540]` / post `[541,740]` 齐备，
`crossing_revision=440`（与 §7 oracle 预测一致）。

| 指标 | plain1 | plain2 | plain3 |
| --- | ---: | ---: | ---: |
| final_known / free / occupied | 2 377 520 / 2 334 388 / 43 132 | 同左 | 同左 |
| known_delta | 2 147 000 | 2 147 000 | 2 147 000 |
| **RSS / 每新增 known 体素** | **121.53 B** | **121.95 B** | **121.41 B** |
| PSS / 每新增 known 体素 | 121.43 B | 121.85 B | 121.30 B |
| USS / 每新增 known 体素 | 121.34 B | — | — |
| RSS / revision | 134.11 KiB | 134.57 KiB | 133.98 KiB |
| RSS peak | 307.3 MiB | 307.2 MiB | 306.9 MiB |

- 三次独立 run 的最终计数**逐位相同**，证明 map 确实扩展且过程确定性、oracle 对齐
  （bounds mismatch 门若不符会拒绝，此处未触发）。
- **每新增 known 体素约 121.5 B（RSS）**，三轮离散度 0.4%。对照 CPU 指标 30% 的
  离散度，实证了 `docs/performance-memory-testing-playbook.md` §2.8 的指标分级。
- expanding 的 RSS 斜率 80 712 KiB/min 属正常地图扩张；analyzer 未对 expanding run
  套用 bounded 的泄漏启发式（无 `suspected_sustained_growth` 字段），满足
  "不把正常地图扩张标记为泄漏"。

**expanding stage-latency 150 秒（item 4 交付）**

`/tmp/alien-c2-formal-exp-stage150-20260731-v11`，`valid=true`，
`final_known=1 812 520`，`capacity=covered`，三桶齐备。

| 阶段 | bounded p50 | **expanding p50** | bounded p99 | **expanding p99** |
| --- | ---: | ---: | ---: | ---: |
| callback 总 | 11.887 | **51.202** | 15.644 | **92.977** |
| ├ snapshot_serialization | 5.623 | 28.577 | 7.860 | 55.365 |
| └ mapper_apply | 5.946 | 21.788 | 7.935 | 39.143 |
| snapshot_total | 5.819 | 28.966 | 8.121 | 55.825 |
| state_publication | 0.028 | 0.051 | 0.150 | 0.160 |
| read_transaction | 0.0025 | 0.003 | 0.0043 | 0.005 |

（单位 ms；bounded 为 §8 批次 B 的 230 520 cells，expanding 为 1 812 520 cells）

**关键发现：callback p99 已占用 100 ms 周期的 93%**，这解释了 §9 首个 300 s run
观测到的约 2.6M cells 容量拐点——地图继续增长即超出周期预算，节点必然掉队，
并非未知资源上限。

- 主导项为 `snapshot_serialization` 且**增长更快**：cells 增长 7.86 倍时，
  serialization 增长 5.08 倍、`mapper_apply` 增长 3.66 倍；读事务与状态发布
  始终可忽略。
- 该结论基于**单 run 内阶段分解比例**，属 §2.8 分级中的可信指标；绝对值仍带
  ±30–46% 漂移不确定度，但比例与标度趋势稳健，且与独立观测到的拐点位置自洽。
- 优化不在本任务范围；hotspot 移交 §10 立独立优化 finding。

**expanding Heaptrack（item 2）：三次尝试均失败，机制已明确**

| 尝试 | 时长 | 失败位置 | 门 | raw |
| --- | --- | --- | --- | --- |
| 1 | 200 s | rev 2100（窗口末检查点） | bounds mismatch | `alien-c2-invalid-exp-heaptrack200-20260731-v9` |
| 2 | 150 s | rev 1500（窗口末检查点） | skew 2.5% | `...-heaptrack150-try1-...` |
| 3 | 150 s | rev 1600（窗口末检查点） | bounds mismatch | `...-heaptrack150-try2-...` |

失败位置跟随**窗口末尾**而非固定地图规模；同为 rev 2100，200 s 的 plain run 干净
通过而 Heaptrack 失败，差别只有 Heaptrack 的额外开销。结合 stage-latency 证据，
机制清楚：**1.81M cells 时 callback p99 已达 93 ms，Heaptrack 的 10–40% 扰动直接
把回调推出 100 ms 预算**，属结构性超预算，不是随机失败。据此改用 100 s 窗口
（rev 约 1000）重试并**一次通过**，验证了该机制预测。全部 invalid raw 按纪律保留，
未放宽任何门。

**expanding Heaptrack 100 秒（item 2 交付）**

`/tmp/alien-c2-formal-exp-heaptrack-20260731-v9`，`valid=true`、
`heaptrack gate_pass=true`、`massif 时间线 gate_pass=true`、`target_verified=true`。

- runtime 114.05 s，`final_known=1 247 520`，`known_delta=1 017 000`（rev 102→1102）。
- peak heap **122.08 MB**（128 010 158 B），peak RSS with overhead 194.83 MB。
- 分配调用 **8 339 973** 次，temporary 仅 42 290（0.5%）。
- Massif 兼容时间线：**10 082 个 snapshot**，peak 于 112.945 s，final 986 668 B，
  elapsed 覆盖完整，checkpoint skew 通过。
- exit 期 `total_memory_leaked=455.54 K`，与 §8 批次 A bounded 的 455.54 KiB **完全一致**
  ——说明该量与地图规模无关，是固定的退出期残留，非随扩张增长的泄漏。

**每新增 known 体素的成本（item 3 交付，跨独立工具交叉验证）**

| 来源 | 每新增 known 体素 |
| --- | ---: |
| plain1/2/3 RSS | 121.53 / 121.95 / 121.41 B |
| plain1/2/3 PSS | 121.43 / 121.85 / 121.30 B |
| plain1 USS | 121.34 B |
| Heaptrack run RSS | 122.55 B |
| **Heaptrack peak heap** | **125.87 B**（含基线堆，故略高） |
| 分配调用 / 新增体素 | 8.20 |

smaps 采样与 Heaptrack 两条独立路径给出同一量级（121–126 B），离散度约 1%。
每 revision 成本：RSS 133.98–134.57 KiB。**正常地图扩张未被标记为泄漏**：
expanding run 无 `suspected_sustained_growth` 字段，泄漏结论仅取 exit 期固定的
455.54 K（与 bounded 一致）。

## 10. 报告、审核与收口

- [x] 生成机器可读 summary 和 `docs/local-map-resource-profiling.md`，引用仓库外 raw
  路径/SHA，记录无效/重跑历史而不把它们计为通过。
- [x] 对 AC1-AC8 做 full-scope check；验证 C1 wrapper 兼容、C2 业务语义无 diff、
  正式数字可由 raw 重算。
- [x] 对每个热点/泄漏/错误或吞吐超限建立独立 finding task，本任务不顺手优化。
- [ ] 使用代码审核 subagent 复核到 `0 Blocking / 0 High`，修复 profiling 资产问题并
  重跑受影响的短测/解析；长时 run 仅在证据失效时重跑。
- [x] 汇报 diff、测试、正式矩阵、限制与 raw 位置；未经用户明确授权不提交、不推送、
  不归档。

## 11. 回滚点

- 公共 runner 抽取失败：恢复到已验证的 C1 wrapper 结构后重新设计复用边界，不能
  复制一份 C2 状态机。
- workload/oracle 等价门失败：保留 invalid run，修复输入或 join 后重跑该模式。
- profiler/tracepoint 不能正常收口：保留工具日志，禁止 SIGKILL 产物进入报告。
- 正式矩阵发现 C2 缺陷：创建独立任务并冻结本任务证据，不修改 occupancy 实现。

## 2026-07-29 §6 阶段延迟校准收口记录

- v8 等价配对构建上的正式校准最终结论：探针无系统性 overhead。CPU 门
  0.067 pp（阈 1.7617 pp）、p50 59 901 ns、p95 935 077 ns、p99（4+4 run 中位数）
  1 324 200 ns（阈 2 330 431.75 ns）全部通过；schema-3 aggregate
  `/tmp/alien-c2-stage-calibration-median-20260729-v8.json`
  （`ff93bd44c433421955fd8f91df97dd7b60d3e692fbcd7e766788aff110d4dbfc`）。
- 首轮 schema-2 单点聚合的 p99 FAIL 已由 finding `07-29-c2-stage-p99-tail-overhead`
  归因为 run-to-run variance（同 mode 跨 run p99 极差 11.3 ms，为单点差 4 倍；
  full 均值与 callback 几乎重合）；宿主机（Windows+WSL2）后台负载为主要噪声源。
- 中位数判据为经用户批准的证据策略修正（方案 A）；阈值公式与 CPU/p50/p95 门未变。
  后续 stage-latency 类正式测量按 §8/§9 计划执行时沿用 schema-3 与该判据。

## 2026-07-30/31 §6 重校准与 CPU 中位数判据（会话交接记录）

### 判据修正二（经用户批准，方案 A 同构）

- 背景：O(1) bounds 修复使稳态 CPU 由 ~35% 降至 ~13%，`max(5% relative, 0.5 pp)`
  的相对项随之从 1.76 pp 缩到 ~0.65–0.74 pp，**跌破本机（Windows+WSL2）宿主噪声
  底**；实测 full 同 mode 跨 run CPU 极差 0.648 pp ≈ 阈值本身，单点比较失效。
- 修正：CPU 门与 p99 同构，改为 **>=3 组 unprobed 中位数 vs >=3 组 full 中位数**
  后按原公式比较。阈值公式、时长、workload、affinity、p50/p95 门一律不变。
- 实现：`scripts/lib/stage_latency_calibration.py` 新增
  `unprobed_replicate_dirs`（追加在 all_dirs 末尾）；schema 版本 4=含 unprobed
  复测、3=仅 p99 复测、2=单三组；payload 增 `cpu_evaluation`；CLI 增
  `--unprobed-replicate`（可重复）。CPU 复测组数允许与 full 不等（各 >=3），
  p99 仍要求 callback/full 数量相等——差异是设计选择，非缺陷。
- 质量门：C2 41 tests、C1 19 tests 全通过；trellis-check 复核 `0 Blocking /
  0 High`（索引切片以非对称用例实证、fail-closed 逐项注入验证、向后兼容确认），
  并补入"unprobed 复测 target provenance 必须等于主 unprobed"负例。
- **证据摘要漂移**（如实记录）：`cpu_evaluation` 无条件输出后，schema-2/3 payload
  与此前归档 digest 不再逐字节可复算（同 `p99_evaluation` 的已知漂移）。v8 的
  `ff93bd44…` 与 v10 的 schema-3 digest 均为**存档值**，权威结论以各自当轮
  quality/JSON 为准。

### 失败与作废记录（不可复用）

- **v10**（`/tmp/alien-c2-stage-pair-20260730-v10`）：pair + 三组 smoke + 12 组
  校准全部有效，schema-4 聚合 `gate_pass=true`（CPU 中位数 13.207 vs 12.787、
  p99 中位数 14.22 vs 14.34 ms、p50/p95 通过）。**但**该轮 CPU 中位数判据代码是
  在 pair 盖章之后才写的，因此 stage-latency 300 s 被身份门正确拒绝
  （`INVALID: workspace closure provenance validation failed`）。教训：**分析侧
  代码变更也属身份域，必须在 pair 构建前冻结**。v10 的 12 组 raw 与聚合作为
  历史证据保留，不得作为当前代码的正式校准。
- **v11**（`/tmp/alien-c2-stage-pair-20260730-v11`，身份
  `9f0c17bb…`／manifest 内 `e3e9fbcc…`）：pair + 三组 smoke 有效。
  - 尝试 1（r1–r3，11:30）：CPU 差 0.7158 pp vs 阈 0.7056 pp，**差 0.01 pp**
    失败；p99/p50/p95 全过。CPU 序列 unprobed 13.52/14.11/14.24、
    full 13.47/14.83/15.65 呈单调上漂（Docker 重启后宿主回温）。
  - 尝试 2（r4–r6，12:30）：CPU 门通过（0.624 vs 0.736 pp），但 **p99 失败**
    （中位数 20.74 → 25.38 ms，差 4.65 ms vs 阈 2.07 ms）。同代码同负载下 p99
    由 v10 的 14.2 ms 涨到 25.4 ms、CPU 抬升约 2 pp，为白天宿主负载所致。
  - 结论：窄阈值门在宿主噪声窗口内不可靠；必须在空闲窗口测量。
- v10/v11 目标 ELF SHA 互不相同（构建非比特级可复现，时间戳/路径嵌入），故
  **不得跨 pair 移植校准结论**。

### 尝试 3（r7–r9）：通过，§6 重校准收口

- 2026-07-31 00:45–01:29 在宿主空闲窗口（启动 load1=0.07）执行
  `/tmp/run-c2-v11-cal-attempt3.sh`，9 组 120 s run 全部 `valid=true` /
  `normal_completion=true`（`/tmp/alien-c2-v11-cal-{unprobed,callback,full}-r{7,8,9}`，
  各含 `sha256sum.txt`），pair 身份 `e3e9fbcc…` 全程复算一致。
- schema-4 聚合 **`gate_pass=true`**：
  `/tmp/alien-c2-stage-calibration-20260731-v11a3.json`
  （`d4d3bf7a34e2b53540dabab37a5de67a010606be9072756835da702fc73764fa`）、
  quality `3eafe819f5fb2ba38339a816a0b36890c2e9275d97be1f532581c16697eb3ab1`。

  | 门 | 基线 | full | 差值 | 阈值 | 结果 |
  |---|---|---|---|---|---|
  | CPU（3 组中位数） | 11.6467% | 11.7883% | 0.1415 pp | 0.5823 pp | PASS |
  | callback p50 | — | — | 369 646 ns | 987 513 ns | PASS |
  | callback p95 | — | — | 347 383 ns | 1 185 877 ns | PASS |
  | callback p99（3 组中位数） | 12.9816 ms | 13.6066 ms | 625 007 ns | 1 298 161 ns | PASS |

- 阈值公式、时长、workload、affinity、event set 一律未改。
  **注意：本轮的"无系统性 overhead"结论已被下方 r10 严格同序重聚合推翻其充分性——
  a3 单独不构成结论，见"r10 严格同序补跑"一节。**
- 尝试 1/2 的失败已由本轮证实为宿主噪声而非探针：同代码同负载下 p99 中位数从
  尝试 2 的 20.7/25.4 ms 回落到 12.98/13.61 ms。空闲窗口是该窄阈值门的前提条件。
- **协议偏离如实记录**：为消除"full 恒定最后跑"的时序偏置，r8 轮内顺序反转为
  full→callback→unprobed（r7/r9 为严格 unprobed→callback→full）。属对照平衡去偏，
  非择优取数；三轮全部计入中位数，无丢弃。

### 2026-07-31 r10 严格同序补跑：CPU 门 FAIL，判据缺陷暴露

- 经用户要求补跑严格同序的 r10（unprobed→callback→full，120 s×3，v11 pair，
  `run_stage` 调用与 attempt3 逐字相同），以 **r7/r9/r10 三个严格同序轮次**重聚合。
- 重聚合结果 `a4`：**`gate_pass=false`**，CPU 中位数差 0.9501 pp > 阈 0.6164 pp；
  p50/p95/p99 全过（p99 差仅 12 309 ns，阈 1 741 966 ns）。
  产物 `/tmp/alien-c2-stage-calibration-20260731-v11a4.json`
  （`a5c9d6ea808084d8686d3e5f14e1e500a16bc6e827d308e8967dd48ee665d6f6`）、
  quality（`1731c2b97496c9f10f798bdbdfa8165b537b5a781e7bc6349816bed6b2f7f357`）。
  r10 三组均 `valid=true` / `normal_completion=true`。

#### 四轮逐 run CPU 均值（%）与 callback p99

| 轮次 | 起始 | unprobed | callback | full | 轮内 full−unprobed | callback p99 |
|---|---|---|---|---|---|---|
| r7 | 00:45 | 10.7975 | 10.8478 | 11.1220 | +0.3245 | 12.569 ms |
| r8 | 01:00 | 11.6467 | 11.5633 | 11.7882 | +0.1415 | 12.982 ms |
| r9 | 01:14 | 12.3287 | 13.4780 | 13.2788 | +0.9501 | 18.323 ms |
| r10 | 03:00 | 14.0693 | 13.8029 | 13.7694 | −0.2999 | 17.420 ms |

#### 归因：判据缺陷，非探针 overhead

1. **宿主单调漂移**：unprobed 由 10.7975 升至 14.0693，+3.27 pp / 约 2.5 h
   （≈1.3 pp/h），为 CPU 阈值（0.62 pp）的两倍以上。同一二进制、同一 workload、
   每轮全新进程；r10 前已清理 9 个残留 ros2 daemon（load1 由 ~1.2 降至 0.00）
   而 r10 仍为最高值，排除 daemon 累积为主因。
2. **schema-3/4 破坏了 schema-2 的配对设计**：schema-2 在同一轮内背靠背比较
   unprobed 与 full，单调漂移天然抵消；schema-3/4 改为各 mode 跨 run 独立取中位数
   后，两个中位数可能落在不同轮次，或同时落在被扰动的轮次上。a4 中两个 CPU 中位数
   都落在 r9，而 r9 正是被扰动轮（p99 18–19 ms，且出现 callback > full 的不合理
   顺序）。
3. **同一 12 组数据换轮次即翻盘**：a3（r7/r8/r9）0.1415 pp PASS，
   a4（r7/r9/r10）0.9501 pp FAIL。**两者均不得单独宣称为结论**；择取 a3 属违纪
   的择优取数。
4. 轮内配对差为 +0.3245 / +0.1415 / +0.9501 / −0.2999，**符号会翻转**，
   中位数 +0.2330 pp、均值 +0.2791 pp，均低于阈值——与"无系统性 probe overhead"
   一致，但单点噪声与阈值同量级。

#### 收口决定（2026-07-31，经用户批准的方案 D：受限交付）

**CPU 探针开销门在本机结构性不可判定，按受限交付收口，不再重跑。**

判定依据（两条实测证据，非推测）：

1. **无硬件 PMU**：`/sys/bus/event_source/devices/` 无 `cpu` 项，
   `cycles:u / instructions:u / branches:u / cache-*:u` 全部 `<not supported>`，
   root 亦不可得（与 `perf_event_paranoid` 无关），内核 `6.10.14-linuxkit`。
   因此不存在对争用不变的替代指标。
2. **指标被污染**：四轮 workload 逐位相同（各 1201 observations / 1202 revisions /
   120.08–120.10 s），实耗 CPU 时间却为 12.966 / 13.986 / 14.806 / 16.897 秒
   （+30%）。待分辨的探针开销约 3% 相对量，比污染低一个数量级。
3. **无法交错测量**：unprobed 用 ELF `976d6af6…`（stage option OFF），
   callback/full 用 ELF `d5be6ef1…`（ON）——不同二进制，无法在细时间尺度上交替
   采样，两者必然相隔约 5 分钟、必然采样到不同宿主状态。

因此任何统计量修正（中位数、配对、增大轮数、更严规则）都无法解决——**问题在指标
不在统计**。此前提出的"轮内配对中位数"与"每轮均须通过 schema-2"两个方案均已
撤回，理由是后验循环论证与"更严的掷硬币"。

**可支持的结论边界**：

- CPU 侧：轮内配对差 −0.2999 ~ +0.9501 pp（基数 11–14 pp），**符号会翻转**、
  无系统性方向。可陈述为"**未观察到系统性探针 CPU 开销，上界约 1 pp**"，
  **不得陈述为"通过 `max(5% 相对, 0.5 pp)` 门"**。
- 延迟侧（callback vs full，**同一 ELF** `d5be6ef1…`，仅 tracepoint 集合不同）：
  四轮 p50/p95/p99 全部通过原门，最大用掉 54% 容差（r9 p99 986 174 ns /
  1 832 276 ns）。可陈述为"**本轮条件下通过原门**"，**不得陈述为无条件成立**——
  该门被同一漂移机制污染（callback p99 基线跨轮从 12.569 漂到 18.323 ms，+46%），
  仅因阈值为 10% 相对（CPU 门为 5%）而容差加倍；v8 首轮正式测量的 p99 即曾以
  2.815 ms > 2.309 ms 失败。

| 轮次 | Δp50 (ms) | 阈值 | Δp95 (ms) | 阈值 | Δp99 (ms) | 阈值 |
|---|---|---|---|---|---|---|
| r7 | +0.370 | 0.988 | +0.347 | 1.186 | +0.520 | 1.257 |
| r8 | +0.203 | 1.065 | +0.119 | 1.226 | +0.625 | 1.298 |
| r9 | −0.194 | 1.211 | +0.032 | 1.603 | +0.986 | 1.832 |
| r10 | −0.192 | 1.269 | +0.131 | 1.510 | +0.012 | 1.742 |

**范围纪律**：阈值公式、workload、affinity、event set、C2 语义**均未修改**；
a3 与 a4 双份聚合、r7–r10 全部 raw 与四份 schema-2 逐轮诊断（`/tmp/diag-schema2-r*.json`）
一并保留，不删除、不择优。环境能力边界已固化到
`docs/performance-memory-testing-playbook.md` §2.8，C1 报告
`docs/perception-resource-profiling.md` 已同步补注，避免后续会话重新论证。

**§8 stage-latency 300 s 的处置**：其 `valid=true` 与分位数作为**测量结果**成立；
"探针不显著扰动被测量"这一前提按上述边界标注为受限，不再作为待办阻塞项。

## 2026-07-31 §8 批次 B：stage-latency 300 秒正式测量

- run `/tmp/alien-c2-formal-stage300-20260731-v11`（含 `sha256sum.txt`），
  `valid=true`、`normal_completion=true`，全部 role 正常退出。
- 身份与目标：v11 `install-stage`，`target_sha256=d5be6ef1…`、
  `build_id=b91bb964…`、`RelWithDebInfo`、`stage_latency_option=ON`、
  `target_affinity=0`、`paired_source_identity_sha256=e3e9fbcc…`。
- 窗口 300.092305204 s，observation 3001、revision 3002（rev 805→3806）、
  snapshot 3001，10 Hz 精确；`final_known=230520` 与批次 A bounded plateau 一致，
  `map_epoch=1`、fingerprint 未变。
- 阶段分位数（full event set，各 3000 样本）：

  | 阶段 | p50 | p95 | p99 | max |
  |---|---|---|---|---|
  | callback 总 | 11.887 ms | 13.833 ms | 15.644 ms | 20.231 ms |
  | ├ mapper_apply | 5.946 ms | 7.125 ms | 7.935 ms | 10.065 ms |
  | └ snapshot_total | 5.819 ms | 6.970 ms | 8.121 ms | 11.018 ms |
  | &nbsp;&nbsp;└ snapshot_serialization | 5.623 ms | 6.749 ms | 7.860 ms | 10.726 ms |
  | state_publication | 27.99 µs | 51.30 µs | 150.17 µs | 480.82 µs |
  | read_transaction | 2.53 µs | 3.32 µs | 4.27 µs | 67.56 µs |

- 分解自洽：`mapper_apply.p50 + snapshot_total.p50 = 11.765 ms ≈ callback.p50
  11.887 ms`；callback p99 占 100 ms 周期 15.6%，全程无 backlog。
- 主导项为 `snapshot_serialization`（占 snapshot_total 的 96.6%）与 `mapper_apply`，
  两者近似各半；读事务与状态发布可忽略。该结论属测量事实，优化另立 finding，
  本任务不顺手优化。
- 内存字段（stage 构建、tracing ON）RSS/PSS/USS 峰值
  91 624 / 46 355 / 41 024 KiB，高于批次 A plain 的 62 484 / 44 586 / 39 884 KiB，
  差额为 LTTng 缓冲与 stage 构建开销；**bounded 内存结论仍以批次 A 三轮 plain 为准**，
  不用本 run 冒充。

### v9 与 v11 前缀的可比性（本次核实）

- 机器比对结论：`ws/src` 在 v9 与 v11 之间**完全相同**——tracked diff 逐字节一致
  （`40b4e1bd…`），untracked `ws/src`（23 文件）`diff -r` 无差异。
- 两前缀身份差异**仅来自 `scripts/`**：`lib/stage_latency_calibration.py`、
  `analyze-local-map-stage-calibration.py`、`test_analyze_local_map_profile.py`
  （CPU 中位数判据实现）及两个 driver 脚本。均为分析侧，不进入被测 ELF。
- 因此批次 A（v9 production）与批次 B（v11 stage）测的是同一份被测源码，数字可比；
  ELF SHA 不同仅因构建非比特级可复现。
- `scripts/profile-local-map.sh:97` 确认 paired-identity 门只作用于 `stage-latency`
  模式，`plain-sample` / `heaptrack` 不要求 —— 这是批次 A/B 前缀分工的依据，
  §9 沿用同一分工。

### 环境与灾备

- 2026-07-30 Docker Desktop 崩溃（引擎管道消失），从 `E:\Docker\Docker Desktop.exe`
  拉起后容器 `/tmp` 证据完好（200 条目）。
- 关键证据摘要已导出宿主机 `log/evidence-archive/`（66 份：各聚合 JSON/quality、
  pair manifest、全部 run 的 `sha256sum.txt` 与 `run-manifest.txt`），容器丢失时
  仍可验证证据链锚点。

---

## 任务收口决定（2026-07-31，经用户批准）

用户指示"不需要重新测试了，直接将结果和备注写入文档"，本任务以**受限交付**收口。

### 交付物

- **`docs/local-map-resource-profiling.md`**（509 行）：C2 冻结基线报告，含指标
  可信度分级、bounded/expanding 正式基线、容量拐点及其机制、探针校准的受限结论、
  AC1–AC8 逐条对照、raw 位置与复算命令。
- `docs/performance-memory-testing-playbook.md` §2.8：环境能力边界（完整论证）。
- `.trellis/spec/backend/performance-measurement-boundaries.md`：可执行约束规则。
- `docs/perception-resource-profiling.md`：C1 报告已补注指标可信度前提。
- `log/evidence-archive/`：151 份证据摘要（宿主机侧锚点）。

### 两项**未完成**且保持未勾选

1. **§5 item 1 — sanitizer 报告门无故障注入覆盖**。
   影响：报告 §4.5 的"无 ASan/LSan 错误"结论依赖一个未被反例验证的门。
   建议闭合方式：给 `scripts/profile-local-map.sh:1411-1414` 的门补一个合成测试，
   与其他工具门同一模式（活体注入不适用——需要往生产源码塞真 bug）。
   注意：改 `scripts/` 会作废 v11 pair 身份，须在任何后续配对测量之前完成。
2. **§10 — 未运行代码审核 subagent 复核到 `0 Blocking / 0 High`**。

两项均已写入报告 §8 的"未闭合的测量链缺口"，不隐藏。

### 未提交

按纪律，本任务**未执行 git commit / push / archive**，等待用户明确授权。

---

## 全部关闭（2026-07-31，用户决定）

用户指示"关于性能与内存测试的都关闭，不处理了，文档中描述现状就好"。
本任务及全部七个子 finding 一并关闭归档。

**两项未完成项保持未勾选，不假装完成**：

- §5 item 1：sanitizer 报告门无故障注入覆盖
- §10：未运行代码审核 subagent 复核

**子 finding 关闭时的实际状态**（各自 prd/implement 内有详细记录）：

| finding | 状态 |
| --- | --- |
| `07-27-c2-valgrind-throughput-blocker` | 已解决（Heaptrack + 测试负载 Memcheck 补偿） |
| `07-29-c2-capacity-bounds-phase-lag` | 已修复（O(1) known_bounds，提交 `eb78d46`） |
| `07-28-c2-stage-calibration-build-equivalence` | 已解决（v11 pair 交付） |
| `07-29-c2-stage-p99-tail-overhead` | 归因成立，但**其建议的中位数判据已被推翻** |
| `07-27-c2-memcheck-runtime-leak` | **未解决**，不再跟进 |
| `07-30-c2-perf-octomap-symbols-blocker` | **未解决**，不再跟进 |
| `07-31-c2-expanding-capacity-knee` | **机制已定位，量化未完成**，不再跟进 |

文档侧已同步：`docs/local-map-resource-profiling.md` §8 改为"当前限制与已知未解决
问题"，明确标注哪些**未解决且无人跟进**、以及"若要继续"的入手方向；
`docs/local-observation-map.md` 同步去掉了指向已关闭 finding 的跟踪表述。

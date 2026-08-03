# C1 外部性能与内存剖析 - 实施计划

## 1. 实施顺序

### 阶段 0：边界和 provenance

- [x] 记录五个感知包、相关文档和现有工作区状态。
- [x] 记录 OS、内核、capability、perf policy、CPU/cgroup/负载和镜像 ID。
- [x] 生成唯一 run-id，确认 build/install/raw 目录不存在。

### 阶段 1：工具与编排

- [x] Dockerfile 增加 perf、Heaptrack、Valgrind/Massif、sysstat、smem、ros2trace。
- [x] Compose 增加最小 PERFMON capability；不使用 privileged。
- [x] 当前容器安装同一工具并逐个做 version/smoke；perf software event 也失败时
  停止 CPU 阶段并记录 blocker；perf stat/record 的 control FIFO/ACK smoke 也
  必须通过。
- [x] 新增固定 mixed YAML，显式包含 fixture 和 input node 全部关键参数。
- [x] 新增 `scripts/profile-perception.sh`，实现精确 PID/进程组、ROS graph 门、
  参数回读、preflight/正式 CSV 分窗计数、按工具 PID/退出状态机、有效性和
  manifest；ASan/LSan smoke 也走同一编排门。
- [x] `bash -n`、短 plain-sample 和故障注入验证脚本能拒绝空载、低计数和强杀。
- [x] graph gate 要求四个 topic 的 publisher/subscription 精确为 `1/1`，拒绝
  ROS domain 碰撞或并发 run 混入的 endpoint。

### 阶段 2：profiling 构建

- [x] 独立构建五包 `RelWithDebInfo` + frame pointer + compile commands。
- [x] 检查目标含 debug info，compile command 含预期参数，保存 build-id/SHA/ldd。
- [x] 运行既有 60 项功能测试。

### 阶段 3：CPU 与 ROS 调度

- [x] 预热后按已校验 tracee PID attach，运行有效 600 秒 `perf stat`；检查同窗
  workload、control FIFO/ACK、实际时长和事件状态。
- [x] 预热后按已校验 tracee PID attach，运行有效 1500 秒 `perf record`；1200 秒
  DWARF run 和 300 秒固定周期探针因样本不足保持 invalid，60 秒 frame-pointer
  探针验证符号质量后，最终 run 得到 1337 samples、6.28% unknown、0 lost；检查
  control FIFO/ACK、同窗 workload、build-id 和热点。
- [x] 运行有效 60 秒 ROS trace；按目标 vpid 和同一窗口分别统计实际 Jazzy
  tracepoint 映射后的 callback/take/publish，任一核心类别缺失即不通过。
- [x] 与既有 0 丢帧、P95 0.477 ms 基线交叉说明，不把工具开销当生产延迟。

### 阶段 4：堆、错误和泄漏

- [x] 运行有效 300 秒 Heaptrack并生成可解析 report。
- [x] 用 `scripts/perception-profile-asan.cmake` 构建 ASan 版本，先运行
  detect_leaks=0 的测试和经统一 workload 门的 60 秒 smoke；Python launch
  runner/sink 显式预加载 `libasan:libstdc++`。
- [x] ASan 无错误后独立运行 detect_leaks=1 的 LSan 测试/smoke；以正常退出、
  workload 达标、日志无 sanitizer error 和 error exitcode 未触发判定干净 run。
  launch 测试仅抑制含 `python3.12` 的宿主泄漏，并保留 suppression 统计。
- [x] 运行有效 60 秒 Memcheck，分类 invalid access、definite/indirect/reachable。
- [x] 运行有效 180 秒 Massif，生成至少 20 个 snapshot 和 peak 时间线。
- [x] 运行三次有效 300 秒 plain-sample，保存 pidstat、smem、smaps 和 workload。

### 阶段 5：分析与复核

- [x] 新增 `docs/perception-resource-profiling.md`，记录环境、工具、命令、
  provenance、指标、热点、增长拟合、限制和 finding。
- [x] 将 64 B 外部动态加载器 finding 独立记录于报告；不修改业务代码。
- [x] 对照 AC1-AC10 检查非空证据、正常退出、时长、计数与符号化门。
- [x] 将 perf stat、perf lost samples、Heaptrack 汇总段和 Massif 峰值纳入机器
  gate；正向样例通过，缺事件/lost/缺段/少 snapshot 的负向样例均被拒绝。
- [x] 纠正 raw manifest 将完整 container ID 误标为 image ID 的 provenance 问题；
  报告记录宿主镜像 ID，runner 今后强制 `sha256:<64 hex>` 格式。
- [x] 将外部 profiler 的 PID/starttime、同窗 workload、正常退出和证据完整性
  合同沉淀到 `.trellis/spec/backend/quality-guidelines.md`。
- [x] 按仓库 AGENTS.md 和最近约定，使用 `gpt-5.6-sol / xhigh` 做最终实现复核；
  reviewer 直接修复高置信问题并运行静态/合成解析器回归。
- [x] 确认感知业务源码无本任务 diff，暂不提交、不推送。

### 最终实现复核（2026-07-26）

- 修复角色 `exec`/PID/starttime/PGID 稳定性、信号前身份复核、sampler 独立进程组、
  perf control stop ACK 和 `normal_completion` 状态机。
- 收紧 trace event list、perf build-id、Heaptrack/Massif/Memcheck 唯一产物与目标
  身份门，并为三次 plain-sample 增加不同目录、独立证据身份、正式时长、workload
  和样本完整性门。
- 新增 `scripts/test_analyze_perception_profile.py`，以合成文本覆盖相关解析器正反例。
- 未运行耗时 profiler；正式 AC 数字仍来自 2026-07-25 旧 runner 的原始产物，
  本轮只对原始已有字段交叉核对，未把新增门伪装为一次正式重跑。

## 2. 有效性门

- profiler 只测目标二进制并显式加载 mixed YAML；perf 预热后按 PID attach，
  启动插桩工具将全生命周期与稳态窗口分开解释。
- 两节点参数回读必须与完整 YAML 一致；PID 角色表必须无歧义。
- ROS graph 必须存在 3 个输入配对和 1 个输出 sink 配对。
- preflight sink 不计数；正式 sink 仅按 `t0-t1` 行偏移统计，必须达到
  `duration*27` 总数与每 ID `duration*9`，否则 run invalid。
- perf stat ≥600 s；perf record ≥120 s 且 ≥1000 samples、unknown≤20%；二者均
  有 enable/disable ACK，正式 sink 的 `t0-t1` 窗口内含于 enabled 窗口。
- trace ≥60 s，实际 Jazzy 事件先明确映射，目标 vpid 同窗的 callback/take/
  publish 三类分别非零；任一类缺失或无法按进程过滤则 invalid。
- Heaptrack/Memcheck/Massif/LSan 必须正常退出并可解析；强杀 run invalid。
- plain-sample 每次 ≥300 s，pidstat/smem 行必须匹配记录的 target PID。
- 所有 raw 产物有 SHA-256，manifest 有源码、镜像、二进制与工具 provenance。

## 3. 验证命令形状

```bash
bash -n scripts/profile-perception.sh

scripts/profile-perception.sh perf-stat <install> <new-output> 600
scripts/profile-perception.sh perf-record <install> <new-output> 1200
scripts/profile-perception.sh ros-trace <install> <new-output> 60
scripts/profile-perception.sh heaptrack <install> <new-output> 300
scripts/profile-perception.sh asan-smoke <asan-install> <new-output> 60
scripts/profile-perception.sh lsan-smoke <asan-install> <new-output> 60
scripts/profile-perception.sh valgrind-memcheck <install> <new-output> 60
scripts/profile-perception.sh valgrind-massif <install> <new-output> 180
scripts/profile-perception.sh plain-sample <install> <new-output> 300
```

ASan/LSan 使用独立 colcon build/test 与 mixed smoke，不复用 profiling install。

## 4. 审核记录

### 首轮方案审核

- 模型：`gpt-5.6-sol`，reasoning effort `xhigh`，只读，10 分钟上限。
- 结论：暂不通过；发现空载节点、未定义 sink、trace 混进程、perf 权限、强杀
  证据截断、缺少 pidstat/smem、workload 不可复现、ASan/LSan 混跑、验收可空
  通过、符号化不足、趋势不可复核和 provenance 不足等 12 项有效问题。
- 修订：显式 mixed 参数和 graph/消息计数门；CSV sensor-data sink；vpid trace
  过滤；perf 阻断条件；正常退出门；独立 ASan/LSan；重复 plain-sample；
  build-id/SHA/image/tool provenance；非空样本、时长和符号化验收。
- 审核关于“指定模型未授权”的 finding 不采纳：仓库 `AGENTS.md` §5.2 要求方案
  与代码审核，最近一次用户明确约定的模型为 `gpt-5.6-sol / xhigh`。

### 第二轮方案复核

- 模型：`gpt-5.6-sol`，reasoning effort `xhigh`，只读；首次调用因 429 未形成
  结论，缩小为规划正文后在 10 分钟窗口内完成。
- 结论：不通过；发现参数 YAML/生效证明、Jazzy tracepoint 名称与门槛、不同
  profiler 的 PID 语义、退出信号顺序、预热污染 sink 计数、perf 包含启动阶段、
  LSan 干净 run 无摘要等 7 项问题。
- 修订：完整双节点 YAML 与参数回读；三类实际 tracepoint 均非零；模式化 PID
  状态表和退出状态机；preflight/正式 sink 分窗；perf 预热后 attach；插桩工具
  分开解释全生命周期与稳态；ASan/LSan 统一 workload 门和 error exitcode 规则。

### 第三轮闭环复核

- 模型：`gpt-5.6-sol`，reasoning effort `xhigh`，只读。
- 结论：其余 6 项已闭环；仅剩 perf 早于正式 sink 启用，无法证明 CPU 数据与
  workload 来自同一窗口。
- 修订：正式 sink 先配对，perf 以 disabled 状态 attach；enable ACK 后记录
  `t0`/起始偏移，记录 `t1`/结束偏移后 disable 并等待 ACK。不支持 control
  FIFO/ACK 时 AC3/AC4 保持未完成。
- 最终窄复核：同一模型确认窗口协议无待修高置信问题，结论通过。

## 5. 回滚点

- Noble 包名不可解析：`apt-cache` 确认后定向修订，不删除其他镜像依赖。
- LinuxKit 阻止 software perf：CPU 阶段阻塞并记录，不改 privileged。
- 脚本清理失败：仅终止 manifest 中的 PID/进程组，核对后重试。
- profiler 暴露业务缺陷：建立独立修复任务，不在分析脚本中绕过。

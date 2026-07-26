# C1 外部性能与内存剖析

## Goal

使用系统 profiler、堆分析器、编译器 sanitizer 和 ROS 2 tracing，补齐 C1
感知输入链路目前只有自写计时测试、没有外部工具证据的缺口。结果用于建立
进入 C2 前的可归因基线；本任务负责测量和定位，不在没有证据时优化实现。

## Background

- 现有固定基线覆盖 Adapter、MapperHealthGate 和 120 帧 ROS 发布链路：
  0 丢帧、P95 0.477 ms。
- C1 计划写有进程 CPU 10% 和内存 100 MB 的回滚触发值，但尚无 CPU
  profiler、RSS/PSS、堆分配、泄漏或 ROS executor 调度证据。
- 开发环境是 Ubuntu 24.04 ROS 2 Jazzy 容器，宿主内核为
  `6.10.14-linuxkit`；容器有 `SYS_PTRACE`、`seccomp:unconfined`，22 个可用
  CPU，无 cgroup CPU quota，`perf_event_paranoid=2`。
- 当前镜像已有 LTTng 与底层 `tracetools`，但没有 `ros2 trace` CLI、`perf`、
  Heaptrack、Valgrind/Massif、`pidstat` 或 `smem`。

## Requirements

### R1：可复现构建与 provenance

- Profiling 使用独立 `RelWithDebInfo` 构建：`-O2 -g -DNDEBUG`、
  `-fno-omit-frame-pointer`、`CMAKE_EXPORT_COMPILE_COMMANDS=ON`。
- ASan/LSan 使用另一套独立构建，不污染性能测量产物。
- 开发镜像显式安装外部工具；运行期 ROS 包依赖不增加 profiler。
- 每个 run 记录源码 revision/dirty path、镜像 ID、OS/内核、编译器、RMW、
  CPU/cgroup/宿主负载、工具版本、目标二进制 SHA-256/build-id/`ldd`、原始
  产物 SHA-256、实际时长和退出状态。

### R2：固定 mixed 工作负载与有效性证明

- 显式参数固定为 2 路 181 点 2D LaserScan + 1 路 16×360 点 3D PointCloud2；
  发布周期 0.1 s、range 5.0 m、seed 17、默认 16 线角表。
- `perception_input_node` 必须显式加载 `sensor_descriptors_mixed.yaml` 的等价
  参数；fixture 与 input 使用包含实际节点名和 `ros__parameters` 的同一参数
  文件，并通过 `--ros-args --params-file` 加载。启动后回读两节点的关键参数，
  任一值不符立即判 invalid，禁止用默认空 `sensor_ids` 运行。
- RMW 固定为 `rmw_fastrtps_cpp`，目标进程固定到一个逻辑 CPU；fixture 和
  sink 固定到另一 CPU。记录 affinity、cgroup quota、load average 和 perf
  multiplex 比例。
- fixture 启动后验证三个输入 topic 各有一个 publisher 和目标订阅；preflight
  sink 使用 `ros2 topic echo --qos-profile sensor_data --csv --field sensor_id`
  验证 front/rear/top 后退出。正式测量使用新的 sink，并以测量窗口起止行偏移
  排除预热和清理期消息。
- 预热后输出期望约 30 Hz；每个有效 run 的 sink 计数必须达到实际测量时长
  `× 30 × 90%`，三种 sensor ID 各达到 `× 10 × 90%`。
- profiler 只测已安装的 `perception_input_node` 二进制。perf 在预热后按已校验
  的目标 PID attach，并通过 control FIFO/ACK 与正式 sink 的起止偏移同步启停；
  Heaptrack/Valgrind/Sanitizer 从进程启动插桩并将启动阶段与稳态窗口分开解释。
  fixture、sink 和 profiler 自身不计入目标进程资源结果，各 profiler 必须分开
  运行。

### R3：CPU 与 ROS 调度证据

- `perf stat` 记录 task-clock、instructions、cycles、branch/cache miss、context
  switch、migration 和 time enabled/running；缺失事件标记 unavailable。
- `perf record` 使用用户态事件与明确 call-graph 模式，保存 build-id，报告采样
  总数、unknown 比例和前 10 个热点。至少 1000 个样本，目标/工作区符号可解析；
  unknown 超过 20% 时结果不完整，必须补符号或说明无法归属的模块。
- LinuxKit 即使软件事件也拒绝时，AC3/AC4 保持未完成；不能用自写计时器替代。
- `ros2 trace` 保存 Jazzy 实际事件列表，并将 callback、take、publish 显式映射
  到 `ros2:callback_start/end`、`ros2:rcl_take`/`ros2:rmw_take`、
  `ros2:rclcpp_publish`/`ros2:rcl_publish` 等实际存在的事件；按目标 PID/vpid
  和测量时间窗过滤 fixture 与 sink。三类事件在目标 vpid 下必须分别非零，任一
  类不可用、空 trace 或无法区分进程时 AC5 不通过。

### R4：进程退出与证据完整性

- 脚本分别记录 `launcher_pid`、`tool_pid`、`tracee_pid` 和 `pgid`；plain/trace/
  perf、Heaptrack 和 Valgrind 使用各自的 PID 解析与校验规则，歧义即 invalid。
  只清理本 run 创建的进程，不按模糊名称 kill。
- perf attach 在固定窗口结束后自行落盘，再向目标发送一次 SIGINT；Heaptrack 向
  唯一 tracee 发送 SIGINT并等待 controller 落盘；Valgrind 向客户进程语义对应
  的 PID 发送一次 SIGINT；ROS trace 通过 tracer 的正常 stop/destroy 流程落盘。
  随后等待 sink 刷盘，最后清理 fixture。每一角色分别记录退出码。
- Heaptrack、Massif 和 Memcheck run 若需要 SIGTERM/SIGKILL、工具执行失败、
  摘要缺失或原始产物不能解析，必须标记 invalid 并重跑。Memcheck 的专用
  error-exitcode 表示完整 run 发现 definite/indirect finding，不等同工具执行失败；
  只有完整摘要存在时可作为有效问题证据。LSan 无泄漏时允许无原生泄漏摘要，
  但必须正常退出、workload 达标、日志无 sanitizer error 且指定的非零 error
  exitcode 未触发。
- perf/trace 被迫结束时也要记录为 invalid；只有完整摘要、达到实际时长和有效
  workload 计数的 run 才能作为基线。

### R5：堆、泄漏和内存错误证据

- Heaptrack 报告峰值堆、总分配、临时分配和主要分配调用栈。
- ASan 先以 `detect_leaks=0:halt_on_error=1` 跑测试和 mixed smoke，确认没有
  越界/UAF；通过后再以 `detect_leaks=1:halt_on_error=1` 正常退出完成 LSan。
  两次 smoke 均使用同一参数回读、graph、正式 sink 和逐传感器计数门。
- Valgrind Memcheck 使用 definite/indirect leak error gate；ROS/DDS 的
  still-reachable 单独记录，不直接判为业务泄漏。
- Massif 记录含栈的时间线。plain-sample 独立调用 `pidstat` 和 `smem`，1 秒
  采样 CPU/RSS，10 秒采样 PSS/USS。
- plain-sample 重复 3 次，每次 300 秒；排除前 60 秒预热后计算后 240 秒的
  RSS/PSS 增长。连续三次同向增长超过 1 MiB/min 才标记“疑似持续增长”，并由
  Heaptrack/Massif 调用栈复核；单次曲线不能直接判泄漏。

### R6：结果与范围

- 原始 `perf.data`、Heaptrack、Massif 和 LTTng trace 保存在唯一 `/tmp` 目录，
  生成 manifest 与 SHA-256；仓库保存复现脚本、参数、文本摘要和 provenance。
- 原始产物只承诺在当前容器生命周期内可复核；需要长期审计时按 manifest 导出，
  不把大型二进制加入 Git。
- 若发现超限、泄漏或热点，建立独立 finding；本任务不顺带修改感知业务行为。
- 保留用户的其他工作区改动，不执行提交或推送。

## Acceptance Criteria

- [x] AC1：所有工具有 version/smoke 证据；`perf` 权限不足时 AC3/AC4 不得通过。
- [x] AC2：五包 profiling 构建含 debug info、frame pointer、compile commands，
  且既有 60 项功能测试通过。
- [x] AC3：`perf stat` 在预热后 attach 目标 PID，通过 control FIFO/ACK 与正式
  sink 起止偏移同步并有效运行至少 600 秒；计数达标，指标含 time enabled/
  running，缺失事件明确标记。
- [x] AC4：`perf record` 使用同一 control FIFO/ACK 同步窗口，有效运行至少
  120 秒、样本不少于 1000，调用栈满足符号化门，报告前 10 个热点。
- [x] AC5：ROS trace 有效运行至少 60 秒，按目标 vpid 和同一时间窗得到分别非零
  的 callback、take、publish 事件并记录实际事件映射与数量；空/混进程或任一
  核心类别缺失均不通过。
- [x] AC6：Heaptrack 有效运行至少 300 秒并正常退出，报告峰值、总分配、临时
  分配、leak 与前 10 个分配调用栈。
- [x] AC7：ASan 测试/smoke 经同一 workload 门且无内存错误后，独立 LSan 经
  同一门正常退出；无 sanitizer error 与 error exitcode 未触发可形成无泄漏结论，
  不要求干净 run 必须打印泄漏摘要。
- [x] AC8：Memcheck 和 Massif 均正常退出、产物可解析；leak 类型、峰值时间线
  和工具限制有明确记录。
- [x] AC9：三次 plain-sample 均达到 300 秒和 workload 门，`pidstat`/`smem`
  目标 PID 正确，稳态 CPU/RSS/PSS 与增长判定可复算。
- [x] AC10：报告含 provenance、原始产物校验和、测量值、开销、限制和 finding；
  感知业务源码无本任务 diff。

## Out Of Scope

- C2 mapper、OctoMap、多机链路、Gazebo、真实 LiDAR 或网络带宽分析。
- 为取得更好数字修改感知业务实现。
- 将大型 profiler 原始产物加入 Git。

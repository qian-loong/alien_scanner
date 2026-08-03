# C2 性能与内存基线

## Goal

为已经完成并归档的 C2 authoritative local observation map 建立可复现、可归因的
性能与内存基线，供后续 C3 keyframe/delta/serialization 做同条件回归比较。任务只
负责测量、定位和形成基线；发现热点、泄漏或超限时建立独立修复任务，不为取得更好
数字修改 C2 occupancy、health、epoch/revision 或射线语义。

## Background

- C2 功能提交为 `8dbdba5`，Release 8 包构建和 `248/248` 测试已通过。
- 现有 `ResourceBaseline` 只报告单帧固定 fixture 的约 `1763 us`、进程历史峰值
  RSS `20280 KiB`、known `1567`、occupied `113`；该结果混合测试进程启动历史，
  不能证明长期吞吐、稳态内存或业务进程泄漏。
- 当前真实场景是 360-beam、10 Hz、20 秒直线扫描，最终约 196 个 map revision；
  它可作为正确性 oracle，但窗口过短，不能单独形成内存增长结论。
- 重构前 `ScanAccumulatorNode` 的默认 `max_points=500000` 限制的是 hit-only
  `/cloud_map` 点数，超限 FIFO 丢弃旧点；当前 C2 不经过该 accumulator，也没有
  500000 known-voxel 硬上限。性能基线仍需把 500000 exact known voxels 当作规模里程碑。
- 容器已具备 `perf`、`heaptrack`、`valgrind`、`pidstat`、`pmap`；归档任务
  `07-25-c1-perception-resource-profiling` 已提供 PID/provenance、workload、退出状态、
  raw artifact 和 profiler 有效性门，可定向复用而不重写一套不一致脚本。

## Requirements

### R1：冻结 provenance 与被测边界

- 以 `perception_local_map_node` 为主要被测进程；正式长时 run 由单一确定性 fixture
  直接发布合法 authoritative health/pose/observation。scanner、C1 input、pose gate
  只参加短程等价验证；fixture/sink/oracle/profiler 均分开记录，不计入 C2 进程结果。
- 性能构建使用独立 `RelWithDebInfo`、`-O2 -g -DNDEBUG`、frame pointer；
  ASan/LSan 使用另一独立前缀，不污染性能数字。
- 每次 run 记录源码/dirty state、镜像、内核、CPU/cgroup、RMW、工具版本、二进制
  SHA/build-id/`ldd`、实际窗口、PID/starttime/affinity、退出状态和产物 SHA-256。

### R2：分离两类内存增长

- **bounded steady-state workload**：重复扫描固定空间，使 known voxel 数收敛；用于
  判断 RSS/PSS/heap 是否继续同向增长，排除把正常地图扩张误判为泄漏。
- **expanding-map workload**：在可无限平移的解析隧道环切面中持续产生合法 FullRay
  observation；同时
  记录 known/free/occupied voxel、revision 和 bounds，用于计算每 revision、每新增
  known voxel 的 RSS/PSS/heap 增长。
- 两类 workload 均固定 seed、beam/range/resolution、输入频率和构建类型，并证明
  observation/revision 数、epoch、fingerprint、diagnostics，以及冻结的 body/sensor
  frame 与静态外参均与预期一致。
- 正式 profiler 矩阵前运行无重量级 profiler 的 expanding capacity ramp；oracle 必须
  提前给出 100k/250k/500k/750k known-voxel 对应的 revision/stamp，最长预测/运行
  600 秒。三个互斥桶固定为 crossing 前、包含首次 crossing、crossing 后各 200
  applied samples；只有三桶完整落入窗口才标记 `covered`，否则区分 `not_reached` 与
  `crossing_without_post_window`。不得调整 beam、resolution、range、频率或解析几何造线。

### R3：延迟、吞吐与 CPU

- 分别报告 C2 observation callback/mapper apply、revision publication、
  revision-locked read transaction 和 visualization-only OctoMap snapshot/serialization
  的样本数、平均、p50、p95、p99 和最大耗时；不得只报告整段 wall time。
- 报告 accepted/applied/no-evidence/rejected、输入频率、revision rate、积压/丢失和
  CPU 使用率；10 Hz 链路不得用重复 stamp 或降低输入负载制造通过。
- `perf stat/record` 与 workload 统计使用同一正式窗口，并保存 enabled/running、
  sample count、unknown ratio、build-id 和前十热点。

### R4：内存与错误检测

- plain sample 重复运行并报告 RSS/PSS/USS、增长斜率、峰值和地图规模；只有固定空间
  已收敛后仍重复出现的同向增长才标记疑似泄漏。
- Heaptrack 报告峰值 heap、总/临时分配和主要调用栈；Massif 报告时间线；Memcheck
  区分 definite/indirect/still-reachable。
- ASan prefix 先覆盖 C2 package closure 全量功能测试与 workload 的内存错误，再独立
  启用 LSan；工具必须正常退出、
  workload 达标且报告可解析，强杀或缺摘要的 run 不形成通过证据。

### R5：结果边界

- 仓库只保存复现脚本、冻结参数、文本/机器可读摘要和 provenance manifest；大型
  profiler 原始产物保存在唯一外部目录并以 SHA-256 引用，不加入 Git。
- 本任务不修改 C2 业务语义、不优化热点、不扩展 C3 协议；发现问题建立独立 finding。
- 保留工作区其他改动，不提交、不推送，除非用户另行明确授权。

### R6：正式完整基线矩阵

- bounded workload 执行完整外部证据矩阵：同窗 `perf stat`、`perf record`、ROS
  tracing、Heaptrack、Massif、Memcheck、ASan/LSan，以及三次独立 plain sample。
- 阶段延迟使用独立 bounded run，正式窗口 `>= 300 s`；该 run 与 perf、堆工具和
  plain sample 不并发，且必须报告探针对同条件无探针运行的扰动校准。
- expanding workload 执行三次独立资源采样和至少一次可归因的堆分配剖析；堆工具
  必须提供可与 map checkpoint 对齐的时间线，形成
  revision、known voxel、bounds 体积与 RSS/PSS/heap 增量的对应关系；不要求对每个
  profiler 重复整套 expanding run。
- expanding 另执行一次阶段延迟定向 run；可达时按 `<500k`、跨越 500k、`>500k`
  exact known voxel 分桶；非 `covered` 状态保留其原始分类并按连续 map-size 分桶，
  避免只用小地图的 bounded latency 代表大地图成本。
- 正式窗口沿用 C1 已验证的量级：`perf stat >= 600 s`、`perf record >= 120 s`
  且样本/符号化门达标、trace `>= 60 s`、Heaptrack `>= 300 s`、Massif
  `>= 180 s`、Memcheck/ASan/LSan 各 `>= 60 s`、plain sample 每次
  `>= 300 s`。若工具开销导致 10 Hz workload/revision 门失败，该 run 无效并重跑，
  不降低业务负载换取通过。
- 预计完整正式运行约 `1.5-2 h`，运行模式严格顺序执行，禁止多个 profiler 并发
  污染 CPU/内存结论。

## Acceptance Criteria

- [ ] AC1：独立 profiling 与 sanitizer 构建可复现，Release 与 ASan prefix 的 C2
  package closure 全量功能测试保持通过。
- [ ] AC2：bounded 与 expanding workload 均通过参数、graph、消息/revision、epoch、
  fingerprint、diagnostics 和地图规模有效性门；graph 明确排除旧 accumulator/cloud_map。
  capacity ramp 按 oracle 结果跨越 500k 后继续增长，或明确记录 `not_reached`/
  `crossing_without_post_window`；不能把旧 FIFO 或调整负载制造的 crossing 计为通过。
- [ ] AC3：延迟/吞吐报告包含 mapper apply、state publication、read transaction、
  OctoMap snapshot 的 average/p50/p95/p99/max 与样本数；阶段延迟正式窗口至少
  300 秒，并证明探针事件完整、扰动有界且没有输入积压被隐瞒。
- [ ] AC4：同窗 `perf stat/record` 有完整 provenance、计数、符号化质量和热点报告。
- [ ] AC5：重复 plain sample 给出 CPU、RSS/PSS/USS、固定空间增长斜率和 expanding
  workload 的每新增 known voxel 内存成本；仅 `covered` 时形成 500k 前/跨越/跨越后
  分段结论并可由 raw 重算，其他状态保存实际最大 known count 且不得冒充分段完成。
- [ ] AC6：Heaptrack、Massif、Memcheck、ASan/LSan 均有正常退出和可解析证据；任何
  finding 明确区分业务缺陷、DDS/ROS still-reachable 与 profiler 开销。
- [ ] AC7：生成面向 C3 的冻结 baseline 报告，明确可比较条件、当前限制和独立 finding；
  C2 业务源码在任务前后无未授权语义 diff。
- [ ] AC8：bounded 完整 profiler 矩阵与 expanding 定向矩阵均达到 R6 的正式窗口、
  workload 和退出完整性门；无任何空跑、强杀或不完整产物被计为通过。

## Out Of Scope

- C2 算法优化、数据结构替换、分辨率/量程/频率降级或验收阈值放宽。
- C3 keyframe/delta/serialization 的实现与性能。
- 多机通信、shared/global map、真实无线链路、Gazebo/真实传感器或功耗分析。
- 把单次 RSS 波动、地图正常扩张或 ROS/DDS still-reachable 直接宣称为泄漏。
- 为复刻旧 `/cloud_map` 行为而给 C2 重新引入 500000 voxel FIFO/cap。

## Notes

- 用户已确认采用正式完整基线矩阵。当前仍处于 planning；方案审核和最终实施计划
  未通过前不启动 profiling 实现或长时运行。

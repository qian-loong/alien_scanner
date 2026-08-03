# C2 性能与内存基线 - 技术设计

## 1. 边界与原则

本任务测量已验收的 `perception_local_map_node`，不优化 C2，也不改变 occupancy、
health、pose、epoch/revision、FullRay 或 authoritative query 语义。正式结果只统计
目标进程；trajectory、scanner、pose gate、C1 input、sink/oracle 和 profiler 都是
独立角色，只用于施加载荷和形成证据。

现有 `cave_full_ray_scene` 保持为 20 秒正确性 oracle。它在直线终点主动停扫，不能
通过放宽冻结配置、循环启动或重复 stamp 改造成长时 workload。

```text
AuthoritativeProfileFixture ---- Health / Pose / LidarObservation ----+
                                                                        v
Profile sink/oracle <------------------------ PerceptionLocalMapNode (target)
                                                   |
                                                   +-- State / OctoMap / diagnostics

短程等价门：TreeCave -> CaveLaserScanNode -> PoseGate -> C1 Input -> C2
```

## 2. Profiling 专用资产

新增独立 `perception_profiling` C++ 包，运行期不引入 Python 节点：

- ROS-free scenario core 用单一 sequence counter 生成 health、pose、完整 FullRay
  observation 和 payload digest，支持 `bounded`、`expanding` 与短程 canonical 模式；
- 薄 fixture 节点在 pose 已领先一个 20 Hz tick 后以 10 Hz 发布 observation，使用
  同一固定 SessionID、stamp 序列和 fingerprint；fixture 内的
  `tf2_ros::StaticTransformBroadcaster` 发布冻结的 `base_link <- profile_scan_link`
  外参，translation 为 `[0, 0, 0]`，quaternion xyzw 为 canonical
  `[0.5, 0.5, 0.5, 0.5]`；
- C++ sink 记录 observation、state、diagnostics 和必要的 transport 完整性数据；
- ROS-free oracle 直接消费 scenario core 的同一 exact sequence，驱动真实
  `LocalObservationMapper + OctoMapBackend`，每 100 revision 用 exact receipt 取得
  read transaction，并由 `for_each_known_cell()` 输出 known/free/occupied、bounds、
  epoch/revision 和 last observation stamp。

sink 保存每帧完整输入或 schema 固定的 digest，production state 的 last stamp、
revision 和 bounds 必须与 oracle 表一致。profile launch 可关闭 target；runner 始终
自行启动唯一 target，确保 Heaptrack/Valgrind/Sanitizer 的 tracee PID 无歧义。

正式长时 run 不经过异步 scanner/C1 链，避免 wall timer、DDS delivery 和 pose-gate
调度使 oracle 不可复现。现有完整 TreeCave -> scanner -> gate -> C1 -> C2 链保留为
短程等价门：验证 canonical 模式的 descriptor、FullRay payload、pose/health、fingerprint
和最终 C2 map oracle；其资源不进入正式 C2 数字。

## 3. 两类 workload

共同冻结：10 Hz unique observation、20 Hz pose、360 beams、FullRay、range
`[0.1, 30] m`、0.2 m resolution、0.5 m/s、同一 clock domain、稳定 producer
session/fingerprint 和单一 mapper epoch。map/body/sensor frame 固定为
`map`/`base_link`/`profile_scan_link`，oracle 使用同一 body-from-scan quaternion。
不得因 profiler 开销降低频率或 beam 数。

正式 payload 是垂直于 +X 的解析隧道环切面：有限 returns 落在冻结的椭圆壁上，固定
岔口扇区使用 `+inf` 表示合法 no-return；不注入 NaN、负无穷、below-min 或 above-max。
no-return 从 sensor origin 穿越的 voxel 开始标 free，并排除 range_max endpoint；
range_max 端点保持 unknown。该横截面
可沿 X 无限平移，不依赖有限的 TreeCave geometry。TreeCave seed 42 只用于短程等价门。

### 3.1 bounded steady-state

在 `x=[1,11]` 做连续三角波，端点直接反转速度而不发布零速或位置跳变，姿态和高度
不变；每帧使用相同解析横截面。正式窗口前，oracle 必须证明连续 600 revisions 的 exact known/free/occupied
计数和 bounds 均不变；否则 workload 无效，不能开始进程内存斜率和 heap 时间线分析。

### 3.2 expanding-map

从 `x=1` 以 0.5 m/s 沿 +X 单调前进，并平移同一解析横截面。每 100 revisions 建 checkpoint；允许单次
revision 因 0.05 m 步长小于 voxel 分辨率而没有新增 cell，但整个 checkpoint 序列的
known count 和 X bounds 必须增长。

### 3.3 旧 500000 上限与 C2 规模里程碑

旧 `drone_scanner::PointCloudAccumulator` 的 `max_points=500000` 是 `/cloud_map` 的
hit-point 容量，超出后 FIFO 丢弃最早点。当前 authoritative C2 launch 明确不启动
`scan_accumulator`/`cloud_map`；`OctoMapBackend` 同时持有 OcTree 和 `known_cells`，但
没有 500000 voxel cap。因此正式 C2 不会在该值自动收敛，不能把 500k 后的平台误判为
旧限制生效。

500k 改作可观测规模里程碑。正式 profiler 前先用 oracle 预测同一冻结 expanding
sequence 最长 600 秒内的 100k/250k/500k/750k exact-known 首次到达 revision/stamp。
定义 `crossing_revision` 为首次 `known > 500000`，`required_end_revision` 为
`crossing_revision + 300`。三个互斥桶在运行前冻结：pre 为
`[crossing-299, crossing-100]`，crossing 为 `[crossing-99, crossing+100]`，post 为
`[crossing+101, crossing+300]`，各恰好 200 applied samples。只有
`crossing_revision >= 300` 且 `required_end_revision <= 6000` 时状态为 `covered`；没有
crossing 为 `not_reached`，有 crossing 但三桶不完整为
`crossing_without_post_window`。

`capacity-ramp` 使用后续非 sanitizer 正式矩阵的同一 RelWithDebInfo install prefix、
target ELF SHA/build-id、参数和 affinity，只开 target 资源采样、sink 和 oracle，不开
perf/trace/堆工具。`covered` 时运行到 required-end checkpoint 加 drain；其余状态运行
到 600 秒或安全门。只有 `covered` 才形成 500k 前/crossing/后的分段证据；其他状态记录
实际最大 known，不调整解析横截面、频率、beam、resolution、range 或 mapper 参数造线。

capacity ramp 的 graph gate 明确拒绝 `scan_accumulator`、任何 `cloud_map` topic、
`/scan_returns` 和 legacy `octomap_builder`。每秒检查 10 Hz/revision/backlog 和资源
headroom：有限 cgroup 下 `memory.current >= 80% memory.max`，或 host
`MemAvailable <= 2 GiB`，或 `memory.events` 的 `oom/oom_kill` 增加时正常停止并建立 C2
finding，不启动后续 1.5-2 小时正式矩阵。该 ramp 不贡献正式 baseline 数字；最长约
10 分钟，用于提前暴露大图成本，避免长 profiler run 中途作废。

两类 run 都要求：正式窗 distinct observation 数达到 10 Hz 容差门、revision delta
等于预期 applied 数、epoch 不变、last stamp 延迟有界、fingerprint/health/capability
正确且 mapper diagnostics 无 rejection/backend fault。

## 4. Revision、voxel 与进程内存对齐

`LocalMapState` 没有 voxel 总数，OctoMap wire 也没有 epoch/revision，且后者只用于
可视化；因此禁止按 topic 到达顺序把 snapshot leaf count 冒充 authoritative count。

对齐键使用 `last_observation_stamp`：

1. fixture core 与 oracle 输出 `sequence/stamp/payload_digest ->
   epoch/revision/exact counts/bounds`；
2. production sink 保存实际 authoritative 输入的 sequence/stamp/digest，以及收到 state
   时的本机 monotonic time、stamp、revision、bounds、changed count 和 fingerprint；
3. checkpoint sampler 在已验证 target PID 上读取 `smaps_rollup`，并连接最近且不晚于
   sample 的 state；
4. analyzer 记录时间和 revision skew，要求 production state 与 oracle checkpoint 的
   epoch/revision/stamp/bounds 一致，缺帧、reset 或拒绝使整次 run invalid。

bounded 只从 plateau 起拟合三轮 RSS/PSS/USS 斜率；heap 趋势来自 Heaptrack/Massif
时间线。expanding 报告每新增 exact known voxel 的 RSS/PSS 增量，并以
`heaptrack_print -M` 产生的 Massif-compatible heap 时间线对齐 checkpoint，报告 heap
增量；同时保留 revision、free/occupied、bounds span/volume 原始列。smaps private
memory 不标作 heap，正斜率是地图扩张成本，不称为泄漏。

expanding 的进程内存、heap 和阶段延迟仅在 capacity 状态为 `covered` 时统一按
上述 pre/crossing/post 固定互斥区间分桶，每桶恰好 200 applied samples；
其他状态保留原状态并继续按连续 known-voxel 自变量分析。500k 不是算法阈值或人为
截断点。

## 5. 阶段延迟

正式 CPU、内存和 hotspot 使用无阶段探针的 RelWithDebInfo 二进制。另设至少 300 秒
的 bounded `stage-latency` run，报告以下 entry/return 配对：

| 指标 | 边界 |
| --- | --- |
| callback envelope | `PerceptionLocalMapNode::on_observation` |
| mapper apply | `LocalObservationMapper::submit_observation` |
| state publication | observation callback 内的 `publish_state` |
| read transaction | receipt overload `acquire_read_transaction` |
| snapshot serialization | `OctoMapSnapshotBridge::materialize` |
| snapshot total | `publish_octomap` |

首选对安装后 ELF 使用 LTTng/perf userspace function probe，并在正式构建上重新保存
`nm -C`、build-id 和 SHA。若容器/kernel 不支持可靠 entry/return uprobe，允许使用
专用 latency prefix：CMake profiling option 默认 OFF，仅在上述边界编译 LTTng-UST
begin/end tracepoints；普通 profiling 和产品构建不含该 provider。此 fallback 不增加
业务字段、topic、日志或 per-cell instrumentation。

probe run 必须无丢事件、无 unmatched pair，按 PID/TID/嵌套关系过滤，只保留 applied
callback 内的 state/snapshot。输出 count/mean/p50/p95/p99/max。正式前运行各至少
120 秒、1200 applied samples 的 unprobed、callback-only probe 和 full nested probe：

- unprobed 与 full probe 的 revision rate 都必须达到 10 Hz、无 backlog；target 平均
  CPU 差异不超过 `max(5% relative, 0.5 percentage point)`；
- callback-only 与 full probe 的 callback p50/p95/p99 差异分别不超过
  `max(10% relative, 50 us)`。

超过任一预注册门时不事后放宽阈值，先降低探针扰动或令 AC3 阻断。标准 ROS trace
仍独立运行至少 60 秒，用于 callback/take/publish 调度证据，不能代替内部阶段耗时。

## 6. Runner 与 analyzer 复用

不复制 73 KiB 的 C1 runner。保留现有四参数 C1 CLI 和行为，通过回归保护后抽取：

- shell common：PID/starttime/PGID、role monitor、provenance、perf control ACK、
  tool-specific normal stop、artifact hash 和 report completeness；
- Python common：manifest、pidstat/smem/smaps、percentile/slope 和 profiler report parser；
- `profile-perception.sh` 继续作为 C1 wrapper；新增薄 `profile-local-map.sh` 只定义 C2
  target、辅助 roles、scenario、graph/parameter/workload gates；
- `analyze-perception-profile.py` 保持 C1 compatibility；新增 C2 analyzer 实现 plateau
  选择、oracle join 和 expanding attribution。

每个 run 使用唯一 ROS domain 和新输出目录。正式 graph gate 精确验证 fixture、target、
sink、`/tf_static` 及 observation/pose/health/state/OctoMap endpoints，并回读
`base_link <- profile_scan_link` 的三轴变换与 quaternion；短程等价门另行验证 scanner、
pose gate 和 C1 input。每个必需 child role 都记录 PID/starttime/PGID，不能只记录
launch parent。

## 7. 构建、provenance 与原始产物

使用三个互不混用的前缀：

- production-like profiling：`RelWithDebInfo -O2 -g -DNDEBUG` + frame pointer；
- latency fallback：同配置，仅开启 profiling tracepoint option；
- sanitizer：`-O1 -g -DNDEBUG -fsanitize=address`，ASan 与 LSan 分开运行。

正式 run 前记录 source revision、完整 dirty patch/hash、镜像 ID、内核、CPU/cgroup、
RMW、capabilities、工具版本、目标及依赖 ELF SHA/build-id/ldd/nm、实际窗口、所有角色
PID/starttime/affinity、退出码和 artifact SHA-256。正式结果可以在明确记录 patch 的
dirty tree 产生，但最终报告必须能唯一重建构建输入。

当前 live container 缺 `PERFMON`，而 Compose 已声明它。重建前先按现有
`sha256sum.txt` 把有价值的 C1 `/tmp` raw evidence 导出到仓库外目录；正式 C2 raw 也
直接写唯一宿主外部目录，不进入 Git。镜像重建后重新 smoke perf wrapper fallback、
control FIFO/ACK、trace/uprobes、Heaptrack、Valgrind、smem 和 sanitizer。

> **2026-07-31 更新（该段前提已过时）**：容器实测
> `CapEff=00000040a80c25fb`，含 `cap_perfmon` 与 `cap_sys_ptrace`，与
> `.devcontainer/docker-compose.yml:31-33` 的 `cap_add` 一致——**`PERFMON` 已到位**。
> 但这不改变硬件 PMU 不可用的结论：PMU 缺失在 LinuxKit 虚拟机层面
> （`/sys/bus/event_source/devices/` 无 `cpu` 项），capability 补不了。
> 详见 `docs/performance-memory-testing-playbook.md` §2.8。

## 8. 正式运行矩阵

所有模式严格顺序运行：

| workload | 模式 | 最低正式窗口 |
| --- | --- | ---: |
| bounded | perf stat | 600 s |
| bounded | perf record | 120 s，且 `>=1000` samples、unknown `<=20%` |
| bounded | ROS trace | 60 s |
| bounded | stage latency | 300 s |
| bounded | Heaptrack | 300 s |
| bounded | Massif | 180 s |
| bounded | Memcheck | 60 s |
| bounded | ASan / LSan | 各 60 s，先 ASan 后 LSan |
| bounded | plain sample | 3 x 300 s |
| expanding | plain sample | 3 x 300 s |
| expanding | Heaptrack + `heaptrack_print -M` | 300 s |
| expanding | stage latency | 300-600 s，由 capacity 路由 |

perf record 可按 smoke 的实际采样率延长，不能降低 1000 samples 门。任何正式窗口中
角色提前退出、10 Hz/revision 门失败、强杀、报告截断或产物不可解析都标记 invalid
并重跑。

expanding stage-latency 在 `covered` 时运行
`max(300 s, required_end_stamp + drain)`，上限 600 秒，且不得超过 capacity 已验证的
安全时长；其他状态固定 300 秒，只形成连续 map-size 证据并明确 segmented evidence
unavailable。

## 9. 分析与报告

仓库保存 runner、冻结 workload、机器可读 schema/summary、C1 兼容回归和
`docs/local-map-resource-profiling.md`。报告至少包含：

- workload/provenance/validity 与 raw artifact 外部路径及 SHA；
- callback 和四个内部阶段的分位数、revision/throughput/backlog；
- perf stat、热点/符号化质量和 ROS 调度证据；
- 三轮 bounded RSS/PSS/USS slope、Heaptrack/Massif heap 时间线与增长判定；
- expanding 每 revision/known voxel/bounds 的进程内存成本、按 checkpoint 对齐的 heap
  时间线，以及可达时 500k 前/跨越/跨越后的阶段延迟；不可达时记录实际最大规模；
- Heaptrack/Massif/Memcheck/ASan/LSan 结果和 profiler 自身开销；
- 面向 C3 的可比较条件、限制和独立 findings。

高 CPU/延迟本身形成 baseline，不自动使任务失败；无法维持冻结 10 Hz、确定性持续
增长、非法访问、definite/indirect leak 或 sanitizer error 建立独立 finding/fix task，
不在本任务修改 C2 算法。

## 10. 回滚与阻断

- shared runner 抽取破坏 C1 CLI/raw 复算：回滚抽取实现，先修兼容测试，不复制 runner。
- uprobe 不可用且 profiling-only tracepoint 构建也无法形成完整配对：AC3 阻断，不能
  用 ROS callback 或 harness 数字冒充 production stage latency。
- bounded oracle 无法形成 plateau：修正 profiling workload/预热，不放宽泄漏定义。
- production 与 oracle stamp/revision/bounds 不一致：run invalid，定位 DDS/TF/health
  缺口后重跑，不插值补齐。
- profiler 暴露业务缺陷：冻结证据并创建独立任务；本任务继续完成其他不受影响矩阵。

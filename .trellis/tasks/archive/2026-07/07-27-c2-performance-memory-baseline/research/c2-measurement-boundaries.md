# Research: C2 measurement boundaries

- Query: 研究已提交 C2 mapper/node/canonical scene/tests，在不修改 occupancy、health、epoch/revision 或射线业务语义的前提下，设计 bounded/expanding 长时负载、阶段延迟、revision/voxel/RSS 对齐，以及 OctoMap snapshot 与 revision-locked read transaction 开销测量。
- Scope: internal / external / mixed
- Date: 2026-07-27

## Findings

### 1. Files found

- `ws/src/alien_perception/perception_local_map/config/cave_full_ray_scene.yaml` - canonical scene 的唯一冻结参数源；20 s、10 m 直线、20 Hz pose、10 Hz/360-beam FullRay、0.2 m map。
- `ws/src/alien_perception/perception_local_map/launch/cave_full_ray_scene_config.py` - 强制拒绝偏离 20 s 直线和 10 Hz/360-beam 扫描的 scene 配置。
- `ws/src/alien_perception/perception_local_map/launch/cave_full_ray_scene.launch.py` - 真实 scanner -> pose gate -> C1 input -> C2 node 进程图与参数映射。
- `ws/src/drone_scanner/src/CaveLaserScanNode.cpp` - 轨迹速度归零后主动停止扫描，说明 canonical launch 不能原样延长为长期负载。
- `ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp` - 单 callback 内顺序执行 mapper、OctoMap snapshot、revision state publication 的生产边界。
- `ws/src/alien_perception/perception_local_map/src/LocalObservationMapper.cpp` - exclusive/shared lock、apply、revision commit、state 和 exact-revision transaction 的实现。
- `ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp` - persistent `OcTree` + `std::set<VoxelIndex>` 双重 map storage，以及 apply/known-bounds/visit 成本。
- `ws/src/alien_perception/perception_local_map/src/OctoMapSnapshotBridge.cpp` - 当前生产实现调用 `binaryMapToMsg()`，不是旧设计文字中的 `fullMapToMsg()`。
- `ws/src/alien_perception/perception_interfaces/msg/LocalMapState.msg` - 当前 wire 可观测 revision/bounds/provenance/capability，未提供 known/free/occupied 总数或阶段耗时。
- `ws/src/alien_perception/perception_local_map/test/TestCaveFullRayScene.cpp` - 可复用 `TreeCaveField`、`FakeLidar`、`LaserScanProjection` 和 mapper 的确定性 ROS-free scene oracle 模式。
- `ws/src/alien_perception/perception_local_map/test/TestLocalMap.cpp` - 当前单帧 ResourceBaseline 及 read transaction 并发语义测试。
- `ws/src/alien_perception/perception_local_map/test/test_cave_full_ray_scene_integration.py` - canonical ROS 正确性 oracle、drain、revision/provenance/fingerprint/graph 断言。
- `scripts/profile-perception.sh`、`scripts/analyze-perception-profile.py` - C1 已验证的 PID/starttime、同窗、perf control、RSS/PSS/USS、退出完整性与趋势分析基础。
- `.trellis/spec/backend/local-observation-map-contract.md` - exact-revision transaction、visualization-only OctoMap 和禁止用 snapshot 回答 authoritative query 的规范。

### 2. 现有 canonical scene 不能直接承担长期负载

冻结基线是 `start=[1,0,1.5]`、`end=[11,0,1.5]`、`duration=20 s`、pose 20 Hz、scan 10 Hz、360 beams、range `[0.1,30] m`、noise seed 0、map resolution 0.2 m（`cave_full_ray_scene.yaml:43-58,80-85`）。config loader 明确拒绝修改 20 s/10 m line 和 10 Hz/360-beam geometry（`cave_full_ray_scene_config.py:49-70`）。轨迹结束后 odometry 仍发新 stamp，但 scanner 看到运动后第一次速度归零就设置 `scanning_stopped_`（`CaveLaserScanNode.cpp:122-151`）。集成测试也把“revision 在结束后 drain 并停止”当作业务 oracle（`test_cave_full_ray_scene_integration.py:160-187,268-294`）。

因此：

- canonical scene 适合短程正确性回归，不适合 300-600 s profiler 正式窗口；简单增大 duration 会被 loader 拒绝，取消 stop 又改变已冻结的 scene 行为。
- 不应循环 launch 或重启 producer；新 SessionID/health recovery 会引入 epoch/recovery 样本，污染 steady-state。
- 不应让 pose 从 x=11 瞬移回 x=1；默认 position jump threshold 为 2 m（`PerceptionLocalMapNode.cpp:450-459`），会触发 reset 语义。

### 3. 推荐 workload：独立 profiling fixture，参数与业务负载冻结

profiling fixture 是目标外单独进程，只发布 C2 已有的 authoritative health/pose/`LidarObservation` 和静态 extrinsic；C2 目标仍是安装后的 `perception_local_map_node`。fixture 复用 canonical 的 `TreeCaveField(seed=42)`、`FakeLidar`、`LaserScanProjection`，已有 ROS-free scene test 展示了同一组合（`TestCaveFullRayScene.cpp:169-185,201-239`）。固定以下共同条件，不因 profiler 变慢而降低：

```text
observation rate       10 Hz, unique stamp step 100 ms
pose rate              20 Hz, pose leads observation and stays same-clock usable
scan                    360 beams, FullRay, [0.1, 30] m, noise=0, seed=42/0
map                     resolution=0.2 m, lattice origin=(0,0,0), frame=map
session/fingerprint     one stable producer session and one stable mapper epoch
publication             one observation per planned stamp; no replayed stamp
```

`bounded`：在 x=[1,11] 上做连续三角波/ping-pong，步长保持 canonical 0.5 m/s（每 scan 0.05 m），body orientation 固定为 canonical identity；端点只反转平移方向，不旋转、不跳变。相同空间被反复观察，revision 继续以 10 Hz 前进，但 exact known voxel 集应在首个往返后停止增加。正式窗口前要求 oracle 连续 600 revisions（60 s）known/free/occupied count 和 bounds 不变，再开始 300/600 s 测量。

`expanding`：保持同一 0.5 m/s、10 Hz 和固定 orientation，从 x=1 单调沿 +X 移动；每步 0.05 m 远低于 jump threshold。继续使用 deterministic cave scan；离开已有 surface 时 FullRay no-return 仍是合法 evidence。每 100 revisions 建 checkpoint，要求 exact known voxel 和 X bounds 跨 checkpoint 增长。不要要求每个 revision 都新增 voxel：0.05 m 小于 0.2 m resolution，revision 可合法推进而 changed count 为 0。

两类负载必须先用短窗口证明：输入 distinct-stamp 数、revision delta、epoch、fingerprint、health、last observation stamp、diagnostics 和预期一致。recovery 前的若干 observation 放在预热期，不计入正式窗口。

### 4. 当前消息能观察什么，不能观察什么

`LocalMapState` 可直接观察 mapper session、`state_sequence`、epoch/revision、health/freshness、resolution/origin、known bounds、fingerprint、last committed observation stamp 和 `last_commit_changed_cell_count`（`LocalMapState.msg:12-29,41-52`；赋值在 `PerceptionLocalMapNode.cpp:608-667`）。但它不包含：

- authoritative known/free/occupied voxel 总数；
- accepted/applied/no-evidence/rejected 累计计数；
- mapper apply、state build/publish、transaction acquire/hold、snapshot serialization 的耗时；
- OctoMap snapshot 对应的 epoch/revision。

`state_sequence` 不能当 revision：state 还会在 health、pose、heartbeat 上发布（`PerceptionLocalMapNode.cpp:306-313,497-554`）。sink 应按 `(mapper_session,map_epoch,revision)` 去重，只把严格 revision 增量视为 commit。`NoEvidence` 没有 diagnostic，普通消息无法与 dropped/rejected gap 无歧义区分；正式 FullRay workload 应设计为每个预热后 observation 都有 evidence，并以 `revision_delta == planned_distinct_observations`、last stamp 无积压、无 mapper warning/error 证明 applied。

OctoMap wire 只有 header/binary/id/resolution/data，不含 revision。规范也明确 snapshot 仅用于 visualization，不能与 state topic 猜配为 authoritative 内容（`.trellis/spec/backend/local-observation-map-contract.md:24-31`；原设计同样说明标准消息无 revision，`07-27-c2-local-observation-map/design.md:419-421`）。因此禁止直接把“最新 OctoMap leaf 数”声明为某个 authoritative revision 的 exact voxel count。

### 5. revision / exact voxel / RSS 的低扰动对齐

推荐“离线 deterministic oracle + 生产进程采样”双通道，避免在 target hot path 遍历全图：

1. ROS-free oracle 用与 production fixture 完全相同的浮点输入序列驱动真实 `LocalObservationMapper + OctoMapBackend`；在每 100 revision 的 exact receipt 上 acquire transaction，以 `for_each_known_cell()` 统计 known/free/occupied，并保存 `observation_stamp -> epoch/revision/counts/bounds` 表。transaction API 按 value 访问且锁住 exact revision（`LocalObservationMapper.hpp:72-87,124-136`）。
2. production run 的 state sink 保存 sink 本机 monotonic receipt time、epoch/revision、last observation stamp、bounds、changed count、fingerprint。以 last observation stamp 连接 oracle 表，不按 wall clock 或 OctoMap topic 顺序猜配。
3. checkpoint controller 在收到目标 revision/state 后立即读取已验证 target PID 的 `/proc/<pid>/smaps_rollup`，记录同一 monotonic timestamp 的 RSS/PSS/private clean+dirty（USS proxy）。若只能周期采样，则只连接“最近且不晚于 sample”的 state，并报告最大时间/revision skew。
4. expanding 报告相邻 checkpoint 的 `delta RSS/PSS/heap / delta exact-known`，同时保留 revision、known/free/occupied、bounds span/volume 原始列；bounded 只在 oracle 已证明 exact counts plateau 后拟合 RSS/PSS/heap slope。

该方案不把 oracle 进程资源计入 C2，也不在 production node 增加 counter/message。其必要 gate 是 deterministic replay 等价：production 每个 checkpoint 的 revision、last stamp、bounds、fingerprint 必须等于 oracle；任一 gap/reset/rejection 使 run invalid。可额外解码 snapshot 作可视化 proxy 或记录 `data.size()`，但不能替代 oracle exact count。

C1 runner 已有合适的进程与内存证据骨架：plain sampler 每 1 s 记录 target-only pidstat、每 10 s 记录 smem + smaps_rollup（`scripts/profile-perception.sh:1033-1060`）；C1 设计要求 PID `/proc/exe`/starttime/cgroup/affinity 冻结（`07-25-c1-perception-resource-profiling/design.md:139-150`）并只在 60-300 s 稳态拟合三次 PSS/RSS（同文件 `:242-245`）。C2 应扩展 workload/state checkpoint 解析，不复制或削弱这些门。

### 6. 延迟阶段：生产外部 probe 为主，profiling harness 为补充

生产 callback 当前串行边界为：decode/TF -> `submit_observation()` -> exact transaction -> `binaryMapToMsg()` -> OctoMap publish -> `state()`/state message publish（`PerceptionLocalMapNode.cpp:534-595,598-675`）。建议单独做一个 bounded latency run，用 ELF uprobes 或等价外部函数 entry/return probe，按 PID/TID 和嵌套关系离线配对：

| 报告项 | production probe 边界 | 含义 |
| --- | --- | --- |
| observation callback envelope | `PerceptionLocalMapNode::on_observation` | TF/decode、mapper、snapshot、两个 publish 的总时长 |
| mapper apply | `LocalObservationMapper::submit_observation` | validation/pose projection/backend apply/commit；若需要纯 backend，再单列 `OctoMapBackend::apply` |
| revision state publication | `PerceptionLocalMapNode::publish_state` | `state()`（含 known-bounds）+ message materialization + rclcpp publish；只保留嵌套于已 Applied `on_observation` 的调用 |
| read transaction acquire | receipt overload `LocalObservationMapper::acquire_read_transaction` | shared-lock acquire + exact receipt check + known-bounds metadata |
| snapshot serialization | `OctoMapSnapshotBridge::materialize` | OctoMap binary materialization本身 |
| snapshot total | `PerceptionLocalMapNode::publish_octomap` | acquire + metadata + materialize + close + rclcpp publish |

当前安装后二进制经本次只读 `nm -C` 检查，上述 `on_observation`、`publish_state`、`publish_octomap`、mapper submit/state/acquire 和 bridge materialize 都有符号；正式 RelWithDebInfo 构建仍必须重新以 build-id/SHA/nm 证明，不能假设链接优化后继续存在。ROS tracing 可提供 callback envelope/take/publish 数量和调度证据，但标准 tracepoint没有 mapper commit/materialize 边界，所以不能单独生成全部阶段 quantile。sink receipt latency是跨 DDS 的 delivery 指标，也不能冒充 `rclcpp::publish()` 或 mapper 内部耗时。

uprobes 只在函数 entry/return 采一次 monotonic timestamp，不探 DDA/voxel loop、不格式化日志、不在线算 quantile。该 latency run 与 `perf stat/record`、Heaptrack、Massif、Memcheck 分开；正式无 probe CPU/RSS/hotspot 数字另跑。probe run 做 A/B 校准：相同 workload 的 unprobed/probed revision rate、target CPU 和 callback envelope 不得出现预注册容差外漂移；丢 probe event、entry/return 不配对、嵌套数不等于 applied revision 数均 invalid。10 Hz 下建议至少 300 s、约 3000 applied 样本再报告 p99；只有 60 s/约 600 样本时 p99 标注 exploratory。

专用 ROS-free profiling harness 用于外部 probe 无法可靠回答的两项，不替代 production latency：

- exact voxel census/oracle，如上一节；
- transaction contention：在固定 receipt 上 acquire reader，异步启动下一 writer，分别测 acquire 成本、transaction hold time、writer wait，在 `close()` 后验证 writer 前进。现有测试已证明 reader 未 close 时 writer 至少等待 20 ms（`TestLocalMap.cpp:666-713`），但没有规模化分位数。

### 7. OctoMap snapshot 和 transaction 的已知热点形状

`OctoMapBackend` 同时持有 `octomap::OcTree` 和 persistent `std::set<VoxelIndex> known_cells`（`OctoMapBackend.cpp:13-22`）。每批 apply 还构造临时 free/occupied `std::set`，对每 cell 做 before query、`updateNode`、known set insert、after query，最后 `updateInnerOccupancy()`（同文件 `:160-201`）。因此每新增 known voxel 的内存不是单一 OctoMap node 成本；必须同时由 RSS/PSS、Heaptrack 和 exact known delta 解释。

`known_bounds()` 每次线性遍历全部 known cells（`OctoMapBackend.cpp:62-80`）。一次 Applied node callback 至少在 exact transaction acquisition（`LocalObservationMapper.cpp:1284-1303`）和随后的 `state()`（同文件 `:1243-1265`）各触发一次，所以 expanding map 下 read acquire 和 revision publication 延迟随 map size 增长是预期成本，不应误判为随机回归。

transaction 从 acquire 起持有 `std::shared_lock`，直到 `close()` 解锁（`LocalObservationMapper.cpp:506-531,578-588`）；writer/apply 使用同一 mutex 的 exclusive lock（同文件 `:1068`）。生产 node 在 materialize 完成后才 close（`PerceptionLocalMapNode.cpp:570-595`），所以 snapshot serialization 是 read-lock hold time 的主体。当前 main 使用 `rclcpp::spin`（同文件 `:678-684`），单 executor thread 下通常没有 callback writer contention；contention harness 的结果是 C3/未来并发风险，不应冒充当前 node 的 observed wait。

当前 bridge 实际调用 `binaryMapToMsg()`（`OctoMapSnapshotBridge.cpp:9-19`）。容器版本为 `liboctomap-dev 1.9.7+dfsg-3.1build3`、`ros-jazzy-octomap-msgs 2.0.1-1noble.20260615.112219`。安装头 `/opt/ros/jazzy/include/octomap_msgs/octomap_msgs/conversions.h:166-177` 显示它执行 `writeBinaryData(stringstream)`，再复制到 `string`，再复制到 `vector<int8_t>`；snapshot latency/temporary heap 随 tree serialized size 增长是合理预期。报告必须同时保存 `message.data.size()`、materialize latency、snapshot-total latency和 Heaptrack temporary allocations。

### 8. 被否决方案

- **直接延长/循环 canonical launch**：被冻结为 20 s 且 scanner 主动 stop；重启还改变 session/recovery，不是 steady workload。
- **endpoint 原地重复或复用相同 stamp**：不能证明 10 Hz 新输入链路，也会触发 replay rejection；不得用其制造低负载通过。
- **在 voxel/DDA loop 内加 chrono、日志或 per-cell tracepoint**：直接改变待测分配、cache 和锁持有时间。
- **把 counters/latency 字段加入 `LocalMapState`**：扩大 wire 业务契约，且每 revision census 本身是 O(N) target 扰动；deterministic oracle 可避免。
- **把 OctoMap snapshot decode 当 authoritative voxel/revision**：消息不带 revision，且规范明确 visualization-only。
- **仅依赖现有 `ResourceBaseline`**：它从 mapper apply 一直计到 full-region query，并读取整个 gtest 进程历史 `ru_maxrss`（`TestLocalMap.cpp:995-1050`），无法分阶段、稳态或定位 production PID。
- **同时运行 uprobe、perf record、Heaptrack、Massif**：叠加开销破坏归因；按 PRD 顺序单工具运行。
- **降低 10 Hz/360 beams/0.2 m、缩短正式窗口或强杀 profiler**：使 run invalid，不构成基线。

### 9. 推荐验收门

1. **构建/provenance**：独立 `RelWithDebInfo -O2 -g -DNDEBUG -fno-omit-frame-pointer`；记录目标 ELF SHA/build-id/ldd/nm、OctoMap versions、RMW、CPU/cgroup/PID/starttime/affinity。sanitizer 前缀独立。
2. **workload**：正式窗 observation distinct stamps 达 `ceil(actual_s*10)` 的预注册容差，不能重复；revision delta 等于预热后 expected applied count，epoch 不变，fingerprint/health/capability正确，last stamp lag 有界，无 rejection/backend diagnostic。
3. **bounded**：正式窗前 oracle 连续 600 revision exact known/free/occupied 和 bounds 不变；正式窗 production bounds 保持一致。三次 plain run 分别从 plateau 起拟合 RSS/PSS/USS/heap slope；只有三次重复同向增长才标疑似，C1 的 `>1 MiB/min` 可作为沿用阈值，但原始 slope/CI 必须保留。
4. **expanding**：每 100 revision checkpoint 的 oracle known 和 X bounds 整体增长；每条 RSS/PSS/heap sample 能以 last stamp 精确连接 oracle，报告 delta bytes/exact-new-known 及 bounds，不把正 slope称泄漏。
5. **latency**：每阶段输出 count/mean/p50/p95/p99/max；300 s latency run 目标约 3000 applied 样本。probe 无丢失、无 unmatched pair，nested `publish_octomap/materialize/acquire/publish_state` 数与 applied callbacks 一致；否则只报告 callback envelope或标未完成。
6. **snapshot**：每个 applied callback 恰有一次非空 binary snapshot publish；记录 bytes 与 latency。snapshot ordinal只作 transport完整性检查，不作 authoritative revision join。
7. **transaction harness**：每次 metadata receipt 与期望 session/epoch/revision 完全相等；acquire、materialize、close/hold/writer-wait 分栏，地图规模分层，关闭后 writer 必须前进且旧 receipt Superseded。
8. **工具隔离/退出**：沿用 C1 perf control ACK、target-only vpid、smaps、artifact SHA、正常退出门；任何积压、10 Hz/revision失败、tool report截断或 SIGKILL run 不计通过。

## Related Specs

- `.trellis/spec/backend/local-observation-map-contract.md:24-31,58-70` - authoritative state、visualization-only snapshot、exact-revision transaction 与 reader/writer 语义。
- `.trellis/spec/backend/perception-ray-evidence-contract.md:31-55` - FullRay no-return 的权限和 whole-batch validation；负载不能通过降级 capability 降低成本。
- `.trellis/spec/backend/quality-guidelines.md` 的 ROS launch/integration contracts - distinct timestamp、QoS、non-empty test 与 process-scoped runtime 约束。

## External References

- OctoMap Debian package: `liboctomap-dev 1.9.7+dfsg-3.1build3`（容器 `alien-scanner-dev`，2026-07-27 查询）。
- ROS Jazzy `octomap_msgs`: `2.0.1-1noble.20260615.112219`。
- `/opt/ros/jazzy/include/octomap_msgs/octomap_msgs/conversions.h:122-177` - binary/full conversion 的 stringstream/string/vector materialization 实现与 compact/full 格式说明。
- `ros2 interface show octomap_msgs/msg/Octomap` - wire 仅有 header、binary、id、resolution、data，无 revision/epoch。
- `.trellis/tasks/archive/2026-07/07-25-c1-perception-resource-profiling/design.md:105-176,181-245` - 已验证的同窗、PID、perf/tracing、退出、RSS/PSS/USS 与 profiler 证据门。

## Caveats / Not Found

- `python ./.trellis/scripts/task.py current --source` 在本研究开始时返回 `Current task: (none)`；写入路径来自上级 dispatch 明确给出的 `.trellis/tasks/07-27-c2-performance-memory-baseline`，task.json 本身状态为 `planning`。主会话应在后续阶段确认/恢复 session active-task pointer。
- 当前没有 production query service、authoritative voxel-count topic 或内建 phase timer；不新增 target instrumentation 时，exact voxel count 必须来自 deterministic oracle，阶段 latency 必须来自外部 function probe。
- 外部 uprobe 的 tracefs/kernel 权限尚未做正式 smoke；容器默认 `perf` wrapper 对 LinuxKit 6.10.14 报 kernel tool mismatch，但 C1 runner 已能发现 `/usr/lib/linux-tools/6.8.0-136-generic/perf`。若 entry/return uprobe不可用或符号消失，phase AC 不能由 ROS trace自动替代；只能报告 ROS callback envelope，并将 harness 数字明确标为非 production microbenchmark。
- 当前 C2 archive design 在 `design.md:532-534` 写的是 `fullMapToMsg()`，已提交实现实际为 `binaryMapToMsg()`；性能研究与基线必须以实现和 wire `binary=true` 为准，不在本研究中修改历史设计或业务代码。
- deterministic oracle 与 production sequence 的等价依赖同一 scan float payload、stamp、pose/extrinsic 和接受顺序。任何 DDS gap/reorder、health recovery或 TF failure都会破坏 join，必须使整次 run invalid，不能插值补齐。

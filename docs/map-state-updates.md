# C3 地图状态与增量更新

`perception_map_update` 位于 C2 权威本机地图与后续 C4 通信数据面之间。它把一个
已提交的本机地图 revision 转成后端无关、可校验和可重建的状态更新；它不重新处理
LiDAR，也不替代 C2 的本机安全查询。

## 生产数据流

```text
C2 CommitReceipt
    -> exact-revision MapReadTransaction
    -> CanonicalSnapshotAdapter
    -> MapUpdateProducer (v2 keyframe / delta)
    -> local_map/updates
    -> MapUpdateApplier (chunk-16 COW + persistent Merkle)
    -> reconstructed source-local occupancy view + VersionedContentDigest
```

- `MapReadTransaction` 同时锁定 source/session、map epoch、revision、geometry 和 cells，
  防止把旧 receipt 的元数据与较新地图内容拼接。
- `CanonicalSnapshot` 是生产内部的完整只读地图表示。known cells 严格排序，缺失 cell
  表示 `Unknown`，并携带稳定 geometry fingerprint；正式 producer 的 content identity
  由固定 edge-16 `MerkleMapState` 计算。flat content hash 仅保留在隔离 oracle/benchmark。
- `Keyframe` 独立建立或替换接收端基线；`Delta` 只在接收端 revision 严格等于
  `base_revision` 时应用。概率变化但三态内容不变时，允许空操作的 revision-only delta。
- 重复更新幂等；乱序、缺口、旧 session/epoch、hash 冲突、损坏 payload 和资源超限
  都在修改接收端前拒绝，最后合法 revision 保持不变。
- 接收端进入 `ResyncRequired` 后，通过短 service 请求 keyframe。service response 不携带
  地图；producer 在原更新 topic 上异步发布带 correlation ID 的 keyframe。

C3 producer 使用单 pending、单 in-flight 的有界 worker。C2 observation callback 只提交
receipt；全图 materialize、diff、hash 和编码不在 mutation callback 中执行。中间 revision
可以 latest-wins 合并为合法的 `base_revision -> latest_revision` 状态 delta。

## ROS 接口

生产者由 `perception_local_map_node` 承载，默认关闭：

- 参数 `map_update_enabled=true` 启用 C3 producer；
- topic `local_map/updates` 发布 `perception_interfaces/msg/MapUpdate`；
- service `local_map/request_resync` 使用
  `perception_interfaces/srv/RequestMapResync`；
- `/local_map/octomap` 继续作为 C2 RViz/兼容快照，不是 C3 revision 协议。

参考接收端 `perception_map_update_receiver_node` 消费 `local_map/updates` 和
`local_map/state`，发布 `map_update_receiver/octomap` 与 `/diagnostics`。公共资源参数统一
使用 `map_update.*` 前缀；默认最多 3,000,000 known/receiver cells、128 MiB 单个
keyframe/delta payload、384 MiB apply 峰值预算和 128 个连续 delta。完整默认值见
`MapUpdateLimits.hpp`。

## C4.3 v2-only 内容身份

当前生产分支统一使用 `protocol_version=2`。每条 `MapUpdate` 都显式携带
`ContentIdentityDescriptor`：`MerklePatriciaSha256V2 / edge=16 /
coordinate_key_version=1 / node_encoding_version=1`。`base_content_hash` 与
`content_hash` 是该 descriptor 下的 base/result Merkle root；接收端在 candidate
store/tree 上本地重算 root，并在 descriptor、count、resource 和 update hash 全部通过后
一次性提交。Relay 只透明转发嵌套 `MapUpdate`，不读取 Patricia 内部节点。

direct `RequestMapResync` 以及 C4 routed `ResyncIntent/ResyncAck` 同样携带完整 descriptor
和 digest。未知 protocol/descriptor、descriptor drift、错误 base/root 或资源超限均
fail closed，不改变最后合法地图，也不存在 v1 fallback、双写、协商或 runtime downgrade。
flat SHA-256 v1 仅用于冻结对照和 correctness oracle，不进入 v2 production profile。

实现代码已在 `phase/4-merkle-v2-production-integration` 完成并通过受影响包测试；正式
生产 Gate 仍需独立的 3 x 300 秒 receiver 矩阵、内存工具证据和 rollback review。Gate GO
前 C5d EdgeAggregator 不绑定 Patricia 内部结构，C5a-C5c 只能消费 descriptor + digest
的稳定边界。

回滚参数：

- `map_update_enabled=false`：关闭 C3 producer，C2 继续独立运行；
- `map_update.delta_enabled=false`：保留协议、资源校验和 resync，但只发布 keyframe；
- `map_update.periodic_keyframe_revision_interval=0`：默认关闭周期 keyframe，late join 必须
  通过 resync 恢复。

## 参考闭环

`map_update_reference.launch.xml` 启动参考 receiver。实际部署应让 producer 和 receiver
加载同一份 `map_update_reference.yaml`，或按部署 namespace 对 topic/service 做一致 remap。

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_map_update map_update_reference.launch.xml
"
```

生产 mapper 需同时以 `map_update_enabled:=true` 或等价参数启动。receiver 可晚于 mapper
加入；它不会等待周期快照，而会通过 resync service 请求带 correlation ID 的异步 keyframe。

## Exact-Revision Replay 与可视化

`MapUpdateReplayOracle` 是验收夹具，不是第二套生产协议。它用固定 `ProfileScenario`
驱动真实 `LocalObservationMapper + OctoMapBackend`，并在每个已提交 revision 上重复生产链：

```text
exact MapReadTransaction
    -> CanonicalSnapshotAdapter
    -> 正式 MapUpdateProducer
    -> 正式 MapUpdateApplier
    -> 与该 revision 的 canonical snapshot 逐 cell 比较
```

比较覆盖 source identity、geometry、revision、content hash 以及完整
Free/Occupied/Unknown 内容。恢复门期间的 `Unavailable` 输入没有 commit receipt 和
revision，因此不构成 checkpoint。固定输入重复运行还会校验 revision、update kind、
content hash 和 update hash 的确定性。

历史 Phase 3 bag 资产当前不可用；本次验收使用当前确定性 C2 exact-revision 数据集，
不是用洞穴 truth 代替权威地图。历史 bag 如果恢复，可作为额外兼容回归输入，但不是
C3 生产功能的运行依赖。

无 GUI 运行：

```powershell
docker exec alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_profiling \
    map_update_replay_visualization.launch.xml show_rviz:=false
"
```

RViz2 运行：

```powershell
docker exec -e DISPLAY=host.docker.internal:0.0 alien-scanner-dev bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /workspaces/alien-scanner/ws/install/setup.bash
  ros2 launch perception_profiling \
    map_update_replay_visualization.launch.xml show_rviz:=true
"
```

RViz 中左侧青色为 exact-revision oracle，右侧黄色为重建图，中间为差异层：红色表示
缺失、橙色表示意外出现、紫色表示 Free/Occupied 状态不一致。正常结果中差异层为空，
`/map_update_replay/diagnostics` 报告 `match=true`。标准 oracle/reconstructed Octomap topic
仍会发布用于字节比较；当前 Jazzy 镜像的 Octomap RViz 插件存在系统 OctoMap ABI 问题，
界面使用有界、确定性采样的 `MarkerArray` 显示 occupied voxels。

replay/oracle 仍只计算一次；两组 MarkerArray 默认以 1 Hz 重发，使 RViz Display
关闭后重新开启可在一个发布周期内恢复。可通过 launch 参数
`visualization_publish_rate_hz:=<正数>` 调整频率；该参数不影响生产
`/local_map/updates`、revision、hash、Octomap 或 diagnostics 的一次性验证语义。

同一 RViz 配置还提供三个相互隔离的 Display Group：

- `C3 Baseline Equivalence` 默认开启，显示原有 oracle/reconstructed 对照与差异层；
- `C3 Resync Recovery` 默认关闭，在一个固定位置循环显示正常基线、故意丢弃 delta、
  future-base gap 被拒绝并保留最后合法地图、带 correlation ID 的 keyframe 恢复；
- `C3 Epoch Reset` 默认关闭，在另一个固定位置循环显示旧 epoch 基线、新 epoch 准入后
  等待 keyframe、旧 epoch delta 被拒绝、新 epoch keyframe 建立新基线。

后两组分别订阅 `/map_update_replay/resync/markers` 和
`/map_update_replay/epoch_reset/markers`，并使用独立 Marker namespace 与 TF frame；可以在
RViz 左侧单独勾选，不会清除或覆盖 baseline。Display 勾选只控制显示，场景生成仍是
一次性测试夹具；可通过 `enable_resync_scenario:=false` 或
`enable_epoch_reset_scenario:=false` 在 launch 时完全关闭对应场景。

### 如何阅读两个验收场景的文字

每个场景在固定位置只显示一个当前阶段。文字 Marker 的格式是：

```text
<阶段标题>
receiver: <接收端状态>  epoch <地图 epoch>  revision <当前合法 revision>
update <本次尝试的 base revision> -> <本次尝试的 new revision>
correlation: <resync correlation ID>  # 仅恢复 keyframe 阶段出现
```

颜色表示阶段性质：青色是已建立的合法基线，橙色是正在等待或被故意跳过的
中间状态，红色是拒绝/需要恢复，绿色是恢复完成。地图点云颜色和文字颜色一起变化，
因此颜色不是新的地图数据类别，而是当前协议阶段的提示。

`C3 Resync Recovery` 表达“delta 丢失后如何恢复同一个 epoch”：

| RViz 标题 | 文字和地图含义 |
| --- | --- |
| `READY BASELINE` | 首个 keyframe 应用成功，`receiver=ready`；接收端建立 epoch 1 的初始 revision。 |
| `DELTA DROPPED` | 中间 delta 被验收夹具故意丢弃；文字里的 `update base -> new` 是被跳过的范围，但地图仍保持上一个合法 revision。 |
| `GAP REJECTED` | 后续 future-base delta 到达；接收端发现当前 revision 不等于 delta base，进入 `resync_required`，地图仍保持最后合法 revision。 |
| `RESYNC RECOVERED` | 带 correlation ID 的完整 keyframe 应用成功，接收端回到 `ready`，revision 跳到最终 snapshot。 |

默认 `sequence_count=60` 时，示例 revision 通常是 `1 -> 29 -> 59`；修改序列长度后，
数字会随实际 exact-revision snapshot 改变，但“丢失、拒绝、keyframe 恢复”的含义不变。

`C3 Epoch Reset` 表达“空间链切换后旧 epoch 如何失效”：

| RViz 标题 | 文字和地图含义 |
| --- | --- |
| `EPOCH 1 READY` | 旧 epoch 的完整地图已建立，接收端处于 `ready`。 |
| `EPOCH 2 ADMITTED` | 新 epoch 身份已准入，但还没有应用新地图；文字中的 epoch/revision 仍显示保留的旧地图，表示准入不等于切换。 |
| `OLD EPOCH REJECTED` | 旧 epoch 又发送 delta，被 admission fence 拒绝；旧地图和旧 revision 继续保留。 |
| `EPOCH 2 READY` | 新 epoch 的 keyframe 原子建立新基线，接收端切换到 epoch 2 / revision 1，并回到 `ready`。 |

两个场景之间只有有限关联：它们复用同一组首个、中间和最终 exact-revision snapshot，
并由同一个播放 timer 同步切换，所以可以并排对照；但它们使用独立的 producer、receiver、
topic、Marker namespace 和 TF frame。`Resync Recovery` 的恢复不会触发 `Epoch Reset`，
`Epoch Reset` 的切换也不会改变 `Resync Recovery` 的状态。两个 Group 同时勾选时，
看到的是两个独立协议故事的同步演示，不是一条共享状态机的连续分支。

场景生成器最多保留 replay 的首个、中间和最终三个真实 exact-revision canonical snapshot，并调用生产
`MapUpdateProducer` 与 `MapUpdateApplier` 构造丢包、gap、resync 和 epoch 切换序列。它不
重新扫描洞穴、不实现第二套 codec/apply 算法，也不向 `/local_map/updates` 发布数据。
两个验收场景默认每 2 秒推进一步，可用正数参数 `scenario_step_period_s` 调整。地图、
receiver state、epoch 和 revision 随当前阶段一起更新；拒绝阶段的 receiver revision 保持
最后合法值，只有恢复 keyframe 到达后才跳变。三组缓存 MarkerArray 另用
`visualization_publish_rate_hz` 周期刷新当前阶段，因此关闭后重新勾选任一 Group 都能
恢复显示，而不会重跑 replay 或状态机。

## 当前边界

C3 已完成 source-local 更新的生成、重建、乱序拒绝和本机 resync。进入 shared view 时
如何绑定 committed alignment，以及 alignment 变化/撤销后如何失效旧贡献并请求新
keyframe，属于后续聚合消费路径的验收，不由本机 replay 可视化冒充完成。

## 常见问题：性能、内存与测试输入

### 当前性能与内存验收具体测了什么？

专项验收的生产边界是 C3 map-update core，以及 C2 到 C3 的
`CanonicalSnapshotAdapter` 和 `AsyncMapUpdateProducer`。整体功能验收实际运行生产
`LocalObservationMapper + OctoMapBackend` 生成精确 revision，再执行 canonical
snapshot、keyframe/delta、hash、异步发布和 receiver 重建。

其中，3 x 300 秒资源矩阵的 tracee 是承载 C2 mapper 和 C3 producer 的
`perception_local_map_node`；reference receiver、sink 和 oracle 用于独立的正确性、计数与
drain 验证，其 CPU/RSS 不计入该矩阵。因而矩阵比较的是“同一个 C2 基线进程关闭或开启
C3 adapter/producer 后的成本”，不是整条未来跨机消费链的总成本。

本轮不把 C2 LiDAR 建图算法、C4 通信路由、shared-view 聚合或 RViz 渲染计入 C3
性能对象。三种运行模式使用同一 RelWithDebInfo 构建和同一输入：

| 模式 | 含义 |
| --- | --- |
| `disabled` | C2 正常建图，C3 producer 关闭且没有 update 证据，作为基线 |
| `enabled` | C3 producer 正常生成事件驱动 keyframe 和 delta |
| `keyframe-only` | 保留 C3 producer 协议，但每次只发送完整 keyframe |

每种模式完成 3 个独立的 300 秒 run。另有 ASan/LSan、Memcheck、约 181 万 cells
容量路径和确定性 replay/oracle；它们回答非法访问、经典泄漏、容量和逐 revision
重建正确性，不与 300 秒 RSS/CPU 矩阵互相替代。

### 300 秒表示真实飞行器连续运行了 300 秒吗？

不是。300 秒是冻结 profiling 协议的正式测量窗口，不是实际任务时长、真实隧道
飞行记录或生产环境耐久保证。正式窗口前还有约 800 revision 的预热；CPU 汇总排除
正式窗口最前面的 60 个样本，仍保留约 240 个稳态 CPU 样本和 30 个十秒级内存
checkpoint。这个窗口用于在可控执行成本内形成可分析的稳态与增长趋势，不是由飞行
物理时长推导出的数学最小值。每种模式运行 3 轮，用于检查独立证据的一致性和窗口内
持续内存增长门。

因此，当前结论只能表述为：在该构建、该确定性 bounded 10 Hz workload 和 300 秒
窗口内，三组均未触发 `1024 KiB/min` 的持续增长门。它不能外推为任意任务时长下
“绝对零增长”；特别是 keyframe-only 的小幅正斜率仍保留为更长时间 soak 的观察项。

### `ProfileScenario` 的输入数据从哪里来？

`ProfileScenario` 是确定性 C++ 验收夹具，不读取 bag。它有三种用途不同的模式：

| 场景 | 输入来源与用途 |
| --- | --- |
| `canonical` | `TreeCaveField(seed=42) + FakeLidar`，固定 200 个序列，用于 exact-revision replay/oracle 和 RViz 等价验证 |
| `bounded` | 公式生成固定 360 束椭圆隧道截面扫描，飞行器在 X=1–11 m 间连续往复，用于 10 Hz 稳态性能与内存矩阵 |
| `expanding` | 使用同类确定性扫描但沿 X 单向前进，用于持续扩大地图和约 181 万 cells 容量路径 |

“夹具”只描述输入来源。C2 mapper/backend、C3 adapter、producer、codec、applier 和
receiver 仍是生产实现；测试没有另写一套地图更新算法。

### 为什么 bounded 场景采用连续往复运动？

连续往复同时满足三个条件：

1. 每帧位移连续，端点只改变运动方向，不触发 pose jump 或 map epoch reset；
2. 飞行器反复扫描同一段空间，使 known-cell 规模进入平台，而 revision 和 C3 工作继续；
3. 10 Hz 输入、时间戳、扫描、位姿和运行时长完全可重复，便于比较三种 C3 模式。

这使测试能够在同一 map epoch 内观察长 delta chain、worker backlog/drain 和 retained
state。如果地图范围仍持续扩张，正常地图占用和 C3 异常内存增长会叠在一起；如果周期
边界重置 epoch，则会反复触发 keyframe，无法隔离 delta 稳态成本。

### 上一阶段 C1/C2 的长时测试也是连续往复运动吗？

不是。上一阶段使用的是完整仿真链路录制 bag 中约 20 秒的 `0 -> 11 m` 直线运动段，
并把该段复制多份后拼接：

```text
第 1 段：0 m ------> 11 m
                       | 位置跳回起点
第 2 段：0 m ------> 11 m
                       | 位置跳回起点
第 3 段：0 m ------> 11 m
```

第 `k` 份的 header stamp 和 TF stamp 增加 `k * 周期时长`，让时间始终单调递增；
session identity 和相对量 `freshness_ns` 不修改。裸 `ros2 bag play --loop` 会让第二圈
时间倒退，被 C2 mapper 的观测/位姿高水位检查拒绝，因此不能为 C1/C2 长跑链路提供
持续的 C2 建图负载。

时间连续不代表位置连续。每段末尾从 11 m 跳回 0 m，C2 会按生产契约 fail closed、
重置 map epoch 并从零重建。所以上一阶段覆盖的是“真实链路固定数据重复 + 周期性
epoch 重置”，不是当前 bounded 场景的平滑三角波往复。

上一阶段所称“真实数据”是完整仿真场景通过生产 ROS 链路录制的数据，包含真实消息
组合和洞穴几何扫描结果；它仍不是物理飞行器和真机 LiDAR 采集数据。

### 为什么当前 C3 的正式矩阵不直接使用上一阶段的 bag？

主要原因不是 bag 永远不可用，而是两阶段要隔离的问题不同：

- C1/C2 测试需要覆盖感知消息和建图链路，bag 适合保存并重复真实链路输入；
- C3 测试需要在精确 C2 revision 上比较 disabled/enabled/keyframe-only，并保持同一
  epoch、稳定地图规模和一致工作量；确定性 bounded fixture 更适合该对照；
- `MapReadTransaction` 是 C2 在某个 revision 锁定的进程内事务，不是 bag 中现成的
  ROS 消息。bag 必须重新喂给当前 C2 才能生成 transaction；
- 上一阶段拼接 bag 的周期位置跳变会重置 epoch，并强制 C3 重新发送 keyframe，导致
  正式矩阵混入反复重建成本；
- bag player、回放速率、重打戳、历史 TF 时序和回放调度会增加当前 C3 资源对照不需要
  的外部变量；当前 fixture 仍通过正常 ROS/DDS topic 驱动 C2，并未绕过进程边界。

历史 Phase 3 source-level bag 当前不可用；已保留的 C1/C2 bag 即使接口兼容，也只能
通过当前 C2 重新生成 C3 输入。它适合作为额外的真实链路兼容回归，但不能替代
revision-aware oracle，也不应替换当前 3 模式、3 x 300 秒的 bounded 性能矩阵。

### 当前夹具证据能证明什么，不能证明什么？

它能够证明：在固定输入下，C2 exact revision 到 C3 重建逐 cell 等价；重复、乱序、
缺包、损坏和 epoch 切换按契约拒绝或 resync；producer/receiver 资源有界；同一 epoch
的 10 Hz bounded workload 能 drain 到 latest；冻结版本在当前窗口内没有触发业务内存
错误或持续增长门。

它不能单独证明：任意真实洞穴扫描分布、真机驱动开销、bag 回放时序、长时间自主探索、
真实无线链路、C4 背压/丢包或 shared-view alignment 消费均已验收。恢复历史 bag 或重新
录制兼容 bag 后，可以增加“bag -> 当前 C2 -> C3”的补充集成回归，但其结论必须与
确定性性能矩阵分开报告。

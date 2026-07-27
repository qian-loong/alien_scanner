# C2 本机观测消费与局部地图

## Goal

在每个 vehicle session 的本机计算域内，将 C1 产出的可追溯
`LidarObservation` 与 `PoseEstimate` 消费为唯一权威的局部 occupancy 地图，
使地图在多传感器、能力降级、位姿异常和 fleet 断开场景下仍具有明确、可测试的
occupied/free/unknown、revision、freshness 与 map epoch 语义。

## Background

- C1 已冻结 observation、pose、sensor/session identity、descriptor inventory、
  `HitOnly < HitRay < FullRay` 和 mapper minimum/degraded 输入契约。C2 只为
  `HealthState` 补充 producer session 与 canonical contract fingerprint 绑定，不改变
  上述语义或枚举数值。
- 父任务已确定 D-008、D-013、D-020 至 D-025：地图契约后端无关，OctoMap
  是默认参考实现；每个 vehicle session 只有一个 active mapper；authoritative
  mapper 位于 vehicle-local compute domain；pose 不连续时 fail closed。
- C2 是关键路径 `C1 -> C2 -> C3` 的第二步。C3 才定义跨进程
  snapshot/keyframe/delta/summary 和 resync 协议。

## Requirements

### R-C2-01：唯一权威本机地图

- 每个 vehicle session 同时只能有一个 active mapping pipeline 和一条
  authoritative local occupancy revision 链。
- active mapper 自己持有并判定冻结的 `MapperInputContract` 与 sensor inventory；
  C1 health 必须携带相同 contract fingerprint 才可作为保守上限，不能单独授权提交。
- 多个 2D/3D sensor 按 batch 与 origin 独立进入同一 pipeline，不得先合并来源
  或启动并行权威地图。
- 替代 mapper 只允许在进程启动时选择；首版不运行实时 shadow mapper。

### R-C2-02：后端无关 occupancy 契约

- ROS-free 算法边界必须表达 occupied、free、unknown 查询及当前地图
  revision、epoch、frame、freshness 和有效 capability。
- C2 `LocalMapState` 必须发布 frame、resolution 与 canonical lattice origin；地图
  content hash、changed payload、base revision 和 update kind 由 C3 在生成
  `LocalMapUpdate` 时负责，不能塞入 C2 状态消息。
- revision 表示一次包含合法 occupancy evidence 的 authoritative commit。更高
  origin stamp 的新 batch 即使产生相同 cell evidence，也推进一次 revision 并刷新
  真实 commit freshness；相同 `(sensor, session, origin_stamp)` 的传输重放、拒绝、
  空输入和全 Invalid batch 不推进、不刷新。
- capability 采用混合表达：有天然全序关系的射线证据继续复用 C1 的闭合枚举
  `HitOnly < HitRay < FullRay`；相互独立的 mapper 能力使用具名结构化字段，
  不使用位掩码，也不把所有组合压成单一枚举。
- 未知射线枚举值、未知/不受支持的结构化能力或矛盾组合必须 fail closed，不得
  推断为更高能力。
- OctoMap 只作为默认参考后端和兼容 adapter 出现；上层算法不得直接依赖
  `octomap::OcTree`。
- 至少一个轻量、确定性的替代 fixture 使用同一 conformance suite。
- canonical voxel lattice 必须固定 origin、负坐标量化、cell/region 半开边界和
  out-of-range 结果；不同 backend 不得各自解释坐标边界。
- ordinary Rejected/NoEvidence 必须保证地图逐 cell 不变。若 backend 在 apply 期间
  发生异常或无法证明无部分写入，立即销毁 backend、封禁旧 view 并进入新 map
  epoch 空图恢复；不得继续发布或查询旧 revision。

### R-C2-03：射线证据能力上限

- `HitOnly` 只能写 occupied endpoint，不得伪造 free space。
- `HitRay` 可写 origin-to-hit free space，但 hit endpoint 只写 occupied；不得把
  no-return 扩展到最大量程。
- `FullRay` 才可消费明确 no-return 的完整 free-space ray；最大量程 boundary
  endpoint voxel 保持 unknown，只把此前穿越的 voxel 写为 free。
- 不能表达上述 required 语义的 backend 不得成为首版 active mapper；optional
  feature 必须显式 unsupported，任何 backend 都不得静默提升 observation capability。

### R-C2-04：输入健康、地图推进与恢复

- Healthy 要求 usable pose 与启动时声明的完整 sensor contract 均满足；
  Degraded 要求 usable pose 仍成立，且剩余输入至少满足一个显式允许的 degraded
  combination。两者均可推进地图，但必须发布当前实际 capability。
- 允许的 degraded combination 最低可降至一个健康 LiDAR 与 `HitOnly`；具体
  sensor type/count/ID 仍由 mapper 启动配置声明，不能在运行期临时放宽。
- pose 缺失/stale/frame 错误/quality 不足，或没有任何允许的 sensor combination
  时为 Unavailable 并停止新 revision；旧地图 freshness 按确定性策略到期，
  不得由 timer 或无效输入伪刷新。
- C2 使用 C1 `MapperHealthGate` 与同一冻结 contract/inventory 自行判定最终 mapper
  health；C1 health 与 producer session、contract fingerprint 绑定，只能保持或
  降低该结果。旧/未知 session、fingerprint 不匹配或缺失的 health 不能授权提交。
- wire origin 只用于追溯，本机到期使用 receive-side monotonic validity budget，
  timer 只允许使状态到期；跨进程只发布剩余 validity budget，不传播 steady
  time point/deadline。
- sensor/pose 恢复后必须重新通过健康稳定门，才能恢复 revision 推进。

### R-C2-05：Pose 连续性与 map epoch

- 经当前 health/contract 授权的 pose source/session/reset epoch 前进或确认的空间
  跳变，立即且只执行一次旧链关闭与 map epoch 推进；旧地图不得接收新观测。
- 重复/旧 pose stamp、retired session、reset epoch 回退或旧 DDS replay 只拒绝并
  fail closed，不重复清图或推进 epoch；active/retired lineage 与 high-water fence
  必须保留到 mapper session 结束。
- 同一事件必须立即使旧 alignment 失效，不得在等待新 alignment 期间继续沿用；
  恢复顺序固定为关闭旧 map revision 链、推进新 map epoch、等待新 pose Ready，
  再由属于新 epoch 的 alignment Ready 解锁 shared-frame 消费。
- 本机 source-local 地图可在新 pose Ready 后从空地图重建，不依赖 shared
  alignment；首版不原地重投影旧体素。
- 时间回退、frame 变化、位置跳变和 pose producer 新 session 必须可确定性回放。
- observation 与 pose 只有在 `clock_domain` 相同且 pose 不晚于 observation
  acquisition stamp 时才可配对；首版不隐式换算不同 clock domain。

### R-C2-06：坐标与 alignment 边界

- authoritative local map 保持 source-local map frame；进入 shared frame 的
  alignment 使用独立 epoch/revision。
- C2 只提供静态/fixture alignment 的本机契约、epoch 绑定与失效边界；它不负责
  shared/global map 聚合，也不以地图共享成功作为本机建图的启动条件。
- 在线地图配准不成为首版隐藏前提；地图共享链路及跨 source 对齐效果留待后续
  子任务验证。
- C2 提供仅供 ROS-free fixture/C3 adapter 调用的 alignment reference API；它校验
  map source/session/epoch、provider session 和单调 alignment revision。C2 不新增
  alignment ROS topic，也不应用 shared transform。

### R-C2-07：本机自治与旧路径对照

- 断开全部 fleet 链路后，本机仍可消费 observation、推进 LocalMapState，并供
  本机安全查询使用。
- 旧 `scan_returns`/OctoMap builder 仅保留为隔离的行为对照，不得与新 pipeline
  同时写同一地图实例。
- 固定 C1 隧道 fixture 与 Phase 3 本机安全 oracle 用于 occupied/free/unknown、
  frontier/known-free 相关回归。

### R-C2-08：首版 required/optional capability

- active mapper 的 required runtime input 是 usable pose、至少一个符合已声明
  Healthy/Degraded contract 的健康 LiDAR，以及最低 `HitOnly` 射线证据。
- mapper backend 的 required capability 是：按证据上限集成 occupied/free、保留
  unknown，提供 occupied/free/unknown 点查询与确定性 bounded-region 查询，支持
  reset/清空并报告 current known bounds。
- mapper engine 的 required behavior 是：维护单调 map epoch/revision、event-driven
  freshness/state sequence，并提供绑定 identity、epoch、revision、geometry 和 known
  bounds 的一致 read transaction；C3/OctoMap adapter 的多次读取不得混合 revision。
- 额外 sensor、`HitRay` 与 `FullRay` 是 optional runtime input；缺失时降低当前
  有效 capability，但只要仍满足已声明组合，就不得仅因缺失这些增强输入而拒绝
  mapper 启动或推进。
- hierarchy/multi-resolution、dirty-region、delta/serialization 和地图持久化是
  optional backend capability；不支持时必须显式声明，不得模拟成功。C3 再决定
  哪些 optional capability 成为地图更新协议的前置条件。
- alignment、shared/global map 与多机融合不属于本机 mapper required capability；
  alignment 未 Ready 或 fleet 断开不得把本机地图降为 Unavailable。
- effective capability 必须包含 satisfied-combination ID、按 2D/3D 与射线等级的
  active sensor counts，以及明确命名的 maximum active ray evidence；该 maximum
  不能被解释为所有 sensor/整张地图的统一证据等级，known-free 资格仍由实际区域
  occupancy query 决定。
- 零 active sensor 必须通过独立 presence 字段表达，不能把 `HitOnly` 伪装成实际
  maximum。正式 wire 必须声明闭合 health/ray 常量以及 unknown 值 fail-closed 行为。

### R-C2-09：连续直线场景建图验收

- 使用既有 `TreeCaveField` 与 `cave_publisher`，冻结 `cave_mode=tree`、`seed=42`、
  `base_radius=2.5 m`。真值 `/cave/points` 只进入 RViz 和自动测试 oracle，不进入
  C1 或 C2 算法输入。
- 复用 `LineTrajectory`/`FakeOdomNode`，冻结轨迹
  `(1,0,1.5) -> (11,0,1.5)`：沿 map `+X` 匀速 `0.5 m/s` 移动 `10 m`，总时长
  `20 s`；全程 y、高度、yaw/pitch/roll 固定，odometry 为 `20 Hz`，
  `motion.mode=line` 且 `altitude_adapt.enable=false`。
- 在 `drone_scanner` 增加薄 `CaveLaserScanNode`，复用 ROS-free `FakeLidar` 与
  `ICaveField`，以 `10 Hz` 发布标准 `sensor_msgs/LaserScan`：`360` 束、范围
  `[0.1,30] m`、零噪声、`FullRay`；`angle_min=-pi`、
  `angle_increment=2*pi/360`、`angle_max=pi-angle_increment`，descriptor FOV 与消息
  实际首末样本完全一致。命中仅在有限且位于闭区间 `[range_min,range_max]` 时写
  有限 range，明确 no-return 写 `+inf`；其他非有限值或越界有限 hit 使整帧
  fail closed，不得提升为 no-return。`intensities` 为空、`time_increment=0`、
  `scan_time=0.1 s`，整帧使用同一 acquisition pose，不做 deskew。
- scan frame 的标准 XY 平面只通过冻结静态 `body <- scan` TF 映射为 body YZ
  垂直环。C1/C2 descriptor 的 `mounting_position/mounting_orientation` 是参与
  fingerprint 和外参一致性校验的期望元数据，不是第二个变换来源；C2 实际使用的
  `body_from_sensor` 唯一来自 observation acquisition stamp 上的 TF sample。
- LaserScan 与对应 map-frame pose 使用同一 acquisition stamp。由于 C1 pose 与
  observation 位于不同 topic，validation scene 必须先观察到 C1 已发布同 stamp、
  同 pose frame/source/clock-domain 的 `PoseEstimate` watermark，再延迟至少 `100 ms`
  才把该 raw scan 释放给 C1；超时、stamp 回退或 lineage 不一致均丢弃并诊断。
  该 validation-only gate 归属 `perception_fixtures`，不得让通用 `drone_scanner`
  依赖 `perception_interfaces`。自动测试须证明 C2 没有 pose missing/stale rejection，
  不能只比较消息 stamp。
- C1/C2 使用完全相同的单 2D sensor `FullRay` descriptor、minimum/degraded contract
  与 pose 要求。轨迹完成后的首个零速度 odometry 不创建新 acquisition；允许排空
  此前 pending scan，随后等待 pose lead delay 与 transport drain 后，revision、
  last-commit provenance 和 freshness 不得被终点停驻帧刷新。
- 新增独立 scene validation launch/RViz，保留现有确定性切片 launch；场景入口不得
  启动 `/scan_returns`、`scan_accumulator`、`scan_accumulator/cloud_map`、legacy
  `OctoMapBuilder` 或第二个 mapper；真值、轨迹、当前标准 scan 与 C2 累积地图只在
  RViz/测试层汇合。
- cave、trajectory、scan、timing 和 descriptor/contract 必须来自一个 canonical
  structured scene config；launch 只读取一次并向各节点 fan-out，自动测试读取同一
  config，不复制运行时常量。

## Acceptance Criteria

- [x] 多个不同 origin 的 2D/3D observation 进入同一 active mapper，来源与
  session 不丢失，且只产生一条权威 map revision 链。
- [x] `HitOnly`、`HitRay`、`FullRay` 固定 fixture 分别产生符合能力上限的
  occupied/free/unknown 结果；低能力输入不能伪造高能力证据。
- [x] 后端无关 query/apply conformance suite 同时通过默认 OctoMap backend 与一个
  轻量替代 fixture，算法测试不依赖 ROS 或 `octomap::OcTree`。
- [x] conformance 覆盖 canonical lattice 的负坐标、边界前后、半开 region 与
  out-of-range；故障注入证明 Rejected/NoEvidence 不变，backend exception 后旧 view
  不可查询且只能从新 epoch 空图恢复。
- [x] 单路和多路 sensor 掉线可确定性产生 Healthy、Degraded、Unavailable；
  `HitOnly` 仍可作为显式允许的最低组合推进地图，Unavailable 不推进 revision、
  不伪刷新 freshness，恢复需重新通过健康门。
- [x] C1/C2 contract fingerprint 不一致、C1 重启后的旧 health session、跨
  clock-domain pose 和 receive-age 已到期输入均 fail closed；传输重放、相同
  evidence 的新 batch 与 NoEvidence 对 revision/freshness 的行为可区分。
- [x] required backend capability 在两个后端上均通过 conformance；optional backend
  capability 可显式为 unsupported，且消费者不会调用、推断或伪造该能力。
- [x] revision read transaction 将 identity/epoch/revision/geometry/known bounds 与
  query snapshot 绑定；并发 apply/reset/fault 不产生混合 revision，C3/OctoMap snapshot
  adapter 不复制整棵图。transaction 不暴露 backend/view/raw reference，close/move-from/
  revoke 后所有操作稳定 fail closed，不产生悬空引用或 UB。
- [x] state 使用 reliable/volatile heartbeat 与严格 state sequence；late join、延迟旧
  样本、publisher crash、deadline 和 remaining-budget 到期均不能复活 map freshness，
  heartbeat 不推进 map revision 或改写 last-commit provenance。
- [x] pose stale、frame 错误、quality 降级和时间回退均 fail closed；旧/replay
  lineage 不重复推进 epoch，只有获授权的新 pose session/reset 前进或确认的空间
  跳变关闭旧链、立即失效 alignment 并推进一次新 map epoch。本机地图在新 pose
  Ready 后可独立重建，测试证明不存在跨 epoch 体素或 alignment 污染。
- [x] alignment fixture API 可提交当前 map epoch 的 Committed reference；pose reset
  后立即 Revoked，旧/回退/错 source alignment 不能恢复 shared-ready 状态。
- [x] fleet 完全断开时本机地图和本机 known-free 安全查询仍连续工作。
- [x] 固定 C1 fixture 与 Phase 3 oracle 的 occupied/free/unknown 和本机安全行为
  对照通过；旧、新 mapper 不会同时写同一地图。
- [x] Release 构建、ROS-free gtest、必要 launch test、确定性 replay、资源基线和
  `git diff --check` 全部通过。
- [x] 冻结场景在 `20 s` 运动窗口内持续产生标准 `LaserScan`，C1/C2 接受多个不同
  acquisition stamp 的 batch，C2 在同一 map epoch 内单调推进至少 `100` 个
  revision，且 diagnostics 中没有 pose missing/stale rejection；终点 drain 后停止
  scan，revision 与 last-real-commit freshness 不再伪刷新，heartbeat sequence 可继续。
- [x] 自动测试证明 C2 `known_bounds` 的 X 向覆盖随无人机从 `x=1` 飞至 `x=11`
  扩展至少 `9 m`。ROS-free scene gtest 使用同一 `TreeCaveField + FakeLidar` 场景配置
  和 mapper read transaction，在固定 map-frame 点/区域同时得到预期 occupied、free、
  unknown；launch test 只验证真实 C1/C2 链的 revision/epoch/bounds/provenance 与 graph，
  不把 OctoMap 二进制解析冒充 authoritative query。洞穴真值只作为独立 oracle，
  不被注入 observation 或 mapper。
- [x] C1 与 C2 冻结的单 2D `FullRay` sensor descriptor、minimum/degraded contract、
  frame/FOV/range/angular resolution、pose 要求和 contract fingerprint 完全一致；
  hit/no-return 分别以有限值/`+inf` 穿过标准 LaserScan 适配链且不提升未知证据。
- [x] 独立 RViz 场景同时显示洞穴真值、飞行轨迹、当前标准 LaserScan 和 C2 累积
  OctoMap；ROS graph 断言不存在 `/scan_returns`、scan accumulator/`cloud_map`、
  legacy builder 或第二个 authoritative mapper，并保留原 C2 静态 debug 场景不变。
  自动 smoke 还必须解析 RViz config 锁定五类 display/topic，证明唯一
  `LD_PRELOAD=liboctomap.so` 只属于 RViz 进程，并在真实 RViz 进程中确认 OctoMap
  plugin 已加载、无 undefined symbol 且订阅预期 topics；GUI 外观仍由用户单独目检。

## Out Of Scope

- C3 的 snapshot/keyframe/delta/summary、跨进程 LocalMapUpdate 和 resync 协议。
- shared/global map 聚合、Relay/EdgeAggregator、任务分配与 fleet 数据面。
- 在线地图配准、回环闭合、实时 shadow mapper、远程 raw LiDAR 建图卸载。
- 地图持久化/加载、真实 VIO/LIO/SLAM 或飞控实现。

## Follow-up TODO

- 在后续地图共享子任务中，使用至少两个 source-local map 与版本化静态
  alignment 验证 shared/global map 聚合、alignment 失效后的旧贡献撤销、
  keyframe/resync，以及跨 source 对齐的 RViz 可视效果；该验证不阻塞 C2
  本机地图验收。

## Notes

- Phase A-H 功能、自动化和人工 RViz 验收均已完成。最终 Release 8 包构建通过，
  `248 tests, 0 errors, 0 failures, 0 skipped`；full-scope 审核结论为
  `0 Blocking / 0 High`，用户确认连续 FullRay 洞穴场景显示正常。
- 本任务已满足验收条件，但提交与归档仍需用户明确授权；后续性能/内存分析作为
  独立基线任务，不回写或扩大本 PRD 的功能范围。
- 本 PRD 已采纳父任务 `C2-OPT-1`、`C2-OPT-2`、`C2-OPT-3`。
- 父级来源：`.trellis/tasks/07-21-perception-swarm-architecture-refactor/` 与
  `docs/decisions/perception-and-swarm-architecture-refactor.md`。

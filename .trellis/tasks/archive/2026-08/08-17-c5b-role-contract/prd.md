# C5b 角色契约：能力、主角色与服务迁移

## 1. 目标

在 C5a 已完成的稳定身份、membership、三张逻辑图和 route epoch 之上，定义多无人机协作
可共同消费的角色契约：静态声明能力、当前有效能力、一个主角色、可组合服务、全局
`role_epoch`、资源预算以及 quiesce/handoff/commit/rollback 生命周期。

C5b 的输出是 ROS-free 的角色状态机、确定性 `RoleSnapshot` 和必要的 ROS 转换。C5c 的
Explorer/纯 Relay、C5d 的 EdgeAggregator 和 C6 的任务分配只能消费该契约，不得各自定义
私有角色字段或把拓扑位置等同于业务角色。

## 2. 已确认背景

- C5a 以 `VehicleIdentity` 区分 stable vehicle 与 vehicle session，并只允许 `Ready` 成员成为
  committed link/route 端点。
- 父架构已确定中央协调器控制角色分配，不做节点自选举或算法热切换。
- 每个 vehicle session 同时只有一个主角色；Relay/Aggregation 是可独立预算、独立健康的
  服务能力。已明确的组合是专用 Explorer、Relay、EdgeAggregator 以及
  `Explorer + RelayService`。
- 角色变化必须使用单调 `role_epoch`；新 session 不继承旧 session 的 role epoch、任务或
  服务所有权。
- C5b 不读取 C4 MapUpdate v2 的 Merkle/chunk 内容，也不实现 C5c/C5d 的数据处理行为。

## 3. 需求

### R1 能力声明与当前有效能力

- 为每个精确 `VehicleIdentity` 定义 canonical `CapabilitySet`，至少能表达 exploration、纯
  relay forwarding 和 map aggregation。
- 区分 vehicle session 内冻结的 declared capabilities 与可随健康证据变化的 effective
  capabilities；effective set 必须是 declared set 的子集。
- 相同 vehicle session 的 declared capability drift 必须拒绝；旧 session、未知成员、重复
  capability 和过期 capability revision 必须 fail closed。
- capability registration 可以在 Joining/Resyncing 阶段完成，但任何角色生效前成员必须是
  C5a committed snapshot 中的 Ready 成员。

### R2 一个主角色与可组合服务

- `PrimaryRole` 至少表达 Explorer、Relay、EdgeAggregator 和 Reserve；没有 committed
  assignment 的成员不通过伪造 `Unassigned` 角色进入角色快照。
- 每个 vehicle session 在一个 committed role snapshot 中最多一个主角色。
- `ServiceAssignment` 至少表达 RelayService 和 AggregationService，并具有独立预算、状态和
  必需 capability。
- Explorer + RelayService 必须可表达；专用 Relay 不得被实现为同时解析/聚合地图的“智能
  Relay”。Reserve 只提供稳定契约与测试替身，不作为首版生产行为。
- 首版 AggregationService 只允许 `PrimaryRole::EdgeAggregator` 持有；Explorer、Relay 和
  Reserve 即使声明 aggregation capability 也不得获得聚合服务 assignment。

### R3 C5a 拓扑与 session 门控

- role candidate 必须引用精确的 C5a `topology_epoch` 和 `VehicleIdentity`；未知、非 Ready、
  Lost/Quarantined/Absent、已退休 session 或 stale topology epoch 均拒绝。
- 主角色和服务所需的逻辑图路径必须存在且健康；加入 `G_map` 不自动授予 aggregation 权限，
  加入 `G_comm` 也不自动授予 RelayService。
- 拓扑或 membership 变化不会静默改写角色。关联成员失效时，角色进入显式 Draining/撤销或
  迁移流程，并通过新的 role epoch 生效。

### R4 全局 role epoch 与原子快照

- 一个 coordinator authority 下使用全局单调 `role_epoch` 表达整个 role snapshot 的原子
  生效边界，支持一次迁移同时修改 source/target 多个 assignment。
- candidate 必须引用当前 base role epoch；旧 epoch、同 epoch 改内容、重复 role/service
  owner、超限 assignment 和引用不闭合必须确定性拒绝。
- candidate 只有在身份、拓扑、capability、预算、组合策略和迁移前置全部通过后才能一次性
  替换 committed snapshot；失败保持最后合法角色状态。

### R5 角色迁移生命周期

- 角色变化必须能表达 prepare、quiesce、handoff、commit 和 rollback；旧 assignment 在
  commit 前仍是最后权威状态，但 quiesce 后不得接受新职责。
- migration/transition 使用稳定 ID 和幂等 acknowledgement；重复 ack 不推进两次，未知、旧
  session 或不属于本次 transition 的 ack 必须拒绝。
- commit 前必须收齐本次变更所需的 quiesce/handoff 前置；rollback 丢弃 candidate 并恢复旧
  assignment 的可服务状态，不产生双 owner 或无依据的新 owner。
- C5b 只定义 ROS-free 生命周期和可序列化 transition descriptor；实际 ROS action adapter
  留给 C5c/C5d 的运行期实现。

### R6 健康与资源预算

- role/service assignment 必须校验 effective capability、VehicleHealth、ResourceHealth 和
  明确预算；Unknown 不等于 Healthy。
- 至少能区分 LowPower、Failsafe、ComputeOverBudget、LinkDegraded 和 Unknown 对角色/服务
  的影响：拒绝新 assignment、仅降级某项服务或触发 Draining。
- 服务预算与健康彼此独立；RelayService 降级不得自动伪造 AggregationService 降级，反之
  亦然。健康变化不得绕过 role epoch 静默获得新权限。
- 所有 role、service、transition、ack 和 capability 集合具有配置上限及有界诊断。

### R7 确定性 ROS 契约

- RoleSnapshot、capability registration、role assignment、service assignment 和 transition
  descriptor 使用稳定排序、显式协议版本和 bounded sequence。
- conversion 在 reserve/分配前检查 schema 上限与运行时上限；未知 enum、非法版本、旧
  topology/role epoch 或非 canonical 顺序原子拒绝。
- ROS-free 核心不得 include `rclcpp` 或 ROS 生成消息；ROS conversion 只负责字段转换和边界
  校验，不实现角色策略。

### R8 诊断与验证

- 诊断至少区分 capability drift、capability unavailable、non-ready member、stale topology、
  stale role epoch、invalid combination、budget exceeded、transition prerequisite、unknown ack
  和 rollback。
- 固定 fixture 至少覆盖 Explorer、专用 Relay、专用 EdgeAggregator、Reserve 测试替身、
  Explorer + RelayService、旧 session、断链、服务降级和双节点 handoff。

## 4. 验收标准

- [x] declared/effective capability、session 冻结和 revision 规则通过 ROS-free 单测。
- [x] 一个主角色、服务唯一性及允许/禁止组合通过确定性策略测试。
- [x] 非 Ready、旧 session、stale topology/role epoch 和 capability 不足均被拒绝，且 committed role snapshot 不变。
- [x] 全局 role epoch 可原子提交涉及多个成员的 assignment 变化，并拒绝同 epoch 内容漂移。
- [x] quiesce/handoff/commit/rollback 和幂等 ack 在固定 fixture 中不产生双 owner 或半提交状态。
- [x] RelayService/AggregationService 的预算和健康可独立降级；Unknown 不获得角色或服务权限。
- [x] RoleSnapshot ROS round-trip 保持 canonical 内容；bounded collection 和未知 enum 在分配前拒绝。
- [x] C5a topology/session contract 和 C4 opaque MapUpdate v2 回归不受影响。
- [x] 任务文档、backend code-spec、验证记录和 C5c/C5d 前置条件同步。

## 5. 非目标

- 不实现 Explorer 的路径/任务行为、真实 Relay 转发进程或 EdgeAggregator 地图聚合。
- 不实现 C6 Region matching、task owner、lease 或任务 action。
- 不实现节点自选举、分布式共识、协调器 HA 或运行期算法热切换。
- 不修改 C3/C4 wire、Merkle、chunk store、route forwarding 或 trust 契约。
- 不把 role assignment 嵌入 C5a `TopologySnapshot`；角色快照只引用已提交 topology epoch。

# C5b 角色契约：技术设计

## 1. 分层与边界

```text
C5a TopologySnapshot + VehicleIdentity/session
    -> C5b capability registration/effective capability
    -> C5b role candidate + transition barrier
    -> committed RoleSnapshot(role_epoch, topology_epoch)
    -> C5c Explorer/Relay runtime
    -> C5d EdgeAggregator runtime
    -> C6 task eligibility/lease
```

- `swarm_data_interfaces`：最小 capability/role/service/transition/snapshot ROS 消息。
- `swarm_data_plane` ROS-free core：角色类型、组合策略、candidate validation、transition 和
  atomic commit；继续复用 C5a identity/topology 类型。
- `swarm_data_plane` ROS conversion：bounded preflight 与 canonical round-trip。
- `swarm_controller`：本步不实现 allocator、角色 action server 或任务分配。

不创建新的算法接口。C5b 当前只有一种明确的中央协调策略，直接使用具体 `RoleState`；未来
出现第二种 policy/allocator 时再提取接口。

## 2. 核心数据模型

建议的 ROS-free 类型：

```text
CapabilityKind
CapabilityRegistration
EffectiveCapabilitySnapshot
PrimaryRole
ServiceKind
ServiceBudget
ServiceAssignment
RoleAssignment
RoleSnapshot
RoleCandidate
RoleTransition
RoleTransitionAck
RoleState
```

`CapabilityRegistration` 绑定精确 `VehicleIdentity`，包含单调 registration/revision 和 canonical
declared capability set。declared set 在 vehicle session 内冻结；动态健康只允许收缩/恢复
effective set，不能新增未声明能力。

`RoleSnapshot` 包含协议版本、`topology_epoch`、全局 `role_epoch` 和 canonical assignments。
没有 assignment 的成员不出现在 snapshot；Reserve 是显式测试角色，不作为“无角色”的占位值。

## 3. 主角色与服务组合

| 主角色 | 必需能力 | 首版允许服务 | 说明 |
| --- | --- | --- | --- |
| Explorer | Exploration | RelayService | 允许预算内兼任纯转发 |
| Relay | RelayForwarding | RelayService | 纯 Relay，不解释 MapUpdate 内容 |
| EdgeAggregator | MapAggregation | AggregationService | 只聚合 revisioned map update |
| Reserve | 无生产要求 | 无 | 稳定契约与 fixture |

每个 assignment 只有一个主角色；service list canonical 排序且 kind 唯一。服务权限来自显式
assignment，不从 graph membership 或 capability declaration 自动推导。

首版固定拒绝 Explorer、Relay 和 Reserve 持有 AggregationService，以保持 C5c 纯 Relay 与
C5d EdgeAggregator 的独立验收边界。未来若需要混合聚合职责，必须作为新的策略版本重新
评审资源抢占、降级优先级和 handoff 语义，不能只放宽 enum 校验。

## 4. Capability 与健康证据

能力分两层：

1. declared capabilities：vehicle session 内冻结的最大能力集合；descriptor drift 要求新 session。
2. effective capabilities：当前健康证据允许使用的 declared 子集，带单调 evidence revision。

角色 core 消费结构化健康证据，不依赖具体 BatteryState/diagnostics adapter。最小证据区分：

```text
VehicleHealth: Healthy / Degraded / LowPower / Failsafe / Unknown
ResourceHealth: Healthy / ComputeOverBudget / MemoryOverBudget /
                LinkDegraded / Unknown
```

每项 service 持有独立 `ServiceBudget` 和状态。预算使用明确整数单位，首版至少包括 queue bytes、
memory bytes、network bits/s 和 contributor/parallel-work 上限；所有加法和比较使用 checked
逻辑。C5b 不建立未经测量的默认性能门，只验证声明预算与配置上限。

健康证据不是角色 authority。`RoleSnapshot` 只保存 committed assignment 及 Active/Draining
等权威生命周期，不内嵌会高频变化的原始健康采样。健康或 effective capability 降级可以立即
阻止新工作并使 transition prerequisite 失败，但撤销、迁移或授予新权限仍必须通过新的
role candidate 和 role epoch；恢复健康也不能静默恢复已经撤销的权限。

## 5. Role epoch 与 candidate commit

全局 `role_epoch` 是一个 committed role snapshot 的 authority revision。candidate 至少包含：

```text
base_role_epoch
topology_epoch
assignments[]
transition_id
```

验证顺序：

1. C5a topology protocol/epoch 合法且成员引用闭合。
2. capability registration 属于同一精确 session，effective revision 非 stale。
3. 主角色、服务组合、graph prerequisite、预算和资源上限合法。
4. candidate 相对 committed snapshot 的变更集合可确定。
5. 变更涉及的旧 owner 已满足 quiesce，责任转移已满足 handoff。
6. 全部通过后推进一次 role epoch，并原子替换完整 snapshot。

同一 role epoch 的不同内容为 conflict；旧 base epoch 为 stale。新 vehicle session 必须重新注册
capability 并完成新 role assignment，不能继承旧 role epoch 下的权限。

## 6. Transition 生命周期

```text
Prepared
  -> Quiescing
  -> HandoffReady
  -> Committed

Prepared/Quiescing/HandoffReady
  -> RolledBack
```

`RoleTransition` 保存旧 snapshot fingerprint、目标 candidate、changed vehicle set 和 required ack
集合。ack 以 transition ID + vehicle identity + ack kind 幂等记录。

- quiesce 表示旧 assignment 不再接受新职责，但 committed owner 尚未更换。
- handoff 表示旧责任已可安全交接；不复制地图 payload，只交接 role/service authority metadata。
- commit 一次推进全局 role epoch。
- rollback 丢弃 candidate 和 ack，恢复旧 assignment 的可服务状态。

C5b 不启动 ROS action。C5c/C5d 后续把 action goal/feedback/cancel 映射为这些幂等领域事件。

## 7. C5a 图前置

- 所有 assignment 目标必须是 `Ready` member。
- RelayService 要求相关 communication/control/map route 前置由配置声明并在 candidate 中验证；
  C5b 不创建 route。
- AggregationService 至少要求 `G_map` 可达，但 `G_map` 可达本身不授予聚合权限。
- topology epoch 改变使尚未 commit 的 transition stale；调用者必须基于新 topology 重新规划。
- committed role snapshot 不被 topology state 静默修改；成员失效通过后续 role candidate 进入
  Draining/撤销。

角色与 topology 的提交顺序采用明确两阶段：协调器先提交满足物理/逻辑连通性的 C5a topology，
再基于该 topology epoch 提交 role candidate。C5b 不在一次调用中同时修改两套 snapshot，避免
形成跨状态机的半提交 API；topology 在 role transition 未完成时再次变化会使 transition stale。

## 8. ROS 消息与转换

最小消息集合预计为：

```text
CapabilityRegistration
ServiceBudget
ServiceAssignment
RoleAssignment
RoleTransitionDescriptor
RoleSnapshot
```

所有数组使用 bounded sequence，并与 `RoleLimits` 默认值对齐。conversion 在构造 vector 前
检查字符串、集合、budget 和嵌套 service 数量，再调用 ROS-free canonical validation。未知
enum/version 不能通过 `static_cast` 静默进入 snapshot。

是否需要自定义 action 延后到 C5c 的真实进程边界决定；C5b 不为尚不存在的 action server
提前冻结 `.action` wire。

## 9. 验证策略

- `TestRoleState`：能力冻结/effective subset、Ready 门、组合矩阵、role epoch、candidate
  atomicity、双成员 handoff、幂等 ack、rollback、预算和健康门。
- `TestRoleConversions`：canonical round-trip、bounded preflight、unknown enum/version、stale
  topology/role epoch 和 nested service limit。
- 现有 `TestTopologyState`/`TestTopologyConversions`/C4 route tests 全量回归。
- 本步不新增 RViz：角色可视化随 C5c 真实 Explorer/Relay fixture 建立，避免显示没有运行期行为
  的静态假角色。

## 10. 回滚与兼容性

- C5b 是 C5a 之上的新旁路契约，不修改 TopologySnapshot 或 C4 envelope。
- Gate 失败时删除/禁用 RoleState adapter，保留静态 C5a topology 和 C4 direct/routed path。
- 不提供旧角色协议兼容、双读或自动 downgrade；项目尚未发布，首版只保留单一明确契约。

## 11. 主要风险

- 把 graph membership 当服务权限会产生越权；所有服务必须显式 assignment。
- 把 health update 直接当 role commit 会造成 epoch 抖动；健康证据只改变 eligibility，权限变化
  仍经 transition/role epoch。
- 使用 per-vehicle epoch 无法证明双节点 handoff 原子性；首版使用全局 role epoch。
- 过早允许任意 role/service 组合会混淆纯 Relay 与 EdgeAggregator 的验收边界。

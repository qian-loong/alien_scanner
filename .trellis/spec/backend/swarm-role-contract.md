# C5b 角色与能力契约

## 1. Scope / Trigger

本规范适用于 `swarm_data_plane` 的 ROS-free capability/role/service 状态机和
`swarm_data_interfaces` 的角色消息转换。它消费 C5a 已提交的 `TopologySnapshot`，为
C5c Explorer/Relay、C5d EdgeAggregator 和 C6 task/lease 提供统一权限边界。

本规范不实现真实转发、地图聚合、任务分配、ROS action 或 RViz 角色可视化；C4
`MapUpdate` v2 在本层保持 opaque。

## 2. Signatures

ROS-free 核心入口：

```cpp
RoleResult RoleState::register_capabilities(
    CapabilityRegistration, const TopologySnapshot &);
RoleResult RoleState::update_capability_evidence(
    CapabilityEvidence, const TopologySnapshot &);
RoleResult RoleState::begin_transition(
    std::string transition_id, RoleCandidate, const TopologySnapshot &);
RoleResult RoleState::acknowledge_transition(
    const std::string &, const VehicleIdentity &,
    RoleTransitionAckKind, const TopologySnapshot &);
RoleResult RoleState::commit_transition(
    const std::string &, const TopologySnapshot &);
RoleResult RoleState::rollback_transition(const std::string &);
bool RoleState::can_accept_new_work(const VehicleIdentity &) const;
bool RoleState::can_service_accept_new_work(
    const VehicleIdentity &, ServiceKind) const;
```

ROS conversion 入口：

```cpp
bool encode_capability_registration(...);
DecodeCapabilityRegistrationResult decode_capability_registration(...);
bool encode_capability_evidence(...);
DecodeCapabilityEvidenceResult decode_capability_evidence(...);
bool encode_role_snapshot(...);
DecodeRoleSnapshotResult decode_role_snapshot(...);
bool encode_role_transition(...);
DecodeRoleTransitionResult decode_role_transition(...);
```

顶层 wire 为 `CapabilityRegistration`、`CapabilityEvidence`、`RoleSnapshot` 和
`RoleTransitionDescriptor`；嵌套 wire 为 `RoleAssignment`、`ServiceAssignment`、
`ServiceBudget`、`ServiceHealthEvidence` 和 `RoleTransitionAck`。

## 3. Contracts

- capability registration 绑定精确 `VehicleIdentity`。同一 vehicle session 的 declared
  capability 集合冻结；generation 可推进，但不能改变集合。Joining、Resyncing 和 Ready
  可登记能力，只有 Ready 可获得角色。
- effective capability 是 declared capability 的动态子集，evidence revision 单调推进。
  健康或 capability 降级可以立即关闭新工作，但不能授予、撤销或恢复 role/service 权限；
  权限变化必须经过新的 role candidate 和全局 `role_epoch`。
- 每个 vehicle session 最多一个 `PrimaryRole`。Explorer 可选 RelayService；专用 Relay
  必须且只能持有 RelayService；EdgeAggregator 必须且只能持有 AggregationService；Reserve
  不持有服务。首版其他角色不得持有 AggregationService。
- graph membership 不是权限。RelayService 还要求健康 `G_comm` participation 且是至少一条
  control/map 多跳 route 的中间成员；AggregationService 要求成为已提交 map route 的 target。
- role candidate 引用当前 base role epoch 和不早于已提交角色快照的 topology epoch。完整
  candidate 通过结构、Ready、capability、健康、预算和 graph 前置后，才一次替换整个
  `RoleSnapshot`。
- 新增或修改 assignment 必须引用当前 capability evidence revision。未变化 assignment 可在
  evidence 后续推进时保留原授权 revision，避免一次健康采样迫使无关角色 quiesce/handoff；
  `can_accept_new_work()` 始终读取最新 evidence。
- transition 状态为 Prepared -> Quiescing -> HandoffReady -> Committed，或在提交前
  RolledBack。旧 owner 在 Quiesced ack 后立即停止接新工作；Ready 的旧 owner 必须同时提交
  Quiesced 和 HandoffReady ack。Lost/非 Ready 旧 owner 可被强制撤销，不阻塞 commit。
- ack 按 transition ID、精确 session 和 kind 幂等。完成/回滚的 ID 保存在
  `max_transition_history` 有界重放窗口内；窗口内重用拒绝。所有 role、service、ack、预算和
  字符串集合受 `RoleLimits` 限制。
- ROS sequence 均为 bounded；conversion 在 reserve 前同时应用 schema 上限和运行时上限。
  role/capability 顶层协议版本固定为 1，未知 enum、非 canonical 顺序或非法版本原子拒绝。
- `RoleTypes` / `RoleState` 不 include ROS；wire 转换不实现角色策略。共享
  `VehicleIdentityConversions` 是 topology/role wire 的唯一身份字段转换实现。

## 4. Validation & Error Matrix

| 条件 | 必须结果 |
| --- | --- |
| declared capability 在同 session 漂移、generation 同值改内容 | `RejectedConflict`，旧 registration 保持 |
| effective capability 不是 declared 子集、未知 capability/service | `RejectedCapability` 或 `RejectedInvalid` |
| evidence revision 回退/同 revision 改内容 | `RejectedStale` / `RejectedConflict` |
| 非 Ready、旧 session、未知 member 获得 assignment | `RejectedAdmission` |
| stale base role epoch、topology epoch 回退 | `RejectedStale`，snapshot 不变 |
| Explorer 持有 aggregation、Relay/EdgeAggregator 缺必需服务、Reserve 持有服务 | `RejectedCombination` |
| Relay 缺 `G_comm` 或中间 route，Aggregator 不是 map target | `RejectedPrerequisite` |
| Unknown/LowPower/Failsafe/ComputeOverBudget 或 capability 不可用 | 新 assignment 拒绝；已提交 assignment 立即停止接新工作 |
| ack 未 quiesce 就 handoff、错误 session/transition、未知 ack kind | `RejectedPrerequisite` / `RejectedAdmission` / `RejectedInvalid` |
| commit 未收齐 Ready 旧 owner 的两类 ack | `RejectedPrerequisite` |
| role/service/ack/budget/string 超限 | `RejectedResourceLimit` 或 decode preflight 失败 |
| ROS protocol、enum、canonical 顺序非法 | decode 不返回 domain object |

## 5. Good / Base / Bad Cases

- Good：Explorer 与 RelayService 显式组合，拓扑只提供 route 前置，service assignment 才提供
  转发权限。
- Good：健康 revision 推进使 RelayService 停止接新工作，但 `RoleSnapshot.role_epoch` 不变；
  后续移除其他成员时，未变化 assignment 不需要伪造一次 revision 更新。
- Good：双成员 handoff 收齐旧 owner ack 后一次推进全局 role epoch；任一前置失败时 committed
  snapshot 完全不变。
- Base：Reserve 作为无服务的稳定测试角色；C5b 不启动任何真实运行期行为。
- Bad：从 `G_map` target 自动推导 AggregationService，或从 declared capability 自动授予角色。
- Bad：健康恢复后直接把已撤销权限恢复为 Active，绕过 candidate/role epoch。
- Bad：让 C5c/C5d 各自定义私有 role 字段，或在 ROS conversion 内决定组合策略。

## 6. Tests Required

- `TestRoleState`：declared 冻结、effective subset、Joining/Resyncing 登记、Ready 门、旧 session、
  角色组合矩阵、Relay/Aggregation 图前置、预算上限和 stale epoch。
- `TestRoleState`：首次原子 assignment、双成员 quiesce/handoff、幂等 ack、unknown ack、rollback、
  Lost owner 强制撤销和 transition ID 重放窗口。
- `TestRoleState`：LowPower、Failsafe、Unknown、ComputeOverBudget、LinkDegraded、capability 收缩、
  服务健康互不串扰，以及未变化 assignment 跨新 evidence revision 的回归。
- `TestRoleConversions`：capability/snapshot/transition canonical round-trip、未知 protocol/enum、
  非 canonical 顺序、nested runtime limits 和 128-byte schema 上限。
- 全量 `swarm_data_plane` 回归必须继续覆盖 C5a topology conversion、C4 routed/aggregate、资源
  smoke、closed-loop 和真实洞穴链路。

## 7. Wrong vs Correct

Wrong：

```cpp
// 图位置和静态声明都不是 authority。
if (is_map_route_target(topology, vehicle)) {
    assignment.services.push_back(AggregationService {});
}
```

Correct：

```cpp
RoleCandidate candidate = coordinator.plan_roles(topology, evidence);
auto prepared = role_state.begin_transition(
    transition_id, std::move(candidate), topology);
// 完整 validation + handoff barrier 通过后再原子 commit。
```

Wrong：

```cpp
// 高频健康证据不能静默改写权限 epoch。
snapshot.role_epoch++;
snapshot.assignments[index].lifecycle = RoleLifecycle::Draining;
```

Correct：

```cpp
role_state.update_capability_evidence(std::move(evidence), topology);
if (!role_state.can_accept_new_work(identity)) {
    // 停止接新工作；权限撤销由新的 role candidate 完成。
}
```

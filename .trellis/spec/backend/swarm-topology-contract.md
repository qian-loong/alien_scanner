# C5a 拓扑基础契约

## 1. Scope / Trigger

本规范适用于 `swarm_data_plane` 的 ROS-free topology/membership/link/route 状态和
`swarm_data_interfaces` 的 `TopologySnapshot` 转换。它位于 C4 routed envelope 之上，
供 C5b/C5c/C5d/C6 消费；本规范不定义角色分配、Relay 行为、EdgeAggregator 聚合或任务
租约。C4 `MapUpdate` v2 在 C5a 中保持 opaque。

## 2. Signatures

ROS-free 核心入口：

```cpp
TopologyResult TopologyState::register_vehicle(VehicleRegistration);
TopologyResult TopologyState::set_prerequisites(
    const VehicleIdentity &, ResyncPrerequisites);
TopologyResult TopologyState::transition_member(
    const VehicleIdentity &, MembershipState);
TopologyResult TopologyState::replace_topology(TopologyCandidate);
TopologyResult TopologyState::validate_route(
    const RouteDescriptor &, std::uint16_t current_hop,
    std::uint64_t accumulated_forwarding_ns) const;
```

ROS conversion 边界：

```cpp
bool encode_topology_snapshot(
    const TopologySnapshot &, swarm_data_interfaces::msg::TopologySnapshot &,
    std::string &, const TopologyLimits & = {});
DecodeTopologySnapshotResult decode_topology_snapshot(
    const swarm_data_interfaces::msg::TopologySnapshot &,
    const TopologyLimits & = {});
```

消息层由 `VehicleIdentity`、`ComponentRegistration`、`SensorDescriptorIdentity`、
`MemberRecord`、`LinkDescriptor`、`GraphEdge`、`RouteHop`、`RouteDescriptor` 和
`TopologySnapshot` 组成。默认 bounded sequence 上限为 member 64、link 256、edge 512、
route 256、hop 64、每成员 component/sensor 32。

## 3. Contracts

- 稳定主键是 `fleet_id + vehicle_id`；每次整机启动必须使用新的
  `vehicle_session`，component/source session 可在同一 vehicle session 内推进。
- 同一 vehicle session 的 sensor descriptor inventory 冻结；registration generation
  只能单调推进，不能用来伪装 descriptor drift。descriptor 变化必须新建 vehicle session。
- 白名单是 admission 前置条件。新成员按 `Joining -> Resyncing -> Ready` 收敛；Ready
  之前不得成为 link/edge/route 的端点。`Draining`、`Lost`、`Quarantined` 为 Frozen 或
  Removed 的显式状态，不静默删除 provenance。
- `G_comm`、`G_control`、`G_map` 共用节点身份但各自拥有 edge 集合；route descriptor 只
  允许 control/map 图，map route 不授予聚合权限。
- candidate 以当前 `topology_epoch` 为 base，先完成资源、身份、epoch 和引用闭合检查，
  再一次性替换 committed snapshot。任一失败都不得改变已提交 snapshot。
- 同一稳定 `link_id` 的 `link_epoch` 不能回退，也不能在同 epoch 改变 link 内容；同一
  `route_id` 的 `route_epoch` 不能回退，也不能在同 epoch 改变路径/端点/预算。link/route
  从 committed snapshot 移除后仍在有界 epoch history 中保留高水位；重新加入必须推进
  对应 epoch，不能用移除前的 epoch 重放。
- 旧 topology/route/link epoch、未知成员、断开的 hop、TTL/hop 耗尽或 freshness budget
  耗尽均 fail closed。C5a 不复制、重算或解释 C4 MapUpdate v2 的 descriptor、digest、
  revision、chunk 或 Merkle 节点。
- core 输出始终按 stable identity、link、edge、route 的 canonical comparator 排序；ROS
  conversion 只接受 canonical snapshot，并在任何 reserve 前检查运行时上限与消息 schema
  上限。

## 4. Validation & Error Matrix

| 条件 | 必须结果 |
| --- | --- |
| 空/超长/非法 UTF-8 identity、零 session、未知 whitelist | `RejectedAdmission` 或 `RejectedInvalid`，snapshot 不变 |
| 重复 component/sensor、同 generation 内容冲突 | `RejectedInvalid` / `RejectedConflict`，不产生半登记 |
| 同 vehicle 的 descriptor drift | `RejectedConflict`，旧 registration 保持不变 |
| Ready 前设置 link/edge/route | snapshot validation 失败，候选不提交 |
| 未完成 resync prerequisites 进入 Ready | `RejectedPrerequisite` |
| 旧 session 或已 fence session 重连 | `RejectedStale`；新 session 只能替换 Absent/Lost/Quarantined |
| stale base topology epoch | `RejectedStale`，committed snapshot 不变 |
| link/route epoch 回退或同 epoch 改内容 | `RejectedStale` / `RejectedConflict` |
| link/route 移除后重放相同 epoch | `RejectedStale`，committed snapshot 不变 |
| unknown member、断开的 route hop、错误 graph、TTL/freshness 超限 | `RejectedRoute` 或 `RejectedInvalid` |
| ROS 数组超过 bounded schema 或运行时 limits | decode 在 reserve 前拒绝 |
| ROS protocol/enum/availability 不认识 | core canonical validation 拒绝，不返回 snapshot |

## 5. Good / Base / Bad Cases

- Good：新成员先进入 `Joining`，完成 link/clock/alignment/map prerequisite 后进入
  `Ready`，三张图可以在不同 epoch 下分别表达路径。
- Good：Relay 后续只改变 route metadata；source/session、map revision、content digest 和
  payload identity 仍由 C4 保持。
- Base：`TopologySnapshot` 的 ROS 序列化作为跨进程编码；固定输入和 registration 顺序
  产生相同的排序、epoch 和字段内容。
- Bad：把 namespace、node name 或 PID 当作 vehicle 主键；把新 session 静默覆盖旧 session。
- Bad：在 `G_map` topology 层读取 Merkle/chunk store，或把 `EdgeAggregator` 字段提前放进
  C5a membership state。

## 6. Tests Required

- `TestTopologyState`：固定 session fixture 覆盖 whitelist、重复 identity、descriptor drift、
  component session 推进、resync barrier、graceful leave、loss/rejoin、session fence、
  candidate failure atomicity、三图独立 edge、link/route epoch、移除后旧 epoch 重放、
  pending endpoint、资源上限、独立图改路、TTL/freshness 和确定性排序。
- `TestTopologyConversions`：snapshot round-trip、bounded collection preflight、未知 enum、
  长度/epoch/route 引用拒绝，并断言 decoded snapshot 与 canonical core snapshot 完全相等。
- 现有 C4 `TestRoutedMapConversions` / closed-loop tests：继续断言 C4 MapUpdate v2 的 payload、
  descriptor、digest、revision 和 update hash 在 topology 边界之外保持不变。
- 容器可用后执行：
  `colcon build --symlink-install --packages-up-to swarm_data_interfaces swarm_data_plane`；
  `colcon test --packages-select swarm_data_plane`；
  `colcon test-result --verbose`。构建身份和结果写入 C5a validation 记录。

## 7. Wrong vs Correct

Wrong：

```cpp
// 直接修改 committed view，并把消息中的未知 enum 当作合法状态。
snapshot.members.push_back(decoded_member);
member.state = static_cast<MembershipState>(ros.state);
```

Correct：

```cpp
// 先检查 bounded counts，再构造 candidate；完整 canonical 校验通过后原子提交。
auto decoded = Ros::decode_topology_snapshot(message, limits);
if (decoded.success) {
    auto result = topology_state.replace_topology(candidate);
    // 失败时 committed snapshot 与 route/source state 均不变。
}
```

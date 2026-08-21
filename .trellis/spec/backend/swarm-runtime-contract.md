# C5c Explorer / Relay Runtime Contract

## 1. Scope / Trigger

本规范适用于 C5c 的 `swarm_data_plane` runtime authority、纯 Relay、topology-driven map source、
`swarm_controller/SingleDroneExplorer` 运行期门控和 RViz runtime marker。它位于 C5a topology、
C5b role/capability 与 C4 routed map envelope 之间；不定义 C5d EdgeAggregator 或 C6 task allocator。

## 2. Signatures

ROS-free 核心入口：

```cpp
PureRelayResult PureRelay::forward(
    const RuntimeSnapshotCache &, const RoutedMapUpdate &, uint64_t forwarding_ns);
RuntimeAuthorityResult RuntimeAuthority::observe_evidence(
    CapabilityEvidence, uint64_t receive_time_ns);
RuntimeAuthorityResult RuntimeAuthority::tick(uint64_t now_ns);
RoleResult RuntimeAuthority::begin_role_transition(std::string, RoleCandidate);
RoleResult RuntimeAuthority::acknowledge_role_transition(
    const std::string &, const VehicleIdentity &, RoleTransitionAckKind);
```

运行期 ROS 端点为 `swarm_runtime_authority`、`map_update_relay`、
`topology_map_route_source`、`swarm_runtime_visualizer` 和 Explorer 的
`ApplyRoleTransition.action` adapter。

## 3. Contracts

- `PureRelay` 只验证本机 RelayService、精确 committed route、route epoch、hop、TTL、freshness 和 identity；只推进 hop/forwarding time，保留 source/session/revision/hash/payload/correlation。
- authority 以 steady receipt time 判断 Relay heartbeat；Relay 失联后原子推进 topology/role/route epoch，备用 Relay 不会因单纯 evidence 恢复自动重新获权。
- topology-driven source 只从 committed `RouteDescriptor` 选择第一跳；route binding 是部署边界信息，不进入 C4 content identity。
- Explorer 在 Active 且健康门通过时接收新职责；Quiescing、Draining、HandoffReady 或健康门失败时停止接新任务并进入 RuntimeHold。
- role transition 的 prepare/quiesce/handoff/commit/rollback 通过领域 transition/ack 映射；Relay 不代理 action 内部 topic/service。
- runtime marker 必须来自真实 topology/role/transition/diagnostic/odometry 状态；不得用静态 fixture 代替故障切换或角色状态。
- C5c N=4 只包含 2 Explorer + 2 Relay；中央 receiver 是 Relay 本地 endpoint，不计入 fleet member。C5d 才增加 EdgeAggregator 并进行 N=5 总验收。

## 4. Validation & Error Matrix

| 条件 | 必须结果 |
| --- | --- |
| 旧 route/topology epoch、错误 hop、TTL 耗尽、过期 freshness、错误本机身份 | Relay fail closed，payload 和 origin freshness 不变 |
| RelayService 未授权、role Draining、service health degraded | 拒绝新转发；不隐式改变 primary role |
| Relay heartbeat receipt timeout | authority 提交新 route/role epoch；旧 route 消息被拒绝 |
| Relay 恢复但未重新提交 candidate | 只更新 evidence，不自动恢复旧 assignment/route |
| resync keyframe 缺少或错配 correlation ID | receiver 保持 barrier，不提交地图 |
| Explorer 非 Active/健康门失败/Quiescing | 新 task/renewal 拒绝并发布 RuntimeHold |
| transition ack 顺序错误、目标 session 不匹配、超时 | transition rollback，committed snapshot 不变 |
| static RViz marker 或错误包路径 | 人工验收失败；launch 必须从 `swarm_controller` 安装目录加载配置 |

## 5. Good / Base / Bad Cases

- Good：Relay 0 fault service 只暂停转发与 heartbeat；authority 自动选择 Relay 1，两个 source 通过新 epoch 和 correlated keyframe 收敛。
- Base：C4 `MapUpdate` v2 在 relay 中保持 opaque，C5c 不解析或重算内容 identity。
- Good：Explorer 完成 Quiesced/HandoffReady 后提交 Draining role，期间不接受新职责。
- Bad：在测试脚本中直接伪造切换后的 committed topology/role snapshot。
- Bad：恢复 heartbeat 后直接把 Relay 0 标回 Active，绕过 topology/role candidate。
- Bad：用 `self` 实例字段保存跨 `launch_testing` 测试方法的旧 route 状态，导致后续方法读取空值。
- Bad：resync service 返回 correlation 后仍发布空 correlation keyframe。

## 6. Tests Required

- `TestPureRelay` / `TestRoleRuntime`：验证 payload identity、route/TTL/freshness、角色门控和 COW-free bounded state。
- `TestRuntimeAuthority`：验证 heartbeat timeout、原子 failover、旧 evidence、恢复不自动重获权限。
- `test_c5c_explorer_relay_runtime.py`：验证 N=4 双来源、Relay 0 fault、Relay 1 takeover、旧 route rejection、correlated resync、Explorer transition 和 clean exit。
- RViz runtime smoke：验证 `/swarm/runtime/markers` 有 transient-local publisher/subscriber，并包含成员、角色、服务、active/standby route、receiver 和 epoch marker。
- resync regression：启动时错过首个 keyframe，后续 delta 必须经 correlation keyframe 恢复并提交 accepted map。

## 7. Wrong vs Correct

Wrong：

```cpp
// Relay 解析地图并自行改写内容身份，或在 fault test 中直接伪造新快照。
message.map_update.content_hash = recompute_hash(message.map_update.payload);
publish_fake_failover_snapshot();
```

Correct：

```cpp
// authority 提交新 route epoch；Relay 只改变允许的 forwarding metadata。
auto result = relay.forward(snapshot_cache, routed, forwarding_ns);
// resync keyframe 携带 service 返回的 correlation_id，receiver 才能清除 barrier。
keyframe.correlation_id = ack.correlation_id;
```

## 8. C5d EdgeAggregator Runtime

- `EdgeAggregator` 保持每个 contributor 一份有界的 `MapUpdateIngress` 状态；它不保留逐消息历史，也不把 Relay 的转发元数据纳入内容身份。
- contributor 输入先应用到独立候选状态，再构建 aggregate cells、manifest 和 aggregate producer update。cell limit、geometry、manifest 或 producer 校验失败时，正式 contributor、aggregate revision、last update 和 producer baseline 均保持不变；候选仅在完整 aggregate 验证成功后交换提交。
- aggregate manifest 的 contributor 条目按 `vehicle_id` 严格排序；同 voxel 冲突使用 `Occupied` 优先。Remove 保留 `active=false` 条目，避免来源消失被误解为未曾加入。
- route epoch、sequence gap、hash conflict 或 local-state failure 使对应 contributor 进入 resync barrier，但保留最后合法地图作为只读快照。EdgeAggregator ROS 节点按输入索引向配置的来源 resync service 请求 correlation；ack 必须同时匹配 producer/source，之后只有该 correlation 的 keyframe 才能恢复 contributor。
- C5d 在 C4 ingress 之上对已提交 contributor 的 route-epoch 前进建立额外 barrier；即使输入本身是 keyframe，也必须先取得 EdgeAggregator 自己请求的 correlation，不能把上游或 Relay 携带的 correlation 当作本聚合器的恢复授权。
- N=5 launch 拓扑是 2 Explorer -> 2 Relay -> EdgeAggregator -> aggregate receiver。Relay 仍为纯转发；故障切换由 RuntimeAuthority 提交新 topology/role/route epoch，旧 route 消息在备用 Relay 端 fail closed。
- C5d 运行期门控只使用 `RuntimeSnapshotCache::service_admission(ServiceKind::Aggregation)`；门控失败拒绝新输入且不改变已提交 aggregate。
- runtime visualizer 的 aggregate Marker 必须显示 aggregate revision、active/total contributor 数、degraded 和 resync；contributor 明细逐项显示 active/inactive。resync 提示由可靠 topology map-route epoch 前进触发，并保留最短可见窗口，不能只依赖瞬态 volatile diagnostic。

# C5c Explorer 与纯 Relay 运行期设计

## 1. Design Goals

- 在 C5a topology 与 C5b role/capability 契约上建立真实运行期，而不是复制一套私有身份、角色或 epoch 状态。
- 复用现有 `SingleDroneExplorer` 探索算法，仅增加运行期授权、quiesce/Hold 和迁移确认。
- 把 C4 routed map update 通过纯 Relay 转发至中央接收端，保持 payload 与 origin identity 不变。
- 使用 N=4（2 Explorer + 2 Relay）验证真实 Relay 健康检测、自动改路和旧路径拒绝。
- 为 C5d 增加 EdgeAggregator 保留稳定入口，但 C5c 不解释或聚合地图内容。

## 2. Non-Goals

- 不实现新的探索策略、Region allocator、task lease 协议或协调器 HA。
- 不修改 C3/C4 MapUpdate v2、Merkle/chunk、canonical hash 或 routed wire 格式。
- 不让 Relay 解析地图、重算内容 hash、生成 aggregate update 或代理 ROS action 内部协议。
- 不在 C5c 宣称完成最终 2 Explorer + 2 Relay + 1 EdgeAggregator 的 N=5 验收。

## 3. Runtime Shape

```text
                         RoleSnapshot / TopologySnapshot
                     + CapabilityEvidence / Transition
                                      |
                                      v
                           SwarmRuntimeAuthority
                             /              \
                            /                \
              Explorer 0 + map source    Explorer 1 + map source
                       |                          |
                  active route               active route
                       |                          |
                  Relay 0  <---- failover ----> Relay 1
                            \                  /
                             \                /
                         central receiver(s)
```

- `SwarmRuntimeAuthority` 是 C5c 单 Active authority。它维护 C5a `TopologyState` 与 C5b
  `RoleState`，发布 committed snapshot，并根据 Relay capability evidence 的接收时间判断健康。
- map source、Relay、Explorer 和可视化只消费 authority 的 committed snapshot；不得直接修改
  topology/role epoch。
- N=4 fixture 使用两个 source identity 和两个独立中央 receiver namespace，避免把 C4 单 source
  ingress 状态误当成多 source 全局状态。中央 receiver 是 Relay 节点的本地 ROS endpoint，不是
  fleet member，也不进入 `TopologySnapshot` 或 N 计数。

## 4. Package Boundaries

### 4.1 `swarm_data_plane` ROS-free core

新增具体类，不为单一实现提前建立接口：

- `RoleRuntimePolicy`
  - 从 `RoleSnapshot`、本机 `CapabilityEvidence` 和活动 `RoleTransition` 计算主角色与服务能否
    接受新工作。
  - `RoleState::can_accept_new_work()` 与运行期消费者共享该策略，避免 C5c 复制 C5b 判断。
  - 输出显式原因：NoAssignment、WrongRole、Draining、HealthBlocked、CapabilityMissing、
    Quiescing、ServiceInactive 等。
- `RuntimeSnapshotCache`
  - 只接受 canonical、单调推进且身份匹配的 topology/role/evidence/transition。
  - stale/conflicting update 原子拒绝，保留最后 committed view。
  - 不成为第二 authority；仅保存运行期只读视图。
- `PureRelay`
  - 输入本机 identity、route binding、runtime snapshot 与 `RoutedMapUpdate`。
  - 先验证本机 RelayService 授权和 exact committed route，再调用现有
    `forward_routed_map_update()`。
  - 仅允许推进 `hop_count` 与 `accumulated_forwarding_ns`；producer、origin、sequence、
    correlation、priority、payload bytes/hash 和 `MapUpdate` shared payload 必须保持相等。
  - 对 TTL、freshness、旧 topology/route epoch、错误 hop、错误本机 route position 和服务降级
    fail closed。
- `RuntimeAuthority`
  - 组合现有 `TopologyState` 与 `RoleState`，不复制其校验规则。
  - 使用固定 N=4 配置建立初始成员、角色与两条 Relay 候选 route。
  - 接收单调 capability evidence，并使用 receipt-time timeout 判断 Relay 丢失。
  - Relay 0 超时后提交不包含失效路径的新 topology candidate，再提交与新 topology 一致的
    role candidate；失败时两者各自保持原 committed snapshot，并输出明确诊断。
  - 不负责探索任务分配或地图聚合。

### 4.2 `swarm_data_plane` ROS adapters

- `swarm_runtime_authority` 薄节点
  - 发布 transient-local `TopologySnapshot`、`RoleSnapshot` 与当前
    `RoleTransitionDescriptor`。
  - 订阅 `CapabilityEvidence`，使用 steady time 维护 receipt timeout。
  - 作为 role transition action 的发起/汇聚端；领域状态仍由 `RuntimeAuthority/RoleState`
    决定。
- `map_update_relay` 薄节点
  - 订阅 route-specific 输入 topic、topology/role/transition/evidence。
  - 调用 `PureRelay` 后发布到 route-specific 下一跳 topic。
  - 周期发布本机 `CapabilityEvidence`；只有真实运行且 service health 为 Healthy 时才持续推进
    evidence revision。
  - 测试模式可通过明确启用的 fault-injection service 暂停转发和健康上报；该 service 不进入
    ROS-free 生产语义。
- `topology_map_route_source` 薄节点
  - 复用 C4 source/update 构建逻辑，按 committed route binding 选择第一跳 publisher。
  - route epoch、TTL 和 validity budget 来自 exact `RouteDescriptor`，不能由测试脚本直接改写。
  - C4 payload/content identity 不重算、不改写。
- `swarm_runtime_visualizer` 薄节点
  - 消费真实 topology/role/transition、Relay diagnostics 和 Explorer odometry/diagnostics。
  - 发布统一 MarkerArray：节点、主角色、服务、活动/备用 route、topology/role/route epoch、
    Active/Draining/Quiescing/HandoffReady 和故障状态。

### 4.3 `swarm_controller`

- `SingleDroneExplorerNode` 订阅本机相关的 role snapshot、capability evidence 和 transition。
- `onExplorationTask()` 在写入现有 `TaskLeaseTracker` 前检查 `RoleRuntimePolicy`；门关闭时拒绝新
  task/renewal，并记录明确诊断。
- `onControlTimer()` 在门关闭或 quiescing 时不推进 Explorer 候选状态，发布安全 Hold；已存在的
  地图、规划和 peer 逻辑保持不变。
- 只有观察到 Hold 且本机速度满足现有停止阈值后，role transition adapter 才确认 Quiesced；
  运行期状态稳定后再确认 HandoffReady。
- rollback 清除本机 quiesce latch，但仍需由 committed role snapshot 与最新健康状态决定是否
  恢复工作。

## 5. Route Binding Without Wire Changes

C4 `RoutedMapUpdate` 只有 route epoch/hop/TTL，没有 route ID。C5c 不修改 wire，而是在 ROS
部署边界使用 route-specific topics 和本地 `route_id` 配置。由于 C5a 的 route endpoint 必须是
fleet member，C5c 的 `G_map` route 只描述 `Explorer -> Relay`，Relay 的转发结果再通过同一
Relay 进程的本地 topic 送到中央 receiver；Relay 的 `G_control` route 仍使用至少两跳路径，
因此满足 C5b 对 RelayService 的 intermediate-route 前置：

1. source 根据 committed snapshot 找到 configured `route_id`，验证 source/target 与本机身份；map
   route 的 target 是承载本地 receiver endpoint 的 Relay。
2. source 将 update 发布到该 route 的第一跳 topic，并写入 descriptor 的 route epoch/TTL。
3. Relay 从自己的 route-specific input 得到 `route_id`，验证自己是该单跳 map route 的终点，或
   位于 control route 的预期 intermediate hop。
4. Relay 调用 C4 forwarding 后发布到本地中央 receiver topic；中央 receiver 仍只依赖 C4 envelope
   验证和 ingress 语义。

route binding 是部署信息，不进入 MapUpdate 内容身份。若 route 被移除或 epoch 改变，旧 topic
上到达的消息仍会因 committed route 不匹配而拒绝。

## 6. Role Transition Adapter

新增必要的 `ApplyRoleTransition.action`，只用于端点 API：

- Goal 携带 committed/Prepared `RoleTransitionDescriptor` 和目标本机 identity。
- Feedback 报告 Preparing、Holding、Quiesced、HandoffReady 或 Rejected 运行期阶段。
- Result 返回领域 ack 与诊断；authority 将 ack 交给 `RoleState::acknowledge_transition()`。
- action cancel 映射为 rollback 请求；是否允许 rollback 仍由 `RoleState` 决定。
- Relay 不转发 action 的 goal/service/topic。跨稀疏链路时只允许后续 adapter 转换出的领域
  transition/ack envelope；C5c fixture 中 action endpoint 与 authority 直接位于 ROS graph。

## 7. Automatic Failover

1. Relay 0 与 Relay 1 周期发布 capability evidence，revision 单调递增。
2. authority 以 steady receipt time 维护 heartbeat deadline，不使用消息 wall-clock 判断失联。
3. fault injection 暂停 Relay 0 转发和 evidence；authority 超时后将其服务健康视为不可接受新工作。
4. authority 构造下一 topology candidate，移除 Relay 0 active route，启用 Relay 1 route，并推进
   topology/route epoch。
5. source 观察 committed snapshot 后切换第一跳；Relay 1 只接受新 route epoch。
6. receiver 拒绝延迟到达的旧 route update；该拒绝不得刷新 origin freshness。
7. Relay 0 恢复只形成健康证据，不自动恢复旧角色/route；重新启用必须经过新的 candidate/epoch。

## 8. Concurrency And Bounded State

- snapshot callback 与 map forwarding callback 使用独立 callback group；核心快照以不可变值交换，
  不在持锁状态执行 MapUpdate 编解码或 publish。
- Relay 不保存完整历史地图，只保存 bounded snapshot、诊断计数和当前 in-flight 数量。
- authority 的 receipt records、transition history、member/route 数量继续受 C5a/C5b limits 约束。
- quiesce 先关闭新工作入口，再等待当前 callback/运动进入安全点，避免 ack 后仍产生新输出。

## 9. Visualization And Manual Acceptance

- 新增 XML 主入口 `c5c_explorer_relay.launch.xml`，参数包含 `show_rviz`、故障注入开关和超时。
- RViz 首屏同时展示 2 Explorer、2 Relay、中央 receiver、活动/备用路径和 epoch 标签。
- 人工操作暂停 Relay 0 后，应依次观察到：Relay 0 degraded/lost、route epoch 推进、活动路径切换
  Relay 1、旧路径拒绝计数增加、两个 Explorer 的 source identity 保持不变。
- 角色迁移演示应展示 Active -> Quiescing -> HandoffReady -> Committed/RolledBack，Explorer 在
  Quiescing 期间保持 Hold。

## 10. Compatibility And Rollback

- 保持 C4 wire 与现有 source/receiver core 不变；旧 C4 launch/test 继续作为回归。
- 新 topology-driven source/Relay 使用独立 executable/topic，避免在 Gate 通过前替换既有 C4
  可执行路径。
- 若 C5c Gate 失败，回滚点是停用 N=4 launch 与新 runtime executables；C5a/C5b core 和 C4
  data plane 不需要回退。
- C5d 只能消费 C5c committed source/Relay runtime contract，不得把聚合逻辑补进 `PureRelay`。

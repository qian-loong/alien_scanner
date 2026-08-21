# C5c Explorer 与纯 Relay 运行期

## Goal

在 C5b role/capability contract 之上实现真实 Explorer 与纯 RelayService 运行期链路，验证 Explorer -> Relay -> 中央接收端、route failover、role transition adapter 和角色可视化；不实现 EdgeAggregator 聚合或 C6 任务分配。

## Background

- C5a 已提供稳定 vehicle/source identity、vehicle session、成员生命周期、三张逻辑图以及带 `route_epoch`、hop、TTL 的路由契约。
- C5b 已提供一个主角色加可组合服务能力的 `RoleSnapshot` 契约，能够表达专用 Explorer、专用 Relay 和 `Explorer + RelayService`。
- C4 已提供 `RoutedMapUpdate` 数据面及 source/session/revision/content identity 语义；C5c 只能消费这些稳定契约，不重新定义身份、角色或地图内容格式。
- 当前已有 `swarm_controller/SingleDroneExplorer` 和 C4 source/receiver，但 Relay 仍主要是测试 fixture，尚未形成受角色与健康状态门控的真实运行期纵向链。
- Explorer 运行期采用现有 `SingleDroneExplorer` 的真实移动/探索行为；C5c 只增加角色、服务能力和健康门控，不重写探索算法。
- C5c 独立集成验收采用 `N=4`：2 个 Explorer + 2 个 Relay；中央接收端不计入无人机数量。C5d 增加 1 个 EdgeAggregator 后再进行最终 `N=5` 总验收。
- C5c 使用单 Active 运行期 authority 维护已提交 topology/role snapshot；Relay 通过受限健康/心跳证据报告可用性，故障注入只制造 Relay 失联，authority 自动推进 topology/route epoch 并选择备用路径。

## Requirements

- **C5C-R1 Explorer runtime**：Explorer 运行期必须消费 C5b committed `RoleSnapshot`、effective capability 和健康状态。只有 Active Explorer 可以接受新探索职责；Quiescing、Draining、HandoffReady 或健康门失败时必须停止接受新职责并进入安全 Hold/收敛状态。
- **C5C-R2 pure RelayService**：实现生产级 ROS-free Relay 核心和薄 ROS 节点。Relay 只转发 C4 `RoutedMapUpdate`，保留原 source/session/revision/hash/payload，仅推进 route hop/TTL 等转发元数据；不得解析、融合或改写地图内容，也不得代理 ROS action 的内部 topic/service。
- **C5C-R3 real process chain**：由真实进程跑通 `Explorer -> Relay -> 中央接收端`，验证原始 source identity、session、route epoch、hop 和 TTL 在跨进程转发后仍符合 C5a/C4 契约。
- **C5C-R4 route failover**：至少提供 Relay 0 与 Relay 1 两条候选路径。Relay 0 失效时推进 topology/route epoch 并切换 Relay 1；旧 route 消息必须被拒绝，且不得刷新 origin freshness。
- **C5C-R5 role transition adapter**：把 C5b 的 prepare、quiesce、handoff、commit、rollback 领域状态映射为运行期事件和确认，不新增私有 role/capability 字段。
- **C5C-R6 service isolation**：支持专用 Relay 与 `Explorer + RelayService` 使用同一契约运行。RelayService 的启动、降级或拒绝不得隐式改变主角色，也不得授予 Explorer 任务资格。
- **C5C-R7 observability and RViz**：可视化必须来自真实 runtime，显示 Explorer、Relay、主角色/服务、route、role epoch 以及 Active、Draining、Quiescing、HandoffReady 和故障切换状态，不能只显示静态 marker fixture。
- **C5C-R8 architecture and tests**：继续遵守 ROS-free 算法库加薄 `rclcpp` 节点分层；核心角色门控、Relay 转发、TTL/epoch 拒绝和故障切换逻辑必须有确定性 gtest，ROS 接线使用集成测试，外观使用 RViz 人工验收。

## Acceptance Criteria

- [ ] Active Explorer 能工作；角色进入 Quiescing/Draining、完成 handoff 或健康门失败后不再接受新职责，并进入预期 Hold/收敛状态。
- [ ] 专用 Relay 和 `Explorer + RelayService` 均可转发同一 C4 map update，接收端观测到的 source/session/revision/hash/payload 与源端一致。
- [ ] Relay 只修改允许的 route metadata；TTL 耗尽、hop 越界、旧 route epoch 和无效 session 被确定性拒绝。
- [ ] 真实进程链 `Explorer -> Relay -> 中央接收端` 可重复运行，并覆盖断链、恢复和 resync。
- [ ] Relay 0 失效后切换至 Relay 1，旧路径消息不能恢复 origin freshness，也不会产生双路径重复接受。
- [ ] Relay 0 失效由健康/心跳超时触发 authority 自动改路；测试脚本只负责注入失联，不直接伪造切换后的 committed snapshot。
- [ ] prepare/quiesce/handoff/commit/rollback 可从 C5b 契约映射到运行期状态与 ack，失败路径可 rollback。
- [ ] RViz 人工验收能观察角色、服务、route/role epoch、迁移状态和 Relay 故障切换，数据来源为真实运行期节点。
- [ ] C5c 相关单元测试、ROS 集成测试和仓库既有回归测试通过，验证记录同步到任务目录。
- [ ] N=4 运行期 fixture 包含 2 个 Explorer、2 个 Relay，至少覆盖多来源地图更新、单 Relay 故障和备用路由切换；不提前声称完成 C5d 的 N=5 聚合验收。

## Out Of Scope

- EdgeAggregator 的地图聚合、contributor manifest 和 N=5 完整聚合链；这些属于 C5d。
- C6 Region/task allocator、lease 和新的探索任务分配协议。
- 协调器 Active/Warm Standby 高可用。
- 修改 C3/C4 wire format、chunk/Merkle 内容身份或 canonical map 语义。
- 把纯 Relay 扩展为理解地图语义的“智能 Relay”。

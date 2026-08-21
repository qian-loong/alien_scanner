# C5c Explorer 与纯 Relay 运行期实施计划

## 1. Preconditions

- [x] 复读本任务 `prd.md`、`design.md`、C5a topology spec、C5b role spec 和 C4 data-plane spec。
- [x] 确认当前分支为 `phase/5c-explorer-relay-runtime`，保留所有无关未跟踪文件。
- [x] 运行 C5b 后的目标包基线测试，记录实现前结果；Docker 不可用时先记录阻塞，不伪造结果。

## 2. Shared Runtime Policy

- [x] 在 `swarm_data_plane` 增加 ROS-free `RoleRuntimePolicy` 与结果/原因类型。
- [x] 重构 `RoleState::can_accept_new_work()` 和 `can_service_accept_new_work()` 复用同一策略，保持 C5b
  行为不变。
- [x] 增加 `RuntimeSnapshotCache`，覆盖 canonical、identity、epoch/revision 单调性、重复与冲突。
- [x] gtest 覆盖 Explorer、专用 Relay、Explorer + RelayService、Draining、health/capability 降级、
  Quiesced ack、rollback 和 stale snapshot。
- [x] 运行既有 `TestRoleState`，确认共享策略重构无回归。

## 3. Pure Relay Core

- [x] 增加 ROS-free `PureRelay` 与 route binding 配置，不建立单实现接口。
- [x] 精确校验 committed route、local identity、route position、epoch、hop、TTL、freshness 和
  RelayService permission。
- [x] 调用现有 `forward_routed_map_update()`，断言只改变 hop/forwarding elapsed。
- [x] 增加 bounded diagnostics counters，不保存地图历史或 payload 副本。
- [x] gtest 覆盖 happy path、专用 Relay、Explorer + RelayService、旧 route、错误 hop、TTL、过期、
  health degraded、quiesce、payload identity 保持和恢复不自动重获权限。

## 4. Runtime Authority Core

- [x] 增加 ROS-free `RuntimeAuthority`，组合 `TopologyState` 与 `RoleState`。
- [x] 使用固定配置建立 N=4 member/capability/role/route；中央 receiver 只作为 Relay 本地 endpoint，
  不登记为 fleet member，保证 canonical 与 limits 校验。
- [x] 接收 Relay capability evidence 与 steady receipt time，执行 bounded heartbeat timeout。
- [x] 实现 Relay 0 timeout -> Relay 1 route candidate -> committed epoch 推进；失败保持旧 snapshot。
- [x] 实现恢复仅更新 evidence，不绕过 role/topology candidate 自动恢复旧 route。
- [x] gtest 覆盖初始提交、超时边界、单次 failover、重复 tick、旧 evidence、冲突 evidence、
  candidate 失败原子性和恢复语义。

## 5. ROS Interfaces And Adapters

- [x] 在 `swarm_data_interfaces` 增加 bounded `ApplyRoleTransition.action`，更新 CMake/package export。
- [x] 实现 `swarm_runtime_authority` 薄节点：snapshot publishers、evidence subscriber、steady timer、
  transition action clients/ack 汇聚和 diagnostics。
- [x] 实现 `map_update_relay` 薄节点：route-specific input/output、snapshot cache、PureRelay 调用、
  capability evidence heartbeat 和显式启用的 fault injection service。
- [x] 实现 topology-driven map source adapter，复用 C4 update 构建/编码，不改内容身份。
- [x] 所有 ROS conversion 在 core 调用前完成 preflight；node 不包含角色策略或 route policy。

## 6. Explorer Runtime Integration

- [x] 为 `swarm_controller` 增加对 data-plane role runtime policy 的构建依赖，保持
  `swarm_exploration` 算法库无 ROS 依赖。
- [x] `SingleDroneExplorerNode` 缓存本机 snapshot/evidence/transition，并在 task callback 前门控。
- [x] 门关闭时不向 `TaskLeaseTracker` 写入新 task/renewal，诊断明确区分角色、健康和迁移原因。
- [x] control timer 在 Draining/Quiescing/HandoffReady 时发布 Hold 且不提交 Explorer candidate。
- [x] 实现停止阈值确认、Quiesced/HandoffReady ack 和 rollback latch 清理。
- [x] 扩展单元/集成测试，确保 Active 路径与既有探索行为不回归。

## 7. N=4 Launch And Visualization

- [x] 新增 `c5c_explorer_relay.launch.xml`：2 Explorer、2 route source、2 Relay、2 receiver、单
  Active authority、visualizer 和可选 RViz。
- [x] 使用 namespace/remap 隔离两条 source/receiver 状态；配置 Relay 0/1 活动与备用 route。
- [x] 新增 `swarm_runtime_visualizer` 和 RViz config，显示真实节点、角色/服务、route 与 epoch。
- [x] fault injection 只暂停 Relay 0 转发/heartbeat；authority 自动推进 snapshot。
- [x] RViz 资源/launch 静态测试确认 topic、display 和 fixed frame 配置正确。

## 8. Automated Validation

- [x] 构建：`colcon build --symlink-install --packages-up-to swarm_controller swarm_data_plane`。
- [x] 单元测试：`colcon test --packages-select swarm_data_plane swarm_controller`。
- [x] 接口/转换测试覆盖 action bounded fields 与未知/非法输入。
- [x] launch test：N=4 happy path，两个 source identity 到达各自 receiver。
- [x] launch test：Relay 0 heartbeat timeout 后自动切换 Relay 1，route/topology epoch 推进一次。
- [x] launch test：旧 route 延迟 update 被拒绝且不刷新 origin freshness；新 route resync/keyframe
  收敛。
- [x] launch test：Explorer Active 接受任务，Quiescing/Draining 拒绝新任务并 Hold，rollback/commit
  行为正确。
- [x] launch test：专用 Relay 与 Explorer + RelayService 使用同一 core 契约。
- [x] 全量结果：`colcon test-result --verbose` 无本任务引入失败。

## 9. Manual Acceptance

- [x] 在 Docker/VcXsrv 环境启动 N=4 RViz 界面，确认地图、节点、角色和路径来自真实 runtime。
- [x] 注入 Relay 0 失联，观察 degraded/lost、epoch 推进、Relay 1 接管和旧路径拒绝。
- [x] 触发 Explorer role transition，观察 Active -> Quiescing -> HandoffReady -> commit/rollback，
  并确认无人机进入 Hold。
- [x] 保存截图、运行命令、关键 topic/diagnostic 证据和人工结论到本任务 `validation.md`。

## 10. Quality Gates And Rollback Points

- [x] 运行 `trellis-check`，修复 spec、lint、构建、测试和跨层数据流问题。
- [x] 对 Qt 无关，本任务不运行 Qt 专用审核；C++ review 聚焦线程、生命周期、bounded state 和
  fail-closed 路径。
- [x] Gate A：shared role policy 重构通过全部 C5b 回归；失败则先回滚该重构，不继续 Relay。
- [x] Gate B：PureRelay 单测证明 payload identity 不变；失败不得进入 ROS/launch 集成。
- [x] Gate C：N=4 自动 failover 与旧 route 拒绝通过；失败保持新 executables 非默认。
- [x] Gate D：RViz 人工验收通过后才同步完成状态；C5d 不在 C5c Gate 前消费未稳定行为。

## 11. Documentation And Handoff

- [x] 更新 backend runtime contract，记录 Explorer/Relay/authority 边界、failover 和错误矩阵。
- [x] 更新父任务 C5 进度，只标记 C5c 已实际通过的能力，不提前标记 C5d/N=5。
- [x] 生成 `validation.md`，列出构建身份、测试数量、故障注入步骤、已知例外和 C5d 前置。
- [x] 保持所有 C5c 改动未提交，直到用户明确要求提交。

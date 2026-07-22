# Research: 当前 Phase 3 基线与延期契约

- Query: 下一阶段架构必须继承哪些已验证契约，哪些工作仍未完成？
- Scope: internal
- Date: 2026-07-21

## Findings

### 当前完成边界

- Phase 3 文档明确只认定 3-1 至 3-8 完成，3-9 在重构后继续，3-10 重定：`docs/phases/phase-03-swarm.md:99`、`docs/phases/phase-03-swarm.md:103`。
- 当前收口不是 Phase 3 完成，保留多 Region 和任务生命周期要求：`docs/phases/phase-03-swarm.md:112`。
- Release 收口已有 `30/30` CTest 通过，作为重构前自动验证基线：`docs/phases/phase-03-swarm.md:2667`。

### 必须保留的行为 oracle

- 中央式三机收口保留 merger revision/原子提交/阶段诊断、Detector oracle、task epoch/revision/lease、allocator 后台管线和 KnownFreePathChecker 安全边界：`docs/phases/phase-03-swarm.md:2584`。
- 这些是行为与证据基线，不要求新架构保留原节点部署。
- 3-9 后续仍必须证明至少两个有效 Region、eligible edge/matching、唯一 Assigned owner、lease/撤销/失效/重分配和真实模式切换：`docs/xenomorph-scanner-plan.md:234`、`docs/xenomorph-scanner-plan.md:238`。

### 地图与性能事实

- 三路 source 当前发布 `fullMapToMsg()` 完整快照；离线只能从相邻 snapshot 推导变化，不能恢复逐 scan 顺序、session epoch 或路由事件：`docs/phases/phase-03-swarm.md:2662`。
- 固定 Release 诊断显示完整 source decode + normalize 约占 merger cycle p50/p90 的 60%；拓扑变化本身不会消除这部分处理：`docs/decisions/phase-03-topology-architecture-notes.md:168`。
- 当前仍未实现 Relay、EdgeAggregator、delta 或稀疏 peer 拓扑：`docs/decisions/phase-03-topology-architecture-notes.md:176`。
- 无 transport loss 的 source-level replay 已冻结，用于未来 snapshot/delta、边缘聚合和拓扑离线比较：`docs/phases/phase-03-swarm.md:2620`。

### 当前代码部署事实

- merger 由静态 `source_topics` 列表订阅各机地图：`ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:224`、`ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:315`。
- 多机 launch 从 `num_drones` 生成 `/drone_i/octomap` source 列表：`ws/src/swarm_controller/launch/multi_drone_exploration.launch.py:213`。
- explorer 通过配置的 `peer_namespaces` 建立 peer 订阅：`ws/src/swarm_controller/src/SingleDroneExplorerNode.cpp:84`、`ws/src/swarm_controller/src/SingleDroneExplorerNode.cpp:334`。
- 当前唯一自定义跨节点任务消息是 `ExplorationTask`，已含 allocator epoch、revision、task ID、mode、target 和 lease：`ws/src/swarm_controller_interfaces/msg/ExplorationTask.msg`。

## Implications

- 新架构可以替换节点和部署，但必须提供 Phase 3 oracle 到新契约的映射。
- 地图数据面与拓扑需要分开验收：减少链路边不等于减少 merger 的完整 source 处理。
- 3-9/3-10 不能因架构重构被删除，也不能仅以合成测试替代真实多机状态验收。

## Caveats / Not Found

- replay bag 位于 Docker named volume，不在 Git；规划阶段未重新打开或校验 bag。
- 当前项目没有真实无线链路、丢包模型或功耗模型，不能据现有三机运行推断真实大规模通信性能。


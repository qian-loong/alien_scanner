# Research: Degraded 地图的消费门控范围

- Query: mapper 进入 Degraded 后，是否使用一个全局健康布尔值控制所有地图、任务和运动消费者？
- Scope: internal
- Date: 2026-07-21

## Findings

- D-021 已定义 mapper 的 Healthy/Degraded/Unavailable，但 Degraded 只表示仍满足最低建图输入，不表示仍满足所有任务、角色或运动方向的能力要求。
- 感知契约要求能力缺失被显式保留；unknown 不能被伪造成 free，因此部分视场丢失可以自然表现为未更新/unknown 区域。
- Phase 3 的 KnownFreePathChecker 是本机严格安全契约，父设计也明确全局地图和协调状态不能替代本机 body/segment/path 检查。
- 当前 allocator 主要使用全局 freshness/health 布尔门；直接沿用会在多能力场景中产生两种问题：把可安全降级的节点全部停掉，或把仅满足最低建图能力的节点视为能执行所有任务。
- 角色与任务已采用 capability registration，因此消费者具备声明最低需求的逻辑位置。

## Implication

推荐保留分层、按消费者的健康门：mapper health 决定能否提交地图；shared-view health 决定贡献能否进入协调视图；role/task eligibility 根据当前有效 capability 判断；本机执行始终独立执行 freshness + body/segment/path known-free 检查。不得由 coordinator 用单一全局布尔值覆盖本机安全，也不得把 Degraded 自动解释为全部任务可用。

## Evidence

- `docs/decisions/perception-observation-interface.md:95`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/prd.md:92`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/prd.md:114`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:239`
- `ws/src/swarm_controller/src/GlobalTaskAllocatorNode.cpp:1410`

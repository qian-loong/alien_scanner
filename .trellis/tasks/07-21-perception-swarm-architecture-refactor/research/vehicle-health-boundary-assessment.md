# Research: Vehicle health 与能量输入边界

- Query: 本轮是否把电量、飞控 failsafe、执行器与计算资源健康纳入统一角色/任务契约？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前 Phase 3 没有 `sensor_msgs/msg/BatteryState` 或统一 VehicleHealth 输入；低电量只在 C7 验收中以“具备模拟能力时”出现。
- D-007 已要求 Relay/Aggregation 服务受独立资源预算和健康状态约束，D-016 允许动态 join/leave，D-027 又引入 external autopilot failsafe，但尚无统一对象连接这些状态。
- Explorer、Relay 和 EdgeAggregator 的健康需求不同：Explorer 依赖感知、定位和执行器，Relay 依赖链路与转发预算，EdgeAggregator 还依赖计算/内存预算。
- 只使用单一 healthy 布尔值会丢失故障来源；完全不纳入则无法在低电量或飞控 failsafe 时提前 Draining、撤销任务和迁移角色。
- 完整能量消耗预测、续航路径优化和充电调度会显著扩大本轮范围，但定义外部状态输入和确定性降级不需要实现这些算法。

## Implication

推荐定义 ROS-free `VehicleHealth`/`ResourceHealth` 逻辑契约，由标准 `sensor_msgs/msg/BatteryState`、autopilot 状态和本机 compute/link 诊断 adapter 提供。字段允许 `Unknown`，但不能把 Unknown 当 Healthy。首版使用固定 healthy fixture，并可注入 LowPower、Failsafe、ActuatorUnavailable、ComputeOverBudget 和 LinkDegraded；状态驱动角色 eligibility、Draining/handoff、任务撤销和本机 Hold。首版不实现能耗预测、充电规划或能源最优任务分配。


## Accepted Direction

父级采用分层 VehicleHealth/ResourceHealth 契约，而不是单一健康布尔值。健康字段允许 Unknown，但 Unknown 不等于 Healthy；具体角色、服务、任务和本机执行消费者各自声明必需字段。首版只做 adapter、诊断和确定性故障 fixture，不做能耗模型、充电规划或能源最优调度。
## Evidence

- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/prd.md:38`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/prd.md:116`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/implement.md:102`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:245`

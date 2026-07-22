# Research: 跨机时间、freshness 与 lease

- Query: 当前 Phase 3 时间语义能否直接用于跨 Relay、跨主机部署？
- Scope: internal
- Date: 2026-07-21

## Findings

- `TaskLeaseTracker` 使用协调器发布的 `issued_time + lease_duration`，并与接收无人机的 `now_ros` 比较；这要求两端 ROS time 可比较。
- 同一 tracker 另用接收端 steady clock watchdog 判断长期未收到续约。该值只在单进程/单主机本地比较，不能跨节点传输。
- `GlobalMapMergerNode` 同时保存 source header stamp、接收 ROS time、ROS clock epoch 和 steady receive time，并已处理 ROS time 回跳；这些机制主要针对单节点本地时钟变化。
- 新设计要求记录 origin、Relay receive/send 和 delivery 时间。若这些时间来自不同 clock domain，原始数值不能直接相减得到链路延迟。
- source epoch/revision、route epoch 和 role epoch 已承担逻辑顺序职责，因此没有必要用 wall/ROS stamp 建立跨 source 的总顺序。

## Implication

父级必须明确：哪些时间只用于事件追溯，哪些可用于本地 freshness/watchdog，哪些只有在时钟同步健康时才可计算跨机延迟。否则转发延迟、任务 lease 和诊断会在仿真中正确、在真实多机部署中静默失真。

## Evidence

- `ws/src/swarm_controller/src/TaskLeaseTracker.cpp:63`
- `ws/src/swarm_controller/src/TaskLeaseTracker.cpp:110`
- `ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:367`
- `ws/src/swarm_controller/src/GlobalMapMergerNode.cpp:406`

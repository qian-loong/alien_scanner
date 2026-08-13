# C4 通信数据面技术设计

## 1. 设计目标

C4 把 C3 的 source-local `MapUpdate` 送入一个可路由、可限流、可恢复的领域数据面，同时保持算法核心与 ROS 传输解耦。首版生产传输复用 ROS 2 RMW/DDS；确定性逻辑链路仅作为测试 seam，用于验证应用数据面语义，不替代或模拟真实 DDS、TCP 或无线物理层。

设计必须保持以下不变量：

1. 一个 vehicle session 的 map source 只有一条 authoritative revision chain。
2. Relay 只转发 envelope 和不可变 payload，不解释或修改地图语义。
3. aggregate update 与 contributor manifest 以同一 revision 原子发布。
4. 传输失败、认证失败和资源耗尽不能伪造 freshness，也不能覆盖本机安全状态。
5. 每一跳的诊断时间可追溯，但 origin stamp、source epoch/revision 和 payload hash 不被重写。

## 2. 分层架构

```text
C2/C3 mapper + MapUpdateProducer
          |
          | ROS-free PerceptionMapUpdate::MapUpdate
          v
  C4 envelope + application admission/resync
          |
          | ROS-free RoutedMessage + immutable payload
          v
  thin ROS endpoint adapter + standard QoS mapping
          |
          | production: rclcpp -> RMW -> DDS
          | test seam: deterministic delivery/fault fixture
          v
  Routed receiver -> C3 MapUpdateApplier
          |
          +--> application resync barrier / keyframe request
          +--> delivery/freshness diagnostics
```

端点 ROS adapter 负责生成消息与 ROS-free 值对象之间的校验转换及标准 QoS 映射；RMW/DDS 负责传输级可靠性、分片、重传、history、durability 和 discovery。应用级去重、epoch/revision、freshness 和恢复状态机不 include `rclcpp`。C4 不把 `GlobalMapMerger` 搬进核心，也不让 `swarm_controller` 的旧 topic 作为新协议。

## 3. 包与依赖边界

### 3.1 推荐包

- `perception_interfaces`：保留现有 `MapUpdate`、`RequestMapResync` 及感知消息，作为 C3 兼容边界。
- `swarm_data_interfaces`：新的 ROSIDL 包，承载 routed envelope、delivery acknowledgement、link diagnostic、resync intent/ack 和 aggregate contributor manifest。它依赖 `perception_interfaces`，但不依赖 `swarm_controller`。
- `swarm_data_plane`：C++17 `ament_cmake` 包，包含 ROS-free 数据面值对象、应用级有界 ledger/backlog、去重/有序接收、TTL budget、resync、trust adapter 和测试；ROS 节点只做端点转换、标准 QoS 映射与收发。确定性 link adapter 只编入测试/fixture target。

具体包名在方案评审前确认。若复用 `swarm_controller_interfaces`，必须保留其 controller/task 领域定位，不能让新通信 core 依赖 controller 节点。

### 3.2 依赖方向

```text
swarm_data_plane::core
  depends on -> perception_map_update::core

swarm_data_interfaces (ROSIDL)
  depends on -> perception_interfaces

swarm_data_plane::ros / thin nodes / launch tests
  depends on -> swarm_data_plane::core
             -> swarm_data_interfaces
             -> perception_map_update::ros
```

实现上可将 endpoint node 放在 `swarm_data_plane` 包内，但算法库 target 必须与 ROS target 分开。新包遵守 C++17、真实 target 归属公共头文件、CMake 导出 target 和 gtest 约定。

## 4. 领域对象与 envelope

候选 ROS-free 值对象如下；字段名和 ROSIDL 拆分在实现前通过接口评审固定，不允许用隐含 topic 顺序代替字段。

```text
RoutedMessage
  message_kind: MapUpdate | State | Control | Diagnostic | ResyncIntent | Ack
  message_id / producer_id / producer_session
  source_epoch / source_revision / sequence
  correlation_id (optional but bounded)
  priority / payload_bytes / payload_hash
  origin_clock_domain / origin_clock_session / origin_time
  validity_budget / route_epoch / hop_count / ttl
  auth_context / schema_version
  immutable_payload
```

地图 payload 是由 C3 canonical codec 产生的 immutable bytes 或经 adapter 验证的 `MapUpdate` 值对象。C4 不重新计算 occupancy hash，不改变 `base_revision`、`new_revision`、`map_epoch`、`content_hash` 或 `update_hash`。

`route_epoch`、`hop_count` 和 `ttl` 在 C4 只作为显式、可校验、可回放的 envelope 字段；C4 不决定 route authority、图成员关系或路径选择。C5 在不改变 payload 和队列契约的前提下固定这些字段的生产与更新规则。

### 4.1 顺序与去重

接收端 ledger 的主键为 producer/source/session/epoch 加 message identity；地图 apply 仍委托 C3 `MapUpdateApplier`，因此 C4 只负责在送入 C3 前过滤：

- 已提交同 identity + 同 hash：`IgnoredDuplicate`，不刷新 freshness。
- 同 identity + 不同 hash：`RejectedConflict`，保持状态并请求 resync。
- 旧 epoch、未来 revision、route epoch 过期或 validity budget 耗尽：拒绝并诊断。
- 新 epoch：先建立 resync barrier，keyframe admitted 后才允许 delta。

任何拒绝路径都不能改变最后合法 C3 map。

### 4.2 时间与 freshness

`origin_time` 只记录来源 clock domain/session；forward receive/send 使用各节点自己的 clock domain。接收端从本地 monotonic clock 开始 watchdog，并扣减剩余 validity budget。没有可证明时钟同步时，只报告每跳 duration 和 uncertainty，不计算跨机绝对 latency，不用 ROS/header stamp 排序。

## 5. QoS、优先级与背压

C4 不实现通用可靠传输队列。逻辑类别优先映射到独立 endpoint 和标准 DDS QoS/resource limit：

| 类别 | ROS API | 基线 QoS/恢复语义 |
|---|---|---|
| Resync intent/ack | short service | Reliable；请求由 bounded ledger 幂等处理，keyframe 不放入 response |
| Map update/keyframe | topic | Reliable + Volatile + bounded KeepLast；history 无法恢复时走应用 resync |
| State/health | topic | Reliable + KeepLast(1) + Lifespan/Deadline；过期不刷新 freshness |
| Summary/diagnostic | topic | 有界 KeepLast；可按部署选择 BestEffort，丢失不改变领域状态 |
| Control/role/task | C5/C6 定义 | C4 仅预留逻辑类别，不提前固定 action 生命周期 |

如果 keyframe 与 delta 需要不同资源预算，可使用独立 endpoint/QoS profile，而不是在应用层复制 DDS scheduler。应用层只保留 C3/C4 业务需要的 bounded resync ledger、source admission window 和处理 backlog；入站在分配前做 checked size arithmetic。任何 DDS history/resource limit 或应用 backlog 淘汰都必须转化为 gap/freshness/resync 结果，不能静默维持健康。

## 6. Resync 与两级聚合边界

```text
receiver detects gap/conflict/epoch change
  -> idempotent ResyncIntent(client_request_id)
  -> short service returns accepted + correlation_id
  -> producer/aggregator schedules keyframe on map stream
  -> receiver admits correlated keyframe atomically
  -> source revision chain resumes
```

service response 不携带大 payload。重试使用 requester/session/client_request_id 去重，重复请求返回相同 correlation。C4 fixture 还验证两级恢复：source→aggregator 的 contributor keyframe，以及 aggregator→central 的 aggregate keyframe。

aggregate update 的 envelope 必须把 `aggregate_revision` 和 contributor manifest 作为一个不可拆分的 committed object；中央只验证 manifest、revision、health 和 hash，不重建每个 source 的完整地图。C5 EdgeAggregator 实现可直接消费该对象。

## 7. 确定性 LogicalLinkAdapter（仅测试）

adapter 位于 ROS-free envelope 的测试 seam，不进入生产 launch。它提供固定 seed 的应用可见事件调度：

- delay、loss、reorder、bandwidth、queue limit；
- partition、reconnect、route epoch 切换；
- clock offset、skew、回跳和新 clock session；
- delivery/drop/fault reason、queue depth 和每跳时间记录。

adapter 只能延迟、丢弃、重排或拒绝消息；不得改写 payload、source epoch/revision、origin time 或 content hash。它不实现 ACK/NACK、重传、分片、拥塞控制或 discovery。测试报告只说明应用数据面在该故障轨迹下的行为，不外推无线吞吐或 DDS 实现性能。

## 8. Trust/Auth adapter

C4 定义最小验证接口（producer identity/session、credential state、authority/epoch、sequence/replay window、payload hash），提供 permissive fixture 与 rejecting fixture。生产实现可在启动配置选择，但运行中不热切换。以下输入必须拒绝且状态 fingerprint 不变：spoof producer、重复 replay、旧 epoch、expired/revoked credential、unknown producer。拒绝只写 bounded diagnostic，不把错误消息转成 source update。

## 9. 传输可替换边界

### 9.1 ROS 2 RMW 厂商替换

Fast DDS 换为 Cyclone DDS 或其他已安装 RMW 时，保持消息、topic/service、ROS-free core 和 C3 语义不变，只切换 `RMW_IMPLEMENTATION` 与厂商部署配置。代码禁止 include Fast DDS/Cyclone 私有类型，QoS 只通过标准 `rclcpp` API 表达。同一 conformance fixture 用于验证 QoS 兼容、late join、断链恢复和 resync 结果。

ROS 2 RMW 是此层的现成抽象，因此不再额外包装一个只转发 `rclcpp` 的 `ITransport`。这避免无收益抽象，也符合出现第二个实际实现后再提取接口的约定。

### 9.2 Router/bridge 替换

DDS Router 或 Zenoh bridge 属于部署/路由适配层：它们可以连接不同 domain、网段或受限链路，但不改变 `MapUpdate`、envelope、epoch/revision 或 resync。具体产品选型和稀疏路径策略由 C5 负责，C4 只提供互操作 conformance 契约。

### 9.3 非 ROS 传输

未来若确认直接 QUIC/TCP/Zenoh API 等第二个生产实现，再在 ROS-free envelope 边界提取正式 `ITransport` 或等价端口，并复用同一 conformance suite。届时只替换 endpoint adapter；地图 producer/applier、source admission 和 resync 状态机保持不变。

## 10. 兼容、迁移与回滚

- 现有 C3 direct `MapUpdate` topic/service 继续工作，C4 adapter 可以从同一 ROS 消息转换到新 envelope；不修改 C3 canonical fields。
- 旧中央 full snapshot/OctoMap merger 保留为参考部署和 A/B 基线，不与 C4 receiver 共用一份可变权威状态。
- 第一个 C4 production vertical slice 只需要一个 source、一个 receiver 和默认 Fast DDS；测试 fixture 在相同 envelope 边界插入 LogicalLinkAdapter。C5 再评估 DDS Router/Zenoh 并接入 Relay/EdgeAggregator。
- 如果 envelope、队列或故障注入有缺陷，关闭 delta/链路实验并回退到低频 keyframe；不得借助 stamp 重写或无限 queue“修复”指标。

## 11. 主要取舍

- 新增 `swarm_data_interfaces` 增加一个包，但避免把感知、通信和 controller 领域互相耦合；这是可替换拓扑和非 ROS consumer 的前提。
- 保留 `perception_interfaces/MapUpdate` 作为 C3 边界，避免在 C4 复制 revision/hash 语义；envelope 只增加路由、生命周期和传输元数据。
- 生产链复用 RMW/DDS，避免重造可靠传输；测试链用逻辑故障 fixture 证明应用恢复，但不能宣称真实网络性能。
- 以独立 endpoint、标准 QoS/resource limit 和最小业务 backlog 表达优先级，避免复制 DDS scheduler；所有资源淘汰都必须转化为可诊断的 gap/freshness/resync。
- RMW 厂商替换依赖 ROS 2 已有抽象，不提前创建 `ITransport`；真正出现非 RMW 的第二个生产实现时再提取正式接口。

## 12. 可视化与人工验收

原四条确定性通信轨迹属于 headless 协议回归，不再作为主要 RViz 人工验收。它们继续覆盖
gap/resync、aggregate 原子性、上行恢复和 TTL，但正确性由 launch test 直接断言。

人工验收使用真实洞穴感知链：

```text
cave truth + fake odom/lidar
  -> A1/A2 perception input
  -> A1/A2 authoritative local map + LocalMapUpdate
  -> routed C4 source endpoints
  -> test-only deterministic fault/receiver fixture at B
  -> B accepted A1/A2 OctoMap views + communication markers
```

A1/A2 从 `x=0`、`y=+1/-1`、`z=1.5` 平齐出发并沿 `+X` 前进约 8 m；B 从
`x=0,y=0,z=1.5` 出发，只前进 3～4 m 后悬停。B 的固定 receiver/relay capability
只属于该 fixture，不表示 C5 的动态角色或距离推断。A2 的首个 delta 在测试链路边界丢弃，
后续高 sequence 触发 gap；恢复请求延迟数秒以便目检，然后通过 C3 resync service 请求当前
相关 keyframe。B 只投影 `MapUpdateIngress` 已提交的重建地图；测试 oracle 可以单独显示
source 与 receiver 的差异，但不得把它混入权威地图。

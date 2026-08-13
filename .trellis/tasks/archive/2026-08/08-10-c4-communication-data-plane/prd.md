# C4 通信数据面

## 1. 目标

在 C3 `LocalMapUpdate` 与本机 resync 语义稳定的基础上，建立可跨进程、可路由、可恢复、可回放、可测量的通信数据面。第一条纵向链仍采用中央协调部署，但逻辑契约不能依赖中央直连、全连接拓扑或具体 DDS 行为。

本子任务只负责通信领域契约、数据面核心、确定性链路适配器和最小端到端恢复验证；不提前实现 C5 的拓扑/角色生命周期，也不把 C3 的 shared-view alignment 消费验收整体回填到本任务。

## 2. 已确认事实

- 父任务已确认 C1/C2 完成，C3 的 snapshot/keyframe/delta/summary、epoch/revision、原子 apply、乱序拒绝、resync 和 exact-revision replay/oracle 已验收。
- `perception_interfaces/msg/MapUpdate.msg` 和 `perception_interfaces/srv/RequestMapResync.srv` 是当前 C3 ROS 边界；service 只返回 accepted、correlation 和当前链状态，keyframe 通过异步 `MapUpdate` 流返回。
- `perception_map_update` 已提供 ROS-free `MapUpdate`、`MapUpdateApplier`、`ResyncRequestLedger` 和生产转换；C4 应复用这些语义，不另写第二套地图 revision/applier 协议。
- 当前 `swarm_controller` 的 `GlobalMapMerger` 是完整 source map 的中央基线，不能作为 C4 的逻辑接口；它只用于迁移对照和性能比较。
- 父设计要求跨机 `G_map` 从 `LocalMapUpdate` 开始，Relay/EdgeAggregator 不订阅、转发或解释 raw `LaserScan`/`PointCloud2` 作为核心权威链路。
- 父设计要求领域消息与 ROS endpoint API 分层：地图/状态走 topic，resync 走短 service，后续任务/角色 action 映射为领域事件；纯 Relay 只转发领域 envelope。
- 当前运行基线使用 ROS 2 Jazzy 的 `rmw_fastrtps_cpp`，相关节点只依赖标准 `rclcpp`/ROSIDL API，没有直接 include Fast DDS 类型。C3 `MapUpdate` 已使用 `Reliable + Volatile + KeepLast`，因此 C4 不需要重写丢包检测、分片、ACK/NACK、重传和 discovery。

## 3. 范围与约束

### 3.1 范围内

- 定义 routed domain envelope、地图/状态/控制/诊断的消息类别、优先级、source/session、epoch/revision/sequence、correlation 和 freshness 语义。
- 提供 ROS-free 数据面核心：入站校验、按 source 独立有序、重复/乱序/旧 epoch 拒绝、应用级有界 ledger/backlog、resync 和过期淘汰。
- 定义与传输实现无关的逻辑 profile，并通过薄 ROS adapter 映射到标准 `rclcpp` QoS；生产路径由 RMW/DDS 负责可靠性、分片、重传、history、durability、discovery 和传输资源限制。
- 提供短 resync 请求到异步 keyframe 流的幂等映射，并保留 full keyframe 回退。
- 为 aggregate update 与 contributor manifest 定义同 revision 原子提交边界；本任务只提供契约和最小 fixture，不实现 C5 的正式 EdgeAggregator 角色。
- 提供仅用于测试的固定 seed 逻辑链路适配器，在应用可见边界注入延迟、丢包、乱序、带宽、队列上限、断链/重连和 clock offset/skew/回跳。
- 提供可注入的 trust/auth validator，验证 spoof、replay、旧 epoch、credential expired/revoked 和 unknown producer 的拒绝语义。
- 记录 network bytes、decode/apply CPU、freshness、queue depth、RSS/PSS/USS 和 resync 计数等可审计指标。

### 3.2 范围外

- 不实现真实 DDS/wireless/射频物理仿真、真实 PKI、加密部署或安全认证基础设施；C4 只稳定验证 adapter 边界和领域拒绝语义。
- 不实现自研 TCP、可靠 UDP、RTPS/DDS、ACK/NACK、分片重组、拥塞控制或通用消息调度器；首版生产传输复用 ROS 2 RMW/DDS。
- 不实现 C5 的 `G_comm/G_control/G_map` 成员管理、角色迁移、route discovery 或自选举。
- 不实现 C6 多 Region allocator、C7 执行器和本机安全闭环；通信层不能拥有 actuator authority，也不能覆盖本机 Hold/LocalFallback。
- 不把 raw LiDAR、TF 或本机高频 setpoint 纳入 fleet 权威数据面。
- 不修改 C3 的 canonical encoding、MapUpdate revision 链或现有本机地图行为；不能引入实时 shadow mapper。

## 4. 需求

### C4-R01 独立领域接口

跨进程消息必须位于独立的 `*_interfaces` 包。现有 `perception_interfaces` 的 C3 `MapUpdate`/resync 类型保持兼容；C4 新增的 routed envelope、delivery acknowledgement、link diagnostic 和 aggregate-manifest 语义不得通过 topic 名、header stamp 或 `PointCloud2` 自定义字段隐式表达。ROS-free 核心不得 include 生成消息头。

### C4-R02 跨机边界

核心 fleet 数据面从 `LocalMapUpdate` 开始。每条地图消息保留 source vehicle、mapper session、map epoch、base/new revision、update kind、content/update hash 和 correlation；链路不得订阅或解释 raw `LaserScan`/`PointCloud2`。source payload、source epoch/revision 和任务语义在转发过程中只读。

### C4-R03 逻辑顺序与有效期

每个 source/session/map epoch 独立有序提交。接收端必须按 source/session/epoch/revision 或等价 sequence 去重，拒绝旧 epoch、重复冲突、未来 revision、过期 route 和 correlation 冲突；完全相同的重复请求或重复交付按幂等结果处理。origin 时间携带 clock domain/session 只用于溯源；每一跳用本地 monotonic clock 扣减 validity budget，不能通过重写 stamp 隐藏延迟。C4 只稳定可路由字段和有界校验，route authority、图成员关系和路径选择仍由 C5 定义。

### C4-R04 优先级与有界资源

数据面必须定义 control/resync、map keyframe、map delta、summary、diagnostic 等逻辑优先级和每类 byte/message 上限，并优先通过独立 endpoint、标准 DDS QoS/history/resource limit 和厂商中立配置实现。C4 只保留业务上必需的有界 ledger/backlog，不建立第二套通用传输队列。控制/resync 不能被普通地图流挤出；过期 summary/delta 可以淘汰，但淘汰必须产生 fault reason 或 resync 信号，不能静默刷新 freshness。

### C4-R05 去重、背压与断链恢复

入站重复、乱序、缺 delta、hash conflict、DDS history/resource limit 耗尽、link down 和 reconnect 都必须得到确定性应用结果，并保留最后合法状态。DDS Reliable 负责连接存续期间的传输级重试；当 history 丢失、进程重启、session/epoch 变化或 freshness 到期时，应用层通过 resync barrier 恢复，不能继续无限等待重传。低频 full keyframe 始终可用，delta 失败不得污染最后合法 revision。

### C4-R06 Resync 交互映射

短 service 请求只确认受理并返回幂等 `correlation_id`、目标 source/session/epoch 和诊断；大体积 keyframe 通过既有地图流异步返回并携带 correlation。endpoint adapter 重启、重试、重复或乱序不得破坏领域状态机收敛。任务/角色 action 的具体生命周期留给 C5/C6，本任务只保留可扩展消息类别。

### C4-R07 聚合原子性

aggregate update 与 contributor manifest 必须使用同一个 aggregate revision 原子提交，不能跨 revision 混用或分开刷新 freshness。manifest 只保留有界 contributor provenance，不携带逐体素 contributor bitset。C4 必须提供至少一个 contributor 删除、epoch 切换和 keyframe 恢复 fixture。

### C4-R08 确定性链路适配器

固定 seed 的逻辑链路适配器必须可在测试边界注入延迟、丢包、乱序、带宽、队列上限、断链/重连、clock offset、skew、回跳和新 clock session。它不进入生产发送路径，只用于验证 DDS 无法理解的应用级 gap/freshness/resync 行为；它只能改变可见交付轨迹，不能修改业务 payload，也不能被描述为真实 DDS 或无线性能仿真。

### C4-R09 信任拒绝边界

可替换 trust/auth adapter 必须拒绝 spoof、replay、旧 epoch、过期/撤销 credential 和未知 producer。拒绝结果只能进入 bounded diagnostics，不得改变 map/view/control 状态，不得使节点进入 Ready 或刷新 source freshness。真实签名、加密和密钥运维留待后续部署任务。

### C4-R10 可观测性与性能

端到端证据必须能区分 origin、forward receive/send、delivery/drop 时间和 fault reason，并分别报告网络字节、decode/apply CPU、freshness、队列峰值、resync 次数和 RSS/PSS/USS。CPU/延迟结论遵守本机性能测量边界，不设置小于可分辨效应的伪精度门。

### C4-R11 传输可替换性

Fast DDS、Cyclone DDS 等 ROS 2 RMW 切换不得要求修改 ROS-free 核心、领域消息或 C3 map-update 语义；RMW 选择和厂商配置只存在于部署环境。代码只使用标准 `rclcpp`/ROSIDL/QoS API，不 include 厂商类型。DDS Router、Zenoh bridge 或未来非 ROS transport 必须通过 endpoint adapter 接入同一 ROS-free envelope/conformance suite；在第二个实际生产实现出现前，不为 YAGNI 单独创建通用 `ITransport` 层。

## 5. 验收标准

- [ ] C4-A01：独立接口包可由 ROSIDL 构建；ROS-free 数据面核心在不链接 `rclcpp` 或生成消息类型的情况下通过 gtest。
- [ ] C4-A02：固定 fixture 的 C3 `MapUpdate` 经 envelope 编解码后 source/session/epoch/revision/hash/payload 完全一致；raw LiDAR 不出现在 fleet 数据路径。
- [ ] C4-A03：重复、冲突重复、乱序、旧 epoch、未来 revision、缺 delta 和 hash conflict 均拒绝且最后合法状态不变。
- [ ] C4-A04：逻辑优先级映射为独立 endpoint 和标准 QoS/resource limit；应用只保留必要的有界 ledger/backlog。满载 fixture 下内存有界，control/resync 不被普通地图流饿死。
- [ ] C4-A05：断链后只接受相关 keyframe 恢复，恢复完成前不接受 delta；恢复后 source revision 链与原始 replay oracle 等价。
- [ ] C4-A06：重复 resync 请求返回同一 correlation；keyframe 不出现在 service response；异步流可按 correlation 交付并在重试后幂等收敛。
- [ ] C4-A07：aggregate update 和 contributor manifest 同 revision 原子可见；contributor 删除、epoch 切换和重新加入不会产生跨 revision 混合。
- [ ] C4-A08：仅测试使用的固定 seed 逻辑链路可重复产生延迟、丢包、乱序、带宽、断链和 clock fault 轨迹，诊断记录完整且 payload hash 不变；生产 launch 不加载该适配器。
- [ ] C4-A09：trust/auth fixture 对 spoof、replay、旧 epoch、credential expired/revoked、unknown producer 均拒绝，且领域状态 fingerprint 不变。
- [ ] C4-A10：性能报告同时包含 bytes、decode/apply、freshness、队列峰值、RSS/PSS/USS；不得以 stamp 重写或无限缓存改善指标。
- [ ] C4-A11：保留旧中央 full snapshot 入口作为独立回滚/对照路径；C4 失败不会改变 C3 本机地图和安全行为。
- [ ] C4-A12：发布最小 launch/integration fixture，验证真实 ROS endpoint adapter + 确定性 link adapter + C3 receiver 的闭环，且恢复不依赖某次 service 客户端会话持续存在。
- [ ] C4-A13：源码与包清单无 Fast DDS 私有依赖；同一最小 conformance fixture 可以仅通过 `RMW_IMPLEMENTATION` 和部署配置切换到另一已安装 RMW，领域结果保持一致。
- [ ] C4-A14：四条确定性协议轨迹以 headless launch test 运行，分别验证 A→B gap/resync、A1/A2→B aggregate、B→C aggregate recovery 和 B1→B2→C TTL；测试直接断言 sequence、revision、hash、correlation、TTL 和最后合法状态，不把 RViz 画面作为正确性依据。
- [ ] C4-A15：提供一个真实洞穴人工目检场景。A1/A2 在 `z=1.5 m`、`y=+1/-1 m` 沿 `+X` 前进约 8 m，B 沿中心线前进 3～4 m 后悬停；A1/A2 的真实 C1→C2→C3 `LocalMapUpdate` 经 C4 传到 B。A2 的一个 delta 被确定性丢弃后，B 必须保持最后合法 A2 地图、显示 gap/resync，并在相关 keyframe 到达后追到最新 revision；A1 全程继续提交。洞穴真值只作低透明度参照，B 的远端地图必须来自 `MapUpdateIngress` 已提交状态。

## 6. 推荐但待确认的包边界

推荐保持 `perception_interfaces` 只负责感知和 C3 地图消息，新增 `swarm_data_interfaces` 承载 routed envelope、delivery/resync acknowledgement、link diagnostic 和 aggregate manifest；新增 `swarm_data_plane` 承载 ROS-free 核心、确定性 link adapter 和薄 ROS endpoint。这样既不迁移已验收的 C3 消息，也不把 C4 与 `swarm_controller` 的任务实现耦合。

该命名和“新增两个包”的选择在方案评审前确认；如果用户要求复用现有包，必须说明如何避免 perception/controller 两个领域互相依赖。

## 7. 回滚边界

- 保留旧中央 full snapshot/OctoMap merger 作为独立参考，不与新数据面共享同一权威状态实例。
- C4 envelope、队列或 link adapter 失败时，回退到低频 full keyframe；不能回退为重写 timestamp、无限缓存或静默接受旧 delta。
- 不回滚或改写 C3 map-update core；C4 只能在 ROS adapter/传输层替换。

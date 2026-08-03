# 下一阶段感知与多机协作架构重构：总体设计

> 状态：Phase 1 候选设计。未决项明确标记为 `Open`；本文不表示代码已实现。

## 1. 设计驱动因素

1. 当前倾斜 FakeLidar、`scan_returns` 和 OctoMap Builder 形成了来源与算法语义耦合。
2. 当前中央式三机路径可复现、可诊断，但 full snapshot 和全量处理不适合作为长期唯一数据面。
3. 3-9 的 Region、分配和任务生命周期需求必须保留，同时部署位置和通信方式允许改变。
4. 后续数量可能超过三机，并出现 Explorer、Relay、Aggregator、Reserve 等异构角色。
5. 本机安全必须在断链、全局状态过期或任务撤销时仍然成立。
6. 重构必须以 Phase 3 oracle 和 replay 为对照，避免同时改变过多变量后无法归因。

## 2. 架构原则

- **先逻辑语义，后传输绑定：** 先定义 observation、map update、topology、task lease 和 execution state，再选择 ROS 消息、共享内存或网络编码。
- **来源和时间可追溯：** 任何转发、聚合或分配结果都保留 source、epoch/revision 和原始观测时间。
- **本地优先安全：** 全局协调只能提供任务意图，不能授权穿越本机 unknown/occupied。
- **控制面与数据面分离：** 地图吞吐优化不隐式改变任务所有权；拓扑变化不自动刷新地图 freshness。
- **角色与部署分离：** Explorer、Relay、Aggregator 是逻辑职责，不先绑定到一个固定 ROS 包或进程。
- **一次只改变一类语义：** 感知、地图协议、拓扑和任务分配分别建立可回滚验证门。
- **基线可比较：** 任何性能结论都需要同输入、同参数、同构建类型和同采样窗口。

### 2.1 开发者扩展模型

开发者友好性由五层契约共同保证：

```text
标准 ROS wire boundary
        -> message/source adapter
        -> 稳定逻辑数据对象
        -> 可替换算法端口
        -> reference/custom implementation
        -> 统一契约测试与诊断
```

同进程、同语言算法优先使用 ROS-free C++ 端口和构造注入；跨进程或跨语言实现使用 ROS topic/service/action 边界。ROS 消息不是内部算法对象，节点负责转换，算法不直接持有 `rclcpp` 类型。

建议支持的主要算法扩展面如下。名称是概念名，最终 C++ 接口名由对应子任务决定：

| 算法族 | 稳定输入/输出 | 二次开发示例 |
|---|---|---|
| 观测解码与同步/融合 | descriptor、batch/window -> observation/fused observation | 单 2D、单 3D、多 2D、2D+3D 融合 |
| 本机地图更新 | observation + prior local state -> map update/state | OctoMap、其他体素/分层实现 |
| 地图差分与聚合 | source update(s) -> merged/aggregated update | full、delta、tile、edge aggregation |
| 拓扑与路由策略 | link/role state -> topology/routes | 中央直连、稀疏邻居、Relay 路由 |
| Frontier/Region | map view -> region evidence | 当前 Detector、后续 3D/dirty-region 方法 |
| 任务分配 | regions + agents + topology -> task leases | 确定性 matching、后续多目标策略 |
| 本机探索与恢复 | task + local map/state -> motion intent/state | 当前短跳、后续局部 planner/recovery |

每个正式扩展点必须有：

- 明确的数据所有权、frame、stamp、epoch/revision 和错误语义；
- 所需 geometry/capability/sync policy 或地图/拓扑能力声明；
- 确定性、线程安全、取消、超时和资源上限要求；
- reference implementation 和最小自定义实现示例；
- 对所有实现复用的 contract/conformance test suite；
- 无法满足能力时的 reject/degrade 诊断，而不是隐式 fallback。

Conformance test suite 使用 **Google Test (gtest)** 框架编写 ROS-free 算法测试；ROS 节点和跨进程集成使用 **ROS 2 launch testing** 框架。每个扩展点的 conformance suite 作为独立测试目标，接受实现实例作为参数，验证契约行为、边界条件和故障注入响应。子任务实现扩展点时同步提供对应 suite；二次开发者实现后运行相同 suite 验证兼容性。

### 2.2 不把所有类接口化

“所有算法可以替换”解释为**主要算法族在模块边界可替换**，而不是每个内部步骤都做虚调用。以下内容默认不是公共扩展面：

- 值对象、枚举、坐标和 revision 校验；
- 只有一个明确实现的消息转换 helper；
- 算法内部为了可读性拆出的纯函数；
- 尚无第二实现需求、也不需要运行时选择的实现细节。

这样既遵守仓库“接口正当性来自第二实现”的约定，也避免二次开发者必须理解大量细碎接口。单元测试直接测试具体算法；只有被测算法依赖另一个昂贵、可替换算法时，才注入该依赖接口。

`Accepted(D-000)`：主要算法在**进程启动时按配置选择**实现。二次开发者实现固定 ROS-free C++ 端口并注册，由 launch/config 选择；选择在节点初始化阶段冻结。未知实现、能力不满足或配置无效必须启动失败并输出明确诊断。首版不支持运行中有状态热切换，也不定义热切换所需的状态迁移、并发切换或回滚协议。

具体注册机制（静态 factory、显式 registry 或 `pluginlib`）由首次需要第二实现的子任务评审决定，不能先以工具选择代替接口设计。开发期承诺逻辑语义和源码级扩展契约，不承诺跨版本 C++ 二进制 ABI。

## 3. 功能模块

| 模块 | 核心职责 | 主要输入 | 主要输出 |
|---|---|---|---|
| 感知输入与观测模型 | 标准消息解码、能力、TF/时间、批次和多雷达视图 | `LaserScan`、`PointCloud2`、descriptor | `LidarObservation`、`ObservationWindow` |
| 位姿/状态输入与本机局部地图 | 标准位姿适配、质量/reset 门控、按独立 origin 应用观测、本机 occupied/free/unknown 状态 | observation、`Odometry`/TF | `VehiclePoseEstimate`、`LocalMapState`、本机安全数据 |
| 地图数据模型与更新 | snapshot/keyframe/delta/summary、epoch/revision、冲突和原子提交 | 本机地图变化 | `LocalMapUpdate`、resync 状态 |
| 通信接口与地图数据面 | 传输优先级、去重、背压、重同步和 freshness | map/state/control update | 有界、可恢复的数据流 |
| 通信拓扑与角色 | `G_comm/G_control/G_map`、邻居、路由和角色生命周期 | link/role/state | `TopologySnapshot`、route/role state |
| 多机协作与任务控制 | Region、matching、owner、lease、撤销与重新分配 | 地图/Region、状态、拓扑 | `TaskLease`、`TaskState` |
| 本机执行与安全 | first-hop、body/segment known-free、Hold/Stalled/Recovery | task、local map、odom | `ExecutionState`、进展/失败 |
| 诊断、回放与验收 | 跨模块身份、阶段耗时、事件和 oracle | 所有逻辑对象 | diagnostics、bag、对比报告 |

诊断是横向能力，不应成为所有模块依赖的业务中心。每个模块发布自身有界状态，由回放与验收层关联。

## 4. 逻辑数据流

```text
FakeLidar / Gazebo bridge / real driver
                  |
                  v
        standard sensor messages
                  |
                  v
     LidarObservation / ObservationWindow
                  |
          +-------+--------+
          |                |
          v                v
    LocalMapState      local safety inputs
          |
          v
 LocalMapUpdate (full/keyframe/delta/summary)
          |
          v
 map data plane -> G_map / roles -> shared/global map view
                                      |
                                      v
                              Frontier / Region
                                      |
                                      v
                         owner / TaskLease / TaskState
                                      |
                                      v
                           local execution and safety
```

原始 LiDAR 默认只在本机处理。跨机默认传本机地图更新、摘要、位姿、任务和必要诊断；拓扑与协作模块不得依赖具体雷达消息。

位姿由独立外部 source 提供：FakeOdom、Gazebo odometry 或真实 VIO/LIO/SLAM estimator 先经标准 Odometry/TF adapter 转换为 `VehiclePoseEstimate`，再被 observation transform、mapper 和 local execution 消费。本轮定义并验证输入契约，不实现 estimator。

`Accepted(D-025)`：C1-C8 的 authoritative mapper 位于 vehicle-local compute domain。传感器、pose adapter 与 mapper 可以是不同本机进程，但 raw LaserScan/PointCloud2 和 observation 不进入 fleet sparse link；`G_map` 从 revisioned `LocalMapUpdate` 开始。EdgeAggregator 只消费/聚合 map update 与 contributor manifest，Relay 只路由领域 envelope。原始数据可本地录 bag；远程 raw analytics、建图卸载或“双层本机安全地图 + 远程高精地图”是后续独立扩展，不能成为首版本机安全依赖。

## 5. 关键逻辑契约

### 5.1 `LidarObservation`

最少包含：

```text
sensor_id
geometry / capabilities
acquisition_stamp
sensor_frame
sensor origin/extrinsic at stamp
native payload and/or ray view
```

二维与三维输入保留原生访问视图；通用地图算法可按 ray 访问 `Hit/NoReturn/Invalid`。不同传感器 batch 不共享隐式 origin。

### 5.2 `VehiclePoseEstimate`

最少包含：

```text
pose_source_id / pose_source_session
parent_frame / child_frame
acquisition_stamp
pose / twist
covariance and/or quality
freshness state
pose_reset_epoch / discontinuity reason
```

ROS adapter 使用 `nav_msgs/msg/Odometry` 和对应 TF，但算法库只消费 ROS-free 值对象。TF 可用于取得某一采样时刻的坐标变换，不能以“当前能查到最新 TF”替代 source identity、质量、时间一致性或 reset 检查。

`Accepted(D-023)`：定位/状态估计算法是本轮外部可替换数据源。FakeOdom、Gazebo odometry 和未来真实 VIO/LIO/SLAM estimator 通过同一 pose adapter 接入；C1-C8 不实现 estimator、IMU/相机融合或回环算法。adapter 必须显式处理 stale、frame 不一致、covariance/quality 降级、时间回退和 pose reset，并把状态提供给 mapper、alignment、task 和 local-execution gate。

`Accepted(D-024)`：pose source session、parent/child frame 或 reset epoch 改变时采用 fail-closed map epoch 切换。mapper 立即停止旧 authoritative revision chain、撤销绑定旧 epoch 的 alignment、推进新的 local map source epoch 并清空旧地图，相关任务进入 Hold/撤销；旧 local/shared contribution 保留旧 pose/alignment epoch 并按 Frozen/Removed 管理。新 pose source 重新 Ready 后，本机 mapper 即可从空状态重建新 local map，不等待 alignment；只有 shared-frame contribution/consumer、shared-map keyframe/resync 与相关任务恢复必须等待属于新 map epoch 的 committed alignment。首版不对旧 occupancy map 做原地重投影；未来若 estimator 能证明跨重启连续性，需要独立 continuity protocol 和验收。

### 5.3 `LocalMapState` / `LocalMapUpdate`

两者共享 source/session、epoch/revision 和 geometry identity，但职责分开：

```text
LocalMapState (C2 state metadata):
  source / vehicle session identity
  source-local map frame
  map geometry / resolution / canonical lattice / current known bounds
  current map epoch / monotonic revision
  current effective ray / backend capabilities
  health / event-driven freshness
  last valid observation origin metadata / local commit time

OccupancyMapView + mapper lifecycle (C2 in-process contract):
  revision-locked occupied / free / unknown and deterministic bounded-region queries
  clear and start a new map epoch

LocalMapUpdate (C3):
  source / vehicle session identity / map epoch
  base_revision / new_revision
  update kind: full / keyframe / delta / summary
  changed keys / dirty or tile bounds
  canonical ordering / serialization
  content hash / hash scope / hash version
  atomic apply result
```

C2 state 不是地图内容消息；map geometry/resolution 与 canonical lattice 由 C2
定义，C3 只在 update 中携带并校验对应描述或 fingerprint。C3 必须针对一个已锁定的
C2 revision 生成 update 和 content hash。source-local update 可以在 alignment 不可用时
生成；只有进入 shared view 时才绑定并校验准确的 alignment epoch/revision。`delta`
必须能检测 base revision 断裂并触发 keyframe/resync，不能在接收端静默套用到错误基线。

`Accepted(D-020)`：每个 `vehicle_session_id` 只运行一个 active mapping pipeline，并产生唯一 authoritative local occupancy source/revision chain。多个 2D/3D sensor 作为独立 observation batch 输入该 pipeline；同步、融合或独立逐批更新是 mapper 实现策略，不改变地图权威性。替代 mapper 按 D-000 在启动时选择，通过 observation replay 和同一 conformance suite 比较；首版不运行实时 shadow mapper，也不允许多个 mapper 同时向本机安全、shared view、Frontier 或任务分配提供权威地图。更换 active mapper 建立新 vehicle session 并完成 map resync。

`Accepted(D-021)`：active mapper 在启动时声明 minimum viable input contract 以及允许的 degraded sensor/capability 组合。运行期不切换 mapper：当前健康输入满足完整契约时为 Healthy；只满足允许的最低组合时为 Degraded，继续提交 authoritative revision 但同时发布当前有效 capability；低于最低组合时为 Unavailable，停止提交新 revision，不能以空更新或改写 stamp 维持 freshness。shared contribution 随 freshness 到期按 Frozen/Removed 策略处理，allocator 撤销或停止续约任务，本机保持 Hold/LocalFallback；输入恢复后必须重新通过 mapper health、map freshness 和必要 resync 门才能恢复 Ready。

`Accepted(D-022)`：Degraded 使用分层、按消费者的健康门，而不是全局 Healthy/Unhealthy 布尔值。mapper gate 判断是否可提交 authoritative update；shared-view gate 校验 update、alignment、route 和 contributor health；role/task gate 将当前有效 capability 与任务要求匹配；local-execution gate 独立校验本机 freshness、body/segment/path known-free 和执行状态。合法的 Degraded 地图可以保留或进入 shared view，但不会自动授予全部角色或任务资格；中央协调器不能覆盖本机拒绝或安全 Hold。

### 5.3 三类逻辑图（G_comm / G_control / G_map）

稀疏拓扑维护三个独立的逻辑图，对应不同的路由语义和数据流：

**G_comm（通信图）**
- 节点间的**物理链路连通性**和**通信能力**
- 用途：健康检查、邻居发现、链路质量评估、freshness 监控
- 边属性：延迟、丢包率、带宽、链路 freshness、质量等级
- 示例拓扑：Explorer A ↔ Relay 0 ↔ EdgeAggregator ↔ Coordinator
- 断链检测：基于心跳超时、freshness 过期

**G_control（控制图）**
- **任务分配、角色管理、控制指令**的路由路径
- 用途：TaskLease、role assignment、ExecutionState、MotionIntent 的下发与上报
- 边属性：route epoch、hop count、TTL、authority chain
- 示例路由：Coordinator → Relay 0 → Explorer A（任务下发）、Explorer A → Relay 0 → Coordinator（状态上报）
- 路由失效：route epoch 切换、TTL 耗尽、authority 撤销

**G_map（地图图）**
- **地图更新、聚合、共享视图**的路由路径
- 用途：LocalMapUpdate 转发、EdgeAggregator 聚合、shared/global view 分发
- 边属性：route epoch、contributor manifest、resync state、map revision
- 示例路由：Explorer A → Relay 0 → EdgeAggregator（贡献聚合）、EdgeAggregator → Coordinator（全局视图）
- 数据流：source-local map → aggregate map → shared/global view

**关键约束与设计原则**：
1. **三个图可以不同**：某条链路可能传输地图但不传控制消息，或反之
2. **Relay 在所有图中保持透传语义**：转发时保留原 source identity、origin timestamp，不修改 payload
3. **EdgeAggregator 只在 G_map 中执行聚合**：不进入 G_control 的 authority chain
4. **route epoch 独立推进**：G_control 和 G_map 的 route epoch 可以独立变化（如 Relay 失效只影响受影响的图）
5. **freshness 跨图独立计算**：G_comm 链路健康不自动保证 G_map 数据 freshness

**跨图不一致时的处理规则**：
- **G_comm freshness 过期**：该链路在所有三个图中同时失效，触发 route epoch 切换和重连恢复
- **G_control 断裂但 G_map 健康**：任务下发停止，但地图贡献继续；allocator 撤销或停止续约受影响 vehicle 的任务，但不删除其 shared view 贡献
- **G_map 断裂但 G_control 健康**：vehicle 仍可接收任务但无法提交地图更新；本机保持 Hold 直到 map route 恢复
- **优先级顺序**：G_comm freshness > G_control authority > G_map contribution。通信层失效传播到控制层和地图层，但反之不成立

### 5.4 `TopologySnapshot`

最少表达：

```text
topology_epoch
role assignments
G_comm links and quality/freshness
G_control routes
G_map routes
route epoch / hop / ttl
```

三个图可以不同。Relay 转发保留原 source 身份，不把 forward time 当 origin time。

### 5.5 `TaskLease` / `TaskState`

沿用 Phase 3 已验证的 allocator epoch、semantic revision、task ID、mode、target 和 lease 语义，并扩展 owner、进展、撤销原因、失效原因和重新分配事件。任务只能表达意图，本机仍执行安全门控。

### 5.6 `ExecutionState`

最少区分：

```text
Executing
WaitingForTask
Hold
Stalled
Recovering
TaskRejectedLocally
LinkLost / LowPower（能力具备时）
```

状态变化绑定 task identity 和本机地图 revision，防止旧任务或旧地图恢复错误执行。

`Accepted(D-026)`：本机规划/安全与具体飞控通过 ROS-free `MotionIntent`/`ExecutionFeedback` 解耦。Intent 表达移动、重扫、Hold、Cancel 及其 identity/revision、frame 和安全依据；feedback 表达 Accepted、Executing、Holding/Stopped、Reached、Rejected、Failed 及对应 intent identity。adapter 负责映射 FakeOdom、Gazebo controller 或真实 autopilot 的 pose/velocity/trajectory 协议和停止确认。本轮不实现姿态控制、电机混控、动力学或厂商飞控；现有 PoseStamped motion_goal 仅为 FakeOdom reference adapter，不能成为探索算法依赖。

`Accepted(D-027)`：每个 vehicle session 只有一个 active local execution authority。Coordinator 只发布 TaskLease/role，local executor 经本机安全门生成 MotionIntent；external emergency/autopilot failsafe 优先于 local safety Hold/Cancel，后者优先于正常 execution intent。authority handoff/restart 推进 `control_authority_epoch`，接收端拒绝旧 epoch、过期 TTL、重复或乱序反馈；Relay 只转发任务/状态，不获得 actuator authority。消息层采用低频监督任务与本机高频控制闭环分离，不能通过 fleet 链路直接发送电机/姿态控制量。

`Accepted(D-028)`：VehicleHealth/ResourceHealth 是 ROS-free、字段可扩展的外部状态契约。它至少区分 battery/energy、autopilot/failsafe、actuator、compute、storage 和 link health，并允许每项为 Unknown；Unknown 不得当作 Healthy。标准 `sensor_msgs/msg/BatteryState`、autopilot telemetry 和本机 diagnostics 通过 adapter 提供，role/task/service/local-execution 各自声明所需字段和门控。LowPower/Failsafe/ActuatorUnavailable/ComputeOverBudget/StoragePressure/LinkDegraded/Unknown 通过确定性 fixture 验证 Draining、handoff、任务撤销、服务降级和 Hold；本轮不实现能耗预测、充电规划或能源最优调度。

`Accepted(D-029)`：跨进程领域 envelope 必须携带并由 trust/auth adapter 验证 producer/coordinator identity、session、authority epoch、sequence、credential state 和 replay window。未知来源、认证失败、credential revoked/expired 或 replay 的消息可以保留诊断证据，但不能提交 membership、map、role、task 或 MotionIntent authority。父级不选择具体 PKI、DDS-Security 或密钥管理；C1-C8 使用确定性 trusted fixture 和 spoof/replay/credential fault injection 验证逻辑拒绝边界，部署子任务再选择真实安全基础设施。

`Accepted(D-030)`：C1-C8 的权威运行状态默认 ephemeral。进程重启生成新 component/coordinator session 和相应 epoch，先 fail closed，再通过 registration、keyframe、alignment、topology、health 和 task resync 重建；不跨重启保留 lease 或 MotionIntent。C9a 复制 committed control prefix 以验证进程级接管，但不承诺同机掉电后的持久恢复；C9b 只要求外部 quorum/fencing 持久保存 authority term。持久化地图快照、控制日志和快速恢复可作为后续独立扩展，不应成为首条纵向链前提。

`Accepted(D-031)`：采用分级仿真验收策略。Level 0（纯软件）：FakeLidar、FakeOdom、确定性链路适配器，完全可重复，用于算法逻辑验证和 CI/CD；C1-C8 在 Level 0 验收。Level 1（软件仿真）：可选 Gazebo 集成用于传感器噪声注入和可视化，如使用必须通过与 Level 0 相同的 conformance suite；Level 1 不是核心验收依赖。Level 2（半物理）和 Level 3（实物）：真实传感器/飞控与算法集成，验证硬件接口、实时性和实际隧道探测任务；留作后续独立扩展，不在 C1-C8 范围内。每级仿真有明确的验收门和回退策略；Level 0 必须证明算法正确性和确定性。

## 6. 部署策略候选

### 方案 A：中央协调迁移，接口先可替换（推荐）

- 保留中央 shared/global map view 和 allocator 作为第一条新架构纵向链；
- 先替换感知、本机地图更新和地图数据面契约；
- 再引入 `G_comm/G_control/G_map` 与纯 Relay；
- 中央组件只是一种实现，不写进逻辑接口；
- 后续根据断链与规模证据增加分区 Aggregator 或分布式协调。

优点：最大化复用 Phase 3 oracle、任务语义和 replay，可逐层归因。缺点：第一版仍有中央瓶颈和单点失效，不能直接宣称完成大规模自治。

### 方案 B：第一版直接分布式协调

- 每个分区或节点参与地图与任务所有权协商；
- 第一版就必须解决 leader/consensus、网络分区、重复 owner、时钟与恢复；
- Phase 3 中央 allocator 只能作为行为参考，无法直接作为部署基线。

优点：目标态更直接。缺点：同时引入地图一致性、任务一致性和网络分区语义，显著扩大首个可运行版本的变量和验收成本。

### 方案 C：静态分区混合协调

- 固定一个中央控制面，地图由静态 EdgeAggregator 分区；
- 比 A 更早验证分区数据面，但角色和故障迁移仍可能被静态配置掩盖。

该方案可作为 A 的后续实验，不建议成为首条纵向链。

`Accepted(D-001)`：采用方案 A“中央协调迁移，接口先可替换”。中央 merger/shared-map view/allocator 是首条纵向链的部署实现，不是逻辑接口的一部分；后续可以在不改变 observation、map update、topology snapshot 和 task lease 语义的前提下替换部署。方案 B 不作为首版目标，方案 C 保留为后续数据面实验。

`Accepted(D-002)`：本父任务覆盖本次整体重构的最终集成，而不只是设计阶段。本次必须实际实现稀疏拓扑与至少 `Explorer`、纯透传 `Relay` 两种角色，并通过 `Explorer -> Relay -> 中央协调实现` 的纵向链验证 source 身份、三个逻辑图、路由 epoch/hop/ttl、断链与恢复。只写角色接口或使用单进程假对象不满足最终验收；EdgeAggregator 和其他角色是否也是本次强制生产实现由 D-003 决定。

`Accepted(D-003)`：本次生产实现角色为 `Explorer + Relay + EdgeAggregator`；`Reserve` 只定义稳定契约和测试替身。Relay 先以纯透传独立验收，EdgeAggregator 再作为独立实现验证 map update 聚合及中央负载变化，不允许用一个“智能 Relay”同时冒充两种角色。

`Accepted(D-004)`：最低稀疏拓扑总验收使用 `N=5` 架无人机，角色组合为 2 Explorer + 2 Relay + 1 EdgeAggregator；中央协调器是外部部署实现，不计入 N。两条 Relay 路径用于验证 route epoch 切换和单 Relay 故障恢复。`N=3` 中央直连作为同版本回归对照，不替代 N=5 验收。

建议的最低拓扑场景为：

```text
Explorer 0 --+
             +-> EdgeAggregator -> Relay 0 --+-> central coordinator
Explorer 1 --+                    Relay 1 --+
```

正常状态只激活一条选定路由，图保持稀疏；故障注入使 Relay 0 失效后，route epoch 推进并切换 Relay 1。恢复验收至少检查：旧 route 消息不能刷新 freshness；原 Explorer source epoch/revision 不因转发改变；缺失 delta 触发 keyframe/resync；任务在输入不健康期间撤销或进入明确 Hold/LocalFallback；恢复后不得出现重复 owner。

`Accepted(D-005)`：地图与任务采用“按 source 有序、按视图原子、按健康门控”的一致性。每个 source 独立维护 session epoch/revision；delta 的 base 不匹配、路由 epoch 过期或 source 不健康时不得静默应用。shared/global view 只发布已原子提交的 source revision 向量；中央 allocator 只基于健康视图发放或续约任务。首版 source/route 不健康时采取保守的全局协调门控，现有任务在 lease/watchdog 到期后进入 Hold/LocalFallback 或显式撤销；恢复需要 keyframe/resync 和新的协调 revision。首版不引入分布式共识，不实现 Region 级部分可用策略。

`Accepted(D-006)`：角色和拓扑允许中央协调器控制的受限动态重配置。节点启动时发布能力声明；协调器以单调 `role_epoch` 发布角色分配和生效边界。变更流程必须先停止或撤销受影响任务，完成必要的 map/keyframe 状态交接和路由更新，再允许新角色承担任务。旧 epoch、未确认交接或未完成 resync 的消息不得刷新状态。节点不自行选举角色，且算法实现仍按 D-000 在启动时选择，不进行运行中算法热切换。

`Accepted(D-007)`：节点使用“一个主角色 + 可组合服务能力”。`CapabilitySet` 在启动时声明可提供的职责；协调器按 `role_epoch` 分配一个 `PrimaryRole` 和零到多个 `ServiceAssignment`。主角色决定探索/待命等任务资格，Relay/Aggregation 服务分别进入相关逻辑图并受独立 CPU、内存、带宽和队列预算约束。专用 Relay/EdgeAggregator 仍通过对应主角色表达；Explorer 可在预算允许时提供纯 Relay 服务，但 Relay 服务不得检查、融合或改写透传业务语义。

服务健康下降可以只撤销该服务并推进拓扑/role epoch，不必自动改变主角色；如果服务资源争用影响本机安全或主任务，则必须优先保持本机安全并显式降级服务。任意多角色无优先级组合不在首版范围内。

`Accepted(D-008)`：公共地图层定义后端无关的 occupancy 语义，OctoMap 是默认参考实现而不是公共容器契约。逻辑算法面向 `OccupancyMapView`/`OccupancyMapUpdate` 概念工作；类型名为候选名，C2/C3 子任务再固定 C++ API。

`LocalMapState` + `OccupancyMapView` + mapper lifecycle（C2）的最低语义包括：

```text
source-local map frame and geometry/resolution
canonical lattice and out-of-range semantics
occupied / free / unknown query
deterministic bounded-region query
source/session identity, map epoch and current revision
health, event-driven freshness and last valid origin/commit metadata
actual ray/backend capabilities
clear and start a new map epoch
```

`LocalMapUpdate`（C3）的最低语义包括：

```text
source/session identity and map epoch
base/new revision and full/keyframe/delta/summary kind
added/removed/flipped cells or equivalent dirty region
canonical ordering/serialization and versioned content hash
atomic apply result, gap detection and keyframe/resync
alignment epoch/revision binding when consumed in shared view
```

其中 state 只携带元数据，query 属于只读 view，clear/new epoch 属于 mapper lifecycle，
不能把后两者误写成 ROS state message 字段。geometry/resolution 与 canonical lattice
由 C2 定义，C3 只携带并校验；source-local update 的生成不依赖 alignment，shared-view
消费才要求 committed alignment。规范不要求所有后端复制成同一容器。OctoMap adapter
负责 `octomap::OcTree`、`octomap_msgs/Octomap`、RViz 和 Phase 3 replay 转换；替代
occupancy 后端通过同一 view/update 与 conformance suite 接入。TSDF、mesh、语义图和
动态物体不是首版统一世界模型，未来需要时作为新的能力视图或上层模块设计。

`Accepted(D-009)`：EdgeAggregator 对中央表现为一个 aggregate source，但输出必须携带与 aggregate revision 原子一致的 contributor manifest。Aggregator 内部继续维护 contributor 级 contribution/snapshot，使单个 Explorer 的删除、epoch 切换和重新加入可以产生正确 aggregate delta。

建议的逻辑 envelope 为：

```text
aggregator_id / aggregator_session_epoch
base_aggregate_revision / new_aggregate_revision
aggregate update kind / dirty bounds / content hash
contributors[]:
  source_id
  source_session_epoch
  included_revision
  origin_stamp
  content_hash
```

manifest 大小与 contributor 数量成正比，不携带逐体素 contributor bitset。中央验证 manifest、aggregate revision 和 health 后直接应用 aggregate update，不重新构建每个 source 的完整地图。恢复分两级：Explorer 到 Aggregator 的断裂请求 contributor keyframe；Aggregator 到中央的断裂请求 aggregate keyframe。aggregate update 和 manifest 不能分开提交或跨 revision 混用。

`Accepted(D-010)`：稀疏拓扑的第一版验收采用确定性逻辑链路适配器/故障注入器，同时运行真实的 Relay 与 EdgeAggregator 进程。适配器位于应用逻辑对象与 ROS 传输绑定之间，在固定 seed 下控制延迟、丢包、乱序、带宽、队列上限以及断链/重连；它不替代 Relay/Aggregator 进程，也不修改业务 payload。

链路诊断至少保留：

```text
link_id / link_epoch
origin_stamp
forward_receive_stamp / forward_send_stamp
route_epoch
delivery or drop result / fault_reason
queue depth / byte budget
```

source payload、source session epoch/revision 和任务语义在链路适配器中只读。该测试层验证应用数据面的背压、freshness、路由切换和恢复，不宣称模拟真实 DDS 实现、射频传播、干扰或无线物理性能。未来网络模拟器、Gazebo bridge 或真实链路可通过同一 link adapter 边界替换确定性实现。

`Accepted(D-011)`：跨进程地图更新、拓扑、角色和任务领域契约放入独立的 `*_interfaces` ROS 接口包。二维与三维原始传感器输入继续使用标准 `sensor_msgs/msg/LaserScan` 和 `sensor_msgs/msg/PointCloud2`；能由标准消息自然表达的类型不重复定义。

接口包只承担进程边界的序列化契约。节点负责验证并转换为 ROS-free 的 observation、occupancy update、topology、role 和 task 值对象；算法库不得 include ROS 生成类型。这样，同一算法端口既可由本进程实现直接调用，也可由 ROS adapter、bag/replay 或二次开发者的跨进程实现接入。具体包名、消息字段拆分及 topic/service/action 选择由拥有相应语义的子任务固定，但不得回退到以 topic 名、header stamp 或 `PointCloud2` 自定义字段隐式编码领域状态。

`Accepted(D-012)`：交互采用“可路由领域协议 + 端点 ROS API”的双层混合模式。

```text
developer/coordinator API
  map and state       -> topic
  request resync      -> short service returning accepted + correlation_id
  exploration/role    -> action with goal/feedback/cancel/result
             |
        endpoint adapter
             |
sparse routed link
  revisioned domain messages only
  map/keyframe stream + lifecycle events + acknowledgements
             |
       pure Relay forwards unchanged
```

重同步 service 不在响应中携带大体积 keyframe；它只创建幂等请求，keyframe 通过既有 map update 流返回并携带 correlation ID。探索任务和角色迁移的 action goal/feedback/cancel/result 映射为显式领域事件，事件携带 task/role ID、epoch/revision 和 correlation ID。跨链路恢复以领域状态机为事实来源，不能依赖 action client/server 会话持续存在。

Relay 只认识路由 envelope 和资源预算，不代理或解释 ROS action 隐含的 send-goal/cancel/result/feedback/status 端点。这会增加端点 adapter 与协议契约测试，但避免让通信拓扑耦合到 ROS action 的内部传输结构，也允许未来非 ROS 链路复用同一领域协议。

`Accepted(D-013)`：本机 occupancy map 保留 source-local map frame，共享地图通过独立、版本化 alignment 契约完成坐标对齐。当前 Phase 3 的共享 `map` 作为 identity/static alignment fixture 保留，不再成为公共 map update 的隐式前提。

建议的逻辑 alignment 对象至少包含：

```text
source_id / source_session_epoch
source_map_frame / shared_frame
alignment_epoch / alignment_revision
transform: shared <- source_map
provider_id / provider_session_epoch
origin_stamp / valid_from / optional valid_until
quality / uncertainty / status
content_hash
```

状态至少区分 Candidate、Committed、Degraded 和 Revoked。map update 与 contributor manifest 引用准确的 alignment epoch/revision；只有 Committed 且通过质量门的 alignment 可以进入 shared view。alignment freshness 与 source map freshness 分开计算，转发不得重写 origin/validity。

alignment 改变会改变 source voxel 在 shared frame 中的空间含义。首版不实现对旧 aggregate contribution 的增量重投影：先原子失效该 contributor 的旧贡献并推进 aggregate revision，再请求 source keyframe，在新 committed alignment 下重建。引用旧 alignment 的 shared-frame task 同步撤销或重新签发。本机地图、安全检查和 Hold/LocalFallback 不依赖 shared alignment。

首条纵向链提供静态/fixture alignment provider 及失效注入，用于验证 map/aggregate/task 的版本门控。在线地图配准、回环闭合和漂移优化保留为正式可替换算法族，但不作为首版架构落地的隐藏依赖。

`Accepted(D-014)`：时间契约采用“origin 追溯 + hop-local monotonic validity”的双时间模型。

```text
origin event:
  origin_time / clock_domain_id / clock_session_epoch
logical order:
  source/session epoch + revision/sequence
each link hop:
  local receive/send monotonic duration
  remaining_validity_budget
receiver:
  local monotonic freshness/watchdog/lease
```

origin time 不因转发而改写；forward 时间只表示对应节点自己的 clock domain。只有在同步健康且偏差上界可证明时，诊断器才允许计算跨域绝对延迟，否则报告每跳 duration 和 uncertainty。链路 adapter 可以扣减 routing envelope 的 validity budget，但不得改变 payload 或 source epoch/revision。

任务 lease、角色交接和 resync 等可过期状态不能以不同节点的 wall/ROS stamp 建立顺序。每次 lease 续约必须推进单调 `renewal_sequence` 或等价 task-state revision；接收端按 source/session/task/sequence 去重，使用本地 monotonic clock 从“接收时的剩余预算”开始 watchdog。预算耗尽、clock session 变化、route/role/source 不健康或未完成 resync 时不得接受旧状态。

测试必须注入 offset、skew、clock 回跳、进程重启导致的新 clock session、传输延迟和重连。统一 `/clock` 只作为仿真 fixture，不是跨机部署的必要前提。

`Accepted(D-015)`：身份采用 stable configured identity + boot/session identity + coordinator registration。建议的层次为：

```text
fleet_id
  vehicle_id                    # 用户配置，稳定业务身份
    vehicle_session_id          # 本次整机运行
    component_id/source_id      # vehicle 内稳定逻辑生产者
      component/source_session  # 生产者实例
    sensor_id                   # vehicle 内用户定义且唯一
```

ROS namespace、node name 和 topic 仅用于寻址与部署，可以 remap，不能用作 contributor、route、role、task owner 或 replay 的主键。领域对象携带适合其层次的完整 identity/session key；新 session 下 revision 可以从初值重新开始，旧 session 消息无论 revision 多大都被拒绝。

节点启动后先发送 registration/capability descriptor；协调器校验稳定 ID 唯一性、session 新鲜度、schema 和能力后返回 registration generation，随后才允许进入拓扑、alignment、map 和 task 状态机。能力 descriptor 在该 session 内冻结，服务健康和资源余量通过独立状态更新，不用 header stamp 冒充更新。

同一 stable vehicle/source ID 的旧 session 仍健康时，新 session 被 Quarantined/Rejected，不能自动覆盖旧贡献或 owner。旧 session 明确撤销或健康过期后，新 session 仍需完成 topology、alignment 和 map resync，才能获得角色或任务。确定性 replay 可注入固定 session fixture；生产默认使用足够低碰撞概率的 boot ID，具体编码由 C5 固定。

`Accepted(D-019)`：同一 `vehicle_session_id` 内冻结传感器 descriptor inventory。已登记 sensor 的健康状态可以变化；相同 `sensor_id` 和相同 geometry/capability/安装 descriptor 的 producer 可以用新的 component/source session 重连，并按健康门恢复。新增、移除 sensor 或改变 descriptor 必须建立新的 vehicle session，重新 registration、算法能力匹配、TF 校验和 map resync；registration generation 只标识协调器接受的登记版本，不提供原 session 内的 inventory 热变更。

`Accepted(D-016)`：fleet membership 使用配置白名单 + 受控动态 join/rejoin/leave。白名单声明允许的 stable vehicle ID、能力约束和资源预算，不要求所有成员在 launch 时同时在线，也不接受开放式 ad-hoc discovery。

```text
Absent
  -> Joining       identity/session/capability accepted
  -> Resyncing     link + clock + alignment + map prerequisites
  -> Ready         eligible for route/service/task
  -> Draining      graceful revoke/handoff
  -> Absent

Ready/Resyncing -> Lost or Quarantined -> Joining with a new accepted session
```

membership 转换由协调器原子提交并推进 membership/topology epoch。加入中的成员不进入当前 committed view；完成 resync 后通过新 view revision 一次纳入。优雅退出先停止新任务和服务、撤销/交接 owner，再从活动图移除；意外失联先使 route/role/task 不健康，再提交移除或恢复决策。新 session 重连不能继承旧 session 的 delta base、role epoch 或 lease。

N=5 是最低集成 fixture，不是运行时上限或静态数组契约。实现必须对 active members、pending registrations、每角色实例数、links、routes、queue bytes 和 contributor 数量设置配置上限，超限申请确定性拒绝并诊断。

`Accepted(D-017)`：成员离线后的最后合法贡献默认保留为 `Frozen`，而不是把 stale timeout 当作删除。Frozen contribution 保留完整 provenance 和 committed alignment，但从 live membership/health 集合中移除，不接收 delta、不刷新 freshness，也不承担 route、role 或 task owner。

Frozen contribution 可继续参与静态 occupancy shared view，避免已探索知识因暂时失联而消失；Frozen-only frontier/Region 不得单独触发新协调任务，必须有 live/healthy source 或本机新观测支撑。这样保留地图价值，同时不把离线成员伪装成可执行主体。

Frozen 状态转换与 aggregate/view revision 原子提交，并受 alignment validity、保留时长/数量/字节预算和显式删除事件约束。alignment 撤销、资源回收或无效审计会移除 Frozen contribution。新 vehicle session 必须以 keyframe 重新建立 source contribution，不能继续旧 session 的 delta base；替换过程同样以一个新的 aggregate revision 提交。

`Accepted(D-018)`：协调器 HA 按三层实现。C1-C8 只部署单 Active；C9a 在同一主机部署独立 Active/Warm Standby 进程，验证状态复制、fencing、promotion 和 resync 协议，但不宣称整机容灾；C9b 将主备部署到独立故障域并接入外部 quorum/fencing，形成主机级高可用。Active/Standby 都是 coordinator deployment instance，不是 fleet vehicle，不加入 `G_comm/G_control/G_map`，不占用 2 Explorer + 2 Relay + 1 EdgeAggregator 配额。

```text
 C1-C8                         C9a                              C9b
 single-active grant          same-host fencing provider       external quorum/fencing
          |                          | one valid term                    | one valid term
       Active                 Active <----> Warm Standby        Active <----> Warm Standby
          |                    process       process              host A         host B
          |                          |                              |
          +--------------------------+------------------------------+
                                     |
                         routed domain contracts
                                     |
                                  N=5 fleet
```

从 C1-C8 起，所有 coordinator-originated control object 都携带 `coordinator_id`、`coordinator_session`、单调 `authority_term` 和 `commit_seq`。C1-C8 的 single-active provider 只授予一个静态 authority grant，不支持自动接管；C9a 将 provider 替换为同机、可确定性注入故障的 fencing 实现；C9b 再接入跨故障域的外部 quorum/fencing。只有持有有效 grant 的 Active 可以提交 role、membership、task 或 shared-view authority；失去 grant 的实例必须停止发布并进入 Fenced。Standby 不做独立 allocator，不接受未提交的 Active 内部状态，也不通过优先级或 header stamp 自行接管。

复制边界优先覆盖控制面事件日志/检查点：membership、topology/route epoch、role assignment、task owner/renewal、view revision、resync barrier 和 authority metadata。地图 payload 由 EdgeAggregator/source 保留，promotion 后以 keyframe/revision resync，不要求同步复制每份完整 OctoMap。为防止丢失最后 owner 决策，任务/角色提交至少在可接受的复制前缀上确认；Standby 落后时只能进入 recovery，不得直接发任务。

promotion 流程为：取得更高 fencing term -> 发布 authority barrier -> 使旧 task/role/action 失效 -> 收集成员 registration 和健康 -> 完成 topology/alignment/map resync -> 发布新 committed view -> 重新签发任务。Active 恢复后即使仍能发送旧消息，也因 fencing term 被拒绝。C9a 覆盖进程 crash/restart、复制落后和协议级旧主恢复；同机掉电属于该层共同故障，不得计作可用性通过。C9b 覆盖主机故障和网络分区；无法联系外部 quorum/fencing authority 时，所有 coordinator 实例停止协调，保留无人机本机安全和 Hold/LocalFallback。

该方案不把共识算法写入业务包。C9a 的同机 provider 只用于验证 authority adapter 和接管协议，可关闭自动 promotion 回退到 C1-C8；C9b 的外部 quorum/fencing 必须选择成熟基础设施，失败时同样回退为单 Active + 禁止自动 promotion，而不回滚 C1-C8 的领域契约。

### 6.1 角色定义的时机与层次

父规划当前只固定：

- 角色职责、禁止行为和是否属于本次生产实现；
- 角色消费/产生哪一类逻辑对象；
- 三个逻辑图中的大致位置；
- 子任务依赖顺序和总体验收责任。

当前不固定具体 C++ 接口名、ROS 消息字段、参数名、状态机转移或进程部署。详细设计顺序为：

```text
C3 地图更新语义
  -> C4 通信数据面
  -> C5a 拓扑基础：identity/link/graph/route epoch
  -> C5b 角色契约：capability/responsibility/lifecycle
  -> C5c Explorer + pure Relay 实现
  -> C5d EdgeAggregator 实现
  -> C6 多机协作消费 TopologySnapshot/RoleDescriptor
```

角色与拓扑应在 C5 内联合迭代，而不是等多机协作完成后再补。否则 allocator 会先依赖未定义的能力、路由和失效语义，形成反向耦合。

一致性与恢复的具体字段、超时和事件顺序由 C3/C4/C5/C6 分别细化；父级只固定上述安全门，不提前锁死某个网络协议或消息编码。

角色重配置与算法替换是两个不同生命周期：前者改变节点在拓扑和任务中的职责，后者不改变已加载的算法实现。C5 必须为角色转移提供可测试的 quiesce、handoff、epoch commit 和失败回退边界。

一致性与恢复的具体字段、超时和事件顺序由 C3/C4/C5/C6 分别细化；父级只固定上述安全门，不提前锁死某个网络协议或消息编码。

## 7. 迁移顺序

1. **感知边界：** 用标准二维/三维输入和逻辑 observation 替代公共 `scan_returns` 假设，保持当前确定性扫描 profile 作为 fixture。
2. **本机地图：** 逐 batch/origin 应用 observation，冻结本机地图和安全行为对照。
3. **地图更新：** 定义 revision/keyframe/delta/summary，并用 source-level replay 证明重建等价。
4. **数据面：** 加入去重、背压、resync 和有界队列；与完整 snapshot 基线比较 CPU、字节数和 freshness。
5. **拓扑角色：** 引入三个逻辑图和纯 Relay，验证断链、改路与原 source 身份。
6. **协作任务：** 恢复多 Region、matching、owner 和 lease 全生命周期，不放宽本机安全。
7. **恢复与规模：** 增加 stalled/失联/低电量策略和 `N > 3` 异构角色验收。
8. **总集成：** 重定 3-10 的一键入口、回放、RViz、诊断和总验收。
9. **C9a 同机 HA 协议验证：** 在 C1-C8 核心验收后，加入同机 Warm Standby、状态复制、fencing、promotion 和进程级恢复；明确不具备整机容灾。
10. **C9b 跨机 HA：** 将主备放入独立故障域，接入外部 quorum/fencing，验证主机故障、网络分区、自动接管和旧主恢复；不改变 N=5 fleet 拓扑。

## 8. 子任务候选

| 顺序 | 子任务 | 独立产出与验收 |
|---:|---|---|
| C1 | 感知输入与观测数据模型 | 2D/3D/混合输入、descriptor、TF/时间、能力和固定 RViz 输出 |
| C2 | 本机观测消费与局部地图 | 多 origin 更新、本机地图 revision、现有地图/安全行为对照 |
| C3 | 地图 snapshot/keyframe/delta 模型 | 离线 delta 推导、等价重建、epoch/revision/resync |
| C4 | 通信数据面 | 优先级、去重、背压、重同步、传输与处理性能 |
| C5 | 拓扑与角色（强制） | 三图模型、实际稀疏拓扑、Explorer/纯 Relay、独立 EdgeAggregator、断链与路由；Reserve 契约/测试替身 |
| C6 | 多 Region 与任务生命周期 | 3-9 强制验收、owner/lease/revoke/reassign |
| C7 | 本机恢复与执行状态 | stalled、任务拒绝、失联和能力降级，不放宽安全 |
| C8 | 集成、`N > 3` 与 Phase 总验收 | 新一键入口、异构角色、replay、RViz、诊断和 3-10 重定 |
| C9a | 同机 HA 协议验证 | 独立主备进程、committed-prefix 复制、可控 fencing、promotion 与 resync；不声明主机级 HA |
| C9b | 跨机协调器 HA | 独立故障域、外部 quorum/fencing、主机故障、网络分区与旧主恢复；不增加 fleet 无人机数量 |

子任务在父设计评审后按需创建；当前不提前创建空任务。C1 是建议的第一项实现任务。

## 9. 保留与可替换边界

### 必须保留的行为契约

- 零真值依赖规划；
- KnownFreePathChecker 的 body/segment known-free 安全；
- 可追溯、单调且防重放的 epoch/revision/lease 语义；
- Detector oracle、Demo 和结构化 replay 对比能力；
- 地图更新的资源上限、失败原子性和诊断；
- 3-9 与 3-10 的延期验收要求。

### 允许替换的实现

- 当前 `scan_returns` 消息及固定倾斜扫描来源；
- 当前节点/包部署方式；
- full snapshot 唯一地图输入；
- peer 全连接接线；
- 中央 merger/allocator 的最终部署位置；
- Component/方向、Region 证据和分配算法，但必须经过独立行为评审。

## 10. 风险与控制

| 风险 | 控制措施 |
|---|---|
| 同时改感知和地图导致 free-space 回归 | C1/C2 分开；固定 ray fixture 和地图重建 oracle |
| delta 节省网络但中央仍全量重建 | 接收端直接应用 delta；分别测传输与处理成本 |
| Relay 与 Aggregator 职责混合 | 第一版 Relay 纯透传；Aggregator 单独子任务/角色 |
| 拓扑变化造成旧数据冒充新鲜数据 | origin stamp、source epoch/revision 与 forward stamp 分离 |
| 分布式任务出现重复 owner | 在选择分布式部署前明确一致性协议；首版建议中央协调迁移 |
| 为产生多 Region 放宽安全或阈值 | Detector/Component 单独行为评审；本机安全不变 |
| `N > 3` 只验证节点能启动 | 加入资源上限、断链、角色变化和任务唯一性判据 |
| 文档与任务产物成为双重真相 | Trellis 用于规划过程；评审后的稳定决策同步到 `docs/decisions/` |
| 为“可替换”给所有类增加接口 | 只公开主要算法端口；内部纯函数和值对象保持具体实现 |
| C++ 插件 ABI 在开发期频繁变化 | 首版承诺语义和源码级契约，不承诺跨版本二进制 ABI；跨进程使用 ROS 契约 |
| 自定义实现通过测试但破坏下游假设 | 每个端口提供统一 conformance suite，并检查能力、错误、资源和确定性 |

## 11. 设计来源

- `docs/decisions/perception-and-swarm-architecture-refactor.md`
- `docs/decisions/perception-observation-interface.md`
- `docs/decisions/phase-03-topology-architecture-notes.md`
- `docs/phases/phase-03-swarm.md`
- `docs/xenomorph-scanner-plan.md`
- 父任务 `research/` 下的当前基线与接口调查。

评审通过后，跨模块扩展规范应单独形成 `docs/decisions/algorithm-extension-contract.md`，总览和各模块详解只引用它，不复制整套规则。

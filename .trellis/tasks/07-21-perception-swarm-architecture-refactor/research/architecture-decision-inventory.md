# Research: 父级架构决策清单

- Query: 哪些结论可以继承，哪些必须由父任务决定，哪些应下放到子任务？
- Scope: internal
- Date: 2026-07-21

## Findings

### 可作为父级不变量

- 感知、地图、通信、拓扑、协作、执行和诊断分层。
- 每个观测与地图更新保留 source、frame、stamp、epoch/revision。
- 原始 LiDAR 默认本机处理，跨机不依赖具体雷达消息。
- `G_comm`、`G_control`、`G_map` 是不同逻辑图。
- Relay 与 EdgeAggregator 是不同职责。
- 本机 known-free 安全不由全局协调替代。
- Phase 3 oracle、replay 和 3-9/3-10 延期要求必须保留。

## 已接受决策点（D-000 到 D-031）

0. `Accepted(D-000)`：主要算法在进程启动时按配置选择实现。二次开发者实现固定 ROS-free C++ 端口并注册，由 launch/config 选择；选择在节点初始化阶段冻结。未知实现、能力不满足或配置无效必须启动失败并输出明确诊断。首版不支持运行中有状态热切换。
1. `Accepted(D-001)`：第一条可运行纵向链采用中央协调迁移，中央部署不属于逻辑接口。
2. `Accepted(D-002)`：本次整体重构必须实现稀疏拓扑以及至少 Explorer/纯 Relay 纵向链，不能只固定接口。
3. `Accepted(D-003)`：生产实现 Explorer/Relay/EdgeAggregator；Reserve 只定义稳定契约和测试替身。
4. `Accepted(D-004)`：最低 `N=5`，角色为 2 Explorer + 2 Relay + 1 EdgeAggregator；验证单 Relay 失效切换，中央协调器不计入 N；`N=3` 保留为回归。
5. `Accepted(D-005)`：按 source 有序、按视图原子、按健康门控；首版不引入分布式共识和 Region 级部分可用。
6. `Accepted(D-006)`：中央协调器控制受限动态角色重配置，以 role_epoch 管理；不做节点自选举或算法热切换。
7. `Accepted(D-007)`：节点采用一个主角色 + 可组合服务能力；服务具有独立资源预算和健康状态。
8. `Accepted(D-008)`：后端无关 occupancy 公共契约；OctoMap 是默认参考实现和兼容 adapter。
9. `Accepted(D-009)`：EdgeAggregator 输出 aggregate update + 有界 contributor manifest；内部保留 contributor 级贡献状态。
10. `Accepted(D-010)`：使用确定性逻辑链路适配器/故障注入器驱动真实 Relay/EdgeAggregator 进程；验证应用数据面和恢复逻辑，不冒充 DDS/无线物理仿真。
11. `Accepted(D-011)`：独立 `*_interfaces` ROS 包承载地图更新、拓扑、角色和任务跨进程契约；原始 LiDAR 保持标准消息，算法核心保持 ROS-free。
12. `Accepted(D-012)`：稀疏链路传 revisioned 领域消息，端点 adapter 对地图/状态提供 topic、对重同步提供短 service、对任务/角色迁移提供 action；纯 Relay 不代理 action 内部协议。
13. `Accepted(D-013)`：每机保留 source-local map frame，以独立 alignment epoch/revision 对齐 shared frame；首版使用静态/fixture provider，在线地图配准作为后续扩展。
14. `Accepted(D-014)`：origin 携带 clock domain/session 仅用于溯源；逻辑顺序由 epoch/revision/sequence 保证；每跳用本地 monotonic validity budget，lease 使用单调 renewal sequence。
15. `Accepted(D-015)`：用户配置 stable vehicle/source ID，每次启动生成新 session，协调器登记 capability；namespace 仅寻址，重复活跃 ID 隔离，旧 session 全面失效。
16. `Accepted(D-016)`：配置白名单允许受控动态 join/rejoin/leave；Ready 前完成 resync，成员变化推进 topology/view epoch，开放式 ad-hoc discovery 不在首版范围。
17. `Accepted(D-017)`：离线 source 默认保留为 Frozen contributor；不计 live health、不接 delta，Frozen-only frontier 不单独分配，按 alignment/资源/删除策略管理。
18. `Accepted(D-018)`：HA 分三层推进。C1-C8 以单 Active 完成 N=5 核心验收；C9a 在同一主机验证 Warm Standby、状态复制、fencing、promotion 和 resync，但不宣称整机容灾；C9b 将主备部署到独立故障域并接入成熟的外部 quorum/fencing，形成主机级 HA。三层均不增加无人机数量，业务包不手写共识。
19. `Accepted(D-019)`：同一 vehicle session 内冻结传感器 descriptor inventory；相同 ID/descriptor 的 producer 可在 component/source session 更新后重连，增删传感器或改变 geometry/capability/安装 descriptor 必须建立新 vehicle session 并重新 registration/resync。
20. `Accepted(D-020)`：每个 vehicle session 只运行一个 active mapping pipeline，产生唯一 authoritative local occupancy source；多雷达作为独立 observation 输入，替代 mapper 在启动时选择并通过 replay/conformance suite 比较，首版不运行实时 shadow mapper。
21. `Accepted(D-021)`：active mapper 启动时声明 minimum viable input 和允许 degraded 组合；仍满足最低能力时以 Degraded + 有效 capability 继续提交地图，低于最低能力时停止 revision、freshness 到期并触发 Frozen/Hold/任务撤销，恢复后重新通过健康门。
22. `Accepted(D-022)`：Degraded 按 mapper、shared-view、role/task 和 local-execution 分层门控；合法地图证据与任务资格分开判断，中央协调器不得用全局布尔状态覆盖本机 known-free 安全拒绝。
23. `Accepted(D-023)`：本轮不实现定位/VIO/LIO/SLAM estimator；以标准 Odometry + TF 绑定可替换的 ROS-free pose estimate，保留 source/session、frame、quality/covariance、freshness 和 reset epoch，并验证异常对地图、任务和本机安全的门控。
24. `Accepted(D-024)`：pose source session/frame/reset epoch 不连续时 fail closed，停止旧 map revision、撤销旧 alignment 并立即推进新 local map epoch；新 pose Ready 后本机从空图重建且不等待 alignment，shared consumption/shared-map keyframe/resync 等待新 alignment；旧贡献按旧 pose/alignment epoch Frozen/Removed，首版不原地重投影。
25. `Accepted(D-025)`：C1-C8 的 authoritative mapper 位于 vehicle-local compute domain，raw LiDAR/pose 在本机处理，fleet G_map 从 LocalMapUpdate 开始；EdgeAggregator 只聚合 map update，远程 raw analytics/建图卸载延期。
26. `Accepted(D-026)`：本机规划/安全使用 ROS-free MotionIntent/ExecutionFeedback，具体 FakeOdom、Gazebo controller 或真实 autopilot 经 adapter 接入；本轮不实现飞控，PoseStamped motion_goal 仅为 FakeOdom 兼容绑定。
27. `Accepted(D-027)`：每 vehicle session 一个 local execution authority；coordinator 只发布任务/角色，local safety 与 external failsafe 可抢占，control_authority_epoch/TTL 拒绝旧命令和反馈，Relay 不获得 actuator authority。
28. `Accepted(D-028)`：VehicleHealth/ResourceHealth 统一表达 battery、failsafe、actuator、compute、storage、link 状态；Unknown 不等于 Healthy，adapter/fixture 驱动角色、服务、任务和本机执行门控，首版不做能源规划。
29. `Accepted(D-029)`：领域 envelope 验证 identity、session、authority epoch、sequence、credential state 和 replay window；未知/未认证/撤销/过期/replay 消息只能诊断，不能改变控制状态；真实 PKI 下放部署子任务。
30. `Accepted(D-030)`：权威运行状态默认 ephemeral；进程重启推进新 session/epoch 并通过 resync 重建，不跨重启保留 lease/intent；C9a 复制控制前缀验证接管，C9b 外部持久保存 authority term；持久化地图/日志作为后续扩展。
31. `Accepted(D-031)`：分级仿真验收策略。Level 0（纯软件）：FakeLidar/FakeOdom/确定性链路适配器，完全可重复，C1-C8 在此验收；Level 1（软件仿真）：可选 Gazebo 用于噪声注入和可视化，非核心依赖；Level 2/3（半物理/实物）：真实硬件集成，留作独立扩展。

以上已决项继续作为后续问题的依赖，不提前固定子任务内部字段或类名。

### 应下放到子任务

- 具体 `LidarObservation` C++ 类型、buffer API 和同步器实现；
- 真实设备 profile、厂商字段和最终雷达布局；
- delta key/tile 编码与压缩算法；
- Relay 的具体 ROS/网络部署；
- Component/方向语义和 Region 算法；
- allocator matching 算法和恢复策略参数；
- RViz 颜色、Marker 细节和 launch 文件组织。

## Recommendation

首条纵向链采用“中央协调迁移、逻辑接口可替换”：先利用现有中央式 oracle 验证新感知、地图和任务契约，再独立引入 Relay/稀疏拓扑。不要把中央节点写入逻辑接口，也不要把分布式一致性问题塞进第一个感知子任务。

## Caveats / Not Found

- 当前材料没有明确用户对“首个新架构版本必须在通信分区下自治”的硬要求。
- 未进行外部多机器人框架/协议选型研究；需在父级部署方向确定后针对具体问题开展，不应先选技术再定义需求。

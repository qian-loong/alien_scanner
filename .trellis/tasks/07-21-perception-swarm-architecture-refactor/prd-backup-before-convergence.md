# 下一阶段感知与多机协作架构重构

> 状态：Phase 1 需求梳理中。本文只定义父任务的目标、约束和验收标准；
> 当前不启动实现，不修改 Phase 3 代码，也不把候选设计写成已实现能力。

## 1. 目标

建立下一阶段可评审、可分步实施的功能架构基线，使以下能力可以在不互相隐式耦合的前提下演进：

- 二维、三维及混合多雷达感知输入；
- 本机状态、局部地图和本机安全闭环；
- 地图 full/keyframe/delta/summary 更新语义；
- 通信数据面、稀疏拓扑和角色模型；
- 多 Region 任务分配及完整任务生命周期；
- 诊断、确定性回放、性能比较和 `N > 3` 验收。

本父任务负责跨模块需求、边界、实施顺序和最终集成门槛。可独立验收的实现由后续子任务负责。

## 2. 已确认事实

1. 当前系统是中央式三机参考基线：无人机均为同质 `Explorer`，中央 merger 接收各机完整 OctoMap，中央 allocator 处理全局 Frontier；尚无 `Relay`、`EdgeAggregator`、`Reserve` 或真实无线链路模型。
2. Phase 3 只认定 3-1 至 3-8 完成。3-9 的基础实现和诊断被冻结，但多 Region、非零 eligible edge/matching、唯一 `Assigned` owner 和真实任务生命周期仍是强制延期项；3-10 等待新架构后重定。
3. Phase 3 已冻结 revision、原子提交、Detector oracle、task epoch/revision/lease、allocator 后台管线、KnownFreePathChecker 安全契约、Demo 和 replay 资产，这些是重构比较基线，不是必须保留的部署结构。
4. 当前 `/drone_i/octomap` 是 `fullMapToMsg()` 完整快照。无损 source-level replay 可离线推导相邻 snapshot 的变化，但不能还原逐 scan 更新、source session epoch、乱序、丢包或 Relay 路由。
5. 固定 Release 三机证据显示 merger 的完整 source decode + normalize 约占 cycle p50/p90 的 60%；仅改变 peer 拓扑或增加 Relay 不会自动消除中央完整地图处理成本。
6. 当前 FakeLidar 发布命中点 `PointCloud2` 和带自定义字段的 `scan_returns` PointCloud2；OctoMap Builder 直接依赖后者。倾斜扫描几何还通过 `ring_pitch_rad` 进入 FakeLidar 与高度适配器。
7. 已形成感知候选设计：二维输入使用标准 `LaserScan`，三维输入使用标准 `PointCloud2`；每个传感器保留独立 `sensor_id`、frame、采样时间和 origin；不把 `/scan_returns` 作为长期公共接口。
8. 当前不接入 Gazebo，也不选择真实雷达品牌。接口应允许 FakeLidar、未来 Gazebo bridge 和真实驱动进入同一标准 ROS 边界。
9. 开发阶段不为旧接口兼容本身牺牲新接口设计；但迁移必须用 adapter、回放和行为 oracle 证明关键算法与安全语义未被无意改变。
10. 主要算法扩展采用“进程启动时按配置选择实现”：二次开发者实现固定 ROS-free C++ 端口并注册实现，由 launch/config 选择；首版不支持运行中有状态热切换。
11. 第一条新架构纵向链采用中央协调迁移：中央 merger/shared-map view/allocator 可以作为首个实现继续存在，但中央部署不属于逻辑接口；感知、地图更新、通信与任务契约必须允许后续替换为 Relay、EdgeAggregator 或分布式实现。
12. 本父任务代表本次整体重构，其最终集成验收必须包含稀疏拓扑和拓扑角色的实际定义、实现与运行验证；至少 `Explorer` 与纯透传 `Relay` 必须形成可运行纵向链，不能只交付接口文档或模拟结论。
13. 本次整体重构的生产角色范围采用 `Explorer + Relay + EdgeAggregator`；`Reserve` 在本次只定义稳定契约和测试替身，不要求生产实现。角色详细协议在地图数据面稳定后，由拓扑与角色子任务联合定义，再由多机协作模块消费。
14. 稀疏拓扑最低总验收规模为 `N=5` 架无人机：2 Explorer + 2 Relay + 1 EdgeAggregator，中央协调器不计入 N；必须验证一个 Relay 失效后的备用路径切换。`N=3` 中央直连继续作为 Phase 3 行为与性能回归对照。
15. 地图与任务采用按 source 有序提交、按视图原子应用和全局健康门控：不要求全 source 同步，但 source/route 不健康时不得发放或无条件续约任务；缺失或断裂的 delta 必须通过 keyframe/resync 恢复。首版不引入分布式共识，也不承诺 Region 级部分可用。
16. 角色和拓扑支持中央协调器控制的受限动态重配置：节点启动时声明能力，角色变更推进单调 `role_epoch`，必须经过任务撤销/交接、安全停顿和必要 resync；节点不自行选举角色，算法实现不在运行中热切换。
17. 节点采用“一个主角色 + 可组合服务能力”模型：主角色决定任务资格，Relay/Aggregation 等服务由协调器在能力和资源预算内独立启停。专用 Relay/EdgeAggregator 仍有对应主角色，Explorer 也可在资源允许时提供纯 Relay 服务；任意服务组合不自动改变主角色。
18. 公共地图逻辑采用后端无关的 occupancy 契约，表达 occupied/free/unknown、frame、几何/分辨率、查询、变化范围、能力和 source epoch/revision；不暴露 `octomap::OcTree`。OctoMap 保留为默认参考后端及 ROS/RViz/Phase 3 兼容适配器，其他 occupancy 后端可在进程启动时选择。
19. EdgeAggregator 输出聚合 map update 和有界 contributor manifest。Aggregator 内部保留每个 Explorer 的贡献状态；中央只消费聚合 epoch/revision/delta 及 contributor 的 source ID、epoch、included revision、origin stamp 和 content hash，不恢复每份完整 source，也不要求逐体素 contributor 列表。
20. 稀疏拓扑的首版故障与恢复验收使用确定性逻辑链路适配器/故障注入器，并运行真实 Relay 与 EdgeAggregator 进程。适配器在固定 seed 下控制延迟、丢包、乱序、带宽、队列上限和断链/重连，但不冒充真实 DDS 或无线物理仿真。
21. 跨进程地图更新、拓扑、角色和任务契约使用独立的 `*_interfaces` ROS 接口包；原始二维/三维雷达输入继续使用标准 `sensor_msgs/msg/LaserScan` 与 `sensor_msgs/msg/PointCloud2`。ROS 消息只存在于节点边界，ROS-free 算法核心使用逻辑值对象和 C++ 端口。
22. 跨进程交互采用双层混合模式：稀疏链路只承载可路由、可回放且带 epoch/revision 的领域消息，纯 Relay 原样转发；端点 adapter 按语义向开发者提供 topic、service 或 action，并负责与链路领域消息互转。
23. 每机 occupancy map 保留 source-local map frame；进入 EdgeAggregator/shared view 前必须引用独立、已提交且健康的 source-to-shared alignment epoch/revision。首版提供静态/fixture alignment provider，不在父任务中强制实现在线跨机地图配准算法。
24. 跨机时间采用 clock-domain-aware 双时间模型：origin 时间携带 clock domain/session 用于溯源；source/route/role/task epoch/revision 负责逻辑顺序；每一跳用本地 monotonic clock 扣减有效期预算，接收端用本地 monotonic clock 执行 freshness、watchdog 和 lease。未验证时钟同步时不得直接用不同节点的 origin stamp 计算延迟或排序。
25. 身份采用用户配置的稳定 `vehicle_id` 与每次启动的新 session 分层；ROS namespace/topic 只负责寻址。传感器、地图和其他逻辑 source ID 在 vehicle 作用域内稳定，协调器登记并校验 session 与能力，重复活跃 ID 被隔离，旧 session 的地图、路由、角色和任务不得继续生效。
26. fleet 成员采用配置白名单与受控动态生命周期：白名单内 vehicle session 可在运行中加入、重连、优雅退出或失联；未知 ID 拒绝。成员必须完成 registration、链路、clock、alignment、地图 resync 和健康门后才进入 Ready，成员变化推进 topology/view epoch，不能把 `N=5` 或启动时静态列表写死为运行契约。
27. 成员退出或失联后的最后合法地图可作为带 provenance 和 committed alignment 的 `Frozen` contributor 保留；它不再刷新 freshness、接收 delta 或计入 live health。Frozen-only evidence 不得单独产生新任务，且必须受 alignment、资源和显式删除策略约束；新 session 通过 keyframe 原子替换旧贡献。
28. 中央协调器的主备属于控制面部署扩展，不计入无人机 `N`，也不属于三个通信/控制/地图逻辑图的 fleet member。高可用按三层推进：C1-C8 在 N=5 基线上只部署一个 Active coordinator；C9a 在同一主机增加 Warm Standby，用于验证状态复制、fencing、promotion 和 resync 协议，但不宣称具备整机容灾；C9b 再将主备部署到独立故障域，并接入外部 quorum/fencing，形成真实主机级高可用。三层均不改变 2 Explorer + 2 Relay + 1 EdgeAggregator 配额。
29. 同一 `vehicle_session_id` 内冻结传感器 descriptor inventory。已登记的相同 `sensor_id`、相同 descriptor 可在 producer/component session 更新后重连，运行期健康状态独立变化；新增、移除传感器或改变 geometry/capability/安装 descriptor 必须开启新的 vehicle session，重新 registration、能力匹配和 resync。
30. 每个 vehicle session 只运行一个 active mapping pipeline，并产生唯一 authoritative local occupancy source。多个二维/三维雷达是该 pipeline 的独立 observation 输入，不各自产生并行权威地图；替代建图算法在启动时按配置选择，通过固定 replay/conformance suite 比较，首版不运行实时 shadow mapper。更换 active mapper 需要新 vehicle session 和 map resync。
31. Active mapper 在启动时声明 minimum viable input contract 和允许的 degraded 输入组合。运行期 sensor 健康变化但不切换 mapper：健康输入仍满足最低契约时继续推进 authoritative map revision，并发布 `Degraded` 与当前有效能力；不再满足时进入 `Unavailable`、停止提交新 revision，地图 freshness 到期，shared contribution 和任务按 Frozen/Hold/撤销规则处理。
32. `Degraded` 采用按消费者分层门控，而不是一个全局布尔值：mapper health 决定能否提交地图，shared-view health 决定贡献能否进入协调视图，role/task eligibility 按当前有效 capability 判断，本机执行独立执行 freshness 与 body/segment/path known-free 检查；中央协调器不得覆盖本机安全判断。
33. 本轮把定位/状态估计作为外部可替换输入，不实现 VIO/LIO/SLAM estimator。ROS binding 使用标准 `nav_msgs/msg/Odometry` + TF，逻辑 pose estimate 保留 source/session、frame、stamp、pose/twist、covariance/quality、freshness 和 discontinuity/reset epoch；FakeOdom、Gazebo odometry 和真实 estimator 经 adapter 接入。
34. Pose source session、frame 或 reset epoch 改变时采用 fail-closed map epoch 切换：停止旧 revision 链和受影响任务，旧 local/shared contribution 绑定旧 pose/alignment epoch 后按 Frozen/Removed 处理；新 pose 与 alignment Ready 后建立新的 local map source epoch，从空状态或合法 keyframe 重建并 resync。首版不原地重投影旧 occupancy map。
35. C1-C8 的唯一 authoritative mapper 必须位于 vehicle-local compute domain，可以与传感器分进程但不经过 fleet sparse link。原始 `LaserScan`/`PointCloud2`、TF/pose 在本机处理，跨机 `G_map` 从 revisioned `LocalMapUpdate` 开始；EdgeAggregator 只聚合地图更新，不解释原始 LiDAR。远程 raw analytics/建图卸载作为独立后续扩展。
36. 本机探索与外部飞控通过 ROS-free `MotionIntent`/`ExecutionFeedback` 契约解耦。Local planner/safety 生成已通过 known-free 检查的意图以及 Hold/Cancel，adapter 映射到 FakeOdom、未来 Gazebo controller 或真实 autopilot 并反馈接受、执行、到达、拒绝和失败；本轮不实现姿态控制、电机混控或具体飞控协议，现有 PoseStamped 只作为 FakeOdom adapter。
37. 每个 vehicle session 只有一个 active local execution authority。Coordinator 只发布 `TaskLease`/role，不直接发布 `MotionIntent`；本机 executor 在安全检查后生成意图。控制优先级固定为 external emergency/autopilot failsafe > local safety Hold/Cancel > active execution intent；接管或重启推进 `control_authority_epoch`，旧 intent/feedback 一律拒绝，Relay 不获得 actuator authority。
38. VehicleHealth/ResourceHealth 是跨模块的外部可替换契约：电池、autopilot failsafe、执行器、计算、存储和链路资源分别报告状态，允许 `Unknown` 但 Unknown 不等于 Healthy。首版用标准 BatteryState/诊断 adapter 和固定故障 fixture 驱动 role eligibility、Draining/handoff、任务撤销、服务降级与本机 Hold，不实现能耗预测、充电规划或能源最优调度。
39. 跨进程领域消息必须有可验证的 producer/coordinator identity、session、authority epoch、sequence、credential state 和 replay window。未知、未认证、已撤销或过期凭据的消息只能进入诊断，不能改变 membership、map、role、task 或 MotionIntent authority；C1-C8 只验证逻辑 trust/auth adapter 和故障注入，不选择或实现完整 PKI。
40. 采用分级仿真验收策略：Level 0（纯软件）使用 FakeLidar、FakeOdom 和确定性链路适配器，完全可重复，用于算法逻辑验证和 CI/CD；C1-C8 在 Level 0 验收。Level 1（软件仿真）可选 Gazebo 用于噪声注入和可视化，如使用必须通过与 Level 0 相同的 conformance suite；不是核心验收依赖。Level 2（半物理）和 Level 3（实物）涉及真实硬件集成，留作后续独立扩展。

## 3. 功能需求

### R-01 功能模块边界

架构必须明确以下模块的职责、输入、输出和依赖方向：感知输入与观测模型、位姿/状态输入适配与本机局部地图、地图数据模型与更新、通信接口与地图数据面、通信拓扑与角色、多机协作与任务控制、本机执行与安全、诊断回放与验收。

### R-02 感知输入契约

二维、三维和混合多雷达必须使用来源无关的标准输入边界，并保留每传感器的 ID、frame、时间、origin、几何类型和能力。输入层不得强制所有算法采用同一种同步或融合方式。

Active mapper 必须在启动时声明 minimum viable input contract 和允许的 degraded sensor/capability 组合。输入层发布每个已登记 sensor 的独立健康状态；不能把“仍有任意一路数据”自动解释为整套建图输入健康。

原始传感器消息与 pose/TF 在 vehicle-local compute domain 内进入 active mapper；核心 fleet 数据面不要求路由 raw LiDAR。原始数据可在本机录制和离线回放，但 C1-C8 不以远程链路作为权威本机建图的必要条件。

### R-03 观测、位姿与本机地图契约

定位/状态估计算法是本轮外部数据源。ROS 边界优先使用标准 `nav_msgs/msg/Odometry` 和 TF，进入算法前转换为 ROS-free pose estimate；契约必须保留 producer identity/session、parent/child frame、采样时间、pose/twist、covariance 或等价 quality、freshness，以及位姿不连续或 estimator 重启的 reset epoch。不得把 topic、node name 或 TF 最新值本身当作位姿业务身份和有效性证明。

FakeOdom、未来 Gazebo odometry、真实 VIO/LIO/SLAM estimator 必须可通过同一 adapter 接入。首版不实现定位算法，但必须检测 stale、frame 不一致、质量降级、时间回退和 pose reset，并将结果传递到建图、alignment、任务与本机安全门。

Pose source session、parent/child frame 或 reset epoch 改变时必须 fail closed：停止向旧 authoritative map revision 链提交观测并暂停依赖它的任务。旧地图始终保留其原 pose/alignment epoch，不得把新 pose 静默套到旧体素；新 pose 与 alignment Ready 后推进 local map source epoch，从空状态或合法新 keyframe 重建并完成 shared-map/task resync。

本机地图更新必须逐传感器观测批次处理，不能把不同 origin 的射线先拼接后按单一原点解释。命中、未命中和无效观测必须可区分；缺少完整射线能力时不得静默伪造 free-space 证据。

每个 vehicle session 只有一个 active mapping pipeline 和一条 authoritative local occupancy revision 链。该 pipeline 可以消费多个 sensor batch，并自行实现独立更新、同步或融合策略；其他建图实现只通过启动配置替换和离线 replay/conformance test 比较，不在生产控制闭环中并行发布第二张权威地图。

健康输入仍满足 mapper 的 minimum contract 时，地图可继续提交 revision，但必须携带 `Degraded` 状态和当前有效 capability；不再满足时停止提交新 authoritative revision，不得通过复用旧 stamp 或空更新维持 freshness。地图到期后不得继续满足 shared-view 或 allocator 健康门，本机进入严格的 Hold/LocalFallback 安全边界。

地图健康不是全局单一布尔值。有效 occupancy update 可以进入 shared-view 候选，但每个消费模块必须针对自身需求校验 freshness、当前 capability、frame/alignment 和 revision；`Degraded` 不自动等于所有任务可用，也不自动使仍满足契约的地图证据失效。

### R-04 地图状态与更新语义

地图层必须定义后端无关的 occupancy view/update，至少表达 occupied/free/unknown、frame、地图几何/分辨率、查询能力、snapshot、keyframe、delta 和 summary，以及 source/session epoch、base/new revision、冲突、删除、原子提交、乱序拒绝和重同步条件。逻辑语义不能依赖具体传输介质，也不能暴露 `octomap::OcTree` 等实现容器。

Pose reset 产生新的 map source epoch，不是旧 revision 的普通下一帧。旧 contribution 只能在旧 epoch 下 Frozen 或 Removed；首版不要求 occupancy 后端支持跨 pose epoch 的原地重投影。

### R-05 通信数据面

通信层必须定义地图、状态、控制和诊断数据的优先级、freshness、去重、背压、断链恢复和 keyframe/resync 流程；不能用更新时间戳掩盖旧观测。每个 source 独立有序提交，shared/global view 原子发布组成它的 revision 状态。EdgeAggregator 的 aggregate update 与 contributor manifest 必须同 revision 原子提交。

`G_map` 的跨机应用边界从 `LocalMapUpdate` 开始。Relay/EdgeAggregator 不订阅、转发或解释 raw LaserScan/PointCloud2 作为核心权威链路；未来远程 raw analytics 或计算卸载必须使用独立扩展契约，不能反向成为本机安全的隐藏依赖。

### R-06 拓扑与角色

架构必须区分 `G_comm`、`G_control` 和 `G_map`，并分别定义 `Explorer`、`Relay`、`EdgeAggregator` 和 `Reserve` 的职责。本次整体重构必须实际实现 `Explorer`、纯透传 `Relay` 和 `EdgeAggregator`；`Reserve` 只交付稳定契约和测试替身。节点启动时声明能力，角色变更由中央协调器以 `role_epoch` 管理。至少一条 `Explorer -> Relay -> 中央协调实现` 和一条独立的 EdgeAggregator 数据路径必须通过验收。Relay 不与边缘聚合隐式合并。

节点具有一个活动主角色和零到多个服务分配。主角色控制探索任务、待命和角色生命周期；Relay/Aggregation 服务分别进入 `G_comm/G_control/G_map`，必须有独立资源预算、健康状态和启停结果。服务启用不授予 Explorer 任务资格，也不允许 Relay 服务修改或聚合透传 payload。

EdgeAggregator 必须在内部保留 contributor 级状态，单个 contributor 删除、epoch 切换或 revision 断裂时生成明确的 aggregate revision；必要时分别执行 contributor-to-aggregator 和 aggregator-to-central 两级 keyframe/resync。中央保留有界 manifest 以审计来源，但不承担逐 source 完整地图重建。

父级当前只固定角色职责、禁止行为、生产实现范围和依赖顺序，不提前固定 ROS 消息字段、C++ 类、配置项或完整状态机。详细角色能力、图成员关系、生命周期和故障语义由拓扑与角色子任务在地图数据面契约稳定后定义；多机协作随后消费该角色/拓扑快照。

### R-07 多机协作与任务生命周期

重构后必须恢复 3-9 的强制验收：至少两个有效 Region、非零 eligible edge/matching、唯一 owner、lease、撤销、失效、进展、重新分配，以及 LocalFallback/Assigned/Standby 的真实状态切换。allocator 只能基于健康、已提交的地图视图发放或续约任务。

任务 eligibility 必须匹配 owner 当前有效 capability；mapper 仍可 Degraded 建图不代表该 vehicle 仍有资格承担所有 Region 或角色任务。能力不足时只撤销受影响资格，不得通过中央任务状态放宽本机安全。

### R-08 本机执行与安全

任务分配、全局地图和通信拓扑不得替代本机 body、segment 和 known-free 路径检查。stalled、失联、低电量和任务不可执行必须有显式状态与恢复边界。

本机执行消费自己的 freshness、地图 revision 和安全能力门，不接受 coordinator 对安全结果的覆盖。即使 shared view 或任务仍健康，本机检查失败也必须拒绝任务、Hold 或进入受控恢复。

本机探索算法只输出 ROS-free `MotionIntent`，并消费带 identity/revision 的 `ExecutionFeedback`；不得直接依赖 FakeOdom、Gazebo、PX4/MAVROS 或某个厂商命令。Intent 至少能区分移动、重扫、Hold 和 Cancel，反馈至少能区分 Accepted、Executing、Holding/Stopped、Reached、Rejected 和 Failed。具体 pose/velocity/trajectory 控制协议由外部 adapter 负责。

当前 `PoseStamped motion_goal` 仅保留为 FakeOdom 参考 adapter，不是长期公共控制接口。本轮不实现姿态/电机控制、飞行动力学或真实 autopilot，但 adapter 的拒绝、超时、取消和停止确认必须进入本机执行状态机。

消息处理采用“低频监督 + 本机高频闭环”：coordinator/任务面传递带 task/intent identity、epoch、sequence、TTL 和取消语义的任务或意图；本机 executor/adapter 负责高频 setpoint、限速、轨迹跟踪和停止确认；反馈至少区分 Accepted、Executing、Holding/Stopped、Reached、Rejected、Failed。不能通过 fleet 链路直接流式发送电机或姿态控制量。

VehicleHealth/ResourceHealth 不由一个全局 healthy 布尔值替代：role/task eligibility、service health、link health 和 local-execution safety 分别消费其所需字段；Unknown 状态必须显式进入诊断和保守门控。

跨进程领域 envelope 还必须验证来源 identity、session、authority epoch、sequence、credential state 和 replay window。认证失败、来源未知、凭据撤销/过期或 replay 的消息只能记录诊断，不能改变 membership、地图、角色、任务或本机运动控制权威。

### R-09 诊断与确定性证据

所有跨模块对象必须可追溯 source、epoch/revision、stamp 和处理阶段。影响地图或 Frontier 语义的实现必须能用固定输入与 Phase 3 oracle 比较；性能比较必须固定构建类型、输入、参数和采样窗口。

### R-10 可扩展性

设计不得把三机数量、peer 全连接或单一中央部署写死为逻辑契约。最终集成验收必须包含 `N > 3`、异构角色、断链/恢复和有界资源行为，但不得在没有网络模型时宣称真实无线性能已经验证。

### R-11 分步迁移

实施必须允许新旧路径在明确边界上进行 A/B 或离线对比。一次子任务只改变一类语义或执行机制，并提供失败回退点；不得把感知接口、地图协议、拓扑、Frontier 行为和任务调度一次性替换。

### R-12 子任务治理

父任务必须给出可独立规划、实现、验证和回滚的子任务地图。每个子任务分别完成 `prd.md`、`design.md`、`implement.md`、方案审核、实现审核和验收后再进入下一依赖步骤。

### R-13 开发者扩展与算法可替换性

所有具有明确第二实现可能的主要算法族必须在稳定逻辑数据契约后提供 ROS-free C++ 扩展边界，使二次开发者可以替换感知融合、本机地图更新、地图聚合、拓扑/路由策略、Frontier/Region、任务分配和本机探索/恢复策略，而不需要改写上下游模块。

地图实现属于正式扩展点。默认 OctoMap 与替代后端必须通过同一 occupancy conformance suite；无法提供某项查询、ray integration、层级遍历或 dirty-region 能力时必须显式声明，并由消费算法接受、降级或拒绝。

每个正式扩展点必须同时提供：输入输出语义、能力声明、错误/降级规则、资源与确定性要求、参考实现、最小示例和可复用契约测试。跨进程或跨语言实现通过 ROS 接口接入；同进程 C++ 实现通过构造注入或工厂选择。

首版扩展机制必须支持在进程启动时根据配置选择实现，并在未知实现名、能力不满足或配置无效时启动失败且给出明确诊断。实现选择在节点构造/初始化阶段冻结；运行中不热切换，也不定义算法状态迁移。

“算法可替换”不等于每个类和辅助函数都建立纯虚接口。只在存在第二实现、需要配置选择或依赖反转的模块边界抽象；坐标转换、校验、值对象和单一确定性 helper 默认保持具体实现。

### R-14 跨进程接口与链路测试边界

标准 ROS 消息能够自然表达的设备数据必须直接使用标准消息。地图 update/envelope、contributor manifest、拓扑快照、角色 epoch 和任务 lease 等领域语义由独立的 `*_interfaces` 包承载，不能塞入 `PointCloud2` 自定义字段或由 topic 名、参数组合和隐含时序共同表达。

生成的 ROS 消息类型不得成为算法库接口。薄节点负责 ROS 消息与 ROS-free 逻辑对象之间的校验和转换；同一逻辑契约应可被进程内 C++ 实现、跨进程 ROS 实现和确定性回放共同消费。

链路适配器必须记录 `link_id/link_epoch`、origin stamp、forward receive/send stamp、route epoch 和 fault reason，并在固定 seed 下可重复注入延迟、丢包、乱序、带宽、队列上限及断链/重连。适配器只能改变传输行为，不能修改 source payload、source epoch/revision 或任务语义；未来真实网络、Gazebo bridge 或硬件链路通过同一适配边界接入。

地图 update、拓扑快照、当前角色/健康状态和任务生命周期事件在稀疏链路上使用可幂等处理的领域消息流。纯 Relay 不代理 ROS action 的内部 service/topic，也不解释取消、反馈或完成语义。端点 adapter 对连续地图/状态暴露 topic；对重同步暴露短 service，该 service 只确认请求并返回 correlation ID，keyframe 仍通过地图流异步返回；对探索任务和角色迁移暴露 action，并将 goal/feedback/cancel/result 映射到带 task/role epoch、revision 和 correlation ID 的领域事件。

端点 adapter 重启、重试或收到重复/乱序事件时，必须依靠领域 ID 与 epoch/revision 收敛，不能依赖某个 action 连接始终存活。领域生命周期状态是跨链路事实来源，service/action 是面向开发者的交互投影。

### R-15 坐标对齐与共享视图

本机 observation、occupancy map 和本机安全检查使用 source-local frame。跨机聚合不得假设所有 source 天然处于同一个 `map` frame；必须通过独立 alignment 对象表达 source map frame、shared frame、刚体变换、provider/session、`alignment_epoch/revision`、有效边界、质量/不确定度和 Candidate/Committed/Degraded/Revoked 状态。

每个 map update、aggregate contributor manifest 和共享坐标任务必须引用其使用的 alignment epoch/revision。聚合层只应用已提交且满足质量门的 alignment；旧、未知、降级或撤销的 alignment 不得刷新 contributor freshness。alignment 改变后不得把新 transform 直接套到旧 delta 链上：首版使旧贡献失效、推进 aggregate revision 并请求该 contributor keyframe，在新 alignment 下重建。

依赖旧 shared-frame alignment 的未完成任务必须撤销、Hold 或重新签发。alignment 不健康不得放宽本机 known-free 安全，也不得阻止无人机基于本机地图执行安全 Hold/LocalFallback。在线配准、回环闭合和漂移优化算法作为独立后续扩展点；首条纵向链使用确定性静态/fixture provider 验证契约与恢复。

### R-16 时间、有效期与续约

每个可过期领域对象必须区分 `origin_time`、各跳本地接收/发送时间和本地 monotonic validity budget，并标识 `clock_domain_id/clock_session_epoch`。跨节点数值时间只有在时钟同步健康、偏差界限已知时才可用于延迟分析；不满足时只能报告各跳 duration 和不确定度。

地图、拓扑、角色和任务的逻辑顺序必须由对应 epoch/revision/sequence 负责。任务 lease 不能依赖跨 clock domain 的绝对 expiry 或 header stamp 排序；续约必须有单调 `renewal_sequence`（或等价 task-state revision），接收端按 source/session/sequence 去重和拒绝回放。链路每跳扣减有效期预算，预算耗尽、route/role/source 不健康或 resync 未完成时进入 Hold/LocalFallback 或拒绝交付。

确定性链路测试必须覆盖 clock offset、skew、回跳、重启新 clock session、延迟和重连，并证明不产生旧 lease 接受、错误 freshness 或伪造跨机 latency。仿真可以使用统一 `/clock`，但验收还必须运行非统一 clock fixture。

### R-17 身份、session 与能力登记

公共契约必须区分稳定业务身份和运行实例：`fleet_id/vehicle_id` 标识物理或逻辑无人机，`vehicle_session_id` 标识本次整机运行，`component_id/source_id/sensor_id` 在 vehicle 内稳定，具体 producer 重启时推进独立 source/component session。ROS namespace、node name、topic 和进程 PID 不得作为业务主键。

`vehicle_id` 与 vehicle 内 source/sensor ID 由部署者配置并在启动时校验；session ID 每次启动唯一，确定性 replay 可显式注入固定 fixture ID。传感器 descriptor inventory 及其 schema/hash 在一个已登记 vehicle session 内冻结，运行期只改变已登记 sensor 的健康状态和 producer/component session。相同 ID、相同 descriptor 可重连；新增、移除 sensor 或改变 geometry/capability/安装 descriptor 必须开启新的 vehicle session，不能通过推进 registration generation 在原 session 内热替换。

协调器必须先登记并接受 identity/session/capability，随后才能把节点加入 topology、alignment、map contributor 或 task owner 集合。仍活跃的相同稳定 ID 出现第二个 session 时，新实例不得静默接管；相关实例进入 Quarantined/Rejected，直到旧 session 被明确撤销或按健康策略失效。所有跨进程对象必须携带足以拒绝旧 session 回放的 identity key。

### R-18 成员准入与生命周期

部署配置必须提供稳定 vehicle 白名单、允许的 capability 约束和 fleet 资源上限。运行时 membership 至少区分 Absent、Joining、Resyncing、Ready、Draining、Lost 和 Quarantined；状态转换由协调器提交并推进 membership/topology epoch，节点不能通过发布 topic 自行成为成员。

Joining/Resyncing 必须完成 identity registration、至少一条健康控制链路、clock/session 检查、必要 alignment、地图 keyframe/resync 和角色前置状态。只有 Ready 成员可被纳入任务 owner、活动 route 或 required shared-view contributor 集合。late join 不得使当前合法 view 半提交，也不得在 resync 完成前获得旧 session 的角色或任务。

优雅退出先进入 Draining，停止新任务并完成撤销/交接后再移出活动图；意外失联立即撤销或到期任务、使相关 route/service 不健康，并提交新的 topology/view membership。相同 vehicle 以新 session 重连时必须重新走 Joining/Resyncing。成员数、pending registration、每角色和每链路资源均有上限；开放式 ad-hoc discovery 不在首版范围内。

### R-19 离线贡献与共享地图保留

成员进入 Lost/Draining/Absent 时，EdgeAggregator 必须原子地把其最后一个合法 source/session contribution 标记为 `Frozen` 或按策略撤销，并推进 aggregate/view revision。Frozen 记录保留 source/session、included map revision、alignment epoch/revision、origin/last-valid metadata 和 content hash；它不接受后续 delta，不刷新 source/map freshness，也不满足 live contributor health gate。

在当前静态 occupancy 场景下，Frozen contribution 可以继续作为 shared occupancy 的历史证据，帮助保留已探索区域；但仅由 Frozen evidence 产生的 frontier/Region 不得直接获得新的协调任务，除非有 live/healthy source 或本机当前观测重新支持。Frozen 数据不能替代本机 body/segment known-free 检查。

Frozen retention 必须有明确的 alignment-validity、保留时间/数量/字节预算和显式删除事件。alignment 被 Revoked/过期、content 被判无效、资源上限触发或管理员删除时，贡献原子移除。相同 vehicle 的新 session 不得沿用旧 delta base；完成 registration、alignment 和 keyframe 后，才可在新 session 下替换 Frozen contribution。

### R-20 协调器权威与高可用扩展

N=5 核心验收只要求一个 Active coordinator，验证 topology、map、task 和本机安全契约；协调器进程不计入 N，也不作为 Explorer/Relay/EdgeAggregator/Reserve 角色。主备实现不得把 standby 当作额外无人机或把它接入 fleet route。

父级现在必须固定可扩展的 authority 边界：每个控制对象携带 `coordinator_id/coordinator_session/authority_term/commit_seq`；只有当前 authority grant 有效的 Active 才可提交任务、角色、membership 和 view。C1-C8 使用不可自动接管的单 Active authority provider；C9a 使用同机可控 fencing provider 验证协议；C9b 才要求外部 quorum/fencing。失去 grant 的 Active 必须自我 Fenced，节点拒绝旧 term。Standby 只消费已提交控制状态，不自行分配任务。

主备不是 C1-C8 的前置。核心实现完成后，C9a 在相同 N=5 拓扑、同一主机的独立 coordinator 进程间验证 committed-prefix 复制、受控 promotion、旧主 fencing 和 topology/alignment/map resync；它覆盖进程故障和协议正确性，不覆盖主机掉电、主机网络隔离或独立故障域。C9b 将 Active 与 Warm Standby 放到不同主机或等价独立故障域，使用成熟的外部 quorum/fencing，验证主机故障、网络分区、旧主恢复和自动接管。两层接管都必须先取得更高 authority term，撤销旧 task/role/action，再完成 resync 后重发任务；不承诺跨故障无缝保留 lease。业务代码不手写共识；失去权威时宁可停发协调任务，也不允许双主。

## 4. 约束与不变量

- 规划与调度不得读取洞穴拓扑真值；真值只用于造数和独立安全审计。
- 算法库继续遵守 ROS-free，ROS 节点保持消息转换和收发的薄边界。
- 可替换算法面向稳定逻辑对象编程，不直接依赖某个设备 topic、ROS node、bag 或中央部署位置。
- 标准 ROS 消息优先；自定义消息仅用于标准消息无法表达且确有跨进程契约的语义。
- 自定义跨进程领域消息集中放入独立的 `*_interfaces` 包；算法库不得 include 生成的 ROS 消息头。
- 本机安全检查不因全局地图、拓扑或任务状态而放宽。
- 来源、观测时间、frame、epoch 和 revision 不得在转发或聚合时丢失或被悄悄刷新。
- 当前 Phase 3 代码、Demo、测试和 bag 是参考基线；本规划不宣称其已满足新架构。
- 当前阶段不运行 `task.py start`，不修改业务代码，不执行 Git 提交。

## 5. 当前规划范围外

- 选择具体真实 LiDAR 品牌、型号和最终安装数量；
- 本轮直接接入 Gazebo 或真实硬件；
- 完整物理飞控、动态障碍预测和长距离 3D 全局路径规划；
- 用编队或表演飞控替代未知环境探索和本机安全；
- 在父任务中直接实现所有模块；
- 在本轮实现或选型真实 VIO/LIO/SLAM、IMU/相机融合、回环闭合算法；
- 将远程 raw LiDAR 建图、双层“本机安全地图 + 远程高精地图”纳入 C1-C8；
- 在本轮实现姿态控制、电机混控、飞行动力学、PX4/MAVROS/MAVSDK 等具体飞控；
- 在 C9a 同机主备阶段宣称已经具备主机级高可用；
- 自研分布式共识、quorum 存储或 fencing 基础设施；
- 仅通过增大 timeout、重写 header stamp 或降低安全阈值通过验收。

## 6. 父任务验收标准

- [ ] `prd.md` 明确目标、强制保留需求、范围外内容和可测试的跨模块验收标准。
- [ ] `design.md` 明确模块边界、依赖方向、逻辑数据契约、部署选择、迁移路径和失败恢复。
- [ ] `implement.md` 给出子任务地图、依赖顺序、每步验证门、回滚点和最终集成门。
- [ ] Phase 3 的 3-9/3-10 延期需求全部映射到后续子任务，没有被删除或降级。
- [ ] 感知、地图、通信、拓扑、协作和安全之间不存在对具体传感器、三机数量或中央节点的隐式契约。
- [ ] 每个 vehicle session 只有一个 authoritative local occupancy source；多雷达进入同一 active mapping pipeline，替代 mapper 可通过相同 replay/conformance suite 比较且不会产生并行权威 revision 链。
- [ ] 固定 sensor descriptor inventory 下可确定性注入单路和多路掉线；仍满足 mapper minimum contract 时地图以 Degraded + 有效 capability 继续推进，不满足时停止新 revision、freshness 到期且任务 Hold/撤销，sensor 恢复后按健康门重新进入 Ready。
- [ ] 同一个 Degraded fixture 能产生可区分的 mapper、shared-view、role/task 和 local-execution gate 结果；能力不足的任务被拒绝，但仍合法的地图证据不被无条件删除，本机安全失败不能被 coordinator 覆盖。
- [ ] FakeOdom fixture 与至少一个标准 Odometry/TF adapter fixture 进入同一 ROS-free pose estimate 契约；stale、frame 错误、quality/covariance 降级、时间回退和 reset epoch 能触发可追溯的地图、任务与本机安全门。
- [ ] Pose source session/frame/reset epoch 变化会停止旧 map revision、暂停相关任务并推进新 map epoch；旧贡献不接收新观测，新 pose/alignment Ready 后通过 keyframe/resync 重建，测试证明没有跨 epoch 污染。
- [ ] C1-C8 的 authoritative mapper 在 vehicle-local compute domain 内运行；断开所有 fleet 链路时本机仍能生成并消费 LocalMapState，Relay/EdgeAggregator 的核心接口不依赖 raw LiDAR，跨机字节统计从 LocalMapUpdate 起算。
- [ ] FakeOdom adapter 与至少一个拒绝/超时 fixture 通过同一 MotionIntent/ExecutionFeedback conformance suite；移动、重扫、Hold、Cancel、接受、执行、停止、到达、拒绝和失败均有 identity/revision，旧反馈不会恢复已取消意图。
- [ ] 同一 vehicle session 下只能有一个 active control authority；coordinator 直发 MotionIntent、旧 control epoch、过期 TTL、乱序反馈和重连旧命令均被拒绝；external failsafe 和 local safety 能抢占正常执行并得到停止确认。
- [ ] VehicleHealth/ResourceHealth fixture 可独立注入 LowPower、Failsafe、ActuatorUnavailable、ComputeOverBudget、StoragePressure、LinkDegraded 和 Unknown；角色/服务/任务/本机执行按字段分别降级或停止，Unknown 不被当作 Healthy，恢复后通过 health/resync 门恢复。
- [ ] spoof、replay、旧 epoch、过期 credential、撤销 credential 和未知 producer fixture 被确定性拒绝；拒绝消息只能进入诊断，不能改变 membership/map/role/task/MotionIntent authority，trusted fixture 的结论不宣称真实网络加密已完成。
- [ ] 稀疏拓扑和至少一个纯 Relay 角色具有实际实现；总验收覆盖原 source 身份、三类逻辑图、route epoch/hop/ttl、断链、路径失效和恢复，不能只验证接口可编译。
- [ ] `N=5`（2 Explorer + 2 Relay + 1 EdgeAggregator）稀疏拓扑完成正常路由、单 Relay 失效、备用路由切换、旧 route 拒绝和地图 keyframe/resync；中央协调器不计入 N。
- [ ] 同版本保留 `N=3` 中央直连回归，证明新接口没有无意改变 Phase 3 oracle 和本机安全语义。
- [ ] source revision 断裂、route 失效和 keyframe/resync 期间，allocator 不产生无条件的新任务或续约；恢复后任务 owner 不重复且状态可追溯。
- [ ] 动态角色变更只能由中央协调器按单调 `role_epoch` 推进；旧 role epoch、未完成交接或未完成 resync 的节点不能继续承担新角色任务。
- [ ] 主角色与服务分配可独立表达；Explorer + RelayService 和专用 Relay 均能通过同一契约运行，服务超出资源预算时显式拒绝或降级且不破坏主角色安全。
- [ ] 默认 OctoMap 和至少一个轻量替代 fixture 通过同一 occupancy view/update conformance suite；算法和数据面不直接依赖 `octomap::OcTree`，Phase 3 OctoMap replay 仍可经 adapter 使用。
- [ ] EdgeAggregator 的 aggregate update 与 contributor manifest 原子一致；单 contributor 删除、epoch 变化、delta 断裂和重新加入均可恢复，中央处理量不退化为重新解码所有完整 source。
- [ ] 固定 seed 的逻辑链路适配器可重复注入延迟、丢包、乱序、带宽、队列上限和断链/重连；真实 Relay/EdgeAggregator 进程在故障后按 route epoch、health gate 和 keyframe/resync 契约恢复。
- [ ] 链路诊断能区分 origin、forward receive/send 和交付时间，并记录 link/route epoch 与 fault reason；测试结论明确限定为应用数据面与恢复逻辑，不宣称真实 DDS 或无线性能。
- [ ] 原始 LiDAR 保持标准 `LaserScan`/`PointCloud2`，跨进程领域对象由独立 `*_interfaces` 包表达；ROS-free 算法端口和契约测试不依赖 ROS 生成类型。
- [ ] 连续地图/状态使用 topic，重同步使用“短 service 确认 + 地图流异步 keyframe”，探索任务与角色迁移使用端点 action；稀疏链路只传领域消息，纯 Relay 不代理 action 内部协议。
- [ ] service/action adapter 在重复、乱序、断链和进程重启后按 correlation ID 与 epoch/revision 恢复，不产生重复任务、重复角色提交或错误完成结果。
- [ ] 本机 map update 可在 source-local frame 发布，并通过显式 alignment epoch/revision 正确进入 shared view；没有健康 committed alignment 的 contributor 不被聚合或用于任务分配。
- [ ] alignment 更新、降级和撤销会原子失效旧贡献、推进 aggregate revision、触发 keyframe 重建并撤销或重签旧 shared-frame 任务；不得把新 transform 静默套用到旧 delta 链。
- [ ] 静态/fixture alignment provider 可确定性复现上述流程；在线地图配准算法明确留作独立扩展，不成为首版验收的隐藏前提。
- [ ] 每个跨进程对象保留 origin clock domain/session、各跳本地时间和 validity budget；clock offset/skew/回跳/重启 fixture 下，跨域时间不被用于错误排序或 latency 计算。
- [ ] task lease 使用单调 renewal sequence 和本地 monotonic watchdog；延迟、乱序、重连或旧 clock session 不会接受过期续约、恢复旧 owner 或生成重复任务。
- [ ] namespace/topic remap 不改变 vehicle/source 业务身份；vehicle 或 producer 重启后新 session 可重新登记，旧 session 的 map delta、alignment、route、role、action 和 lease 均被拒绝。
- [ ] 重复活跃 `vehicle_id`、vehicle 内重复 source/sensor ID、未知能力 schema 或 capability 不满足会确定性隔离并输出诊断，不能静默覆盖已有状态。
- [ ] 同一 vehicle session 内传感器 descriptor inventory 不可变；相同 ID/descriptor 的 producer 重连可恢复，新增、移除或 descriptor 改变会被拒绝并要求新 vehicle session、registration 和 resync。
- [ ] 白名单成员可在运行中 late join、重连、优雅退出和意外失联；只有完成 Resyncing 的 Ready session 可进入活动 route/shared view 或获得任务。
- [ ] 每次成员变化原子推进 membership/topology/view epoch；旧成员、旧 route 和旧 owner 被拒绝，资源上限内重复运行结果确定，未知 ID 永不自动加入。
- [ ] Lost/Draining source 按原子 revision 转为 Frozen 或撤销；Frozen 不刷新 live health、不能接收 delta，Frozen-only frontier 不得单独分配任务，保留/删除受确定性预算和 alignment 状态控制。
- [ ] 新 session 必须以 keyframe 替换旧 Frozen contribution；旧 session delta、alignment 或 task owner 不得跨 session 继承，贡献状态和 provenance 可在 replay/诊断中追溯。
- [ ] C1-C8 的 N=5 单 Active coordinator 基线不依赖 standby 或外部 quorum；所有控制对象已携带 authority 元数据，关闭后续 HA 能力不会改变 fleet 数量、逻辑拓扑或本机安全契约。
- [ ] C9a 在同一主机的独立 Active/Standby 进程间完成 committed-prefix 复制、进程故障、受控 promotion、旧主 fencing、coordinator 重启和 resync 验收；不产生双主、重复 owner 或旧 lease 接受，并明确标注“不具备整机容灾”。
- [ ] C9b 将 Active/Standby 部署到独立故障域并接入外部 quorum/fencing；主机故障、网络分区、旧主恢复和自动接管不会产生双主、重复 owner 或旧 lease 接受，恢复后按 resync barrier 重新发任务。
- [ ] C9a/C9b 的 standby 均不计入无人机 N、不获得 fleet role 或 task owner 资格；任一 HA 层失败均可回退到单 Active + 禁止自动 promotion，而不回退 C1-C8 的领域契约。
- [ ] Phase 3 Demo、确定性 analyzer、source-level replay 和 Release 性能基线均有明确复用位置。
- [ ] 主要算法族均有明确的扩展点归属；每个扩展点定义能力、错误、确定性、资源边界和契约测试，二次开发者无需修改上下游即可接入第二实现。
- [ ] 文档明确区分同进程 C++ 扩展与跨进程/跨语言 ROS 扩展，并明确哪些内部 helper 不属于公共扩展面。
- [ ] 每个未定型决策都记录候选方案、推荐项、取舍和负责它的子任务。
- [ ] 总体方案通过用户确认和指定方案审核模型评审后，才允许创建并启动实现子任务。
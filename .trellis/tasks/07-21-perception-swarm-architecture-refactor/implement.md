# 下一阶段感知与多机协作架构重构：父任务实施计划

> 状态：父任务持续跟踪中。Phase 1 规划已经完成并通过评审；C1/C2 已完成
> 并归档，C3 实现、本机质量门、冻结版本性能/内存专项和确定性 exact-revision replay/oracle 已完成。
> 历史 Phase 3 bag 资产不可用；shared-view alignment 消费证据仍保持开放。
> C4 通信数据面及其质量门已完成；C4.1 分块与 COW 评估为 no-go；C4.2 增量 Merkle v2
> 算法 Gate 为 go。C4.3 的 v2-only 实现、正式矩阵、内存工具和 rollback dry-run 已完成。
> 严格 Memcheck 原始结果保留第三方 `liblttng-ust` 768 B `possibly lost`，但该 finding
> 已由 C1/C2/C4 历史证据确认且不经过业务栈，business memory Gate 通过；C4.3 production
> Gate 为 GO（含已知第三方 runtime 例外），C5d 前置阻塞解除。本文维护
> 后续子任务顺序、验证门和回滚边界；Git 提交仍需用户明确授权。

## 1. 父任务规划步骤

- [x] 创建父任务、完成规划评审并激活分步实施。
- [x] 盘点 Phase 3 收口、拓扑讨论、感知草案和当前代码接线。
- [x] 建立初版 `prd.md`、`design.md`、`implement.md` 和内部 research 目录。
- [x] 确定算法替换级别 D-000：进程启动时按配置选择，首版不支持运行中热切换。
- [x] 确定协调部署形态 D-001：中央协调迁移，但中央部署不写入逻辑接口。
- [x] 确定 D-002：整体重构最终必须包含实际稀疏拓扑与 Explorer/纯 Relay 纵向验收。
- [x] 确定 D-003：生产实现 Explorer/Relay/EdgeAggregator，Reserve 只做稳定契约和测试替身。
- [x] 确定 D-004：`N=5`（2 Explorer + 2 Relay + 1 EdgeAggregator），验证单 Relay 失效切换；`N=3` 保留为回归对照。
- [x] 确定 D-005：按 source 有序、按视图原子、按健康门控；首版无分布式共识和 Region 级部分可用。
- [x] 确定 D-006：中央协调器控制受限动态重配置，以 `role_epoch` 管理；不做节点自选举或算法热切换。
- [x] 确定 D-007：一个主角色 + 可组合服务能力，服务受独立资源预算和健康状态约束。
- [x] 确定 D-008：后端无关 occupancy 契约，OctoMap 为默认参考实现和兼容 adapter。
- [x] 确定 D-009：aggregate update + 有界 contributor manifest；贡献状态保留在 Aggregator 内部。
- [x] 确定 D-010：确定性逻辑链路适配器/故障注入器 + 真实 Relay/EdgeAggregator 进程。
- [x] 确定 D-011：独立 `*_interfaces` 包承载跨进程领域契约，原始 LiDAR 保持标准 ROS 消息，算法核心保持 ROS-free。
- [x] 确定 D-012：链路传 revisioned 领域消息，端点 adapter 提供 topic/service/action，纯 Relay 不代理 action 内部协议。
- [x] 确定 D-013：source-local map frame + 独立版本化 alignment；首版静态/fixture provider，在线配准延期。
- [x] 确定 D-014：origin clock domain + hop-local monotonic validity；lease 使用 renewal sequence，不依赖跨机绝对时间排序。
- [x] 确定 D-015：用户配置 stable vehicle/source ID + boot session + coordinator capability registration；namespace 仅寻址。
- [x] 确定 D-016：配置白名单 + 受控动态 join/rejoin/leave；Ready 前完成 resync，拒绝开放式 ad-hoc discovery。
- [x] 确定 D-017：离线 source 默认保留为 Frozen contributor；不计 live health、不接 delta，受 alignment/资源/删除策略约束。
- [x] 确定 D-018：HA 分三层推进；C1-C8 单 Active，C9a 同机验证主备协议，C9b 跨故障域接入外部 quorum/fencing；均不增加 N=5 fleet 节点。
- [x] 确定 D-019：vehicle session 内冻结 sensor descriptor inventory；同 descriptor 可重连，增删或改变 descriptor 需要新 vehicle session 和 resync。
- [x] 确定 D-020：每个 vehicle session 只有一个 active mapper/authoritative local occupancy source；多雷达进入同一 pipeline，算法对比走 replay/conformance suite。
- [x] 确定 D-021：mapper 声明 minimum viable input/degraded 组合；满足最低契约时 Degraded 运行，不满足时停止 map revision 并触发 freshness/task 安全门。
- [x] 确定 D-022：Degraded 按 mapper/shared-view/role-task/local-execution 分层门控，不使用一个全局健康布尔值，coordinator 不覆盖本机安全。
- [x] 确定 D-023：定位算法保持外部；标准 Odometry/TF adapter 转为 ROS-free pose estimate，携带质量、freshness 和 reset epoch。
- [x] 确定 D-024：pose source/session/frame/reset 不连续时 fail closed，立即关闭旧链、撤销旧 alignment、推进新 local map epoch 并清空旧地图；新 pose Ready 后本机从空图重建，shared consumption/task 与 shared-map keyframe/resync 等待新 committed alignment；首版不原地重投影旧地图。
- [x] 确定 D-025：authoritative mapper 位于 vehicle-local compute domain，fleet G_map 从 LocalMapUpdate 开始，EdgeAggregator 不处理 raw LiDAR。
- [x] 确定 D-026：本机规划/安全输出 MotionIntent、消费 ExecutionFeedback；具体 FakeOdom/Gazebo/autopilot 通过 adapter 接入，本轮不实现飞控。
- [x] 确定 D-027：每 vehicle session 一个 local execution authority；coordinator 只发任务/角色，local safety 可抢占，control_authority_epoch 拒绝旧命令和反馈。
- [x] 确定 D-028：VehicleHealth/ResourceHealth 作为外部可替换健康契约；Unknown 不等于 Healthy，健康故障驱动角色/服务/任务/本机执行分层门控，不做能耗规划。
- [x] 确定 D-029：领域 envelope 验证 identity/session/authority/sequence/credential/replay；未认证或过期消息只能诊断，不能改变控制状态；真实 PKI 下放部署子任务。
- [x] 确定 D-030：运行状态默认 ephemeral，进程重启推进新 session/epoch 并 resync 重建；C9a 复制控制前缀，C9b 外部持久 authority term；持久化地图/日志留作扩展。
- [x] 确定 D-031：分级仿真验收策略；C1-C8 在 Level 0（FakeLidar/FakeOdom/确定性适配器）验收，Level 1（Gazebo）可选，Level 2/3（半物理/实物）留作扩展。
- [x] 对 PRD 做 convergence pass，删除重复事实和已解决问题。
- [x] 对总体模块、数据契约、迁移顺序和子任务边界进行指定模型方案评审。
- [x] 根据评审修订父任务产物，并由用户确认规划基线。
- [x] 将稳定结论同步回 `docs/decisions/` 总览文档。
- [x] 按依赖只创建近期要实施的子任务，首先是 C1 感知输入与观测数据模型。

父任务自身不承载业务代码；真正实施时创建并启动拥有该交付物的子任务。

## 2. 当前实施进度

| 交付物 | 状态 | 说明 |
|---|---|---|
| C1 感知输入与观测数据模型 | 已完成 | 标准 2D/3D 输入、射线证据、session/health 契约与测试已归档 |
| C2 本机观测消费与局部地图 | 已完成 | 单一权威本机地图、pose/health fail-closed、revision-locked view 已归档 |
| C1/C2 性能与内存复测 | 已完成 | 真实链路基线、sanitizer 路径覆盖和 30 分钟周期回放已归档 |
| C3 地图状态与增量更新 | 本机验收完成 | snapshot/keyframe/delta、原子 apply、乱序拒绝、resync、确定性 exact-revision replay/oracle 和冻结版本性能/内存专项已完成；shared-view alignment 消费证据保持开放 |
| C4 通信数据面 | 已完成 | routed envelope、admission/resync、故障注入、信任拒绝、可视化与性能/内存质量门已归档 |
| C4.1 接收地图分块与 COW | 已完成（no-go） | edge 16 为三档折中研究候选；R5 与短 A/B 未过门，默认仍为 Vector，保留 storage-independent view 与可选 COW 实现 |
| C4.2 增量 Merkle 内容身份 v2 | 已完成（算法 go / 生产 no-go） | 固定 4 个 touched chunks 时相对 chunk+flat 改善 20.5x/189x/858x；PSS 增加 1.7%/5.2%/6.9%，默认仍为 Vector/v1，生产迁移另立任务 |
| C4.3 Merkle v2 生产集成 | 已完成（GO，含已知第三方 runtime 例外） | v2-only descriptor、本地 root 重算、资源 admission、3 x 300 秒矩阵和 rollback 已完成；原始 Memcheck 严格 gate 保留第三方 768 B finding，business memory Gate 通过 |
| C5a 拓扑基础 | 已完成 | N=4 stable identity/session、三图、membership lifecycle、route epoch 与 topology conversion 已通过回归并归档 |
| C5b 角色与能力契约 | 已完成 | primary role、service capability、health gate、role epoch 与 transition barrier 已通过回归并归档 |
| C5c Explorer + 纯 Relay 运行期 | 已完成（N=4） | 2 Explorer + 2 Relay 真实进程链、Relay failover、旧 route 拒绝、correlated resync、Explorer Hold/transition、runtime MarkerArray 已通过；证据见 C5c validation.md |
| C5d EdgeAggregator | 已完成 | 2 Explorer + 2 Relay + 1 EdgeAggregator 的 N=5 聚合链、bounded contributor manifest、Relay failover、correlated resync、aggregate receiver 和 RViz Marker 验收已通过；证据见 C5d validation.md |
| C6-C8 | 后续 | C5d 完成后再进入 Region/task lifecycle、执行恢复和 N=5 总集成 |
| C9a/C9b | 后续扩展 | HA 扩展不阻塞 C1-C8 核心路径 |

## 3. 后续子任务顺序

### C1 感知输入与观测数据模型

前置：父级模块边界通过评审。

验收门：标准 `LaserScan`/`PointCloud2` 输入；每 sensor 独立 ID/frame/stamp/origin/health；vehicle session 内 descriptor inventory 冻结；相同 descriptor 的 producer 掉线/重连；标准 `Odometry`/TF 转 ROS-free pose estimate，保留 source/session/frame/stamp/quality/freshness/reset epoch；mapper minimum viable input 与允许 degraded 组合可校验；vehicle-local 断开 fleet 链路仍可产出本机 observation/map；2D/3D/混合 fixture；当前倾斜 profile 方向等价；固定 RViz 输出；缺失 free-ray 能力显式降级。

回滚点：保留当前 FakeLidar + OctoMap Builder 基线入口，adapter 只在对比路径中启用，直到地图行为通过。

### C2 本机观测消费与局部地图

前置：C1 逻辑 observation 契约稳定。

验收门：逐 batch/origin 更新；多个 2D/3D sensor 进入同一 active mapping pipeline；每个 vehicle session 同时只有一条 authoritative local occupancy revision 链；source-local map frame；后端无关 occupied/free/unknown view；地图 revision/capability；单路/多路 sensor 掉线时 Healthy/Degraded/Unavailable 转换正确，Unavailable 不产生伪 freshness；pose stale/frame 错误/quality 降级/时间回退可确定性门控地图；pose source/session/frame/reset 不连续会立即关闭旧链、撤销旧 alignment、推进新 map epoch 并清空旧地图，旧体素不被新观测污染；新 pose Ready 后本机从空图重建且不等待 alignment；恢复后重新通过健康门；OctoMap 与轻量替代 fixture 通过同一 replay/conformance suite；当前 deterministic fixture 和本机安全回归；无实时 shadow mapper 或并行权威地图。

回滚点：可切回旧 `scan_returns` builder 对照，不让新旧路径同时写同一地图实例。

### C3 地图状态与增量更新

前置：C2 本机地图状态稳定。

验收门：后端无关 full/keyframe/delta/summary；epoch/base/new revision；source-local update 在无 alignment 时仍可生成；进入 shared view 的 update 必须引用准确的 pose/map/alignment epoch/revision；added/removed/flipped 或等价 dirty region；canonical ordering/serialization 与版本化 content hash；乱序/重复/断裂；pose reset 和 alignment 变化使旧共享贡献失效，并在新 committed alignment 后通过 keyframe/resync 重建；Frozen/Removed contributor 的原子 revision；source-level OctoMap replay 经 identity/static adapter 等价重建。地图共享端到端与 RViz 效果按 `C3-TODO-1` 留待 C3/C8 验证，不阻塞 C2。

回滚点：完整 keyframe 始终可恢复；delta 协议失败不能污染最后合法状态。

### C4 通信数据面

前置：C3 更新语义稳定。

验收门：独立 `*_interfaces` 领域消息；核心 fleet 数据面不承载 raw LiDAR，跨机测量从 LocalMapUpdate 起算；优先级、有界队列、去重、背压、断链/恢复、resync；短 service 请求映射为 correlation ID，keyframe 异步走地图流；aggregate update 与 contributor manifest 原子一致；固定 seed 的逻辑链路适配器可注入延迟、丢包、乱序、带宽、clock offset/skew 和断链；分别测网络字节、decode/apply CPU、freshness 和内存。 trust/auth adapter 可注入 spoof、replay、旧 epoch、credential expired/revoked 和未知 producer，拒绝不改变领域状态。

回滚点：保留低频 full keyframe 路径；不能通过 stamp 重写隐藏延迟。

### C5 拓扑与角色

前置：D-001/D-002 已决、C4 有可路由数据对象。

实施顺序：先定义 stable vehicle/source identity、boot session、registration generation、link、三个 graph 和 route epoch；再定义 CapabilitySet、PrimaryRole、ServiceAssignment、role_epoch、资源预算和 quiesce/handoff/commit；随后分别实现 Explorer、纯 RelayService 与 AggregationService。专用 Relay/EdgeAggregator 使用对应主角色，Reserve 只提供稳定契约和测试替身。多机协作不得先行发明私有角色字段。

验收门：真实进程组成的稀疏拓扑；stable identity 与 session 登记、重复 ID 隔离、namespace remap 和重启旧 session 拒绝；sensor inventory 变更必须建立新 vehicle session，原 session 内只接受相同 descriptor 的 producer 重连；白名单成员 late join/rejoin/graceful leave/loss，Ready 前完成 resync；Lost source 的 Frozen/Remove 生命周期和新 session keyframe 替换；Explorer、专用 Relay/EdgeAggregator 与 Explorer + RelayService 均可表达；`Explorer -> Relay -> 中央协调实现` 纵向链；aggregate update + contributor manifest 数据路径；三个逻辑图；source 身份保持；route epoch/hop/ttl；中央协调器控制的 role_epoch 变更；角色迁移 action 经 adapter 映射为幂等领域事件；服务资源预算与独立降级；quiesce/handoff/commit/rollback；断链、路径切换和角色失效；两级 keyframe/resync；健康门控期间不发放/无条件续约任务；不把 Relay 结果当 Aggregator 结果。 VehicleHealth/ResourceHealth 的 LowPower/Failsafe/ComputeOverBudget/LinkDegraded/Unknown 可分别触发 role Draining、service 降级或拒绝。 领域 envelope 认证失败或 producer credential 无效时，节点不能进入 Ready、active route 或 role owner。

回滚点：中央直接链路仍可作为参考部署，不写入逻辑契约。

### C6 多 Region 与任务生命周期

前置：C5a-C5d 已完成，TopologySnapshot、RoleSnapshot、EdgeAggregator aggregate view 和 N=5 route/failover 契约可用。

前置：shared/global map view 和 topology snapshot 可用。

验收门：至少两个有效 Region；eligible/matching 非零；唯一 owner；lease/续约/撤销/过期/重新分配；task requirement 与 owner 当前有效 capability 匹配；Degraded mapper 不自动失去全部资格，也不获得能力不足的任务；探索 action 与跨链路任务事件正确映射，按 renewal sequence 处理续约，重连不产生重复 goal/result；alignment 失效会撤销或重签 shared-frame 任务；LocalFallback/Assigned/Standby；任务 Marker 与 Region Marker 分离。 VehicleHealth/ResourceHealth 进入 eligibility 和 lease 生命周期，LowPower/不可用执行器不接收新任务，Draining 任务可撤销或交接，Unknown 不绕过健康门。

回滚点：协调不健康时显式回到 LocalFallback/Hold，不保留过期 Assigned。

### C7 本机恢复与执行状态

前置：C6 task identity 和状态事件稳定。

验收门：ROS-free MotionIntent/ExecutionFeedback；单一 local execution authority 与 control_authority_epoch；coordinator 只发任务/角色；local safety 和 external failsafe 抢占验证；FakeOdom reference adapter；移动、重扫、Hold、Cancel 与 Accepted/Executing/Holding/Stopped/Reached/Rejected/Failed 生命周期；stalled、任务不可执行、失联、低电量（具备模拟能力时）和恢复；mapper Degraded/Unavailable、map freshness 到期与任务 Hold/撤销联动；shared/task 健康不能覆盖本机拒绝；重复/乱序/旧 feedback 不恢复取消意图；任务变化与本机地图变化触发条件明确；KnownFreePathChecker 不放宽。 VehicleHealth/ResourceHealth 的 LowPower/Failsafe/ActuatorUnavailable/Unknown 可触发本机 Hold/停止确认。

回滚点：无法证明恢复安全时保持 Hold。

### C8 集成与总验收

前置：所有必需子任务独立验收通过。

验收门：重定 3-10；一键入口；`N=5`（2 Explorer + 2 Relay + 1 EdgeAggregator）；单 Active authority provider 且不依赖 standby/外部 quorum；Relay 0 失效后切换 Relay 1；旧 route 拒绝；地图 keyframe/resync；FakeOdom 与标准 pose adapter fixture；静态/fixture alignment 提交、变化和撤销；clock offset/skew/回跳 fixture；健康门控；任务唯一性；`N=3` identity-alignment 回归；地图重建；RViz；诊断；重复运行确定性；资源上限。

回滚点：保留 Phase 3 中央式三机启动和 replay 作为独立参考，不在失败时覆盖基线资产。

### C9a 同机 HA 协议验证

前置：C1-C8 核心链路、D-014 时间契约、D-015 identity/session 和 D-018 authority boundary 已验收。

验收门：在同一主机、同一 `N=5`（2 Explorer + 2 Relay + 1 EdgeAggregator）fixture 上启动独立 Active 和 Warm Standby 进程；standby 不计入 N、不进入 fleet graph；控制面事件按 committed prefix 复制，复制落后时不得 promotion；同机可控 fencing provider 与 authority term 阻断旧主；Active 进程 crash/restart、Standby promotion、旧主恢复、coordinator 重启和 map/task resync 均不产生双主、重复 owner、旧 lease 接受或未经过健康门的任务。报告必须明确该层只覆盖进程和协议故障，不具备整机容灾。

回滚点：关闭自动 promotion 和同机 provider，保留 C1-C8 的单 Active + 安全停机/resync；C1-C8 不依赖 standby 存在。

### C9b 跨机协调器 HA

前置：C9a 的状态复制、authority adapter、promotion 和 resync 协议已验收；已选择成熟的外部 quorum/fencing 基础设施，并明确独立故障域和运维边界。

验收门：在同一 N=5 fleet 外，将 Active 与 Warm Standby 部署到不同主机或等价独立故障域；外部 quorum/fencing 只允许一个有效 authority term；主机故障、网络分区、旧主恢复、协调器重启和自动 promotion 均不产生双主、重复 owner、旧 lease 接受或未经过健康门的任务；无法取得 quorum/fencing 时 fail closed；promotion 后通过 topology/alignment/map keyframe 与 task resync 恢复，不要求同步复制完整地图 payload。

回滚点：关闭自动 promotion 并固定一个 Active，保留 C9a 已验证的 authority/复制协议和 C1-C8 领域契约；外部 quorum/fencing adapter 可替换，业务包不得实现自有共识。

## 4. 每个子任务的统一质量门

1. 子任务分别完成 `prd.md`、`design.md` 和 `implement.md`。
2. 方案由用户指定的审核模型评审，无高置信未解决问题后再启动。
3. 实现遵守 ROS-free 算法库 + 薄节点；消息转换不得渗入算法契约。
4. 跨进程领域消息集中在独立的 `*_interfaces` 包；标准消息足够时不得重复定义，算法库不得依赖 ROS 生成类型。
5. 新增或替换的主要算法端口提供 reference implementation、最小二开示例和可复用 conformance suite；内部 helper 不为测试便利被强制接口化。
6. 扩展实现可在进程启动时由配置选择；未知实现、能力不满足和无效配置有确定性失败测试，运行中不提供热切换入口。
7. 行为变化有纯算法 gtest；ROS 接线有 launch test；可视化有固定 RViz 目检入口。
8. 影响地图/Frontier 的变更使用固定 fixture 或 replay 与旧 oracle 比较。
9. 影响性能的变更使用 Release、固定 seed/参数/输入和足够窗口，分别报告各阶段而非只报总耗时。
10. 审核、修复、复测完成后等待用户验收与明确提交指令。

## 5. 父级集成验收矩阵

| 维度 | 最低证据 |
|---|---|
| 感知来源替换 | FakeLidar 与至少一个标准 2D/3D fixture 进入同一逻辑边界 |
| 多雷达 | 两个不同 origin 的 batch 不被错误合并；同步策略可区分 |
| 本机地图权威性 | 每个 vehicle session 只有一个 active mapper 和 authoritative occupancy revision chain；替代实现使用同一 replay/conformance suite，不并行进入控制闭环 |
| 输入能力降级 | mapper 声明 minimum/degraded 输入契约；单路失效可 Degraded，低于最低能力则停止 revision、地图过期并触发任务安全门，恢复需重新验收 |
| 分层健康门 | mapper、shared view、role/task 与 local execution 对同一 Degraded fixture 独立给出可追溯结果；中央状态不覆盖本机安全拒绝 |
| 位姿输入 | FakeOdom 与标准 Odometry/TF fixture 进入同一 ROS-free pose estimate；source/session、quality/freshness、frame 和 reset 语义可回放并正确门控地图/任务/安全 |
| Pose reset | source session/frame/reset 变化立即关闭旧 map chain、撤销旧 alignment、推进新 local epoch 并清图；新 pose Ready 即可本机重建，shared consumption/task 仅在新 committed alignment 与 shared-map keyframe/resync 后恢复；旧 contribution 不接受新观测，首版无隐式重投影 |
| Mapper 部署 | authoritative mapper 在 vehicle-local domain；断开 fleet 链路仍可更新本机地图，G_map 从 LocalMapUpdate 开始且 Relay/Aggregator 不依赖 raw LiDAR |
| 飞控边界 | MotionIntent/ExecutionFeedback 与具体控制协议隔离；FakeOdom adapter 覆盖接受、执行、Hold/Cancel、停止、到达、拒绝、超时和失败，旧反馈不可复活命令 |
| 控制权威 | 单一 local authority、control epoch、TTL、抢占和停止确认可回放；coordinator/Relay 无 actuator authority，旧命令与反馈不可复活 |
| Vehicle health | BatteryState/diagnostics adapter 与固定故障 fixture 可回放；角色、服务、任务和本机执行按字段门控，Unknown 不被视为 Healthy |
| 消息信任 | spoof/replay/过期或撤销 credential/未知 producer 的 envelope 被拒绝且只留诊断；父级不宣称真实 PKI 或链路加密已验收 |
| 地图等价 | 后端无关 keyframe + delta 重建与完整 snapshot 内容一致；source revision 断裂不会静默应用；OctoMap 只经 adapter 出现 |
| 数据面恢复 | 重复、乱序、缺 delta、epoch 切换和 contributor 删除/重入可恢复；aggregate/manifest 原子且状态有界 |
| 链路故障 | 固定 seed 的逻辑链路适配器驱动真实 Relay/Aggregator 进程；诊断区分 origin/forward/delivery 时间与 fault reason，结论不外推为无线性能 |
| 跨进程契约 | 标准 LiDAR 消息与独立 `*_interfaces` 领域消息边界明确；ROS-free 算法与 ROS 生成类型隔离 |
| 交互映射 | map/state topic、短 resync service、task/role action 均映射到可路由领域协议；Relay 无需代理 action 内部端点，重连后生命周期幂等收敛 |
| 坐标对齐 | source-local map 经 committed alignment 进入 shared view；alignment 变化/撤销触发贡献失效、keyframe 重建和任务撤销，不污染本机安全状态 |
| 时间语义 | origin clock domain 与每跳本地 duration 分离；有效期预算和 renewal sequence 在 offset/skew/回跳/重启下仍防止旧 lease 与错误 freshness |
| 身份/session | stable vehicle/source ID 与 namespace 分离；重复 ID 隔离，重启后旧 session 的地图、路由、角色和任务全部拒绝 |
| 动态成员 | 白名单 session 可 late join/rejoin/leave/loss；Resyncing 不进入 committed view，Ready 转换原子推进 topology/view epoch且资源有界 |
| 离线贡献 | 最后合法 source 可 Frozen 保留 provenance 但不计 live health、不接 delta；Frozen-only frontier 不单独分配，按 alignment/资源策略原子删除 |
| 协调器权威基线 | C1-C8 单 Active N=5 不依赖 standby/外部 quorum，但控制对象携带 authority term/commit sequence |
| 同机 HA 协议 | C9a 的 standby 不计 fleet N；独立进程完成复制、fencing、promotion 和重同步，报告不宣称主机级 HA |
| 跨机 HA | C9b 的主备位于独立故障域；外部 quorum/fencing 在主机故障和网络分区下防止双主与重复 owner |
| 拓扑 | 主角色/服务可独立表达；稀疏拓扑和纯 Relay 链可运行；路径切换保留 source 身份；role_epoch 交接可回退；过期路由不刷新 origin freshness |
| 任务 | 多 Region、唯一 owner、lease/revoke/reassign 完成真实状态验收；不健康视图不发放或无条件续约 |
| 本机安全 | 断链和任务错误时 body/segment known-free 契约仍成立 |
| 扩展性 | `N=5` 异构角色稀疏拓扑运行并完成单 Relay 故障切换，无写死三机/全连接逻辑契约 |
| 性能 | 与 Phase 3 Release 基线同条件对比 decode/normalize/apply/serialize/detect/freshness |
| 回放 | 原始观测、地图更新、拓扑事件、任务和执行状态均可追溯 |
| 二次开发 | 至少两个关键算法端口用 reference + alternate fixture 运行同一 conformance suite，替换时不修改上下游 |

## 6. 当前执行边界

- C3/C4 语义和质量门已经稳定；C4.1/C4.2 已完成且未切换生产默认。C5 必须消费
  storage-independent canonical view，不得依赖 vector/chunk 内部布局，也不得借 C5
  修改 C3/C4 wire、hash、revision、resync 或 trust 契约。Merkle v2 生产 schema、协商、
  admission、回滚与 3 x 300 秒 ROS 矩阵必须由 C4.3 完成；C5d EdgeAggregator 在其
  Gate 通过前不得固定新的内容身份 wire 语义。
- 历史 Phase 3 bag 资产不可用，但当前确定性 C2 exact-revision replay/oracle 已完成
  C3 功能验收；shared-view alignment 消费证据由具备聚合边界的后续任务验收。
- 只按依赖创建近期子任务，不批量创建 C5-C9 空目录。
- `C3-TODO-1` 的多进程与 RViz 总集成仍由 C8 最终验收。
- 不因 C1-C3 局部完成而把 C4-C9 描述为已实现能力。
- 不处理或删除未跟踪的性能证据和本地工具目录，除非用户明确指定。
- 未经用户明确授权，不执行 `git add`、`git commit` 或 `git push`。

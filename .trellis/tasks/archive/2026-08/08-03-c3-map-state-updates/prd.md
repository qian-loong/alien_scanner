# C3 地图状态与增量更新

> 状态：功能实现、单元/集成测试、确定性 C2 exact-revision replay/oracle、RViz
> 可视化以及冻结版本的性能与内存专项验收均已完成。用户已授权本轮收口，任务将在
> 定向提交后归档；历史 Phase 3 bag 资产仍不可用但不阻塞功能验收，
> shared-view alignment 消费验收由后续聚合路径完成。

## 1. 目标

在 C2 的权威本机占据地图之上建立后端无关、ROS-free、可确定性验证的地图
更新契约，使一个接收端能够通过完整基线和连续增量可靠地重建某个 source-local
地图，并在重复、乱序、缺失、epoch 切换或状态损坏时 fail closed、请求重同步，
且不污染最后一个合法 revision。

C3 解决“地图变化如何被表达、校验、应用和恢复”；它不重新处理 LiDAR，也不
负责跨机链路、Relay 路由或多机任务分配。

## 2. 背景与已确认事实

- C2 是每个 vehicle session 唯一的权威本机 occupancy writer，已经提供
  `(mapper_session, map_epoch, revision)`、原子 mutation、epoch reset 和精确
  revision 的 `MapReadTransaction`。
- `map_epoch` 表示一条空间连续链；新 epoch 从 revision 0 开始。只有接受并实际
  改变 occupancy 的证据才推进 revision。
- 当前 `octomap_msgs/Octomap` 是可视化完整快照，不是权威查询、同步或恢复契约。
- 当前 OctoMap backend 支持确定性 known-cell 遍历和 serialization，但声明
  `supports_dirty_region=false`；轻量测试 backend 同样没有 dirty-region 能力。
- 历史 Phase 3 source-level bag 资产当前不可用；既有无损快照即使恢复，也只能离线
  推导 added、removed、free/occupied flipped、dirty bounds 和 content hash，不能单独
  表达原生 revision 依赖、session/epoch、丢包、乱序或重同步。
- 现有中央 merger 内部会比较完整 source snapshot 并计算 added/removed/flipped；
  这可作为行为 oracle，但它仍接收完整 OctoMap，不能直接充当 C3 协议。
- 父级决策要求 C3 先于 C4 通信数据面。C3 负责地图更新语义消息和直接本地
  producer/receiver topic；C4 再负责 routed envelope、队列、优先级、背压、
  链路故障和传输恢复策略。

详细证据见 `research/current-c3-boundary.md`。

## 3. 功能需求

### R-01 后端无关逻辑模型

定义 ROS-free 的 source map update、接收端状态和 apply result。算法接口不得包含
`rclcpp`、ROS 生成消息或 `octomap::OcTree`。OctoMap 只能通过 adapter 接入。

C3 同时提供独立地图更新 ROS 接口和薄转换层，使单 source producer/receiver 能通过
直接本地 topic 运行。ROS 消息只表达 C3 已定义的语义，不把 ROS 类型反向渗入算法库。

### R-02 身份、几何与版本链

每个更新至少绑定：

- `vehicle_id`、mapper/source session；
- source-local map frame、geometry/resolution 及其稳定 fingerprint；
- `map_epoch`、`base_revision`、`new_revision`；
- update kind、schema/hash version 和 content hash；
- 生成该更新的确定性 source revision 元数据。

不同 session、map epoch、frame、geometry 或 fingerprint 的更新不得静默接到当前链。

### R-03 更新种类

契约必须区分：

- `Snapshot`：某个精确 revision 的完整只读状态表示，用于本机生成、测试和对比；
- `Keyframe`：可独立建立或替换接收端基线的完整更新；
- `Delta`：只包含 `base_revision -> new_revision` 的确定性变化；
- `Summary`：不携带可重建的完整 occupancy，只用于状态、诊断或调度提示，不能
  冒充 keyframe/delta 推进接收端 occupancy revision；
- `Remove` 或等价 tombstone：显式结束一个 source contribution，不用 freshness
  超时伪装删除。

Keyframe 默认按事件触发：新 mapper session/map epoch 的首个有效 revision、接受的
resync 请求、producer 丢失 delta base、delta 超出 configured bytes/changed-cell 上限，
或 delta chain 超过配置的最大长度。固定时间/revision 周期 keyframe 仅作为可选策略，
默认关闭，不能成为 late join 或 gap 恢复的唯一机制。

### R-04 Canonical delta 与内容哈希

Delta 至少能表达 known cell 的新增、删除以及 Free/Occupied 翻转，或提供语义等价
且可确定性应用的表示。相同输入必须产生相同 canonical ordering、版本化 serialization
和 content hash。哈希范围必须覆盖影响重建结果的身份、geometry、版本和 payload。

C2 backend 可能接受概率更新并推进 revision，但 canonical Free/Occupied/Unknown 结果
保持不变。此时允许 operations 为空但 `new_revision > base_revision` 的 revision-only
Delta；它必须推进 receiver revision、保持 content hash 不变，不能误写成 Summary 或 gap。

### R-05 原子应用与幂等

接收端在修改当前状态前完成 envelope、资源、版本、hash 和 payload 校验。任何失败
不得产生部分写入，不得推进 revision，也不得刷新 source freshness。

- 完全相同的重复更新幂等接受或忽略；
- 相同 identity/revision 但不同 hash 作为冲突拒绝；
- 旧 revision、未来 base、缺失 delta 或错误 epoch 拒绝；
- `Delta.base_revision` 必须严格等于接收端当前 revision；
- keyframe 必须按明确的 session/epoch 和替换规则提交，不能绕过 identity fence。

### R-06 Gap 检测与 resync 状态机

接收端必须明确区分 `Ready`、`NeedsKeyframe`/`ResyncRequired` 和不可恢复拒绝状态。
检测到 gap、epoch 切换、hash 冲突或本地状态损坏后，停止应用后续 delta，保留最后
合法状态并产生幂等 resync intent/correlation key。C3 定义逻辑状态与请求语义；C4
再把它路由到稀疏链路并增加重试、超时和链路故障处理。

C3 的单 source reference 闭环必须提供本机短 resync service。service response 只返回
accepted/rejected、correlation ID 和必要诊断，不携带大体积 keyframe；producer 接受请求
后仍通过 map-update topic 异步发布绑定 correlation ID 的 keyframe。重复请求必须幂等，
旧 session/epoch 或不合法目标 revision 必须拒绝。

### R-07 Epoch、pose reset 与 alignment

Pose/source reset 后的新 map epoch 不能延续旧 delta base。新 source-local map 可以在
没有 alignment 时生成 keyframe/delta；进入 shared view 的消费路径必须引用准确且已提交
的 alignment epoch/revision。alignment 变化或撤销不得把新 transform 套到旧 delta 链，
必须使旧共享贡献失效并要求在新 alignment 下使用 keyframe 重建。

C3 验证上述协议证据，不在本任务实现在线配准、EdgeAggregator 或最终 shared/global
view。

### R-08 资源边界

Snapshot、keyframe、delta、接收端 retained state、单次 changed-cell 数和 canonical
serialization bytes 必须有配置上限，并使用溢出安全的预检。超限更新原子拒绝，不能
以无界缓存等待未来补包。

### R-09 C2 接入与回放等价

生产者只能从一个精确 revision 的 C2 `MapReadTransaction` 生成更新。若目标 receipt
已 superseded，必须明确重试最新 revision 或丢弃，不能把不同 revision 的 metadata 与
occupancy 混合。

使用当前确定性 `ProfileScenario` 驱动真实 C2 mapper 产生的 exact-revision 数据集证明。
历史 Phase 3 bag 若恢复，作为额外兼容回归输入，不是 C3 生产功能的运行依赖：

- keyframe 重建与对应完整 snapshot 的 occupied/free/unknown 内容一致；
- 连续 delta 应用后的每个检查点与完整 snapshot oracle 一致；
- 重复、乱序、缺包、旧 session/epoch 和 hash corruption 不改变最后合法状态；
- gap 后通过 keyframe resync 恢复到正确 revision。

### R-10 单 source 可运行闭环

提供一条不经过 routed link 的参考闭环：

```text
C2 exact-revision MapReadTransaction
    -> C3 producer
    -> direct local map-update topic
    -> C3 reference receiver/applier
    -> reconstructed source-local occupancy view
```

闭环必须验证 ROS 转换、发布/接收、原子 apply、诊断和重建等价。它是 C4 的稳定
输入基线，并覆盖本机 resync service -> 异步 keyframe 恢复；不实现 Relay、route epoch、
链路背压或网络故障注入。

### R-11 Producer 调度与 C2 回调隔离

C3 producer 不得在 C2 的 observation/pose mutation callback 内执行全图 canonical
遍历、delta compare、hash 或大对象 serialization。生产路径使用有界异步 worker，至少
保留一个待处理 revision 和一个正在处理的 revision；超出能力时只合并或丢弃中间待处理
项，不丢失 producer 当前基线到最新可处理 revision 的语义。

被合并的 revision 必须生成 `base_revision -> new_revision` 合法 delta，或退回完整
keyframe；不能把非连续 revision 伪装成连续单步 delta。worker 的 pending、in-flight、
published、superseded、resync-required 状态和队列/字节上限必须可诊断。

地图更新流复制最终 occupancy 状态，不承担逐 observation 事件日志职责。合法 delta 的
`new_revision` 可以大于 `base_revision + 1`，但必须携带 revision span/coalesced count
或等价诊断；接收端只在当前 revision 严格等于 `base_revision` 时应用。中间 revision
的逐步审计由本机原始 observation/replay 负责，不能从合并 delta 反向伪造。

当前 `/local_map/octomap` 可视化 topic 作为 C2 兼容路径保留。C3 不得以关闭本机安全、
降低 freshness 门或改写 header stamp 的方式掩盖额外开销。

### R-12 性能与资源验收边界

C3 使用确定性、结构性和容量门作为阻塞验收，不设置本机无法可靠分辨的 `<10%`
CPU/延迟相对收益门。

阻塞门至少包括：

- C2 mutation callback 中不存在 C3 全图 diff/hash；
- bounded reference 场景持续 10 Hz 输入无 observation backlog；
- producer pending/in-flight、retained snapshot、payload 和 receiver state 严格有界；
- worker 落后只产生明确 revision 合并，输入停止后在配置 drain window 内收敛到 latest；
- 稀疏变化 fixture 的 delta bytes 确定性小于对应 keyframe；
- sanitizer/memcheck 无业务泄漏或非法访问。

generate/canonicalize/compare/hash/apply 的绝对延迟与 CPU、约 1.81M-cell expanding
capacity knee 和相对完整 OctoMap snapshot 的 CPU 差异只测量并报告。只有效应显著高于
本机约 30-50% 分辨率下限时才允许做方向性结论；首版不以 CPU 改善百分比阻塞正确性
基线。

### R-13 兼容与回滚

保留现有 C2 可视化 OctoMap 输出，并让当前 exact-revision replay/oracle 作为独立对照。
历史 Phase 3 bag 若恢复则增加兼容回归，不替代当前 revision-aware oracle。完整 keyframe
必须始终能够绕过 delta 优化恢复接收端；delta 失败不能要求降低本机安全或 freshness 门限。

## 4. 非功能约束

- ROS-free 算法库 + 薄 ROS adapter；单元测试直接覆盖算法库。
- 固定输入和 seed 下输出必须字节级或逻辑级确定，hash 不依赖容器遍历顺序。
- 生成和应用阶段分别记录可测耗时与字节/单元数量，但 C3 不提前宣称真实网络收益。
- 资源和异常路径优先 fail closed；不得通过重写 observation/header stamp 刷新数据。
- 对现有 C2 revision、epoch、pose reset 和 known-free 安全契约保持兼容。

## 5. 当前范围外

- routed envelope、跨机 resync 路由/重试、QoS 策略、优先级队列、背压、带宽整形和
  丢包/延迟注入（C4）；
- Relay、稀疏路由、EdgeAggregator 生产实现和 contributor manifest（C5）；
- 多 Region、allocator、task lease 和本机执行策略（C6/C7）；
- N=5 多进程、最终 shared/global view 与 RViz 总验收（C8）；
- 在线 alignment/回环闭合、真实网络压缩协议和无线性能声明；
- 用新的地图更新链替换 C2 本机安全所依赖的权威地图。

## 6. 验收标准

- [x] ROS-free update model 明确 Snapshot/Keyframe/Delta/Summary/Remove 的语义和禁止行为。
- [x] session、map epoch、geometry fingerprint、base/new revision 和版本化 hash 均有确定性校验。
- [x] canonical delta 可表达 added/removed/flipped，并在两个独立 backend fixture 上通过同一 conformance suite。
- [x] 三态内容未变化但 C2 revision 前进时生成 revision-only 空 delta，receiver 合法推进
  revision 且保持 content hash，不触发错误 gap/resync。
- [x] keyframe 可从空状态建立正确基线；连续 delta 在每个 checkpoint 与 snapshot oracle 等价。
- [x] 事件驱动 keyframe 覆盖新 session/epoch、resync、base 丢失、delta 超限和 chain
  超限；周期策略默认关闭且关闭后 late join 仍可通过 resync 恢复。
- [x] 重复更新幂等；乱序、gap、旧 epoch/session、错误 base、冲突 hash 和损坏 payload 原子拒绝。
- [x] gap/epoch/hash/local-state 故障进入明确 resync 状态，保留最后合法 revision；
  新 keyframe 可恢复。
- [ ] shared-view consumer 能按 committed alignment epoch/revision 准入，并在 alignment
  变化或撤销时失效旧贡献、通过新 keyframe 重建。
- [x] 本机短 resync service 对重复请求幂等，response 不携带地图 payload；keyframe 通过
  update topic 异步返回并绑定 correlation ID。
- [x] pose reset 后旧链不接收新 delta，新 epoch 的 source-local 更新不依赖 alignment 生成。
- [x] 所有 payload/state/changed-cell 资源上限和整数溢出路径有确定性拒绝测试。
- [x] C2 精确 revision transaction 接入不存在 metadata/content 混合，superseded receipt 有明确处理。
- [x] 当前确定性 C2 exact-revision replay 完成 snapshot -> keyframe/delta -> reconstructed
  snapshot 的逐 revision 等价验证；历史 Phase 3 bag 资产不可用但不是功能验收前提。
- [x] 单 source 直接 topic 闭环完成 C2 producer -> C3 receiver 的逐 revision 重建和诊断验证。
- [x] C2 mutation callback 不执行 C3 全图 diff/hash；有界 worker 在 pending/in-flight/superseded
  和 producer lag 场景下保持安全、可恢复且无无界增长。
- [x] 跨 revision 合并 delta 能从 receiver 当前 base 直接重建 latest revision，并明确报告
  revision span；缺失中间 revision 不被误判为协议 gap。
- [x] 现有 C2 可视化 OctoMap、C2 单元/集成测试和当前 exact-revision oracle 不发生行为回归。
- [x] 生成、canonicalize/hash、apply 阶段已暴露独立指标，不宣称网络收益。
- [x] 失败时可关闭 delta 生产/消费并退回完整 keyframe 对照路径，不污染最后合法状态。
- [x] bounded reference 场景 10 Hz 无 observation backlog，停止输入后在 drain window 内
  收敛 latest revision；所有 worker/retained/payload/receiver 状态满足配置上限。
- [x] 在冻结版本上完成 expanding capacity、CPU、绝对延迟和 ASan/LSan/Memcheck 专项验收，
  按测量边界报告且不使用低于环境分辨率的小效应门。

## 7. 已确认决策

### D-01 Delta 生成策略：canonical snapshot comparison

当前两个 C2 backend 都没有 dirty-region 能力。C3 首版需要决定：

- 方案 A：先通过相邻 revision 的 canonical snapshot 比较生成真实 delta，建立正确性
  和恢复基线；后续允许支持 dirty-region 的 backend 使用优化路径，但必须输出完全
  等价的逻辑更新。
- 方案 B：C3 同时修改 OctoMap backend，直接记录每次 mutation 的原生 dirty set，
  首版即避免 O(n) snapshot 比较。

`Accepted(D-01)`：采用方案 A。它只依赖 C2 已稳定的 revision-locked view，让 OctoMap
和轻量 backend 共用同一协议与 conformance suite。首版明确接受 O(n) canonical snapshot
比较和上一 revision 状态保留成本，必须分别测量 generate/compare/hash/apply，但不能提前
宣称 CPU 优化。

后续先优化 C3 自身的排序、哈希、内存复用和 apply；只有证据证明 O(n) 比较仍是瓶颈时，
才以独立优化任务为支持 `supports_dirty_region` 的 backend 增加原子 change journal/dirty set，
并让 C3 producer 使用可选快速路径。快速路径必须逐 revision 与方案 A 输出等价；不支持该
能力的 backend 继续走方案 A。方案 A 永久保留为兼容路径、正确性 oracle 和回滚路径。

### D-02 交付深度：单 source 可运行闭环

`Accepted(D-02)`：C3 不停留在算法库或离线 replay。首版同时交付 ROS-free producer/
applier、地图更新语义消息、薄 ROS adapter，以及直接本地 topic 上的单 source reference
receiver，端到端验证 C2 exact revision 到 reconstructed occupancy view。

C3 的 ROS 边界承载地图更新语义与本机短 resync service；C4 再增加 routed envelope、
跨机 resync 路由/重试、QoS/队列/背压和链路故障恢复，C5 再引入
Relay/EdgeAggregator。这样能在进入通信复杂度前暴露消息转换和 apply 边界问题，
同时不把直接 topic 冒充稀疏网络验收。

### D-03 Producer 调度：异步、有界、可合并

`Accepted(D-03)`：C3 producer 使用有界异步 worker 与单槽/等价有限队列，不在 C2
mutation callback 内执行全图 diff、hash 或大对象 serialization。中间 revision 可以
被合并；合并后必须以 `base_revision -> new_revision` 生成合法 delta，无法证明基线连续
时必须发送 keyframe 或进入 resync-required。worker 的 pending/in-flight/published/
superseded/lag 状态进入诊断，队列和字节上限超出时 fail closed，不得无界缓存。

现有 C2 可视化 `/local_map/octomap` 保留为兼容路径；C3 首版不以改变安全门、freshness
或 header stamp 来隐藏额外工作。后续若性能证据证明重复 materialization 成为瓶颈，再
单独评审“共享一次 canonical snapshot，同时服务可视化和 C3”的联合优化。

### D-04 Revision 合并：允许 base 到 latest 的状态 delta

`Accepted(D-04)`：地图更新是状态复制流，不是逐 observation 事件日志。异步 producer
允许把多个尚未发布的 C2 revision 合并成一个 `base_revision -> latest_revision` delta；
消息必须记录 revision span/coalesced count 或等价诊断，不能声称包含中间 revision 的
逐步历史。接收端只校验自己的当前 revision 是否等于 base，不要求 `new = base + 1`。

若 producer 已失去用于生成该跨度的合法 base，必须发送 keyframe 或进入 resync-required，
不得猜测中间状态。逐 revision/逐 observation 审计继续依赖本机输入记录和确定性 replay。

### D-05 Resync API：本机短 service + 异步 keyframe

`Accepted(D-05)`：C3 单 source 闭环提供本机短 resync service。receiver 检测 gap、epoch
切换、hash 冲突或本地状态损坏后保留最后合法地图并进入 `ResyncRequired`，以幂等
correlation ID 请求 keyframe。service 只确认接受/拒绝和返回诊断，不在 response 中传输
地图；producer 通过原 map-update topic 异步返回 keyframe，receiver 原子替换基线后恢复
`Ready`。

C4 复用同一领域 intent/correlation 语义，把请求和 keyframe 流路由到稀疏链路并增加
重试、TTL、背压和故障注入；纯 Relay 不解释 service 或 keyframe 业务语义。

### D-06 Keyframe 策略：事件驱动，周期策略默认关闭

`Accepted(D-06)`：C3 首版不依赖固定周期全量发布。新 mapper session/map epoch 的
首个有效 revision、已接受 resync、producer base 丢失、delta bytes/changed-cell 超限，
以及 delta chain 长度超限时生成 keyframe。周期 keyframe 作为可选配置保留但默认关闭；
late join 和 gap 恢复必须通过 `ResyncRequired -> service -> async keyframe` 流程成立。

该策略避免用周期性 O(n) 全量遍历掩盖恢复协议缺口。C4 后续可以依据链路预算配置
周期或主动 keyframe，但不得改变 C3 的 identity/epoch/revision 和原子替换语义。

### D-07 性能门：确定性/有界性阻塞，CPU 小效应只报告

`Accepted(D-07)`：C3 的阻塞性能门是 callback 隔离、10 Hz bounded 场景无 backlog、
有界 worker/retained/payload/receiver 状态、drain 后收敛 latest revision、稀疏 delta bytes
小于 keyframe，以及 sanitizer/memcheck 安全。方案 A 的 generate/compare/hash/apply CPU
与绝对延迟、1.81M-cell capacity knee 和相对完整 snapshot 的 CPU 差异只测量并报告。

本机 CPU/延迟跨轮漂移约 30-46%，可分辨效应下限约 30-50%；禁止用 `<10%` CPU
A/B 或类似伪精确阈值阻塞 C3。若方案 A 无法满足结构性容量门，再回到 D-01 的 native
dirty-region 独立优化评审，而不是降低安全或 freshness 契约。

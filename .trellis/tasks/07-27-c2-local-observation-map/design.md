# C2 本机观测消费与局部地图技术设计

## 1. 设计目标

在单个 vehicle-local compute domain 内，把 C1 的标准化 observation、pose 和
health 转换为一条权威 local occupancy revision 链。核心算法保持 ROS-free；
OctoMap 是默认后端，不能成为上层地图契约。

本设计只完成 source-local map。本机建图和本机安全不依赖 fleet、shared frame
或 alignment Ready。C3 才定义地图内容的跨进程 keyframe/delta 协议。

## 2. 关键决策

### D-C2-001：新增独立包

新增 `ws/src/alien_perception/perception_local_map`：

```text
perception_local_map/
├── include/perception_local_map/  # ROS-free public API
├── src/                            # engine + OctoMap backend
├── src/PerceptionLocalMapNode.cpp  # thin rclcpp binding
├── test/                           # gtest + launch_testing
├── launch/                         # XML public/debug launch
└── config/                         # mapper parameters + RViz
```

包内分为 ROS-free `perception_local_map_core`、ROS-free
`perception_local_map_octomap` 和薄节点 `perception_local_map_node` 三个 target，
并导出 package-scoped library targets。节点是唯一接触 rclcpp、ROS messages、
QoS、TF 和 `octomap_msgs` 的位置。

不把新实现放回 `swarm_controller`：旧包同时拥有全局 merger、探索和历史
OctoMap 类型，继续扩展会让本机感知边界与 fleet 逻辑重新耦合。

### D-C2-002：接口仅位于真实替换点

定义只读 `IOccupancyMapView` 与写入侧 `ILocalOccupancyBackend`，因为：

- 父架构要求公共地图契约后端无关；
- 默认 OctoMap 与轻量确定性 backend fixture 必须通过同一 conformance suite；
- 后续后端只能在启动时选择，engine 不依赖具体树类型。

不为 `LocalObservationMapper` 自身创建接口。它是被直接测试的具体领域对象。

### D-C2-003：capability 混合表达

- `Perception::RayEvidenceCapability` 继续表达
  `HitOnly < HitRay < FullRay`，不复制枚举。
- `BackendCapabilities` 使用具名字段表达独立能力：point query、bounded-region
  query、hierarchy、dirty region、serialization 等。
- `EffectiveMapCapabilities` 使用具名字段表达当前可用 sensor/pose/free-ray
  能力；它保留 `maximum_active_ray_evidence`、satisfied-combination ID，以及按
  2D/3D 和 HitOnly/HitRay/FullRay 的 active counts。maximum 只描述 active input
  上界，不代表所有 sensor 或整张地图都具有该等级。
- satisfied-combination ID 在 frozen contract 内确定：完整 minimum contract 为
  `minimum`，degraded combinations 按声明优先级为 `degraded/<zero-based-index>`，
  Unavailable 为空。ID 必须与 contract schema version/fingerprint 一起解释，不使用
  可变的诊断 description 作为 identity。
- 不使用 bitmask，不构造所有组合的笛卡尔积枚举。未知枚举、未知 backend
  feature 或矛盾组合一律 fail closed。

### D-C2-004：本机地图元数据与地图内容分离

ROS-free `LocalMapState` 至少包含：

```text
vehicle_id / mapper_session
source_local_map_frame
map_epoch / revision
health: Healthy / Degraded / Unavailable
effective capabilities
mapper contract fingerprint / satisfied-combination ID
last accepted observation origin stamp
last commit provenance: sensor ID/session + observation stamp
last commit monotonic time / freshness deadline
backend capabilities
alignment reference: optional, epoch/revision/status only
```

地图内容通过 `ILocalOccupancyBackend` 查询；状态对象不持有
`octomap::OcTree`。revision 仅在原子接受了至少一项合法 occupancy evidence 后
递增；timer、拒绝、相同 `(sensor,session,origin_stamp)` 的传输重放和仅有
`Invalid` 的 batch 不推进 revision，也不刷新 last-commit freshness。更高 stamp 的
新 batch 即使产生相同 occupancy evidence 或 log-odds 已饱和，仍算一次 accepted
commit：它推进 revision、刷新 freshness，并把 `changed_cell_count=0` 与 NoEvidence
区分。

## 3. ROS-free 类型与职责

下列类型名可在实现审核时微调，但职责不可合并回 ROS 节点。

### 3.1 基础地图类型

- `OccupancyState { Unknown, Free, Occupied }`
- `MapPoint`、`VoxelIndex`、`AxisAlignedBounds`、`MapQueryResult`
- `OccupancyCell { VoxelIndex, center, state }`
- `MapEpoch`、`MapRevision`：底层为 `uint64_t`，revision 在单个 epoch 内单调。
- `MapGeometry { resolution_m, lattice_origin, frame_id }`

bounded-region 查询返回按 `VoxelIndex(x,y,z)` 字典序排序的已知 cells；区域内未
返回的格子仍为 unknown。这样 conformance/replay 不依赖 OctoMap 遍历顺序。

canonical lattice 对每个轴使用
`index = floor((coordinate - lattice_origin) / resolution)`；负坐标必须使用数学
floor，不得向零截断。cell `i` 覆盖半开区间
`[origin + i*r, origin + (i+1)*r)`，region 同样使用 `[min,max)`；边界点属于右侧
cell。非有限、`min >= max` 或 backend key range 之外返回明确 Invalid/OutOfRange，
不能伪装成 Unknown。

`known_bounds` 是当前 revision 中全部已知 Free/Occupied cells 的最小 lattice-aligned
AABB，使用 `[min,max)`；空图为 `std::nullopt`。它不是配置的最大建图范围，也不能把
bounds 内未返回的 cell 从 Unknown 推断成 Free。

### 3.2 Backend API

`IOccupancyMapView` 只暴露 geometry/capabilities、point query 和 bounded-region
query，本机 safety/frontier 等消费者不得取得 reset/apply 权限。

`ILocalOccupancyBackend` 继承只读 view，并增加最小写入职责：

```cpp
virtual MapGeometry geometry() const noexcept = 0;
virtual BackendCapabilities capabilities() const noexcept = 0;
virtual std::optional<AxisAlignedBounds> known_bounds() const = 0;
virtual void reset(const MapGeometry& geometry) = 0;
virtual ApplyResult apply(const EvidenceBatch& batch) = 0;
virtual MapQueryResult query(const MapPoint& point) const = 0;
virtual RegionQueryResult query_region(
    const AxisAlignedBounds& bounds) const = 0;
virtual ~ILocalOccupancyBackend() = default;
```

`EvidenceBatch` 已完成全部 provenance、frame、pose 和 ray capability 校验；backend
只执行 voxel evidence。ordinary Rejected/NoEvidence 具有 strong no-change guarantee。
`ApplyResult` 明确 Applied/NoEvidence/Rejected，并携带
`changed_cell_count`。Applied 表示合法 evidence 已被消费，即使 cell 分类或
log-odds 饱和后没有可见变化；NoEvidence 只表示 batch 没有可消费证据。普通输入
拒绝不用异常表达。构造参数错误或 backend 不变量破坏可以抛异常，由节点启动或
callback 边界捕获并进入 Unavailable。

OctoMap 不要求每批复制整棵树。backend 先完成所有可预见校验/key staging，再进入
mutation；若 mutation 期间仍抛异常或无法证明无部分写入，mapper 立即销毁该
backend 实例、关闭 revision 链并推进新 map epoch。mapper 不向消费者返回
`IOccupancyMapView`、concrete backend、裸指针或引用；该 interface 只供 backend
conformance 和 mapper 内部调用。消费者只能通过下述 transaction 的值返回方法查询。
backend factory 在恢复门后创建空实例，旧 revision 永不重新暴露。

跨多次 query 的一致读取使用 mapper-owned、不可复制的 `MapReadTransaction`：

```text
identity: vehicle/source + mapper session
map_epoch / revision
geometry / optional known_bounds
query(point) -> MapQueryResult by value
query_region(bounds) -> RegionQueryResult by value
for_each_known_cell(visitor(OccupancyCell by value))
close() -> idempotent revoke
```

`LocalObservationMapper::acquire_read_transaction()` 在 mapper shared mutex 下捕获上述
身份并持有 shared read lock；apply/reset/backend fault 使用同一 mutex 的 exclusive
lock。因此 transaction 生命周期内 revision、geometry、bounds 和全部 query 结果保持
一致，不复制整棵 OctoMap。`MapReadTransaction` 是 move-only RAII value，内部 control
block 持有 lock/liveness token，但 public API 不暴露 backend/view 引用；`close()`、
move-from 或 token revoke 后，其所有 query/visit 均稳定返回 TransactionClosed/
Unavailable，不触发悬空引用或 UB。C3 adapter 与 OctoMap snapshot adapter 都必须在
一次 transaction 内完成一个 revision 的遍历、序列化或 content-hash 输入收集。实现
对 transaction 持有时间做诊断/测试预算，业务 callback 不得跨等待、网络 I/O 或
executor spin 持有它。backend fault 获得 exclusive lock 前会等待已有 reader 退出；
一旦 fault 被提交，旧 epoch/revision token 的新 transaction 立即拒绝，已关闭或
move-from 的旧 handle 也不能复活。

public `for_each_known_cell` 给 visitor 的是 cell value，不是 backend leaf 引用。
package-private OctoMap snapshot bridge 是 transaction 的 friend/内部操作：它在 read
lock 内直接生成完整 `octomap_msgs` value 后返回，绝不把 `octomap::OcTree&` 交给节点
或保存在 callback 外。compile/API tests 使用 type traits 与 public-header consumer
证明 transaction 不存在返回 backend/view/raw-reference 的 accessor。

Applied commit 返回不可伪造的 `CommitReceipt {mapper_session,map_epoch,revision}`。
snapshot adapter 使用 `acquire_read_transaction(receipt)` 请求 exact revision；commit 与
acquire 之间若已有其他 callback 推进 revision，调用返回 Superseded，绝不能把 receipt
的旧 revision 标签配到当前新内容。snapshot publisher 可以合并中间 revision，并用
最新 receipt 重试一次/排队到单一 snapshot worker，但只允许发布 transaction identity
与 receipt 完全相等的内容；同一 mapper 的 apply、reset、transaction acquisition 和
pending-snapshot bookkeeping 受同一锁/串行化边界保护。这样不依赖 ROS executor 恰好
单线程，也不要求为每个中间 commit 复制或发布整棵图。

这是同进程 C++ 扩展点：后续 C3 `LocalMapUpdate` producer 必须链接 core 并与 mapper
engine 共进程/组件内通过 transaction 读取，然后才把 update 跨进程发布；不得在另一
进程把 `local_map/state` 与 visualization-only `local_map/octomap` 按时间戳猜配成
authoritative delta。C2 本轮只实现 transaction 和 OctoMap snapshot adapter，不实现
C3 update producer/protocol。

`OctoMapBackend` 是默认实现，依赖非 ROS 的 `octomap` 库。测试目录提供轻量
`DeterministicVoxelBackend`，只用于 conformance 和 engine 测试，不形成第二条
运行期权威地图。OctoMap concrete header 使用 Pimpl 或保持为 package-private；
安装的 core public headers 不得 include OctoMap。

### 3.3 LocalObservationMapper

具体类拥有唯一 active backend、backend factory、冻结 contract/inventory 和
`SensorExtrinsicRegistry`，负责：

1. 使用 C1 `MapperHealthGate`、相同 `MapperInputContract` 和 descriptor inventory
   自行计算 active mapper health；
2. 验证 C1 health 的 producer session 与 canonical contract fingerprint，并只把
   它作为不可抬高的上游 health ceiling；
3. 保存有界 pose history、active/retired pose lineage 与 high-water fences；
4. 按 sensor ID/session 保存健康、外参和最后 origin stamp；
5. 将 observation 与不晚于 acquisition stamp 的最近 pose 配对；
6. 在 ROS-free registry 中冻结/比较 `body <- sensor`，再组合 `map <- sensor`；
7. 生成 `EvidenceBatch` 并调用 backend；
8. 使用注入的 monotonic `now` 原子推进或到期
   state 的 epoch/revision/freshness/capability；
9. 处理 pose discontinuity、backend fault、恢复稳定门和 alignment invalidation。

算法类不查 TF、不订阅消息、不读取 ROS 参数、不发布 OctoMap。

## 4. 输入与坐标数据流

```text
perception_interfaces/LidarObservation ─┐
perception_interfaces/PoseEstimate ─────┼─ PerceptionLocalMapNode
perception_interfaces/HealthState ──────┘        │ explicit decode
TF body <- sensor sample ─────────────────────────┤
                                                  ▼
          Perception::LidarObservation / PoseEstimate / InputHealth
                                                  │
                                                  ▼
                       LocalObservationMapper + SensorExtrinsicRegistry
                       map <- body * frozen(body <- sensor)
                                                  │ EvidenceBatch
                                                  ▼
                                      ILocalOccupancyBackend
                                                  │
                     LocalMapState + query view + optional OctoMap snapshot
```

### 4.1 Pose 配对

- `source_local_map_frame` 和 `body_frame` 是启动时冻结参数。
- pose `frame_id` 必须等于 source-local map frame。
- observation 与 pose 的 `clock_domain` 必须相同；首版没有显式 clock-domain
  converter，不同 domain 不能仅凭数值 stamp 配对。
- observation 使用 acquisition stamp；选择 stamp 不晚于 observation 且年龄不超过
  `max_pose_age` 的最近 pose。首版不使用未来 pose、不插值、不做 beam deskew。
- pose history 有固定样本数和时间窗口，超限确定性淘汰最旧项。
- observation 比对应 sensor 的最后已接受 stamp 更旧或相等时拒绝；不同 sensor
  之间不建立伪全序，因此相同 stamp 可分别提交。
- 每个 ROS callback 记录本机 steady receive time；pose、health 和 map freshness
  以 hop-local monotonic validity budget 计算，不能把最后一次 wire
  `freshness_ns` 当作持续时钟。origin stamp 只用于溯源与同 clock-domain 配对。

### 4.2 Sensor 外参

- observation `frame_id` 是每批权威 frame；node 只在 acquisition stamp 查询 TF、
  转换为普通 `SensorExtrinsicSample` 并交给 engine。
- ROS-free `SensorExtrinsicRegistry` 验证有限性、四元数归一化/可逆性、sensor/frame
  identity 与配置容差；同一 sensor ID 首次成功后冻结外参。后续 drift/frame 改变
  由 engine 拒绝，需要新 vehicle session 才能改变 inventory。
- 禁止直接查询 `map <- sensor` TF 来绕过 `PoseEstimate` 的 session、quality、
  freshness 和 reset epoch 门。

## 5. 射线证据到 occupancy 的映射

所有 ROS wire enum 在节点用闭合 `switch` 解码；禁止直接 `static_cast`。

### 5.1 Scan2D

| Capability | Return kind | Free evidence | Occupied evidence |
| --- | --- | --- | --- |
| HitOnly | Hit | 无 | hit endpoint |
| HitOnly | NoReturn/Invalid | 无 | 无 |
| HitRay | Hit | origin 到 endpoint，不含 endpoint | hit endpoint |
| HitRay | NoReturn/Invalid | 无 | 无 |
| FullRay | Hit | origin 到 endpoint，不含 endpoint | hit endpoint |
| FullRay | NoReturn | origin 到 `range_max` boundary，不含 boundary endpoint voxel | 无 |
| FullRay | Invalid | 无 | 无 |

有限但小于 `range_min`、大于 `range_max`、NaN、负无穷均保持 Invalid；不得按
no-return 消费。有限值恰好等于 min/max 按 C1 `return_kind()` 仍是 Hit。
NoReturn 的最大量程边界 voxel 保持 Unknown，与旧 OctoMap oracle 的
`computeRayKeys()` endpoint 语义一致；不能把“未命中”改写为边界处 free endpoint。

### 5.2 Cloud3D

当前公共 schema 只接受 `HitOnly`。每个有限 XYZ 点产生 occupied endpoint，不产生
origin-to-point free evidence。声明 `HitRay/FullRay`、非有限点或 cross-type residue
拒绝完整 batch。

### 5.3 同批冲突与提交

- 先完成完整 batch 校验和 key 计算，再修改 backend。
- 同一 batch 内 occupied key 胜过 free key，保持旧 builder 的安全语义。
- 合法但没有任何 evidence 的 batch 返回 NoEvidence，不推进 revision/freshness；
  相同 `(sensor, session, origin_stamp)` 的传输重放在 apply 前拒绝。更高 stamp 的
  新 batch 即使产生相同 evidence、`changed_cell_count == 0`，仍返回 Applied，
  推进一次 revision 并刷新 last-real-commit freshness。
- backend Applied 后才提交 revision；ordinary Rejected/NoEvidence 保证旧 map 逐
  cell 不变。backend exception/fault 不宣称保留旧 revision，而是销毁实例、封禁
  view、关闭链并推进新 epoch，不能发布或查询半成功/旧 revision。

## 6. 健康与 capability 状态机

### 6.1 状态

- `Healthy`：usable pose 和启动时声明的完整 sensor contract 满足。
- `Degraded`：usable pose 仍成立，且当前输入满足一个显式 degraded combination；
  最低允许一台健康 LiDAR + `HitOnly`。
- `Unavailable`：pose 不可用、没有任何允许的 sensor combination、frame/session
  不变量破坏或 backend 失败。

alignment/fleet 状态不参与本机 health 计算。

C1/C2 共同复用 `perception_core::MapperHealthGate` 和 canonical
`MapperContractFingerprint`。fingerprint 覆盖排序后的 frozen descriptor inventory、
minimum contract、degraded combinations、pose requirements 与 recovery samples；
canonical serialization 使用显式 schema version、字段名、长度前缀和固定字节序，
禁止依赖 YAML 文本、容器遍历顺序或实现相关 `std::hash`；wire 使用小写十六进制
SHA-256。fingerprint 只覆盖影响判定的语义字段，不覆盖 degraded combination 的
自由文本诊断 description。descriptor inventory、单个 combination 内的 requirements
和 specific-sensor 集合使用 canonical 排序，但 degraded combination 的声明优先级是
gate 语义，必须按原顺序进入 fingerprint，不能把有意义的重排规范化成相同摘要。
fingerprint 用于检测配置漂移，不作为认证/签名。

C2 是 active mapper health 的唯一权威判定者：它从冻结 inventory、每 sensor
receive health 和 pose 运行同一个 gate。C1 `HealthState.msg` 增加 producer
source/session 与 contract fingerprint，只作为不可抬高的 upstream ceiling：
fingerprint 不同、旧/空 session、receive-age 过期或上游 Unavailable 都使最终结果
Unavailable；上游 Degraded 最多允许最终 Degraded，绝不能提升 C2 gate 结果。

C2 的每 sensor receive health 由该 sensor 通过完整 wire/payload/session 校验的最近
observation 及本机 monotonic receive age 推导；拒绝的 batch 不激活 sensor，也不刷新
其 age。C1 aggregate health 不用于猜测具体 sensor 状态；它只在上述本地 gate 结果上
取更保守等级。因此 `specific_sensors`、combination ID 和 capability counts 均来自 C2
自己的 frozen inventory + receive-health 判定，可由确定性 observation fixture 验证。
若本地 gate 满足 `minimum` 但 upstream ceiling 把最终状态降为 Degraded，combination
ID 仍为 `minimum`，准确说明本地实际满足的组合；最终 health 单独表达上游降级。

### 6.2 推进规则

- Healthy/Degraded 可接受 batch 并推进 revision。
- Degraded 发布 satisfied-combination ID、按 sensor type/evidence level 的 counts
  和 `maximum_active_ray_evidence`。maximum 不能授予全局 known-free；消费者仍须
  查询实际 path/region 的 occupied/free/unknown。
- Unavailable 拒绝新提交；输入/pose stale 时旧地图仍可查询但 freshness 按 last
  real commit 到期。BackendFault 时旧 view 被封禁，query 返回 Unavailable。
- 故障立即生效；恢复沿用 C1 的 `recovery_stability_samples` 语义。恢复门成功前不
  接受新 revision。
- `tick(monotonic_now)` 可以使 health/map freshness 到期并发布状态变化，但不得
  修改 map epoch/revision、last observation provenance 或 last-commit time。

## 7. Pose discontinuity、map epoch 与 alignment

只有以下已获当前 health/contract 授权的 lineage 事件关闭当前链：

- 新 pose source/session 被接受；
- reset epoch 严格前进；
- 与新 lineage 一致的 frame 变化；
- 位置或姿态跳变超过阈值，并创建新的内部 continuity generation。

重复/non-increasing stamp、retired session、reset epoch 回退和旧 DDS replay 只拒绝，
不推进 map epoch。mapper 保存 active lineage、retired `(source,session,reset,frame)`
集合和每 lineage high-water stamp；同一旧事件无法重复触发 reset。same-session frame
改变若没有 reset 前进只视为协议错误并保持 fail closed，不擅自接受新坐标系。

处理顺序固定：

1. 标记当前 revision 链 closed；
2. 立即撤销绑定旧 map epoch 的 alignment reference；
3. 将旧 lineage 写入 retired fence，清空 backend 和 pose history；
4. 推进 map epoch，并把 revision 置 0；
5. 状态进入 Unavailable/Recovering；
6. 新 pose 通过稳定门后，本机地图即可接受新 observation；
7. 新 epoch 的 alignment Committed 只解锁未来 shared-frame consumer。

首版不保留可继续写的旧 backend、不重投影旧体素、不在 C2 聚合 shared map。

sensor producer session 更新不自动推进 map epoch：同一冻结 sensor ID/frame 的新
producer session 可重连，C2 记录新 session 并拒绝随后到达的旧 session batch。
只有 pose 坐标连续性变化才改变本机地图的空间 epoch。

`AlignmentReference` 记录 map source/session/epoch、provider ID/session、alignment
epoch/revision 和 Candidate/Committed/Degraded/Revoked 状态。ROS-free
`submit_alignment(reference)` 只接受当前 map lineage、合法 provider session 和
单调 revision；旧/回退/错 source reference 拒绝。C2 不应用 transform，也不新增
alignment ROS topic；fixture 直接调用该 API 验证 Committed 以及 reset 后 Revoked。

## 8. ROS binding 与输出

### 8.1 输入

默认相对 topic：

- `perception/observations`
- `perception/pose`
- `perception/health`

不订阅 `scan_returns`、raw `LaserScan`、raw `PointCloud2` 或 Odometry；这些均属于
C1 producer/adapters。

### 8.2 输出

- `local_map/state`：新增 `perception_interfaces/msg/LocalMapState.msg`，发布 map
  identity、epoch/revision、health/freshness、last-commit provenance 和 capability
  摘要，不含地图 cells。
- `local_map/octomap`：默认 OctoMap backend 的标准
  `octomap_msgs/msg/Octomap` 兼容/可视化快照，transient-local。
- `/diagnostics`：拒绝原因、reset、backend error 和 freshness 状态。

`LocalMapState` 在状态改变、真实 revision 提交或固定 `state_heartbeat_period` 到期时
发布；heartbeat 只推进独立 `state_sequence` 并重新计算剩余 budget，不改写 origin
stamp、map epoch/revision、last-commit time 或 map freshness。OctoMap 只在真实 map
commit 后通过同一 revision read transaction 发布。heartbeat/tick 由本机 steady timer
驱动，不受 ROS `/clock` 暂停或回退影响；header emission stamp 只用于诊断，不参与
state 顺序或 monotonic deadline 判定。

标准 `octomap_msgs/Octomap` 不携带 C2 revision，因此它仅用于 RViz/legacy inspection，
不能与另一个 topic 的 state 猜配为原子内容。节点内部仍记录 snapshot receipt 并测试
其与 transaction 相等；未来 C3 update 在同进程 transaction 内显式携带 revision。

`LocalMapState` 是状态摘要，不是 C3 `LocalMapUpdate`，不能携带 keyframe/delta 或
被 fleet merger 当作地图内容。

首版 `HealthState.msg` 增加：

```text
string producer_source_id
uint64 producer_session_boot_time_ns
uint32 producer_session_random_suffix
uint32 mapper_contract_schema_version
string mapper_contract_fingerprint       # lowercase SHA-256 hex
```

C1 在启动时对实际冻结的 descriptor inventory 与 mapper contract 计算一次 fingerprint，
后续每条 health 都携带同一 version/value。version 不受支持、空值、非 64 位小写
十六进制或与 C2 本地冻结值不相等均 fail closed。现有 aggregate state/capability
字段继续保留用于诊断与 ceiling，但不能代替 C2 的 per-sensor receive-health 授权。
该 schema 是 C1/C2 共享 wire 契约，不允许节点各自拼接字符串生成摘要。

首版 `LocalMapState.msg` 字段固定为：

```text
uint8 STATE_HEALTHY = 0
uint8 STATE_DEGRADED = 1
uint8 STATE_UNAVAILABLE = 2
uint8 RAY_EVIDENCE_HIT_ONLY = 0
uint8 RAY_EVIDENCE_HIT_RAY = 1
uint8 RAY_EVIDENCE_FULL_RAY = 2

std_msgs/Header header                 # emission stamp + source-local map frame
string vehicle_id
uint64 mapper_session_boot_time_ns
uint32 mapper_session_random_suffix
uint64 state_sequence                  # strictly increasing per mapper session
uint64 map_epoch
uint64 revision
uint8 health                           # closed Healthy/Degraded/Unavailable
bool map_fresh
int64 validity_remaining_ns            # emission-time hop-local budget; >= 0
float64 resolution_m
geometry_msgs/Point lattice_origin
bool has_known_bounds
geometry_msgs/Point known_bounds_min   # valid only when has_known_bounds
geometry_msgs/Point known_bounds_max   # exclusive; valid only when has_known_bounds
uint32 mapper_contract_schema_version
string mapper_contract_fingerprint     # lowercase SHA-256 hex
string satisfied_combination_id        # empty while Unavailable/no combination
bool has_active_ray_evidence
uint8 maximum_active_ray_evidence      # closed C1 enum; active-input maximum only
uint32 active_sensor_count
uint32 active_2d_hit_only_sensor_count
uint32 active_2d_hit_ray_sensor_count
uint32 active_2d_full_ray_sensor_count
uint32 active_3d_hit_only_sensor_count
uint32 active_3d_hit_ray_sensor_count
uint32 active_3d_full_ray_sensor_count
string last_sensor_id
uint64 last_sensor_session_boot_time_ns
uint32 last_sensor_session_random_suffix
builtin_interfaces/Time last_observation_stamp
string last_observation_clock_domain
uint32 last_commit_changed_cell_count
bool supports_point_query
bool supports_bounded_region_query
bool supports_hierarchy
bool supports_dirty_region
bool supports_serialization
bool has_committed_alignment
uint64 alignment_epoch
uint64 alignment_revision
```

六个 capability count 按 sensor 的声明类型和当前有效证据等级做互斥计数，不做
“至少该等级”的累加；其和必须等于 `active_sensor_count`，否则消费者 fail closed。
当前 C1 Cloud3D 只能使 3D HitOnly 非零，但保留完整字段使 closed schema 可验证，不能
借此接受 C1 已禁止的 3D HitRay/FullRay payload。

`active_sensor_count == 0` 时 `has_active_ray_evidence=false`，maximum 字段使用合法但
不授权任何能力的 `RAY_EVIDENCE_HIT_ONLY` sentinel；消费者必须先检查 bool，不能仅凭
enum 推断能力。count 大于零时 bool 必须为 true 且 maximum 等于六个互斥 counts 中
实际存在的最高等级。空图使用 `has_known_bounds=false`，min/max 为零 sentinel；非空图
bounds 必须对齐 canonical lattice 且使用 `[min,max)`。

`validity_remaining_ns` 是发布时刻的剩余 hop-local budget，不是 steady clock age 或
deadline；`map_fresh == false` 时固定为 0。接收端在消息通过 identity/session/revision
校验后，以本机 monotonic receive time 加该 budget 重建本 hop 的 deadline，并受本地
transport timeout 进一步收紧；任何进程都不得传播或比较另一个进程的 steady time
point。所有计数/时长执行有界转换；未发生 commit 时 provenance 字段使用明确零/空
sentinel，`revision == 0`。unknown health/ray/alignment wire value 在消费者处 fail
closed。

state topic 固定为 `KeepLast(1) + reliable + volatile`，禁止 transient-local 缓存；
DDS lifespan 不大于 `state_heartbeat_period`，deadline 为 heartbeat 的明确容差倍数。
late subscriber 等待下一次 heartbeat，不读取历史缓存。消费者对同一 mapper session
只接受严格递增 `state_sequence`，低序、重复、超过 transport timeout/lifespan 的样本
拒绝；首次见到某 mapper session 时至少观察到第二个递增 sequence 才允许把
`map_fresh=true` 用作 safety/shared-ready 证据，故障/Unavailable 则立即生效。publisher
崩溃后不会产生新 heartbeat，接收端按本机 deadline/remaining budget 到期，不能用最后
样本自动续期。传输延迟的最坏保守误差由 DDS lifespan 限定，不宣称跨主机硬实时时钟
同步。

`LocalMapState` 负责 C2 的 frame、resolution、lattice origin、epoch/revision 与运行
状态元数据和 current known bounds，不携带 changed keys、tile bounds、content hash 或
update kind。C3 通过 `MapReadTransaction` 对一个已锁定 revision 生成
keyframe/delta/summary，计算并发布 canonical content hash，并负责 base/new revision
与 dirty payload；不得把 C2 state 当作地图内容更新。

### 8.3 OctoMap 适配隔离

package-private OctoMap snapshot bridge 在有效 `MapReadTransaction` 内调用
`octomap_msgs::fullMapToMsg()` 并只返回 materialized message value。该 backend-specific
bridge 不进入 `IOccupancyMapView`/`ILocalOccupancyBackend`，不暴露 OcTree reference，
也不得被 frontier/safety/C3 领域算法 include。

## 9. 启动与唯一权威链

- `backend_type`、vehicle/map identity、map geometry、sensor contract、pose 门和
  jump thresholds 都在启动时冻结；未知 backend 值使启动失败。
- C2 首版生产 runtime 只注册 `octomap`；轻量 backend 是测试 fixture。
- 公共 C2 launch 只启动一个 local mapper，并使用 `local_map/*` 输出。
- 旧 `swarm_controller/octomap_builder_launch.py` 不被 include；旧 builder 保留在
  自己的 legacy launch/test 中。
- C2 不修改现有 swarm exploration/multi-drone public launch 的默认 legacy 链；
  在 C3 map update 和后续总集成前把它们切到新 mapper 会混用新旧地图契约。
- launch test 检查 authoritative state/map topic 各只有一个 publisher，ROS graph
  不出现 `scan_returns`，且旧、新 mapper 不会写同一 topic。

## 10. 测试设计

### 10.1 Backend conformance gtest

同一测试集运行于 `OctoMapBackend` 与 `DeterministicVoxelBackend`：

- 初始 unknown、occupied/free 更新、点查询；
- bounded-region 的确定性排序；
- empty/non-empty known bounds 与 canonical lattice 对齐；
- 同批 occupied 胜过 free；
- reset 清空已知 cells；
- canonical lattice 的负坐标、cell/region 半开边界、边界前后点和 OutOfRange；
- invalid geometry/bounds fail closed；
- fault-injection backend 证明 Rejected/NoEvidence strong no-change，并证明 mutation
  fault 后全部旧 transaction/token 被封禁、新 epoch 只能从空 backend 恢复；
- revision read transaction 在并发 apply/reset/fault 下保持同一 identity/epoch/revision/
  geometry/bounds/query snapshot；close/move-from 后稳定拒绝，public API 不泄漏
  backend/view/reference，且无整图复制；
- commit-to-snapshot race 返回 Superseded 并 coalesce/retry latest；从不把旧 receipt 的
  revision 标到后续 transaction 内容，也不要求每个中间 commit 都发布完整 snapshot；
- required capabilities 为 true，optional unsupported 不被调用。

### 10.2 Engine gtest

- HitOnly/HitRay/FullRay 全矩阵和 Invalid 边界；
- Cloud3D HitOnly 与高能力拒绝；
- 多个 2D/3D sensor、不同 origin、各自 session/stamp 顺序；
- Healthy/Degraded/Unavailable、恢复稳定门和无伪 freshness；
- contract fingerprint、C1 upstream ceiling、pose stale/frame/quality、时间回退、跳变、
  source session/reset epoch；
- active/retired pose lineage、high-water fence、map epoch 隔离，以及旧
  observation/pose/session replay 只拒绝而不重复清图；
- `submit_alignment()` 的 identity/revision/epoch 校验、reset 后立即 Revoked，且
  alignment 不阻塞新本机地图；
- `SensorExtrinsicRegistry` 的首次冻结、容差、frame/session 漂移拒绝；
- 固定 replay 两次得到相同 state/query 结果。

### 10.3 ROS launch tests

- 直接发布确定性 C1 messages，验证显式 wire decode、QoS 和输出；
- C1 fixture/input node 到 C2 的端到端路径，验证不出现 `scan_returns`；
- map revision 与 OctoMap 只随真实提交推进，state heartbeat 只推进 state sequence；
- state QoS/sequence 覆盖 volatile late join、延迟旧样本、publisher crash、deadline 与
  remaining-budget 到期；OctoMap transient-local late subscriber 必须等待有效 state
  heartbeat 后才能作为 fresh map 使用；
- 单一 publisher、frame、epoch/revision 和 revision-matched snapshot；
- 节点参数非法、unknown enums、TF/外参漂移和 pose reset 故障注入。

### 10.4 旧路径对照与 RViz

- 复用旧 `TestOctoMapBuilder` 代表查询语义作为 oracle，不复制旧私有消息。
- C1 隧道环切面 fixture 生成真实 C2 occupancy；岔口 no-return 扇区只在
  FullRay 下写 free，超过量程/Invalid 不写地图。
- RViz 同时显示右手坐标轴、C1 ray-evidence markers 与 C2 OctoMap；地图截面应
  垂直 map +X，网格含义在 RViz display 名称中明确。
- GUI 目检单独执行，不替代 gtest/launch assertions。

## 11. 兼容、迁移与回滚

- 不删除旧 builder、旧测试或旧 launch；C2 先建立旁路但使用独立 topic。
- 新 launch/验收中旧链完全关闭。后续总集成任务再把现有探索消费者迁到新 map
  契约或 C3 update。
- 运行期旁路回滚可停止新 launch/节点并独立启动 legacy launch；若回滚源码/接口，
  还必须恢复 `HealthState.msg` schema、C1 publisher/fixture/tests 与 fingerprint utility，
  删除 `LocalMapState.msg` 注册并重建 `perception_interfaces` 全部反向依赖，禁止保留
  混合 type support。
- 不在 C2 修改 global merger、allocator、fleet topology 或在线 alignment。

## 12. 主要风险与控制

| 风险 | 控制 |
| --- | --- |
| 从 2D payload/正无穷推断过高能力 | 复用闭合枚举和 C1 return classification，完整矩阵测试 |
| TF 绕过 pose reset | 仅解析冻结 body-sensor 外参；map-body 必须来自 PoseEstimate |
| 多 sensor 建立伪全序 | 每 sensor 单独检查 stamp/session，map revision 只表示 commit 顺序 |
| timer 伪刷新 | revision/freshness 只在 backend Applied 后更新 |
| OctoMap 泄漏到领域算法 | backend interface + backend-specific ROS adapter 隔离 |
| reset 后跨 epoch 污染 | 先关闭/失效 alignment，再 reset backend/pose history，测试旧消息 replay |
| 两条 mapper 同时权威 | launch/topic 隔离 + publisher cardinality assertion |
| C2 偷跑地图共享 | LocalMapState 不含 map delta；shared TODO 保留到后续任务 |
| LaserScan 角序列与 FakeLidar 束序反向/错位 | 冻结基向量、四元数和 index remap，ROS-free golden test 覆盖 360 束 |
| TF 对标准 scan 再施加一次垂直环旋转 | FakeLidar 只负责 body YZ raycast；唯一实际变换来自 acquisition stamp 的静态 `body <- scan` TF，descriptor mounting 仅作期望元数据/fingerprint 校验 |
| no-return 被编码成有限最大量程 | publisher 只允许 hit 为有限 range、no-return 为 `+inf`，端到端分类测试锁定 |
| cave/trajectory/sensor/contract 参数在节点间漂移 | 独立 scene config 作为单一真源，C1/C2 共用同一 descriptor/contract 参数组 |
| 同 stamp 的 pose/scan 跨 topic 乱序 | validation-only gate 等待 C1 PoseEstimate watermark，再施加明确 lead delay；C2 diagnostics 断言无 pose missing/stale rejection |
| launch test 无 authoritative query API | 三态查询放在 ROS-free mapper transaction scene gtest；launch test 只锁定真实链的状态、bounds、provenance 与 graph |
| RViz config 存在但 plugin/订阅未生效 | 结构解析 + 独立 domain 真实 RViz process/plugin/maps/graph smoke，GUI 外观另行目检 |
| 终点停驻帧伪刷新地图 | 观察到运动结束后停止 LaserScan；测试断言 revision/last commit 不再变化 |

## 13. 连续 FullRay 洞穴场景验证

### 13.1 场景边界、单一配置源与数据流

新增独立 `cave_full_ray_scene` 入口，不替换 `local_map_debug.launch.xml`。完整数据流为：

```text
TreeCaveField(seed=42, radius=2.5) ──> /cave/points ─────────────> RViz/oracle only
             │
             └─> CaveLaserScanNode ── raw LaserScan ─┐
LineTrajectory/FakeOdomNode ── Odometry ──> C1 ── PoseEstimate watermark
                  │                                  │
                  └──────────────────────────────────┴─> PoseGatedLaserScanRelay
                                                          │ released LaserScan
                                                          ▼
                                      C1 observation adapter ──> LidarObservation ─┐
                                      C1 health/pose ──────────────────────────────┼─> C2
                                                                                  ┘
```

`CaveLaserScanNode` 是薄 ROS binding：从 scene config 构造既有 `ICaveField`，把
map-frame odometry 普通值转换为 `Pose3D`，调用 ROS-free `FakeLidar::scanReturns()`，
再编码标准 `sensor_msgs/LaserScan`。它不实现第二套 raycast、地图累积、证据分类或
mapper，也不依赖 `perception_interfaces`。

`PoseGatedLaserScanRelay` 是 `perception_fixtures` 中仅供 validation scene 使用的薄
gate。它缓存 raw scan，订阅 C1 `PoseEstimate` 作为 watermark；只有 watermark 与
scan acquisition stamp 相等、frame/source/clock-domain 和当前 producer lineage 均
符合冻结场景配置时，才把 scan 放入延迟 release queue。这样 perception-specific
时序职责不会污染通用 `drone_scanner` 包。

场景参数冻结为：`tree`、seed `42`、base radius `2.5 m`；轨迹
`(1,0,1.5) -> (11,0,1.5)`、`20 s`、`0.5 m/s`，固定 y/z/yaw/pitch/roll，odometry
`20 Hz`，`motion.mode=line`，`altitude_adapt.enable=false`；scan `10 Hz`、`360` 束、
range `[0.1,30] m`、noise `0`。

这些 cave、trajectory、scan、timing、descriptor/contract 值只存在于一个 canonical
structured scene config。Python launch 读取一次，验证字段并计算派生的角度末值、
四元数参数顺序和 timing budget，然后向 cave publisher、CaveLaserScanNode、gate、
C1 与 C2 fan-out；cave publisher 与 CaveLaserScanNode 必须获得同一 cave 参数对象，
C1/C2 descriptor/contract 必须由同一参数组生成。这里采用 Python launch 是因为 XML
无法安全表达结构化参数复用和派生不变量；launch 不承载 raycast、证据分类或 mapper
业务逻辑。ROS-free/launch tests 读取同一 config，不复制运行时常量。

洞穴真值点云不得订阅到 C1/C2，scene graph 中也不得出现 `/scan_returns`、
`scan_accumulator`、`cloud_map` 或 legacy `OctoMapBuilder`。

### 13.2 标准 LaserScan 几何与编码

LaserScan 保持标准 sensor-frame XY 平面，消息与 descriptor 使用完全相同的实际采样
边界：

```text
N = 360
angle_min = -pi
angle_increment = 2*pi/N
angle_max = angle_min + (N-1)*angle_increment = pi-angle_increment
ranges.size() = N
d_scan(theta) = (cos(theta), sin(theta), 0)
```

样本覆盖半开 `[-pi,pi)`；`LaserScan.angle_max` 仍是最后一个实际样本，绝不能写成
`pi`。冻结静态旋转 `body <- scan = Ry(+pi/2) * Rz(+pi/2)`，右侧 `Rz` 先作用，
ROS TF 四元数顺序为 `(x,y,z,w)=(0.5,0.5,0.5,0.5)`。其基向量映射为
`scan +X -> body +Y`、`scan +Y -> body +Z`、`scan +Z -> body +X`，因此：

```text
d_body(theta) = (0, cos(theta), sin(theta))
```

该方向与既有 `FakeLidar` 的 YZ 垂直环一致。FakeLidar 原生 index `j` 使用
`theta_fake=2*pi*j/N`，而 LaserScan 从 `-pi` 起始；publisher 对冻结偶数束数执行
`j=(i+N/2) mod N` 后写入 `ranges[i]`。golden 必须锁定：

```text
i=0   -> j=180 -> body -Y
i=90  -> j=270 -> body -Z
i=180 -> j=0   -> body +Y
i=270 -> j=90  -> body +Z
```

hit 仅在 finite 且位于 `[range_min,range_max]` 时写有限距离；no-return 写 IEEE
`+inf`。非有限 hit、低于 min 或越界有限 hit 使整帧 fail closed，不能转写成
no-return。`intensities` 为空，`time_increment=0`，`scan_time=0.1 s`，
`header.frame_id=scan_link`，`header.stamp` 保留原 odometry acquisition stamp；整帧
使用同一 pose，不做 beam deskew。`LaserScan` 自身没有 `clock_domain`，该值由同一个
C1 node 参数赋给 Observation/Pose，并由 gate 对 PoseEstimate watermark 校验。

`range_min/range_max`、angle 三元组和数组长度必须通过既有
`LaserScanAdapter` 的完整校验。ROS-free helper test 使用可解析 `ICaveField` fake
锁定 360 束方向、index remap、hit/`+inf` 分类、整帧拒绝和首尾方向不重复。

### 13.3 Pose-before-scan 与结束行为

scan acquisition 由 odometry 样本确定性降采样到 `10 Hz`；每帧使用触发它的
odometry header stamp 和 map-frame pose 执行 raycast，raw/released LaserScan 均保留
该 stamp。仅仅“相同 stamp”不足以保证 DDS 跨 topic 到达顺序，因此 gate 必须：

1. 按 stamp 建立有界 pending scan；
2. 等待 C1 已发布同 stamp、`frame_id=map`、冻结 pose source/clock-domain 和当前
   producer session 的 PoseEstimate watermark；
3. watermark 到达后再等待 `pose_lead_delay=100 ms`，且该值不得少于两个 odometry
   period，然后才发布 released LaserScan；
4. 对 watermark 超时、stamp 回退、lineage/配置不一致和 pending overflow 丢弃整帧，
   发布节流诊断且不刷新任何 acquisition；
5. C2 launch test 必须监听 diagnostics，证明没有 pose missing/stale rejection，而不
   只断言两个 message stamp 相等。

C2 仍按 acquisition stamp 选择不晚于 scan 的最近 pose；不得用 receive time、最新
停驻 pose 或无时间对应关系的 TF 替代。这里不修改 C2 核心排序/配对逻辑。

节点观察到非零运动后，在轨迹完成的首个零速度 odometry 上停止创建新 scan
acquisition。gate 可排空此前已经建立且 watermark 合法的 pending scan；随后等待一个
lead delay 加明确 transport drain 窗口，再开始终点稳定断言。FakeOdom 可继续发布
终点 pose 和轨迹供 RViz 使用，但 C2 不再收到新 observation，因此 revision、
last-commit provenance 和 freshness 不得被停驻帧刷新；state heartbeat sequence 可以
继续递增，最终 map freshness 必须按真实 last commit 到期。

### 13.4 TF、descriptor、fingerprint 与启动组合

C1 与 C2 必须加载完全相同的单 sensor contract：sensor type `2d`、capability
`FullRay`、相同 sensor/frame ID、半开 full-circle FOV、`pi/180` angular resolution、
range `[0.1,30] m`；minimum 与 degraded requirement 均为一个 2D `FullRay` sensor，
`requires_pose=true`、expected pose frame `map`，其他影响 fingerprint 的
recovery/pose 字段也相同。

唯一实际空间变换来自静态 TF `body <- scan`，C2 在 observation acquisition stamp
调用 `lookupTransform(body_frame, observation.frame_id, stamp)`，转换成普通
`SensorExtrinsicSample`，再由 `SensorExtrinsicRegistry` 与 descriptor 中的期望
`mounting_position/mounting_orientation` 比较并冻结。C1/C2 descriptor 参数按 Eigen
构造顺序写 `mounting_qw,qx,qy,qz = 0.5,0.5,0.5,0.5`；ROS TF 参数按
`x,y,z,w = 0.5,0.5,0.5,0.5`。descriptor metadata 不在 range 生成或 mapper 中再次
旋转，TF sample 也不直接进入 fingerprint。

canonical fingerprint 精确覆盖 descriptor identity/type/frame、mounting position/
orientation、FOV、angular resolution、range、ray evidence，minimum/degraded
requirements，以及 `requires_pose`、pose freshness、expected frame、minimum quality、
recovery samples。它不覆盖 topic 名、cave/trajectory/rates、clock domain、map
freshness 或运行时 TF sample。启动测试比较 C1 health 携带的 canonical fingerprint
与 C2 state fingerprint，不以 YAML 文本相等代替语义校验。

scene launch 只启动一个 `perception_local_map_node`，使用独立 topic/namespace；组合
`cave_publisher`、`FakeOdomNode`、`CaveLaserScanNode`、validation-only gate、C1 input、
C2 和可选 RViz。RViz 使用独立 config，同屏显示 `/cave/points`、飞行 Path、当前
released LaserScan、TF 与 C2 `local_map/octomap`；`LD_PRELOAD=liboctomap.so` 仍只设置
在 RViz 进程。

### 13.5 自动验收、RViz smoke、兼容与回滚

自动验收分层，不把 visualization-only OctoMap 二进制当作 authoritative query：

- ROS-free geometry/helper gtest 读取 canonical config，锁定全部束方向、remap、
  LaserScan metadata、return encoding 和 fail-closed 边界。
- ROS-free scene gtest 使用同一 `TreeCaveField + FakeLidar` 配置，通过
  `LocalObservationMapper` 的 read transaction 对固定 map-frame 点/区域查询，分别
  断言 Occupied、Free、Unknown；真值对象只在测试 oracle 内调用，不注入 mapper。
- 正式 `20 s` launch test 使用约 `40-45 s` timeout，运行真实 C1/C2 链；要求至少
  `100` 个唯一 accepted observation、同一非零 map epoch 内 revision 单调、known
  bounds X span 至少 `9 m`、last-commit provenance 与 acquisition stamp 前进、C1/C2
  fingerprint 相等，且无 pose missing/stale rejection。终点 drain 后 revision/
  provenance 固定，heartbeat sequence 可继续，freshness 最终到期。
- graph test 要求 authoritative state/map 各一个 publisher，并证明 `/scan_returns`、
  accumulator/`cloud_map`、legacy builder 和第二 mapper 缺席。
- 静态测试解析 RViz config，锁定 `/cave/points`、Path、released LaserScan、TF、
  OctoMap 五类 display/topic；解析 launch 证明唯一 `LD_PRELOAD=liboctomap.so` 只属于
  RViz。独立 ROS domain 的真实 RViz smoke 要求进程启动，`/proc/<pid>/maps` 含
  `liboctomap_rviz_plugins.so`，日志无 undefined symbol，并由 ROS graph 证明 RViz
  实际订阅 config 中的预期 topics。GUI 外观目检仍单独执行。

Release gate 扩展到 `cave_world`、`drone_scanner`、`perception_fixtures` 和完整 C1/C2
dependency closure，运行新增 ROS-free gtest、launch/RViz smoke、既有 C2 全套测试和
`git diff --check`；随后执行 full-scope `trellis-check` 和用户 RViz 目检。

该场景是新增旁路资产，不改变既有 `drone_sensing_stack.launch.py`、静态 C2 debug
入口或 legacy mapper 默认选择。运行期回滚只需停止新 scene launch。源码回滚必须
同时移除 CaveLaserScan ROS-free helper/node/main、validation-only gate、各自 CMake/
package/test 注册、canonical scene config/loader、scene launch/RViz、launch/RViz smoke
和 `docs/local-observation-map.md` 的 Phase H 段落；既有 FakeLidar、C1/C2 core/wire、
原 A-G 测试与静态 debug 资产保持不变。

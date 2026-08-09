# C3 地图状态与增量更新：技术设计

> 状态：功能设计已实施，单元/集成测试、确定性 C2 exact-revision replay/oracle、
> RViz 可视化以及冻结版本的性能与内存专项验收均已通过。历史 Phase 3 bag 资产
> 不可用但不阻塞功能验收，shared-view alignment 消费验收由后续聚合路径完成。

## 1. 设计目标

C3 在 C2 权威本机占据地图与 C4 通信数据面之间建立一个稳定的状态复制层：

```text
C2 exact-revision read transaction
    -> canonical source snapshot
    -> keyframe / delta producer
    -> C3 semantic ROS message
    -> reference receiver / atomic applier
    -> reconstructed source-local occupancy view
```

该层必须同时满足以下不变量：

1. C2 仍是唯一权威本机 occupancy writer；C3 不回写 C2，也不参与本机安全查询。
2. 更新顺序只由 source identity、map epoch 和 revision 决定，不能由 ROS header stamp 决定。
3. receiver 只从完整 keyframe 或当前 base 上的合法 delta 推进；失败保留最后合法状态。
4. producer 的全图遍历、diff、hash 和 canonical serialization 不在 C2 mutation callback 中执行。
5. 所有状态、队列、payload 和临时内存都有显式上限；超限 fail closed。
6. source-local 更新不依赖 alignment；进入 shared view 时必须在独立消费边界绑定已提交 alignment。
7. 方案 A 的完整 snapshot comparison 永久作为正确性 oracle、兼容路径和回滚路径。

## 2. 包与所有权边界

### 2.1 新增 `perception_map_update`

在 `ws/src/alien_perception/perception_map_update/` 新建 `ament_cmake` 包，命名空间使用
`PerceptionMapUpdate`。包内按职责拆成四个 target：

| Target | 依赖 | 职责 |
|---|---|---|
| `perception_map_update_core` | C++17、`perception_core`、OpenSSL Crypto；无 ROS | 值对象、canonical codec、SHA-256、diff、producer 决策、applier、resync 状态机 |
| `perception_map_update_ros` | core、`perception_interfaces`；不依赖 `rclcpp` | ROS message/service 字段与 core 值对象的唯一 encode/decode 边界 |
| `perception_map_update_parameters` | core、`rclcpp` | 集中声明并校验 producer/receiver 共用的资源上限参数，避免两个薄节点出现默认值漂移 |
| `perception_map_update_octomap` | core、OctoMap | 把 reconstructed canonical state 转为 visualization-only OctoMap |

包内提供薄节点 `perception_map_update_receiver_node`，只负责参数、订阅、service client、
消息转换、诊断和 reconstructed OctoMap 发布。节点不重新实现协议校验。

公共 CMake target 至少导出：

```text
perception_map_update::core
perception_map_update::ros
perception_map_update::parameters
perception_map_update::octomap
```

### 2.2 扩展 `perception_interfaces`

新增：

- `msg/MapUpdate.msg`：C3 语义更新 envelope + canonical binary payload；
- `srv/RequestMapResync.srv`：本机短 resync 请求与确认。

不新增逐 cell ROS message。高容量 cell/op 数据使用版本化 `uint8[] payload`，由 core codec
唯一解释，避免 C++ core 依赖 ROS 类型，也避免不同消费者各自重写 cell 解码规则。

### 2.3 修改 `perception_local_map`

`perception_local_map` 新增两个 ROS-free 适配组件：

- `CanonicalSnapshotAdapter`：从一个打开的 `MapReadTransaction` 构造 C3 snapshot；
- `AsyncMapUpdateProducer`：单 pending 槽 + 单 in-flight worker，调用 C3 core producer。

`PerceptionLocalMapNode` 只在 observation 成功提交后 O(1) enqueue `CommitReceipt`，并负责：

- 发布 `MapUpdate`；
- 提供 `RequestMapResync` service；
- 暴露 producer 诊断；
- 保留现有 `/local_map/octomap` 和 `LocalMapState` 行为。

以下箭头表示“左侧被右侧消费”，依赖方向固定为：

```text
perception_core -------------> perception_map_update_core
                                      |
perception_interfaces -------> perception_map_update_ros
                                      |
perception_map_update core/ros ------> perception_local_map
```

`perception_map_update` 不依赖 `perception_local_map`，避免循环依赖。C2 类型到 C3 类型的
转换归 `perception_local_map` 所有。

## 3. 逻辑数据模型

### 3.1 Source identity

```cpp
struct SourceIdentity {
    std::string vehicle_id;
    Perception::SessionID mapper_session;
    std::uint64_t map_epoch;
};
```

revision 不属于稳定 source key，而属于 snapshot/update。`vehicle_id` 非空，session 必须
有效，`map_epoch >= 1`。同一 receiver 同时只维护一个 admitted source chain。

### 3.2 Geometry

```cpp
struct MapGeometry {
    double resolution_m;
    Point3d lattice_origin;
    std::string frame_id;
};
```

geometry 必须有限、resolution 为正、frame 非空。`geometry_fingerprint` 为 geometry 的
versioned canonical bytes 的 SHA-256。fingerprint 只描述 lattice/frame，不包含 known bounds；
known bounds 从 canonical cells 推导，避免同一内容出现两个边界真相。

### 3.3 Canonical snapshot

```cpp
enum class CellState : std::uint8_t { Free = 1, Occupied = 2 };

struct CanonicalCell {
    VoxelIndex index;
    CellState state;
};

struct CanonicalSnapshot {
    SourceIdentity source;
    MapGeometry geometry;
    std::uint64_t revision;
    RevisionProvenance latest_commit;
    std::vector<CanonicalCell> cells;
    Hash256 geometry_fingerprint;
    Hash256 content_hash;
};
```

规则：

- Unknown 不存储；缺失 index 即 Unknown。
- cells 按 `(x, y, z)` 严格升序，禁止重复 index。
- `content_hash` 覆盖 source identity、map epoch、geometry fingerprint 和有序 cells，
  但不包含 revision。这样概率更新导致三态内容不变时，前后 revision 可以共享同一
  content hash，而 revision 顺序仍由 envelope 单独保证。
- `latest_commit` 来自目标 revision 的精确 transaction metadata，只描述跨度终点，
  不伪造被合并 revision 的逐步 provenance。
- Snapshot 是 producer、测试和 oracle 使用的完整只读值对象，不直接作为 update topic kind。

### 3.4 Update kind

```cpp
enum class UpdateKind : std::uint8_t {
    Keyframe = 1,
    Delta = 2,
    Summary = 3,
    Remove = 4
};
```

语义如下：

| Kind | Payload | Occupancy 作用 |
|---|---|---|
| `Keyframe` | 完整有序 known cells | 无依赖地建立或原子替换基线 |
| `Delta` | 有序 cell operations | 只在 receiver current revision/hash 等于 base 时应用 |
| `Summary` | 空或小型版本化 summary payload | 不改变 cells，不推进 occupancy revision/freshness |
| `Remove` | 空 tombstone payload | 原子清除 contribution，保留 replay fence |

Snapshot 与 Keyframe 的区别是：Snapshot 是内部完整状态；Keyframe 是可传输、可应用的
完整更新。

### 3.5 Delta operations

```cpp
enum class DeltaOp : std::uint8_t {
    UpsertFree = 1,
    UpsertOccupied = 2,
    RemoveToUnknown = 3
};
```

operations 按 voxel index 严格升序，每个 index 最多一个 op。相邻 canonical snapshot
通过双指针 O(n) merge 比较生成：

- Unknown -> Free/Occupied：对应 upsert；
- Free/Occupied -> Unknown：`RemoveToUnknown`；
- Free <-> Occupied：对应目标状态的 upsert；
- 状态相同：无 op。

C2 的 OctoMap backend 可能在概率值发生变化但三态阈值结果不变时返回 `Applied`，C2
仍会推进 revision。因此 `Delta` 允许 operations 为空且 `new_revision > base_revision`；
这是合法的 revision-only state transition，不是 `Summary`，receiver 必须推进 revision，
但 content hash 保持不变。

### 3.6 Update envelope 与哈希

逻辑 `MapUpdate` 至少包含：

```text
protocol_version / canonical_encoding_version / hash_algorithm
update_kind
source identity
source map frame / geometry / geometry_fingerprint
base_revision / new_revision / revision_span
observed_coalesced_receipt_count
base_content_hash / content_hash / update_hash
latest commit provenance
known_cell_count / operation_count / canonical_payload_bytes
optional resync correlation_id
canonical payload
```

三个 hash 的职责不同：

- `geometry_fingerprint`：验证 lattice/frame 完全一致；
- `content_hash`：标识 `new_revision` 对应的完整 occupancy 内容；
- `update_hash`：验证 kind、identity、geometry、base/new revision、base/content hash、
  provenance、计数和 payload 的完整性。

`correlation_id`、ROS header stamp、本机 receive/send time 和诊断字符串不进入 `update_hash`。
因此同一 revision/content 为不同 resync 请求重发 keyframe，不会制造 revision/hash 冲突。

Delta 必须携带 `base_content_hash`；receiver 同时比较 base revision 与 base content hash。
Keyframe 的 base revision 为 0，base content hash 为全零。Summary 不推进 occupancy revision；
Remove 使用明确 tombstone 规则，不把空 keyframe 当作删除。

## 4. Canonical encoding

### 4.1 版本与字节序

首版固定：

- protocol version：1；
- canonical encoding version：1；
- hash algorithm：SHA-256；
- 实现：容器内已存在的 OpenSSL 3 `EVP` API；
- 整数：固定宽度、大端编码；
- 字符串：UTF-8 bytes + 无符号长度前缀，禁止嵌入式 NUL，长度受配置约束；
- double：拒绝 NaN/Inf，把 `-0.0` 规范为 `+0.0` 后编码 IEEE-754 bit pattern；
- array：先编码元素数量，再编码严格 canonical order 的元素。

hash 通过流式 writer 计算，不要求为完整 snapshot 再复制一份大 byte buffer。需要发布的
keyframe/delta payload 才物化为 `std::vector<std::uint8_t>`。

### 4.2 Decoder 所有权

core decoder 是 binary payload 的唯一所有者，负责：

1. envelope 资源预检；
2. 版本和枚举校验；
3. overflow-safe 长度计算；
4. 完整消费 payload，禁止 trailing bytes；
5. index 严格递增、无重复、状态/op 合法；
6. 重新计算 `update_hash`。

ROS adapter 不解析 cell/op，只验证固定数组长度和 bounded string，再交 core decoder。

## 5. Producer 设计

### 5.1 Pure producer state

core `MapUpdateProducer` 是具体类，不抽象自身。它保存：

- 最后一次成功交给 publisher 的 canonical baseline；
- 自上次 keyframe 以来已发布 delta 数；
- force-keyframe/resync intent；
- 配置与累计诊断。

输入是目标 exact-revision `CanonicalSnapshot`，输出是 `ProduceResult`：

```text
ProducedKeyframe
ProducedDelta
NoNewRevision
RejectedInvalidSnapshot
RejectedResourceLimit
NeedsKeyframeButUnavailable
```

只有发布 callback 返回成功后才 `commit_published(result)` 并推进 producer baseline。
生成失败、转换失败或发布失败不得推进 baseline。

### 5.2 Keyframe 决策

以下条件强制 keyframe：

1. producer 没有 baseline；
2. mapper session 或 map epoch 改变；
3. geometry fingerprint 改变；
4. 接受了 resync intent；
5. producer baseline 丢失或状态校验失败；
6. delta operation count/bytes 超限；
7. delta chain 达到 `max_delta_chain_length`；
8. `delta_enabled=false` 回滚模式。

可选 `periodic_keyframe_revision_interval` 和 duration 默认均为 0，表示关闭。late join 不依赖
周期 keyframe，必须走 resync。

若 keyframe 自身超出 cell/byte/retained-state 上限，producer 进入 bounded blocked 状态并
发布诊断，不发布截断地图，也不修改 C2 或安全 freshness。

### 5.3 Cross-revision coalescing

只要求目标 `new_revision > baseline.revision`，不要求相差 1。producer 从最后已发布
baseline 直接与 latest target 比较，生成合法 `base -> latest` delta。

```text
revision_span = new_revision - base_revision
observed_coalesced_receipt_count = pending slot 覆盖的 receipt 数
```

revision span 是协议事实；coalesced receipt count 是调度诊断，两者不得混为逐 observation
审计记录。

### 5.4 C2 async worker

`AsyncMapUpdateProducer` 使用 C++17 `std::thread`、mutex 和 condition variable：

```text
callback thread                 worker thread
---------------                 -------------
enqueue(receipt) --O(1)-------> take latest pending
                                acquire exact transaction(receipt)
                                materialize canonical snapshot
                                diff / encode / hash
                                publish callback
                                commit producer baseline
```

边界固定为：

- pending receipt：最多 1 个，latest-wins；
- in-flight receipt/transaction：最多 1 个；
- ready update：不建立无界队列，worker 直接调用线程安全 publisher callback；
- resync intent：有界集合/单槽，受 `max_recent_resync_requests` 限制；
- shutdown：停止接收、唤醒 worker、关闭 transaction、join，不 detach。

exact receipt 已 superseded 时，worker 不把当前 map 冒充旧 revision。它记录 superseded，
优先处理已有较新 pending；若 pending 为空，可从 mapper 当前 state 取得 latest identity 并在
下一轮尝试该精确 receipt。重试次数/循环受界，不在一次 worker iteration 忙等。

`MapReadTransaction` 在 canonical snapshot 遍历期间持有 C2 read lock，writer 可能等待；
首版接受该方案 A 约束，但通过 10 Hz backlog/drain 门验证。未来 immutable snapshot 或
dirty journal 优化不得改变本协议输出。

## 6. Receiver 与原子 apply

### 6.1 状态机

```text
Empty
  | valid admitted keyframe
  v
Ready ---------------------> Removed
  | gap/hash conflict/local validation failure
  v
ResyncRequired
  | valid admitted keyframe
  +------------------------> Ready
```

- `Empty`：没有 baseline，只接受 admitted source 的 keyframe；收到 delta 产生 resync intent。
- `Ready`：接受合法 delta、更新的 keyframe、summary 和 remove。
- `ResyncRequired`：保留最后合法 cells/revision/hash，拒绝所有 delta；只允许合法 keyframe 恢复。
- `Removed`：cells 已清空，但保留 tombstone identity/revision/hash fence；旧消息不能复活贡献。

### 6.2 Source/session admission

applier 不根据 ROS topic、node name 或 header stamp 猜测新 session。它维护显式
`ExpectedSource`：

- `vehicle_id` 必须匹配配置/registration；
- 同 session 内更高 map epoch 只能由 keyframe 建立，旧 epoch 永久拒绝；
- mapper session 切换需要 reference receiver 从本机 `LocalMapState` 更新 expected source；
  后续 C5 用 registration/membership 事件提供同一 admission 输入；
- 已退休 session 保存在有界 replay fence 中。

reference receiver 初次启动可以通过 `RequestMapResync` 的 latest 模式获取 producer 当前
identity；一旦接受 keyframe，就固定该 chain。它也订阅本机 metadata-only `LocalMapState`
用于明确 session/epoch reset，不把该 state topic 当作地图内容。

### 6.3 Apply pipeline

所有更新遵循同一顺序：

1. ROS adapter 固定字段/长度预检；
2. core envelope、版本、identity、geometry、计数和 hash 校验；
3. revision/base/content hash 状态机校验；
4. 在 scratch vector 中构造候选状态；
5. 生成候选 canonical content hash 并与 update 声明比较；
6. 全部成功后一次 swap，提交 revision/hash/freshness/diagnostics。

receiver state 使用严格有序 `std::vector<CanonicalCell>`。Delta apply 通过 old cells 与
sorted ops 的 merge 构造新 vector，复杂度 O(n+k)。keyframe decode 后直接验证候选 vector。
apply 期间允许旧状态 + 新状态 + payload 同时存在，因此内存预检必须按峰值计算。

### 6.4 Revision 与重复矩阵

| 输入 | 结果 |
|---|---|
| 相同 identity/new revision/content hash 的重复 | 幂等忽略；不刷新 freshness |
| 相同 identity/new revision、不同 content hash | 冲突拒绝，进入 `ResyncRequired` |
| Delta base revision/hash 等于 current | 原子 apply |
| Delta base 大于 current | gap，进入 `ResyncRequired` |
| Delta base 小于 current 或 new revision 不前进 | stale/overlap 拒绝，不回滚 |
| 空 operations、new revision 前进、content hash 不变 | 合法 revision-only delta |
| 旧 session/epoch | 拒绝，不刷新 freshness |
| 新 admitted epoch 的 keyframe | 原子替换并退休旧 epoch |
| hash/payload/资源错误 | 原子拒绝；必要时进入 resync |

Summary 只更新独立诊断字段，不改变 occupancy revision/content/freshness。Remove 必须满足
当前 chain 的单调 tombstone 规则，提交后清空 cells；相同/旧 revision 的 map update 不得恢复。

## 7. Resync 设计

### 7.1 Domain intent

core `ResyncIntent` 至少包含：

```text
requester identity/session
client_request_id
optional expected source identity/map epoch
receiver current revision/content hash
reason: InitialBaseline / Gap / EpochChange / HashConflict / LocalStateInvalid
```

reference receiver 每次进入新的 resync generation 生成稳定 `client_request_id`；service 重试
复用同一 ID。producer 维护有界 recent-request ledger：

- 相同 ID + 相同请求：返回相同 correlation ID，不重复扩张状态；
- 相同 ID + 不同请求：冲突拒绝；
- 新合法请求：O(1) 记录 force-keyframe intent，response 立即返回；
- materialization/keyframe 发布始终异步发生。

### 7.2 Service 行为

`RequestMapResync.srv` response 只包含 accepted/rejected、correlation ID、当前 source
identity/revision 摘要和诊断，不包含地图 payload。

允许两种请求：

- exact：请求中 identity/session/epoch 必须与 producer 当前状态一致；
- latest bootstrap：只允许 `InitialBaseline`，由本机 producer 返回当前 identity 并调度 keyframe。

旧 session/epoch、未来目标 revision、空/超长 request ID 和资源上限失败均拒绝。接受后，
keyframe 通过原 `MapUpdate` topic 返回并携带 correlation ID。receiver 只有在 correlation
匹配当前 resync generation，或 keyframe 满足显式 unsolicited-new-baseline 规则时恢复。

C4 后续复用这些领域字段，但负责跨机 route、TTL、retry、backpressure 和故障注入。

## 8. ROS wire contract

### 8.1 `MapUpdate.msg`

建议字段分组如下，最终 `.msg` 使用现有 session wire 形式：

```text
std_msgs/Header header                 # transport/display only

protocol / encoding / hash versions
update kind

vehicle_id
mapper_session_boot_time_ns / random_suffix
map_epoch
base_revision / new_revision / revision_span
observed_coalesced_receipt_count

frame_id (Header.frame_id mirrors it) / resolution / lattice origin
geometry_fingerprint[32]
base_content_hash[32]
content_hash[32]
update_hash[32]

latest commit sensor/session/origin stamp/clock domain
known_cell_count / operation_count / canonical_payload_bytes
correlation_id
uint8[] payload
```

`header.stamp` 是本机 publish time，不进入排序、freshness 或 hash。source origin stamp 与
clock domain 单独携带且不得在转发时重写。

### 8.2 Topic 与 QoS

reference 闭环使用可 remap 的相对名称：

- producer：`local_map/updates`；
- resync service：`local_map/request_resync`；
- receiver reconstructed view：`map_update_receiver/octomap`；
- diagnostics：标准 `/diagnostics`。

C3 reference QoS 固定为本机可复现基线，例如 reliable + volatile + keep-last(1)。这不是
C4 的最终网络 QoS 策略；late join 仍必须通过 resync 恢复。

### 8.3 反序列化资源边界

C3 可以在 ROS callback 入口拒绝超长 strings、payload array 和声明计数，但 DDS/CDR 已在
callback 前完成 sample 分配。跨机 serialized sample、history cache 和 queue byte budget
属于 C4；C3 不把 application-level 上限误写成完整传输防护。

## 9. C2 精确 metadata 扩展

为避免 `state()` 与 transaction 跨 revision 混读，`MapReadMetadata` 扩展为在同一 read
transaction 下捕获：

- `MapIdentity`；
- `MapGeometry` 和 known bounds；
- exact revision 的 `last_commit` provenance；
- mapper contract schema/fingerprint。

worker 不在 transaction 外补读会影响 update hash 的 source metadata。alignment 不进入
source map content/update hash，因为 alignment 可在不推进 map revision 时变化；shared-view
消费通过独立 `AlignmentReference` + exact source revision/content hash 绑定。

## 10. Alignment、Summary 与 Remove 边界

### 10.1 Alignment

C3 producer 始终可在无 alignment 时生成 source-local update。C3 提供 shared admission
校验函数/测试契约：

```text
source update identity + revision + content hash
    must pair with
Committed alignment identity + alignment epoch/revision
```

alignment 改变或撤销时，上层必须使旧 shared contribution 失效，并在新 alignment 下用
keyframe 重建；不能给既有 delta chain 重新套 transform。在线配准与 aggregate commit 在 C5+。

### 10.2 Summary

首版 core 与 wire 定义并验证 Summary 的禁止行为；C2 reference producer 不要求主动发布
Summary。健康与 metadata 仍由 `LocalMapState`/diagnostics 提供，避免建立第二套状态真相。

### 10.3 Remove

首版 core/applier 定义并测试 Remove/tombstone；C2 mapper 生命周期不主动生成 Remove。
成员离开、Frozen/Removed contribution 和 aggregate revision 由 C5 生产实现调用该语义。

## 11. 配置与资源模型

建议配置集中为 `MapUpdateLimits`，至少包括：

```text
max_identity_string_bytes
max_frame_id_bytes
max_correlation_id_bytes
max_known_cells
max_delta_operations
max_keyframe_payload_bytes
max_delta_payload_bytes
max_receiver_cells
max_peak_apply_bytes
max_retained_snapshot_bytes
max_delta_chain_length
max_recent_resync_requests
max_revision_span
periodic_keyframe_revision_interval = 0
delta_enabled = true
```

所有 `count * element_size + header_size` 使用 checked add/multiply。producer 峰值预算至少
覆盖 baseline snapshot + target snapshot + payload；receiver 峰值预算覆盖 current state +
candidate state + payload。任何失败都不做部分 commit。

## 12. 诊断与指标

producer 至少报告：

```text
pending_revision / in_flight_revision / published_revision
pending_coalesced / superseded_receipts
published_keyframes / published_deltas / revision_only_deltas
forced_keyframe_reason / delta_chain_length
snapshot_cells / delta_operations / payload_bytes
acquire / materialize / sort / diff / hash / encode / publish duration
resource_rejections / publish_failures / resync_requests
```

receiver 至少报告：

```text
state / current source identity / epoch / revision / content hash
accepted / duplicate / stale / gap / conflict / malformed / resource rejected counts
keyframe / delta / revision-only delta / remove counts
decode / validate / apply / content-hash / octomap-materialize duration
resync generation / reason / correlation / request status
current cells / payload bytes / peak candidate bytes
```

指标使用本地 monotonic duration；ROS/header/origin stamp 不用于跨域绝对延迟推断。

## 13. 验证设计

### 13.1 Core gtest

- canonical ordering、double normalization、golden bytes 和固定 SHA-256；
- keyframe/delta codec round-trip、truncation/trailing bytes/overflow；
- added/removed/flipped/revision-only delta；
- duplicate、old/future base、gap、session/epoch/geometry/hash conflict；
- scratch apply 失败原子性；
- keyframe event matrix、chain/bytes/cell limit；
- resync idempotency、correlation conflict 和 bounded ledger；
- randomized fixed-seed snapshot -> delta -> reconstructed equivalence。

### 13.2 C2 adapter/conformance gtest

在 `perception_local_map` 中让 `DeterministicVoxelBackend` 与 `OctoMapBackend` 运行同一组：

- exact transaction -> canonical snapshot 等价；
- canonical cells/hash 与 backend query 一致；
- transaction 持有期间 writer 等待；
- superseded receipt 不产生混合 update；
- 概率变化但三态不变时生成 revision-only delta。

### 13.3 ROS launch testing

- C2 -> update topic -> receiver -> reconstructed OctoMap 正常闭环；
- receiver 晚启动且周期 keyframe 关闭，通过 service 异步恢复；
- 注入 duplicate、乱序、gap、损坏 hash/payload，last valid revision 不变；
- pose reset/new epoch 后旧 delta 被拒绝，新 keyframe 恢复；
- resync response 不含 payload，重复请求 correlation 幂等；
- existing `/local_map/octomap`、state、TF 与 C1/C2 测试不回归。

### 13.4 Replay 与性能

- 当前固定 `ProfileScenario` 驱动真实 C2 mapper，并从每个 commit receipt 获取 exact
  `MapReadTransaction`，再通过正式 adapter 生成 canonical snapshot；
- 正式 producer/applier 在每个 checkpoint 比较 source、geometry、revision、content hash
  与完整 free/occupied/unknown cells；固定输入重复运行比较 update kind/hash 确定性；
- 历史 Phase 3 source-level bag 当前不可用；未来恢复后只作为额外兼容输入，不替代
  revision-aware replay/oracle；
- bounded 10 Hz 场景检查 observation backlog、pending/in-flight 上限和 drain convergence；
- sparse fixture 要求 delta canonical bytes 确定性小于 keyframe；
- expanding 场景报告约 1.81M-cell capacity knee、绝对延迟、CPU 和峰值内存；
- ASan/LSan/Valgrind/memcheck 按仓库 playbook 执行，非法访问和业务泄漏为阻塞项。

CPU 相对差异只有显著高于本机 30-50% 分辨率下限时才作方向性结论，不设 `<10%` 门。

### 13.5 冻结版本性能与内存结论

冻结提交 `7bb76643e8c800fa938406e87e42ee9923151d92` 使用同一
RelWithDebInfo 构建、同一 bounded 10 Hz workload，分别完成 C3 disabled、enabled 和
keyframe-only 三种模式各 3 轮、每轮 300 秒的独立采样。9 轮均正常结束并通过原始证据
分析、角色/计数/资源门；enabled 与 keyframe-only 均在停止输入后收敛到 latest revision。

- disabled 的稳态 CPU 均值为 11.88-12.50%，RSS 峰值为 62,648-63,032 KiB；
- enabled 的稳态 CPU 均值为 66.91-67.54%，RSS 峰值为 102,392-104,516 KiB；
- keyframe-only 的稳态 CPU 均值为 76.04-77.50%，RSS 峰值为 102,728-104,472 KiB；
- 三组 aggregate 均为 `suspected_sustained_growth=false`。keyframe-only 的 RSS 斜率
  为 520.3-866.6 KiB/min，低于 1,024 KiB/min 门，但作为后续长时运行观察项保留；
- enabled 的 materialize P95 为约 54 ms，diff P95 为约 2.6-2.8 ms；这些绝对延迟只
  表示当前本机量级；
- ASan/LSan core 26/26、adapter/async producer 5/5，Memcheck 无业务内存错误或
  definite/indirect/possible leak；1,812,520-cell capacity harness 通过。

enabled 相对 disabled 的 CPU 差异明显高于环境分辨率，可报告为 C3 方案 A 的额外成本；
keyframe-only 相对 enabled 约 15% 的差异低于本机 30-50% 分辨率，不作优劣结论。
300 秒证据支持“正式窗口内未发现持续增长”，不外推为无限时长零增长保证。

## 14. 兼容、迁移与回滚

1. `map_update_enabled=false`：完全关闭 C3 producer/receiver 接入，C2 保持原状。
2. `delta_enabled=false`：worker 对每个实际处理的 latest revision 发布完整 keyframe，作为
   协议内回滚路径。
3. 现有 `/local_map/octomap`、当前 exact-revision replay 和 C2 安全查询始终保留为独立 oracle；
   历史 Phase 3 bag 若恢复则作为额外兼容回归。
4. 新接口使用 version 字段；未知 protocol/encoding/hash version 直接拒绝并 resync，
   不做隐式兼容猜测。
5. native dirty-region、共享一次 materialization、压缩和 routed envelope 均为后续任务，
   必须与 canonical snapshot comparison 输出逐 revision 等价。

## 15. 已关闭的技术选择

| 决策 | 选择 | 原因 |
|---|---|---|
| Delta 来源 | 相邻/跨度 canonical snapshot comparison | 当前两个 backend 均无 dirty region；后端无关且可作 oracle |
| Cell 表示 | tri-state sparse，Unknown 缺失 | 与 C2 查询语义一致，可表达 remove-to-unknown |
| Receiver 容器 | sorted vector + merge/swap | 确定性、顺序 hash、原子 apply 和容量计算简单 |
| Wire payload | versioned canonical binary bytes | 单一 decoder、低对象开销、C4 可 opaque route |
| Hash | OpenSSL EVP SHA-256 | 容器已具备成熟实现，跨平台稳定，避免自写 hash |
| Hash 模型 | geometry/content/update 三层 | 分离 geometry、结果状态 identity 与 payload 完整性 |
| Producer 调度 | 单 pending + 单 in-flight worker | callback O(1)、latest-wins、严格有界 |
| Session 切换 | 显式 admission，不按 stamp 猜测 | 防止旧 session/乱序消息接管 source |
| Keyframe | 事件驱动，周期默认关闭 | 恢复协议可验证，避免周期全量掩盖缺口 |
| Resync | 短 service + topic async keyframe | service 不承载大 payload，便于 C4 路由复用 |

当前没有阻塞实施规划的开放产品问题。实现中若发现 wire 字段无法满足上述不变量，必须
回到 planning 修订本文和 PRD，不能在代码中形成第二套隐式协议。

# C4.3 Merkle v2 生产集成：技术设计

## 1. 决策摘要

本任务把尚未发布的生产地图更新协议直接提升为 v2-only：

```text
C2 exact CanonicalSnapshot
  -> producer diff
  -> chunk 16 immutable candidate + persistent Merkle Patricia candidate
  -> MapUpdate protocol v2
  -> C4 transparent route
  -> receiver local candidate rebuild + root verification
  -> atomic committed CanonicalCellView + VersionedContentDigest
```

- 所有 ROS 节点统一重编译，不支持旧、新 `.msg` 二进制共存。
- production 不保留 flat v1 双读、双写、协商或 downgrade 分支。
- SHA-256 原语、canonical keyframe/delta payload 和 geometry fingerprint 语义保持不变。
- content identity 改为固定 edge-16 Merkle Patricia v2；update hash 使用新的 v2 domain 并提交
  descriptor 与 base/result digest。
- v1 仅存在于隔离的 oracle/benchmark 与冻结基线证据中。

## 2. 分层与所有权

### 2.1 `perception_map_update` ROS-free core

正式拥有以下契约和算法：

- `ContentIdentityDescriptor` / `VersionedContentDigest`；
- `CellSnapshotStore` 的 chunk-16 immutable COW；
- `MerklePatriciaTree`；
- 新的生产 `MerkleMapState`，把 store 与 tree 作为一个不可分割的 committed/candidate 状态；
- v2 `MapUpdateProducer` 与 `MapUpdateApplier`；
- v2 update hash、resource preflight、resync domain types。

现有 `MerklePrototypeApplier` / `MerklePrototypeProtocol` 的已验证逻辑应晋升或被生产类型吸收，
完成后不保留第二套 prototype 状态机。Patricia 算法、golden vector 和 full-rebuild oracle 继续复用，
不重写树。

### 2.2 ROS conversion 与节点

- `perception_interfaces` 只定义 v2 wire schema。
- `perception_map_update::ros` 负责无分配前置校验、descriptor conversion 和未知值 fail closed。
- `perception_local_map` 节点只读取参数、把 C2 snapshot 交给 producer、发布消息和处理 resync。
- `swarm_data_plane` 继续把 `MapUpdate` 当作 opaque domain payload；Relay 不访问 chunk/tree。
- profiling fixture 可包含 flat v1 oracle，但 production node/target 不能链接一条可选 v1 apply path。

## 3. v2 wire contract

### 3.1 ROS 消息

新增可复用 `ContentIdentityDescriptor.msg`：

```text
uint16 SCHEME_MERKLE_PATRICIA_SHA256_V2 = 2
uint16 scheme
uint32 chunk_edge
uint16 coordinate_key_version
uint16 node_encoding_version
```

`MapUpdate.msg`：

- `protocol_version` 固定为 `2`；`canonical_encoding_version` 仍为 `1`；
- `hash_algorithm` 仍为 SHA-256；
- 新增 `content_identity_descriptor`；
- 保留 `base_content_hash` / `content_hash` 字段名；Merkle root 本身仍是 SHA-256 hash，32-byte
  值的具体语义由同一 update 中的 descriptor 决定。避免纯机械重命名扩大消费者变更面；
- `update_hash` 保持 32-byte SHA-256，但 domain 改为 `alien-scanner/map-update/v2`，并提交
  protocol、descriptor、base/result digest 和现有 envelope/payload 字段。

本项目未发布，因此允许直接变更现有 `.msg`，不新建平行 `MapUpdateV2` topic。

### 3.2 ROS-free 类型

- `ContentIdentityDescriptor` 和常量从具体树头文件提升到稳定的 map-update domain type 边界，
  避免 `MapUpdateTypes.hpp` 反向依赖 Patricia 实现。
- `MapUpdate` 持有一个 descriptor、base digest 和 result digest。
- `ReconstructedMap` 持有 `VersionedContentDigest`，消费者仍只通过 `CanonicalCellView` 读取 cells。
- `ProducerBaselineToken` 提交 source、revision 和完整 versioned digest。

### 3.3 状态语义

| Update kind | Base identity | Result identity | Store/tree 行为 |
| --- | --- | --- | --- |
| Keyframe | 同 descriptor + zero digest | 本地全量建树 root | 新建 store/tree，原子替换 |
| Delta | committed versioned digest | 本地增量 root | COW touched chunks + Patricia paths |
| Summary | committed digest | 与 base 相同 | 复用 store/tree，零 candidate nodes |
| Remove | committed digest | 同 descriptor + zero digest | 清空 state，提交 tombstone |

空 keyframe 的 v2 root 非零；zero digest 只表示无 base 或 remove tombstone。descriptor drift、
unknown version 或 v2 delta 的 zero base 都是协议错误。

## 4. 生产 `MerkleMapState`

`MerkleMapState` 是 ROS-free immutable snapshot，内部同时持有：

- `CellSnapshotStore`，配置固定为 `Chunked / edge 16 / 256 buckets`；
- `std::shared_ptr<const MerklePatriciaTree>`；
- source、geometry fingerprint、versioned digest；
- store/tree 的 committed 与最近 candidate metrics。

接口只暴露：

```cpp
static BuildResult build(...cells, limits);
ApplyResult apply(...operations, limits) const;
CanonicalCellView cells() const;
const VersionedContentDigest & identity() const;
```

`build/apply` 返回独立 candidate。调用方只有在 envelope、count、digest 和资源检查全部通过后才
替换 committed shared pointer。任何异常、溢出或校验失败只销毁 candidate。

## 5. Producer 数据流

### 5.1 Snapshot adapter

`CanonicalSnapshotAdapter` 仍负责精确 revision 的完整 known-cell materialize、排序、cell 校验和
geometry fingerprint；它不再计算 flat content hash。对应计时改为能准确表达的
`geometry_fingerprint_duration_ns`，不能把已删除的 flat hash 阶段继续报告成零成本成功。

这一步仍是 `O(N)`。本任务不修改 C2 backend 的 exact-revision API，也不把 producer diff 改成
C2 事件日志；该剩余成本必须在 Gate 中独立报告。

### 5.2 Prepare/commit

`MapUpdateProducer` 同时维护：

- 完整 canonical baseline snapshot，供 `SnapshotDiffer` 计算 delta；
- committed `MerkleMapState`，供增量 root 计算；
- revision/delta-chain/keyframe-correlation 状态。

Keyframe prepare 对 target cells 执行 full build；delta prepare 先 diff，再对 committed state 应用
operations。`PreparedUpdate` 持有 target snapshot 和 candidate map state。publish callback 成功后，
`commit_published()` 才以 baseline token CAS 语义同时提交两者。失败发布、stale prepared result 或
错误 update hash 均不推进 baseline/tree。

revision-only delta 的 operations 为空，candidate 复用 committed store/tree/root。producer 不在
production prepare 中运行 flat oracle 或 full-rebuild v2 oracle。

## 6. Receiver 数据流

`MapUpdateApplier` 变为 v2-only，并固定使用 `MerkleMapState`：

1. 校验 protocol、descriptor、source/geometry/revision、count、payload length 和 update hash；
2. 在分配前完成可计算的资源 upper-bound；
3. keyframe decode 后 full build，delta decode 后 persistent apply；
4. 本地比较 result digest、known-cell count 和 descriptor；
5. 一次性提交 candidate、revision、freshness token 和 last update hash。

现有 `ReceiverState::{Empty,Ready,ResyncRequired,Removed}` 和 duplicate/gap/conflict 语义保持。
descriptor drift 视为 `RejectedInvalid` 并要求 resync；同链 keyframe 只能在现有 correlation barrier
规则允许时恢复。`ReconstructedMap::cells` 仍是 storage-independent view，OctoMap 和可视化消费者
无需知道 chunk/Merkle。

## 7. Resync 与 C4 路由

不存在版本协商。direct `RequestMapResync.srv`、routed `ResyncIntent.msg` 和 `ResyncAck.msg` 应携带
完整 receiver/current versioned identity，使 ledger 的 duplicate/conflict 比较不再只比较裸 digest。

- 初始 bootstrap 使用规范 v2 descriptor + zero receiver digest；
- gap/conflict 请求携带 receiver committed descriptor/digest；
- ack 返回 producer 当前 descriptor/digest 与 revision；
- correlated keyframe 必须使用相同 descriptor，才能解除 barrier。

`RoutedMapUpdate` envelope 自身无需加入 Merkle 字段，因为嵌套 `MapUpdate` 已包含正式身份。
`payload_hash` 继续等于 v2 update hash，route/TTL/freshness/duplicate 规则不变。

## 8. 资源准入

扩展 `MapUpdateLimits`：

- `max_live_chunks`；
- `max_merkle_nodes`；
- `max_merkle_owned_bytes`；
- `max_candidate_merkle_bytes`；
- 现有 `max_peak_apply_bytes` 继续约束 committed store/tree、candidate store/tree、decoded payload、
  operations 和 cursor scratch 的组合峰值。

preflight 使用 checked arithmetic：

- keyframe 从已排序 cells 统计 distinct chunks；`K=0` 时 live nodes 为 0，否则 Patricia live
  nodes 上界为 `2K-1`；
- delta 从已排序 operations 统计 touched chunks，persistent path candidate 使用每个 mutation 最多
  192 key bits 的保守节点上界；
- store upper-bound 复用 `CellSnapshotStore::estimate_replace/apply_upper_bound()`；
- 分配后再用实际 metrics 复核 committed/candidate 上限，超限仍不得提交。

配置默认值必须覆盖现有确定性 cave/profile fixtures，同时与 `max_peak_apply_bytes` 一致地拒绝
高度离散、chunk/node 成本过大的合法 cell 序列。`max_known_cells=3,000,000` 不是绕过内存峰值
上限的保证。所有参数通过 `declare_map_update_limits()` 暴露并验证正数/范围。

## 9. 诊断与测量

producer/receiver diagnostics 至少新增或重命名：

- snapshot traversal/canonicalize/geometry fingerprint；
- diff、store candidate、Merkle leaf/path/content root、update hash、commit；
- live/touched/shared chunks、copied cells；
- live/allocated/path-rebuilt nodes；
- committed/candidate bytes；
- descriptor/protocol/resource rejection counters。

正式结果分三层，不混淆：

1. 同构建 ROS-free `perception_merkle_profile`：flat v1 oracle 与 v2 算法单因素；
2. 冻结 commit `6865c08` 的 production v1 端到端证据，与 candidate v2 使用相同 workload 参数，
   通过 manifest 明确不同 ELF/source identity；
3. candidate v2-only 的 `3 x 300 秒` 独立 receiver ROS 矩阵和内存工具。

由于 production 不保留 v1 runtime，本任务不伪造“同一 production binary 切换 v1/v2”。报告必须
明确跨 commit 对比的限制，并以 v2 的正确性、资源有界性、绝对成本、scaling 及 C4.2 同构建算法
A/B 共同决策。

## 10. 测试与 Gate

### 10.1 正确性

- descriptor/message conversion round-trip 与 unknown-value rejection；
- v2 golden roots/update hash；
- full-rebuild 与 incremental root deterministic replay；
- producer publish success/failure/stale commit 原子性；
- receiver keyframe/delta/revision-only/remove、gap/conflict/descriptor drift/resource rejection；
- direct/routed resync identity 与 correlation barrier；
- C4 route payload/update hash 一致；
- canonical cell 与 OctoMap/visualization 输出相对 flat v1 oracle 完全一致。

### 10.2 正式 Gate

- 所有受影响包串行 build/test 通过；
- v2 production 短筛选没有 correctness、unbounded scaling 或明显资源回归；
- 三次 300 秒运行全部 provenance 一致、无丢失最终 revision、无未解释增长；
- ASan/LSan/UBSan、严格 Memcheck、Heaptrack 无业务泄漏或非法访问；
- rollback dry-run 能把完整 C4.3 change set 移除并恢复冻结 v1 build/test，不要求在线 downgrade。

Gate GO 后接受 v2-only 为 production contract 并解除 C5d 阻塞；NO-GO 则不合并/回退本任务，
C5a-C5c 仍可继续，C5d 保持阻塞。

## 11. 排除方案

- 并行 `MapUpdateV2` topic/bridge：没有已发布节点需要兼容，成本无收益。
- runtime v1/v2 flag、dual-read/dual-write：污染 hot path 和性能归因。
- 只用 `protocol_version=2` 隐式固定树参数：无法让 digest 自描述并对未来 encoding drift fail closed。
- producer 在每个 snapshot full-rebuild Merkle：会把 hash 重新变成 `O(N)`，只允许 keyframe 使用。
- 本任务顺带改 C2 为增量 event log：是独立的 producer diff 优化，超出 hash v2 生产集成边界。

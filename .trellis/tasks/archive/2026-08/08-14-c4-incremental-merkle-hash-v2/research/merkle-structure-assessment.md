# Merkle 结构、坐标键与原型版本边界评估

## 1. 问题

在 chunk 16 COW 已把 256-cell delta 的 candidate build 降到 `0.140 ms` 后，接收端仍用
flat SHA-256 遍历完整 canonical map，hash mean 为 `4.169 ms`，约占 apply 的 95%。本轮
需要让 delta 内容身份维护随 touched chunks 及其树路径增长，同时不能把未验证的 v2 身份
隐式伪装成现有 wire/hash v1。

## 2. 仓库证据

- `ContentHasher.cpp:95-106` 把 cell count 和每个 canonical cell 顺序写入单个 SHA-256，
  因而每次内容哈希都是 `O(known cells)`。
- `CellSnapshotStore.cpp:943-1038` 已按 chunk coordinate 聚合 delta，并复制 touched chunks
  和 buckets；这正是增量叶子集合的可靠来源。
- `MapUpdateApplier.cpp:461-480` 在 candidate store 构造后重算完整哈希，再原子提交；Merkle
  candidate 必须复用相同的先验证后提交边界。
- `MapUpdateTypes.hpp:14-25` 只有 protocol v1、canonical encoding v1 和
  `HashAlgorithm::Sha256`，不能表达 flat SHA-256 与 Merkle SHA-256 的身份差异。
- 已提交 estimator 中 edge 16 的单来源一维 `10k/100k/500k` 分别有
  `625/6250/31250` 个 chunk，而每个 256-cell delta 均只触达 16 个 chunk。生产三维 replay
  当前最多只有 20 个 chunk，只能做正确性证据，不能代表容量上限。

## 3. 结构比较

令 `K` 为 live chunks，`T` 为 touched chunks。

| 结构 | 常驻节点 | delta 路径 | 确定性与风险 | 结论 |
| --- | --- | --- | --- | --- |
| 256/192 层未压缩 Sparse Merkle Trie | 随非空前缀增长，近似 `K * (depth - log2 K)` | 固定深度 | 默认节点可预计算，但 12.5k chunks 可产生数百万个非空路径节点 | 拒绝，内存与 hash 次数过高 |
| 持久化压缩 Merkle Patricia Trie | 精确上界约 `2K-1` | 平均接近 `O(T log K)`，最坏受 192 bit 限制 | 固定 key set 的压缩 trie 唯一；插入顺序不改变最终 root | 选择 |
| 持久化 Merkle B-tree | `O(K)`，缓存局部性好 | `O(T log_b K)` | split/merge 与 packing 容易让 root 依赖历史；要另做 canonical packing | 首轮拒绝，复杂度不匹配当前目标 |

按每节点含 32-byte digest、分支位和 COW 子指针粗估，Patricia 节点含 allocator/control block
约为 64-96 B。`2 x 100k` 一维基线约 12.5k chunks，即约 25k 节点、1.6-2.4 MiB；
`2 x 500k` 约 62.5k chunks，即约 125k 节点、8-12 MiB。实际值必须由
`node_count/committed_owned_bytes/candidate_owned_bytes` 计数替代估算后再过 Gate。

## 4. 选择的 canonical key

- key 固定为 192 bit：按 `x, y, z` 依次编码三个 signed int64。
- 每轴先把 two's-complement bit pattern 的 sign bit XOR `0x8000000000000000`，再写 big-endian。
  该变换无碰撞，并与当前 `ChunkCoordinate::operator<` 的 signed lexicographic 顺序一致。
- 不使用 `chunk_bucket_index()`：它只用于内存 bucket 定位，存在碰撞且不是内容身份。
- 不先 SHA-256 坐标生成 trie key：直接 key 无碰撞、少一次 digest、少 8 byte/leaf；Patricia
  已压缩共同前缀，不需要借随机化 key 控制未压缩深度。

## 5. 哈希域

- empty trie：`SHA256(domain-empty-v2)`，只作为树内部空值。
- leaf：`SHA256(domain-leaf-v2 || key || coordinate || cell_count || canonical cells)`。
- branch：`SHA256(domain-branch-v2 || branch_bit || left_hash || right_hash)`。
- content root：`SHA256(domain-content-v2 || descriptor || source || geometry_fingerprint ||
  total_cell_count || trie_root)`。
- 空 keyframe 的 content root 是非零规范值；Remove 仍使用全零 tombstone，明确区分“合法空地图”
  与“来源已移除”。

所有 domain 都通过 length-prefixed canonical string 写入。branch bit 沿根到叶严格递增；leaf
保留原始 coordinate，任何重复 key 或不一致 coordinate 都在提交前拒绝。

## 6. 增量与事务

- keyframe 从 chunked store 全量建树，成本仍为 `O(K + cells)`。
- delta 先按 canonical key 排序并去重 touched coordinates，重算对应 leaf；空 chunk 生成删除。
- 多个叶子用批量递归更新，共享 dirty path 只复制一次，避免逐叶更新反复复制共同祖先。
- candidate store 与 candidate root 都验证通过后同时 move-commit；失败只释放候选对象。
- revision-only/summary 复用 root，不创建节点；resync/keyframe replacement 全量重建；Remove
  清空 tree 并提交零 tombstone。

## 7. 版本边界

本任务不修改 ROS 消息或线上 schema。原型定义显式、ROS-free 的
`ContentIdentityDescriptor{scheme=MerklePatriciaSha256V2, chunk_edge=16,
coordinate_key_version=1, node_encoding_version=1}` 和 versioned digest wrapper。v2 update hash
必须覆盖 descriptor；v1/v2 base digest 不混链。

v2 原型只进入 core unit/conformance/performance harness。现有 `MapUpdate`、ROS conversions、
C4 route 和生产默认继续只接受 v1。Gate 通过后另立 schema/协商/rollout 任务，决定 descriptor
如何进入 wire；本任务不能靠两端相同的隐藏参数声称协议兼容。

## 8. 决策

采用“chunk 16 + batch persistent Merkle Patricia Trie + 192-bit canonical coordinate key”作为
唯一首轮候选。先完成独立 full-rebuild/incremental conformance，再接入 opt-in applier
harness；若节点内存或 dirty-path hash 次数不满足短 Gate，不继续生产迁移工作。

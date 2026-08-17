# 增量 Merkle 内容哈希 v2：技术设计

> 状态：原型、正确性与短 Gate 已完成；生产 schema/rollout 仍未实施。

## 1. 对照边界

```text
B: chunk 16 immutable COW + flat content hash v1
C: chunk 16 immutable COW + incremental Merkle content root v2
```

两组复用同一 `CellSnapshotStore`、canonical content 和 update chain。生产默认在 Gate 前仍为
Vector/v1；C4.1 的 storage-independent `CanonicalCellView` 保持消费者边界。

## 2. 结构与身份

首轮候选固定为 batch persistent Merkle Patricia Trie，不再并行实现多种树：

- chunk edge 固定 16；key 是无碰撞的 192-bit signed-coordinate canonical bytes；
- 压缩 trie 的 live node 数约 `2K-1`，branch bit 沿路径严格递增；
- leaf hash 提交 coordinate、cell count 和 chunk 内 canonical cells；
- branch hash 提交 branch bit、left hash 和 right hash；
- 最外层 content root 再提交 descriptor、source、geometry fingerprint、total cell count 和 trie root；
- delta 对排序后的 dirty leaves 做批量路径 COW，共同祖先只复制/重算一次。

完整比较、估算和拒绝项见 `research/merkle-structure-assessment.md`。

## 3. 候选数据流

```text
decoded keyframe/delta
  -> chunked COW candidate
  -> sorted unique touched chunk leaf mutations
  -> batch persistent Patricia candidate paths
  -> locally recomputed candidate root
  -> count/root/descriptor/resource validation
  -> atomic snapshot + tree commit
```

正确性 fixture 同时从完整 canonical content 独立重建 v2 root，并保留 flat v1 作为内容
oracle；性能 fixture 只执行 v2 路径。

## 4. 原型版本边界

- 新增 ROS-free `ContentIdentityDescriptor` 和 `VersionedContentDigest` 原型类型，不修改现有
  `MapUpdate` 或任何 ROS message。
- descriptor 固定 scheme、chunk edge、coordinate-key version 和 node-encoding version；v2
  update digest 必须包含 descriptor。
- v2 prototype update/applier harness 使用显式 wrapper，不能通过构造参数把裸 32-byte hash
  偷换为 v2。
- 现有 producer/receiver/C4 conversions 继续是 flat v1；只验证它们不接受或误路由 prototype v2。
- Gate 通过后另立 production schema/negotiation/rollout 任务。

## 5. 状态机

- keyframe：从 candidate chunk store 全量建树；空 keyframe 得到非零规范 root。
- delta：base descriptor/root 必须匹配；从 touched chunks 计算 batch candidate tree。
- revision-only/summary：cells 不变，复用 tree/root，节点分配为 0。
- keyframe/epoch replacement：不复用旧 root，全量建立新 source/geometry content identity。
- resync：只以显式 keyframe 恢复 v2 chain，不跨 v1/v2 接续 delta。
- remove：释放 tree，提交全零 tombstone；它不同于空 keyframe root。

candidate chunk store、candidate tree、revision 和 freshness 是一个提交单元。任何 decode、count、
descriptor、base/root、资源或 root oracle 失败都不得改变 committed state。

## 6. 兼容与回滚原则

- v1/v2 base identity 不混链；跨版本 delta 原子拒绝并要求显式 keyframe/resync。
- 未知版本 fail closed；update hash 必须覆盖内容身份版本和树参数。
- 任何 Gate 失败都可删除/禁用 opt-in v2，不影响默认 Vector/v1。
- C4 route、trust、QoS 和 C5 aggregate 在本任务中不接触 v2 字段。
- 原型文件和 opt-in harness 可整体删除，不影响生产 Vector/v1 与 C4.1 no-go 结论。

## 7. 测量原则

- B/C 同二进制身份和 workload，只改变 content identity implementation/version。
- 单独报告 leaf、path/root、keyframe build、callback、CPU、节点/owned bytes 和 PSS/USS。
- 先确定性 correctness，再短 A/B，再决定是否运行正式工具矩阵。
- 除 B/C 单因素归因外，保留当前生产 A=`Vector + flat v1` 作为影子参照，报告端到端、PSS/USS
  和阶段差异；本任务不为 A 设固定回退硬门。即使 C 通过 B/C Gate，生产迁移仍需独立任务
  基于 A/C 结果决定。

## 8. 最终 Gate 决策

- 保留 SHA-256 原语，只把 flat 完整流重算升级为 versioned incremental Merkle content
  identity；因此 CPU 改善可以独立归因于增量叶子/路径维护，而不是摘要算法替换。
- 固定 4 个 touched chunks 时，C 对 B 的 10k/100k/500k delta apply 加速为
  `20.5x / 189x / 858x`，证明 delta 成本已从 known-cell 总量解耦。
- 树的额外常驻 PSS 在三档规模为 `+1.7% / +5.2% / +6.9%`，且 owned bytes 随 live
  chunks 而非历史 revision 增长；keyframe 仍承担一次性 `O(N)` 建树成本。
- 该结果只批准后续生产迁移设计，不批准当前任务切换默认。生产 schema/协商、双读写、
  admission/rollback 和正式 ROS 矩阵完成前，Vector/v1 是唯一默认路径。后续 C4.3
  先完成生产集成；C5d 只消费 C4.3 通过后的正式 versioned content identity，不接触
  Patricia 内部节点。

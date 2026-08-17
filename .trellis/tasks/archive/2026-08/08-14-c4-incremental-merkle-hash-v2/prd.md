# 增量 Merkle 内容哈希 v2 可行性与验证

> 状态：原型与短 Gate 已完成。该任务以 C4.1 已提交的 chunk 16 COW 可选实现为固定存储基线，
> 只评估内容身份与哈希维护方式，不改变当前生产默认 `Vector + flat SHA-256`。
>
> 规划收敛：本任务交付 opt-in 原型、正确性验证、B/C Gate 和 go/no-go；生产 schema/rollout
> 不在本任务内实施。

## 1. 目标

设计并验证增量 Merkle 内容身份 v2，判断它能否把 delta 的内容哈希成本从完整 canonical
地图遍历收敛到 touched chunks 及其树路径，同时量化树节点常驻内存、keyframe 初次建树、
版本迁移和端到端 apply 成本。

## 2. 已确认事实

1. C4.1 同构建短 A/B 中，chunk 16 的总 apply mean 为 `4.369 ms`，其中 flat canonical
   hash mean 为 `4.169 ms`，约占 apply 的 95%；candidate build 已降至 `0.140 ms`。
2. chunk 16 COW 已作为可选 ROS-free store 落盘，生产默认仍是 Vector；本任务不重新选择
   `8/16/32`，也不把 C4.1 no-go 追溯改写为通过。
3. 协议 v1 只有 `protocol_version=1`、`canonical_encoding_version=1` 和
   `HashAlgorithm::Sha256`。flat 与 Merkle 都使用 SHA-256，现有算法枚举无法区分两种内容
   身份结构。
4. `MapUpdate`、resync、routed update、contributor revision 和 aggregate manifest 均携带
   裸 32 字节 hash；内容身份版本若改变，必须明确 producer/receiver/C4/C5 的拒绝、迁移和
   回滚边界。
5. 正确性模式可以同时计算 flat hash 与 Merkle root 做 oracle 校验；性能模式不能继续计算
   flat 全量 hash，否则无法测到 v2-only 的 CPU 收益。
6. 初始 keyframe 建树仍为 `O(N)`；增量收益只适用于后续 delta。revision-only、remove、
   resync keyframe 和 source/epoch replacement 必须单独定义。
7. 本机 CPU/绝对延迟受约 +/-30% 争用影响；确定性业务计数、阶段内比例、PSS/USS、节点数
   和 owned bytes 是主要证据，CPU 门只接受明显大于噪声的效应。

## 3. 需求

### R1 单因素归因

- 固定对照 B 为 `chunk 16 COW + flat SHA-256`，候选 C 为
  `chunk 16 COW + Merkle SHA-256 v2`。
- 两组使用相同 canonical content、delta、revision、构建类型、ELF 身份、workload、warmup
  和采样规则；不得同时改变 chunk edge、bucket count、wire payload 或聚合算法。

### R2 确定性 Merkle 身份

- 固定 chunk-coordinate key 编码、负坐标归一化、leaf/internal/empty-node domain separation、
  子节点顺序和空树根；禁止依赖进程随机 hash、指针、容器迭代顺序或 ABI padding。
- 插入、删除和稀疏空间扩张不能移动无关叶子或要求重算全部已有 chunk。
- 采用 batch persistent Merkle Patricia Trie。坐标键为三个 signed int64 经 sign-bit flip、
  big-endian 后按 `x/y/z` 连接的无碰撞 192-bit key；不使用 bucket hash 或坐标摘要代替 key。
- 压缩树常驻节点上界约为 `2K-1`；未压缩 Sparse Merkle Trie 因数百万前缀节点风险不进入
  首轮实现。具体比较和容量估算见 `research/merkle-structure-assessment.md`。

### R3 增量事务与原子性

- keyframe 从完整 chunked snapshot 一次性建树；delta 只重算 touched leaf 和祖先路径；
  revision-only 保持 root；空 keyframe 使用非零规范 root，remove 清空树并提交全零 tombstone。
- candidate tree 与 candidate chunks 同一事务提交。count、root、descriptor 或资源校验失败时，
  最后合法 snapshot、root、revision 和 freshness 均不改变。
- receiver 不通过信任 producer root 跳过验证；它必须从本地已提交 tree 和本次合法 delta
  独立重算候选 root。

### R4 版本与兼容

- 不能仅复用 `HashAlgorithm::Sha256` 声称兼容；必须为 flat content v1 与 Merkle content v2
  定义可判别的版本身份，并纳入 `update_hash`。
- 未知版本、base root 不匹配、错误 empty hash、树参数漂移和跨版本 delta 必须在 mutation
  前或候选提交前原子拒绝。
- 原型使用显式 ROS-free `ContentIdentityDescriptor` 与 versioned digest wrapper；v2 update hash
  覆盖 descriptor，不允许仅靠两端相同的隐藏测试参数区分身份。
- 生产默认、现有 `MapUpdate`/ROS schema 和 v1 路径保持不变；descriptor 的 wire 字段、双发、
  协商和 rollout 由 Gate 通过后的独立任务决定。

### R5 正确性验证

- 正确性模式对每个 deterministic replay checkpoint 同时物化 canonical cells，计算 flat v1
  oracle，并验证独立全量重建的 Merkle root 与增量 root 相同。
- 覆盖负坐标、chunk 边界、空树、单叶、奇数叶、插入、状态翻转、删除最后 cell、
  revision-only、keyframe replacement、epoch replacement、remove 和 resync recovery。
- 固定 golden vectors 必须跨进程、跨运行和相同构建重复一致。

### R6 性能与内存

- 分别记录 leaf hash、path update/root、candidate build、commit、callback、节点分配/共享、
  Merkle owned bytes、PSS/USS 和 keyframe 建树成本。
- 先运行短 B/C 筛选。delta hash/apply 必须表现为随 touched chunks 而非 known cell 总数增长；
  若 CPU 改善不明显大于本机噪声，或新增常驻内存抵消收益，判为 no-go。
- 当前生产 A=`Vector + flat SHA-256` 只作为影子参照，报告端到端、PSS/USS 和关键阶段差异，
  不预设“回退不超过 10%”等硬约束；生产迁移是否值得由 Gate 结果和后续独立任务判断。
- 只有短筛选通过后才投入 3 x 300 秒正式矩阵、Heaptrack、ASan/LSan 和严格 Memcheck；
  未执行项必须如实标记，不能写成通过。

### R7 本任务交付边界

- 本任务止于 opt-in Merkle v2 原型、正确性证明、性能/内存 Gate 和 go/no-go 结论。
- 即使 Gate 通过，本任务也只输出生产迁移建议；生产 hash v2 schema、跨节点协商、双发/切换
  和 rollout 另建任务实施。
- 原型所需的版本身份可以在 ROS-free 边界内显式建模，但不得借原型参数隐式改变线上协议语义。

## 4. 验收标准

- [x] 选定并记录树结构、coordinate key、domain separation、empty hash 和版本策略。
- [x] golden vectors 与 full-rebuild/incremental root conformance 全部通过。
- [x] prototype producer/applier 对合法 keyframe/delta/root 完全一致，已覆盖的拒绝状态保持原子。
- [x] 正确性模式证明 Merkle root 对应同一 canonical content；性能模式不计算 flat 全量 hash。
- [x] B/C 短 A/B 使用同一构建和 workload，完整报告阶段耗时、CPU、PSS/USS 和树节点成本。
- [x] 给出 go/no-go：算法 Gate go，但生产 rollout no-go，默认继续 `Vector + flat SHA-256`。
- [x] C5 仍只消费 storage-independent canonical view/content identity，不依赖 Merkle 内部节点。

## 4.1 最终量化结论

- 本任务没有替换 SHA-256 算法；优化的是内容身份结构，从每次 delta 遍历完整 canonical
  map 的 flat SHA-256，改为 edge-16 chunk 叶子与 persistent Patricia 路径的增量 SHA-256。
- 固定每次更新 4 个已有 chunk 时，B=`chunk 16 + flat` 到 C=`chunk 16 + Merkle v2`
  的 10k/100k/500k apply 分别从 `483.3 us / 6.131 ms / 40.22 ms` 降到
  `23.6 us / 32.4 us / 46.9 us`，即 `20.5x / 189x / 858x`。
- C 相对 B 的 PSS 增量为 `92 / 446 / 1586 KiB`（`1.7% / 5.2% / 6.9%`）；
  ASan/LSan/UBSan、严格 Memcheck 和 Heaptrack 未发现业务泄漏或内存错误。
- keyframe 初次建树仍为 `O(N)`，10k/100k 下分别比 B 慢约 `23% / 8%`，500k 下为
  `0.92x`；本机时间噪声约为 +/-30%，因此不把这些小差异解释为稳定方向性收益。
- 最终决策为 **B/C 算法 Gate GO、生产 rollout NO-GO**。生产继续
  `Vector + flat SHA-256 v1`；v2 schema、协商、资源 admission、回滚和 3 x 300 秒 ROS
  集成矩阵由 C4.3 生产集成任务完成。C5a-C5c 不依赖内容身份内部结构，可以独立推进；
  C5d EdgeAggregator 必须等待 C4.3 Gate，避免聚合链路建成后再次迁移 wire identity。

## 5. 非目标

- 不重新选择 chunk edge 或修改 C4.1 的 Gate B 结论。
- 不实现 C5 EdgeAggregator、跨来源融合或哈希证明交换。
- 不以第三方攻击防护、远程 inclusion proof 或内容寻址存储为首轮目标。
- 不在证据门前删除 flat v1、改变生产默认或把双算结果当作性能收益。

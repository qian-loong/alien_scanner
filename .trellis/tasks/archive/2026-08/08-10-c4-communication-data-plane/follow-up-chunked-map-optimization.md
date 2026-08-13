# 后续分块地图与写时复制决策

## 1. 推荐时机

不在缺少数据时立即重构，也不等 C5/C8 整条链路完成后再处理。推荐顺序：

```text
C4 功能基线（add6ca3）
-> C4 性能/内存质量门
-> 分块决策门
-> 必要时独立实现分块 + COW
-> C5 正式 EdgeAggregator
```

这样可以避免 C5 聚合器、可视化和后续消费者绑定当前完整
`ReconstructedMap::cells` vector 表示，降低后续重构扩散面。

## 2. 当前实现的成本模型

传输已经通过 C3 delta 增量化，但 B 端 `MapUpdateApplier` 应用 delta 时仍遍历旧完整
cell vector、构造完整候选 vector、验证后 swap。

设每来源完整地图为 `N` 个已知体素，本次变化为 `K` 个操作：

```text
当前 delta 时间：O(N + K)
当前 delta 瞬时内存：旧地图 N + 候选地图约 N + delta/decode K
当前稳态内存：每来源一份完整地图 O(N)
```

delta 传输降低网络字节，但不会自动消除 B 端完整候选地图的复制成本。

## 3. 候选优化

首选候选是固定空间块加不可变块、写时复制：

- 例如以 `16 x 16 x 16` 体素坐标范围作为一个逻辑空间块；具体尺寸必须由基准验证，
  不能提前写死为生产契约。
- 快照保存有序块索引和共享的不可变块对象。
- delta 只复制受影响的块和顶层索引，未变化块继续共享。
- 新快照完成 hash/资源校验后原子替换；旧快照在最后一个 reader 释放后回收。
- keyframe、canonical hash 和 wire encoding 语义保持不变，块只是接收端内部表示。

预期成本模型：

```text
稳态仍至少 O(N)，因为当前阶段要求保留已知地图
delta 临时内存约 O(changed blocks + top-level index)
delta 更新成本主要随受影响块数增长，而不是每次复制全部 N
```

分块不能让持续探索的地图容量凭空变成常数。若产品要求总内存硬上限，还需要另外定义
有界空间、区域淘汰、分层摘要或磁盘换出策略；这些策略会改变历史空间可用性，不属于
写时复制本身。

## 4. 启动优化的触发条件

C4 质量门出现以下任一结果时，在 C5 前启动独立优化任务：

- bounded 无泄漏，但小 delta 的 apply 成本随完整地图 `N` 明显增长。
- delta 的典型修改比例很小（建议观察 `<5%`），却仍复制接近完整地图。
- keyframe/delta 瞬时 PSS/USS 或 peak heap 接近 B 的部署内存预算。
- 2/4/8 来源矩阵出现吞吐不守恒、DDS history 压力、freshness 下降或 resync 增加。
- 同 run 阶段分解显示 decode/apply 是接收端主要成本，且预期改善远大于本机 30--50%
  可分辨效应下限。

若未触发，允许推迟实现，但 C5 必须通过只读遍历/快照访问边界消费接收地图，不得让
聚合算法直接拥有或依赖 `std::vector<CanonicalCell>` 的具体存储布局。

## 5. 实施边界

未来优化任务必须保持：

- C3 wire schema、canonical encoding、revision/hash、resync 和原子 apply 语义不变。
- C4 routed envelope、信任拒绝、TTL 和 QoS 契约不变。
- 同一输入的完整 canonical cells、content hash 和 update hash 与当前实现等价。
- ROS-free 算法库加薄 ROS node 分层；不为测试给被测类新增无正当性的接口。
- 保留当前 vector applier 作为基准/回滚实现，直到分块实现完成等价性和资源验收。

## 6. 优化验证矩阵

分块实现必须复用 `performance-memory-quality-gate.md` 中完全相同的 seed、source 数、
地图规模、delta 密度和正式窗口，进行当前 vector 与分块+COW 的 A/B：

- 每个 revision 的 source、geometry、content hash、完整 canonical cells 完全相同。
- 比较稳态 PSS/USS、delta/keyframe peak heap、分配次数和 throughput/freshness。
- CPU/延迟只采信数量级改善；小于 10% 的差异不作为本机判定门。
- ASan/LSan/Memcheck 对两种实现使用相同最终体素规模。

块尺寸至少比较两个候选值，并报告块元数据开销、稀疏填充率、changed-block 数和峰值；
不能只选对某一个合成几何最有利的块尺寸。

## 7. 当前决策状态

- C4 质量门已经完成，优化触发条件已满足：固定 256-cell delta 下，地图从每来源
  10k 增至 100k、500k 时，apply p95 从约 `0.52 ms` 增至 `4.19 ms`、
  `16.78--33.55 ms`；apply 始终占 callback 的 `97--99.9%`。
- 固定容量的 18 个正式样本没有时间相关无界增长，但 `2 x 10k / 100k / 500k` 的
  平均 PSS 从 `19.76 MiB` 增至 `31.72 MiB`、`84.70 MiB`；大 keyframe 的 peak heap
  为 `90.75 MiB`。这属于完整地图容量和候选复制成本，不是泄漏。
- 决策：建议在 C5 正式 `EdgeAggregator` 实现前创建独立 Trellis 子任务，实施分块
  不可变快照 + COW，并复用本文第 6 节矩阵做 vector 基线 A/B。
- 当前 C4 任务只记录决策和基线，不直接修改 C3/C4 存储表示；分块实现仍需单独规划、
  方案审核和用户启动确认。

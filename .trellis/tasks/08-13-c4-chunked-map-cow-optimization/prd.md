# C4.1 分块地图布局估算与 COW 优化

> 状态：规划中。C4 多来源接收数据面已经完成并通过功能、性能和内存质量门；
> 本任务是 C5 正式 EdgeAggregator 之前的接收端存储前置优化，不改变 C4 wire
> 协议，也不实现地图聚合。

## 1. 目标

把 B 端 `MapUpdateApplier` 的 delta 候选状态从“每次复制完整 canonical cell
vector”改为“只复制受影响空间块并共享未变化块”，降低小 delta 在大地图上的复制量、
临时内存峰值和分配压力，同时保持 C3/C4 的 revision、hash、resync 和原子提交语义。

任务先用只读布局估算器比较 `8 x 8 x 8`、`16 x 16 x 16`、`32 x 32 x 32`
三档空间块。`16` 是当前首选默认值，但不是生产契约；实现必须由代表性三维数据和
分散更新最坏场景的证据决定。

## 2. 已确认事实

1. 当前 delta apply 对每来源完整地图执行合并并构造完整候选 vector，时间和瞬时内存
   模型约为 `O(N + K)`，其中 `N` 是已知 cell 数，`K` 是本次 operation 数。
2. C4 正式质量门中，固定 256-cell delta 在每来源 10k、100k、500k cells 时，apply
   p95 约为 `0.52 / 4.19 / 16.78--33.55 ms`，apply 占 callback `97--99.9%`。
3. `2 x 10k / 100k / 500k` 平均 PSS 约为 `19.76 / 31.72 / 84.70 MiB`；
   大 keyframe replacement 的 peak heap 为 `90.75 MiB`。固定容量正式样本没有
   随时间无界增长，因此当前问题是容量和完整候选复制，不是泄漏。
4. 当前 C4 资源 workload 把所有 cell 放在 `(x, 0, 0)`，并修改连续的前 256 个 x
   cell。它会偏向块元数据更少的 `32`，不能单独决定真实三维地图的块边长。
5. C3 `content_hash` 是完整 canonical cell 流的 SHA-256。只要 wire/hash v1 保持不变，
   接收端校验仍至少需要遍历全部已知 cell；分块能消除完整候选复制，但不能承诺整个
   apply 时间完全与 `N` 无关。
6. `VoxelIndex` 允许负坐标，分块坐标必须使用数学 floor division；C++ 有符号整数
   直接除法的向零截断不满足该契约。

## 3. 需求

### R1 只读布局估算

- 在不改变生产 apply 行为的前提下，对同一 snapshot/delta 分别模拟块边长
  `8 / 16 / 32`。
- 每档至少输出：总块数、每块 cell 数 P50/P95/max、稀疏填充率、每次 delta 触及块数、
  受影响块内现有 cell 数、预计复制 cell 数、写放大率、未变化块共享率、目录/块元数据
  估算，以及固定分片目录的 bucket occupancy P50/P95/max。
- 估算必须覆盖负坐标、边界坐标、插入、删除、状态翻转和 revision-only delta。
- 估算输入必须包含三类：现有一维 C4 workload、生产 C2/C3 replay 产生的三维
  canonical snapshots/deltas、人工空间分散最坏场景。
- 报告必须单列三类输入，不允许用一维 workload 的综合平均值掩盖三维和最坏场景。

### R2 候选选择门

- 首要比较代表性三维 replay 的复制 cell 数、触块数和元数据；其次检查人工分散更新
  的最坏写放大；最后才比较现有一维兼容基线。
- 若某档在代表性三维 workload 的复制 cell P95 至少比 `16` 改善 20%，且元数据估算
  与人工分散场景的 P95 均不恶化超过 20%，允许替换 `16`。
- 若没有候选形成上述明确优势，选择 `16`，原因记录为“折中默认”，不得写成普适最优。
- 若三档在代表性局部更新上都不能明显降低相对完整 vector 的复制量，停止正式替换，
  保留估算证据并重新评审索引/哈希方案。

### R3 分块不可变快照与 COW

- 每个空间块覆盖所选边长的体素坐标范围，块内只保存已知 cell，保持确定性有序。
- 已提交快照和块对象不可变；delta 只复制受影响块，未变化块通过共享所有权复用。
- 顶层索引不得在每次 delta 时按总 chunk 数完整深拷贝。首选固定大小、确定性分片目录：
  候选快照只复制固定目录和受影响 bucket/chunk；bucket 分布必须纳入 R1/R5 证据。
- keyframe 可从解码 vector 一次性建立分块快照；delta 不得重新构造完整 candidate
  cell vector。
- 对外提供与存储布局无关的只读 canonical 遍历、cell count 和显式物化边界；C5
  消费者不得依赖 `std::vector<CanonicalCell>` 的所有权或布局。
- 当前 vector 路径保留为测试/性能 A/B 和回滚实现，直到本任务验收完成；运行时不得
  同时持有两份权威地图状态。

### R4 语义兼容

- 不改变 C3/C4 ROS 消息、canonical encoding version、content/update hash、
  revision、epoch、tombstone、resync、route、trust 或 QoS 契约。
- 同一合法输入在 vector 与分块路径上必须得到完全相同的 source、geometry、revision、
  known cell count、canonical cells、content hash、update hash 和 apply status。
- gap、冲突、损坏 payload、资源超限、旧 epoch 和 tombstone 后更新必须保持原子拒绝；
  失败候选不得改变最后合法快照或 freshness。
- peak apply admission 必须按分块候选的真实持有量重新建模，且不得弱化现有资源上限。

### R5 性能与内存验收

- 先运行相同进程、相同输入下的 vector/chunked 短 A/B，分别记录 decode、候选构建、
  canonical hash、commit 和 callback，不能只报告总 apply。
- 本任务的归因对照固定为 `vector + flat SHA-256` 与
  `chunked COW + flat SHA-256`；两组必须使用相同 wire/hash v1、输入、构建类型和采样
  规则。两组差值用于衡量存储复制、索引和分配优化，不得归因于哈希算法变化。
- 正式复用 C4 的 seed、source 数、地图规模、256 operations、10 Hz、RelWithDebInfo
  身份和 300 秒三轮窗口；至少覆盖 `2 x 100k`、`8 x 100k`、`2 x 500k` bounded，
  以及 expanding 和 keyframe replacement。
- 分块 delta 的实际复制 cell 数必须与估算器一致，代表性三维局部更新的 P95 必须低于
  当前完整地图 cell 数的 5%；若无法满足，任务不得宣称复制复杂度已收敛。
- 分块路径不得出现消息/revision/hash/cell count 不守恒、backlog、额外 resync 或
  freshness 回退。端到端耗时必须如实披露完整 hash 的 `O(N)` 下界，不设置低于本机
  可分辨噪声的虚假硬门。
- 比较 PSS/USS、delta/keyframe peak heap、分配次数和阶段耗时；若 keyframe 因 wire
  全量语义没有改善，必须单独说明，不能用 delta 结果代替。
- ASan/LSan 与严格 Memcheck 必须为 0 业务内存错误和 0 definite/indirect/possible leak。

## 4. 验收标准

- [ ] 估算器对 `8/16/32` 和三类 workload 生成可复现、带输入身份的报告。
- [ ] 负坐标 floor division、块边界和指标计算有确定性 gtest。
- [ ] 块边长按 R2 规则选出并记录；无明确优势时使用 `16`。
- [ ] 分块快照只复制触及块及对应目录 bucket，测试证明未变化块对象身份保持共享。
- [ ] delta 路径不分配完整 candidate vector，顶层索引复制量不随总 chunk 数线性增长。
- [ ] vector/chunked conformance 覆盖所有 C3 apply 成功与拒绝状态并逐 revision 等价。
- [ ] C2/C3 三维 replay 每个 checkpoint 的 cells/hash/revision 完全一致。
- [ ] C4 core、route、resync、trust、cave visual launch tests 全部通过，无跳过。
- [ ] 短 A/B、正式性能/内存矩阵、Heaptrack、ASan/LSan、Memcheck 证据完整且结论边界正确。
- [ ] C5 前获得稳定的存储无关只读遍历边界，不再要求直接拥有 cell vector。

## 5. 非目标

- 不修改 wire schema 或引入新的分块网络协议。
- 不实现 C5 EdgeAggregator、地图对齐、跨来源融合或全局地图显示。
- 不通过淘汰历史区域、磁盘换出或摘要化把持续扩张地图变成常量内存；这些会改变产品语义。
- 不把 `content_hash` 改成 Merkle hash，也不降低接收端完整内容校验强度。
- 不在本任务的生产实现或正式对照中混入 Merkle；Merkle 作为独立后续任务，以本任务
  验收后的 `chunked COW + flat SHA-256` 为基线，单独衡量哈希阶段和端到端收益。
- 不以当前一维 workload 的结果宣称真实洞穴或无线部署性能。
- 不删除旧 vector 基线，直到用户完成本任务验收并明确同意后续清理。

## 6. 证据位置

- C4 决策与历史基线：
  `.trellis/tasks/archive/2026-08/08-10-c4-communication-data-plane/`
- C4 原始本地证据：
  `profiling-archive/c4-communication-data-plane-20260812/raw/`
- 本任务可审计摘要：任务目录下 `validation/`
- 本任务完整本地原始证据：完成采集后迁移到独立的
  `profiling-archive/c4-chunked-map-cow-<date>/raw/`，不把大体积 raw 纳入 Git。

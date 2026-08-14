# C4.1 分块地图布局估算与 COW 优化：技术设计

## 1. 边界与数据流

本任务只改变 C3 `MapUpdateApplier` 在接收端保存和构造 reconstructed occupancy 的
内部方式：

```text
C4 routed envelope
  -> 现有 decode / admission / base revision 校验
  -> vector 或 chunked candidate store
  -> 按 canonical (x,y,z) 顺序重算 v1 content hash
  -> count/hash/resource 校验
  -> 原子提交 immutable snapshot
  -> 存储无关只读 canonical view
```

生产节点、ROS conversion、wire payload 和 C4 routing 均不感知空间块。分块不是新的
跨进程协议，也不是 EdgeAggregator 的融合算法。

## 2. 估算阶段

### 2.1 输入

1. **一维兼容基线**：复现 `C4ResourceProfileSource.cpp` 的 `(x,0,0)` cell 和连续
   256-cell alternating/expanding delta，用于解释历史 C4 数字，不能主导选型。
2. **三维生产 replay**：复用 `perception_profiling::MapUpdateReplayOracle` 驱动真实
   C2 mapper、canonical adapter 和 C3 producer，取得连续 exact-revision snapshots，
   用生产 `SnapshotDiffer` 得到 operations。这样不再发明第二套洞穴地图生成器。
3. **人工分散最坏场景**：在固定三维 snapshot 上让每个 operation 尽量命中不同空间块，
   检查 touched chunks、bucket copy 和元数据上界。

### 2.2 坐标与指标

块坐标定义为：

```text
chunk_axis = floor(voxel_axis / chunk_edge)
local_axis = voxel_axis - chunk_axis * chunk_edge
```

其中 `local_axis` 始终位于 `[0, chunk_edge)`。实现必须对 `INT64_MIN/MAX` 做检查，不能
依赖向零截断。

对每个候选边长和每个 delta 计算：

```text
touched_chunks
copied_cells = sum(每个 touched chunk 更新前的已知 cell 数)
write_amplification = copied_cells / max(1, operation_count)
shared_chunk_ratio = 1 - touched_existing_chunks / total_chunks
sparse_fill_ratio = known_cells / (chunk_count * edge^3)
```

同时估算固定目录、bucket entry、共享指针、chunk 对象和 cell payload 字节。所有
`sizeof` 估算都标注编译器/ABI，只用于同构建候选比较，不外推为部署内存承诺。

## 3. 分块快照

### 3.1 逻辑对象

- `ChunkCoordinate`：三轴有符号块坐标和确定性比较/哈希。
- `ImmutableCellChunk`：一个空间块内严格有序的已知 `CanonicalCell`；空块不保留。
- `ChunkDirectoryBucket`：按完整 `ChunkCoordinate` 排序的 chunk entry vector，每个
  entry 指向不可变 chunk。
- `ChunkedCellSnapshot`：固定数量的不可变 bucket 指针目录、总 cell/chunk 数和布局
  参数；已提交后不可修改。
- `CanonicalCellView`：提供 `size`、canonical traversal 和显式 materialization，
  隐藏 vector/chunked 的所有权与布局。

固定 bucket 数在实现前由 estimator 同时报告 occupancy 分布；bucket 选择使用版本内
固定的确定性整数混合，不使用进程随机 salt。目录大小是与地图容量无关的常量；创建
候选时浅复制目录，只深复制被 delta 命中的 bucket 和 chunk。

### 3.2 Delta 事务

1. 完成现有 envelope、base revision/hash、payload size 和 decode 校验。
2. 按 `ChunkCoordinate` 对已排序 operations 分组，拒绝重复/无序/非法 operation。
3. 浅复制固定目录；每个受影响 bucket 只复制一次。
4. 对每个受影响 chunk 合并原 chunk cells 与该组 operations，构造新的不可变 chunk；
   未变化 chunk 的 `shared_ptr` 保持对象身份。
5. 校验 candidate count 和分块 peak-memory admission。
6. 通过 candidate 的 canonical cursor 以协议 v1 的全局 `(x,y,z)` 顺序重算
   `content_hash`。空间 chunk 的自然遍历顺序不等于全局 cell 顺序，因此 cursor 必须
   做确定性归并，不能按 chunk 顺序直接 hash。
7. hash 完全匹配后一次替换 committed snapshot；任何失败都丢弃 candidate overlay。

完整 canonical SHA-256 仍为 `O(N)`。实现需把 candidate build 与 hash traversal 分段
计时，以证明分块实际优化了哪一部分；本任务不以修改 hash 契约掩盖该下界。

### 3.3 Keyframe、summary 与 remove

- keyframe：解码仍得到完整 vector；校验后一次性按 chunk 分组建立新 snapshot，旧
  snapshot 在最后 reader 释放后回收。wire 全量 payload 峰值仍存在。
- summary：只检查 metadata/count/hash，不复制任何 chunk。
- remove：提交空 snapshot/tombstone，释放当前 snapshot 所有权；旧 reader 可自然退出。
- revision-only delta：只更新 metadata，全部 bucket/chunk 保持共享。

## 4. Vector 基线与切换

`MapUpdateApplier` 继续拥有 protocol/admission 状态机，cell candidate storage 抽到一个
ROS-free 存储边界。由于本任务确实存在 vector 和 chunked 两种实现并需要 A/B/回滚，
该边界满足仓库的接口使用条件；不为节点、codec 或被测 applier 本身额外造接口。

首轮实现允许通过构造配置选择 `Vector` 或 `Chunked`，默认在布局门通过后设为所选
chunk edge。单个 applier 只持有一种权威 storage；禁止为了比较在生产 hot path 同时
更新两份地图。A/B 由两个独立 receiver run 或测试实例完成。

对外消费者改用 `CanonicalCellView`：

- hash、OctoMap materialization 和 oracle compare 使用 streaming traversal；
- 仅测试输出或明确兼容边界允许调用 `materialize()`；
- C5 后续只能消费 view，不得向下转型或引用 chunk/vector 内部类型。

## 5. 资源 admission

现有 `max_receiver_cells`、payload、operation 和 revision 上限保持不变。原先
`current vector + candidate vector + operations + payload` 的 peak 估算替换为：

```text
committed snapshot live bytes
+ fixed candidate directory
+ copied bucket metadata
+ cloned/new chunk payload
+ decoded operations
+ payload/decode buffers
+ canonical cursor scratch
```

所有加乘使用 checked arithmetic。配置上限约束候选的可计费 owned bytes，不能因为
共享块而把仍然存活的 committed map 从部署容量核算中消失。

## 6. 兼容与回滚

- wire/schema/hash 版本不变，因此旧 producer 与新 receiver 可直接互通。
- vector storage 保留到本任务结束，任何语义或资源门失败可切回 vector，不回滚 C4。
- 若 estimator 否决三档，停止在估算阶段，不修改默认 storage。
- 若 chunked conformance 通过但性能未改善，保留实现和证据但默认仍为 vector，由用户
  决定是否继续哈希/索引专项；不得仅凭“代码已完成”切换默认。

## 7. 主要风险

1. **canonical 顺序错误**：chunk 顺序与 cell 全局顺序不同。用 golden hash、负坐标和
   跨块交错数据验证 cursor。
2. **顶层索引仍线性复制**：使用固定目录 + bucket COW，并把 copied bucket entries
   作为强制指标；不接受完整 `std::map` clone。
3. **稀疏更新写放大**：人工每 operation 一块的 workload 单列报告，必要时回退到 8。
4. **bucket 倾斜**：记录 occupancy max/P95；确定性 hash 的碰撞最坏情况受资源门约束。
5. **读取边界物化回归**：搜索所有 `.cells` 消费者，生产路径改成 view traversal；禁止
   在 OctoMap/诊断定时器中无意缓存第二份完整 vector。
6. **只优化复制未优化 hash**：阶段计时单列 candidate/hash，不宣称端到端 `O(K)`。

## 8. 证据与可重复性

布局报告记录 git commit、ELF/hash、候选参数、fixture seed、snapshot/update hash 和
输入 cell/operation count。正式 A/B 复用 C4 runner 的身份/守恒/采样规则，新增
`storage_mode`、`chunk_edge`、chunk/bucket/copy 指标。任务目录只保留摘要与 artifact
hash，完整 raw 迁移到独立 profiling archive。

## 9. 优化归因与后续哈希任务

两类优化按以下顺序独立验收：

```text
A: vector storage + flat canonical SHA-256       （同环境重跑基线）
B: chunked COW    + flat canonical SHA-256       （本任务）
C: chunked COW    + incremental Merkle hash v2   （独立后续任务）
```

`A -> B` 只改变候选存储，衡量 copied cells/bytes、bucket/chunk 分配、candidate build、
PSS/USS 和 peak heap；完整 hash 的协议与算法保持不变。`B -> C` 保持同一分块存储和
workload，主要衡量 canonical hash 阶段、callback CPU 和吞吐变化。每次都报告分阶段
耗时，不能只比较总 apply。

这些收益是“在固定另一因素下”的条件收益，不保证可简单相加；缓存局部性和 traversal
方式会产生交互。因此正式报告还要保留 `A -> C` 的端到端总改善，但不把总改善重复
分摊给两个子项。为了消除机器负载和构建漂移，A 不能只引用历史 C4 数字，需与 B 在
同一构建身份和测量窗口中重跑。

不优先实现 `vector + Merkle` 第四组合：增量 Merkle 需要持久的空间叶子/脏路径状态，
在 vector 路径旁维护影子分块索引会引入额外内存和同步成本，反而不能代表任一生产
方案。只有后续需要严格的 2x2 因子实验时，才把它作为测量专用实现单独评审。

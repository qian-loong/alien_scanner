# C4 性能与内存质量门

## 1. 目的与结论边界

本质量门测量当前 C4 在接收端 B 的通信数据面成本：ROS routed envelope 解码、
协议校验、按来源维护的 `MapUpdateIngress`、C3 `MapUpdateApplier` 以及最后合法地图。

本质量门不测量未来 C5 `EdgeAggregator` 的正式地图聚合算法，也不把洞穴生成、
source fixture、RViz、Marker 或其他可视化进程的资源计入 B。所有结论必须表述为
“C4 多来源接收数据面”，不得表述为“地图聚合性能”。

当前 B 的预期内存组成是：

```text
ROS/RMW 基础开销
+ 每来源 (Ingress 状态 + 有界去重账本 + 一份最后合法完整地图)
+ DDS 有界 history
+ 当前消息 decode/apply 的临时内存峰值
```

必须区分两类增长：

- 固定来源数和固定已知体素数下，内存仍随运行时间增长：非预期增长，需定位泄漏、
  无界历史或资源未释放。
- 探索空间扩张导致已知体素数增加：当前完整接收地图设计下的预期容量增长，需报告
  每来源和每体素成本，不能称为泄漏。

## 2. 专用 workload

质量门使用关闭 RViz 的独立 workload，不复用真实洞穴验收 fixture：

- source fixture：固定 seed 生成合法、确定性的 C3 keyframe/delta，并通过生产 C4
  envelope conversion 发布。
- receiver target：使用生产 conversion、validation、`MapUpdateIngress` 和
  `MapUpdateApplier`，每个来源维护一个独立接收状态。
- sampler/runner：只采样 receiver target 的 PID；fixture、采样器和分析器不计入业务
  CPU/RSS/PSS/USS。

冻结参数：

- 默认 10 Hz/source，固定 seed；每次 delta 修改固定 256 个体素。
- 标准 `Reliable + Volatile + KeepLast(4)`；不使用厂商私有 API。
- 默认无故障注入；fault/resync 是独立正确性场景，不混入容量基线。
- source 数、目标体素数、正式窗口和 build identity 写入每个 run manifest。
- 每个来源的发送、解码、应用、最终 revision、content hash 和 cell count 必须守恒。

当前生产 receiver 没有应用层异步队列，订阅回调同步执行 decode/apply。因此报告
`application_queue_peak=0`，并单独报告 DDS history 配置；标准 `rclcpp` 无法读取的
DDS 实时队列深度不得伪造。

## 3. 测试矩阵

### 3.1 短 smoke

- `2 sources x 10k cells/source`，30 秒。
- 验证 PID 身份、消息计数、revision/hash、资源采样、正常退出和无残留进程。
- smoke 只证明接线和证据链有效，不产生正式容量结论。

### 3.2 来源数扩展

| Sources | Cells/source | 用途 |
| ---: | ---: | --- |
| 1 | 100k | 单来源基准 |
| 2 | 100k | 当前 A1/A2 -> B 形态 |
| 4 | 100k | 中等多来源 |
| 8 | 100k | 来源数扩展趋势 |

### 3.3 地图规模扩展

| Sources | Cells/source | 用途 |
| ---: | ---: | --- |
| 2 | 10k | 小图基准 |
| 2 | 100k | 当前阶段主基准 |
| 2 | 500k | 大图容量与峰值 |

### 3.4 场景类型

- bounded：达到目标规模后只修改既有体素。用于稳态内存、时间相关增长和吞吐守恒。
- expanding：从 10k 增长到 500k cells/source。用于每体素成本和增长曲线，不用于泄漏
  斜率判定。
- keyframe replacement：固定大图连续替换 keyframe。用于旧地图、候选地图、payload
  和 decode buffer 同时存活时的瞬时峰值。

正式 bounded 场景在地图平台期之后采集 300 秒，运行三轮独立样本。长矩阵开始前必须
先通过 smoke；不得因为短 smoke 数字看起来合理而省略正式窗口。

## 4. 构建与证据身份

性能与内存矩阵使用独立 RelWithDebInfo 前缀：

```text
-O2 -g -DNDEBUG -fno-omit-frame-pointer
```

每次 run 记录：

- git commit、镜像 ID、内核、RMW、CPU/cgroup 信息。
- target ELF SHA-256、build-id、compile command、`ldd` closure。
- workload seed、source 数、cells/source、delta operations、频率和窗口。
- target/fixture/sampler PID、starttime、退出码和产物摘要 hash。

性能前缀、ASan 前缀和普通开发工作区不得混用。

## 5. 指标与工具

### 5.1 业务与传输指标

- application serialized bytes：对 routed ROS message 做 CDR serialization 后的字节数。
  该值不含 DDS/RTPS、IP 和链路层头部，必须明确标注。
- payload bytes、keyframe/delta 数、source/revision/hash/cell count。
- envelope decode、ingress/apply 和端到端 callback 的 steady-clock 阶段时间。
- origin age/freshness、发送/接收/应用计数、resync/rejection 数。
- `application_queue_peak=0` 和配置的 DDS history depth。

CPU 与延迟绝对值受当前 LinuxKit 宿主约 +/-30% 争用影响，只报告量级；同一 run 内
decode/apply 阶段比例可用于判断主要成本。不得建立小于 10% 的 CPU A/B 门。

### 5.2 RSS/PSS/USS

runner 每秒读取 receiver 的 `/proc/<pid>/smaps_rollup`，并用 `smem` 交叉检查：

- RSS、PSS、USS。
- Private Clean/Dirty、Shared Clean/Dirty。
- 正式窗口的均值、峰值和线性回归斜率。

多进程容量求和以 PSS 为主，RSS 只作单进程驻留量，USS 作为退出后可回收内存下界。

### 5.3 Heaptrack

至少覆盖 `2 x 100k bounded` 和 `2 x 500k keyframe replacement`：

- peak heap、正式结束时存活堆、分配次数。
- 最大存活和临时分配调用栈。
- keyframe/delta 是否存在不必要的完整地图重复复制或分配抖动。

Heaptrack 的时间数字不作为实时性能结论；分配次数、peak heap 和调用栈用于解释
RSS/PSS/USS。

### 5.4 ASan/LSan

独立构建使用 `-O1 -g -fsanitize=address -fno-omit-frame-pointer`，至少覆盖 bounded、
expanding、keyframe replacement 和多来源正常销毁。允许降频，但 expanding 必须达到
普通矩阵相同的最终体素规模。

要求：0 heap buffer overflow、0 use-after-free、0 double-free、0 direct/indirect leak。

### 5.5 Valgrind Memcheck

对 ROS-free 或最小 receiver workload 降频运行，要求：

- 0 invalid read/write、0 uninitialized read。
- definitely/indirectly/possibly lost 均为 0。
- 第三方静态注册产生的 `still reachable` 单独披露，不与业务泄漏混淆。

Memcheck 不用于性能或吞吐结论。

## 6. 有效性与判定门

run 必须同时满足：

- target、fixture、sampler 身份稳定且正常退出；无残留进程。
- 所有 source 的发送、解码、应用计数和最终 revision/hash/cell count 守恒。
- 无 rejection、gap、resync 或 backlog，除非该 run 明确是 fault 场景。
- bounded 正式窗口已在目标体素平台期内。
- 三轮 bounded 的 PSS/USS 持续增长斜率均小于仓库现行门槛 `1024 KiB/min`。
- 去重账本不超过 `max_recent_messages x source_count`，当前默认是 `256 x sources`。
- ASan/LSan/Memcheck 无业务内存错误。

若任何阻塞门失败，保留原始产物并标记 `valid=false`，不得把该 run 纳入基线。

## 7. 输出与分块决策输入

最终报告至少给出：

```text
稳态内存 = ROS/RMW 基础量 + 每来源固定量 + 每已知体素成本
keyframe/delta 最坏瞬时内存峰值
固定地图下是否存在时间相关的非预期增长
来源数和地图规模对 decode/apply、freshness 与吞吐守恒的影响
```

测试结果随后进入 `follow-up-chunked-map-optimization.md` 的决策门。若当前完整 vector
实现仍有足够余量，可以推迟分块；若小 delta 成本明显依赖完整地图规模、峰值接近预算
或多来源出现积压，则在 C5 正式聚合节点实现前启动独立分块优化任务。

## 8. 2026-08-12 正式结果

### 8.1 结论

当前 C4 多来源接收数据面通过本质量门，但结果同时触发了后续分块+COW优化决策：

- 固定来源数和固定体素数时，未发现随运行时间无界增长。6 个 bounded 场景共 18 个
  独立 300 秒样本全部 `valid=true`；PSS/USS 最大增长斜率为 `117.44 KiB/min`，低于
  `1024 KiB/min` 门槛。
- 内存仍随保存的完整地图容量近似线性增长。`2 x 10k`、`2 x 100k`、`2 x 500k`
  的平均 PSS 分别约为 `19.76 / 31.72 / 84.70 MiB`。
- 256-cell 小 delta 的 apply 成本明显依赖完整地图规模。上述三个场景的 apply p95
  分别落在约 `0.52 / 4.19 / 16.78--33.55 ms` 档；apply 占 callback 的
  `97.1% / 99.7% / 99.9%`。这个数量级变化远大于本机 30--50% 的延迟噪声边界。
- 8 个来源、每来源 100k cells、10 Hz/source 时，三轮消息、revision、hash 和 cell
  count 全部守恒，freshness p95 仍在约 `0.52--1.05 ms` 档，未出现 backlog、gap、
  resync、duplicate 或 rejection。

因此当前实现没有“时间越久必然泄漏”的证据，但存在明确的容量和 apply 复杂度问题。
建议在 C5 正式 `EdgeAggregator` 绑定当前完整 vector 表示前，创建独立的分块不可变快照
与写时复制任务。

### 8.2 正式 bounded 矩阵

所有行均为三轮独立样本的平均值或三轮范围。每轮为 5 秒 warmup + 300 秒正式窗口；
延迟只表示本机观测量级，不是硬实时承诺。

| 场景 | 最终 cells | PSS 平均/范围 MiB | USS 平均 MiB | 最大斜率 KiB/min | apply p95 ms | apply/callback | 发送速率 | 应用 CDR |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `2 x 10k` | 20,000 | 19.76 / 19.50--20.21 | 10.17 | 6.31 | 0.52 | 97.1% | 20 msg/s | 1.14 Mbit/s |
| `1 x 100k` | 100,000 | 27.23 / 27.00--27.39 | 17.53 | 117.44 | 4.19 | 99.6% | 10 msg/s | 0.63 Mbit/s |
| `2 x 100k` | 200,000 | 31.72 / 31.61--31.82 | 22.02 | 16.95 | 4.19 | 99.7% | 20 msg/s | 1.26 Mbit/s |
| `4 x 100k` | 400,000 | 38.52 / 37.70--40.02 | 28.83 | 116.40 | 4.19 | 99.7% | 40 msg/s | 2.51 Mbit/s |
| `8 x 100k` | 800,000 | 52.70 / 52.33--53.24 | 42.98 | 109.04 | 4.19 | 99.7% | 80 msg/s | 5.03 Mbit/s |
| `2 x 500k` | 1,000,000 | 84.70 / 84.50--84.97 | 75.04 | 98.79 | 16.78--33.55 | 99.9% | 20 msg/s | 1.78 Mbit/s |

`应用 CDR` 只包含 routed ROS message 的 CDR 字节，不含 DDS/RTPS、IP 和链路层开销。
固定 2 来源的三个容量点给出约 `69 bytes/新增 cell` 的本 workload 增量 PSS；该值包含
接收地图、索引/allocator 以及进程内相关容量，不应解释为 `CanonicalCell` 的
`sizeof`。来源矩阵还包含固定 ROS/RMW 和共享库基线，因此不强行用一个截距/斜率模型
同时拟合两类实验。

18 个样本具有 18 个唯一 receiver PID/starttime；receiver/source 的 ELF SHA-256 和
build-id 在整个矩阵内各自唯一。每轮恰有 300 个内存采样点，发送、接收、应用、最终
revision/hash/cell count 和 application bytes 全部守恒；全矩阵 mismatch、duplicate、
rejection 和 endpoint anomaly 均为 0。

### 8.3 Heaptrack

| 场景 | peak heap | allocations | 结束时 heap | 主要解释 |
| --- | ---: | ---: | ---: | --- |
| `2 x 100k bounded` | 19.16 MiB | 57,997 | 421 KiB | apply 占 callback 99.05% |
| `2 x 500k keyframe replacement` | 90.75 MiB | 50,353 | 421 KiB | 48 个大 keyframe 全部应用 |

大 keyframe 场景的峰值栈包含约 48 MiB canonical keyframe decode vector，以及约
12.5 MiB Fast DDS payload pool。时间线退出后都回落到约 421 KiB，支持“瞬时完整候选
与 payload 峰值”而不是业务对象持续累积的解释。

Heaptrack 报告的 `357.78 KiB total memory leaked` 是工具对进程退出存活分配的分类，
不能单独作为 C4 泄漏结论；泄漏门以 ASan/LSan 和严格 Memcheck 结果为准。

### 8.4 ASan/LSan 与 Memcheck

独立 ASan 前缀使用 `-O1 -g -fsanitize=address -fno-omit-frame-pointer`，下列三条链
均 `valid=true`，消息/hash/cell 守恒，正常销毁且 sanitizer gate 通过：

- `8 x 100k bounded`；
- `2 x 10k -> 2 x 500k expanding`；
- `2 x 500k keyframe replacement`。

ROS-free `TestDataPlaneCore` 的 Memcheck 为 9 tests passed、5,974 alloc/free、退出时
0 bytes、0 errors，严格 gate 通过。

真实 ROS receiver smoke 的业务链同样守恒，且 invalid/uninitialized、definite lost、
indirect lost 均为 0；但严格 gate 未通过，因为进程退出时有 `768 bytes possibly lost`
和 `194,137 bytes still reachable`。两个 possible-lost 栈均为
`allocate_dtv -> pthread_create -> liblttng-ust` 初始化线程 TLS，不经过 C3/C4 业务代码。
本报告保留该第三方退出行为，不使用 suppression 把真实 ROS 全链伪装成严格通过。

### 8.5 证据路径与无效尝试

Git 可审计摘要位于本任务的 `validation/`，完整 raw 位于本地
`profiling-archive/c4-communication-data-plane-20260812/raw/`。两处使用相同的下列
相对目录名；旧采集路径到新路径的映射和 raw 树摘要见
`validation/relocation-provenance.txt`：

- `formal-main-2x100k/`
- `formal-scale-2x500k-rerun/`
- `formal-sources-8x100k/`
- `formal-remaining-bounded/`
- `heaptrack-rel-bounded-2x100k-20s/`
- `heaptrack-rel-keyframe-2x500k-20s/`
- `asan-bounded-8x100k-10s/`
- `asan-expanding-2x500k/`
- `asan-keyframe-2x500k-10s/`
- `memcheck-core/`
- `memcheck-smoke-bounded-2x1k-3s/`

`formal-scale-2x500k/` 是一次保留的无效启动：命令未加载 ROS/profile install 环境，
source 和 receiver 因缺少共享库在 readiness 前退出。它没有正式采样，也未纳入任何
矩阵结论；修正环境后使用新 domain ID 和新目录 `formal-scale-2x500k-rerun/` 完成三轮。
无效启动的完整日志只保留在 raw archive，不复制到 Git 摘要镜像。

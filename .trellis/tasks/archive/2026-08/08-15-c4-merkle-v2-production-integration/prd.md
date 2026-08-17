# Merkle v2 生产集成与 rollout Gate

## 1. 目标

将 C4.2 已通过算法 Gate 的 `chunk edge 16 + persistent Merkle Patricia + SHA-256 v2`
内容身份接入正式 ROS 地图更新链路，以 v2-only 新 schema 替换尚未正式发布的 v1 运行时协议，
完成生产端与接收端一致性、资源准入和正式集成验证，为是否接受该生产实现以及 C5d
EdgeAggregator 是否可以绑定正式内容身份契约提供可审计结论。

## 2. 已确认背景

- C4.2 没有替换 SHA-256 原语；它把每次 delta 遍历完整 canonical map 的 flat v1，改为
  对 dirty chunk 叶子和 Patricia 路径执行增量 SHA-256。
- C4.2 的 10k/100k/500k 固定四 chunk delta apply 相对 `chunk 16 + flat v1` 分别改善
  `20.5x / 189x / 858x`，算法 Gate 为 GO；当时原型没有修改 ROS schema 或生产 receiver，
  因此生产 rollout 尚未决策。C4.3 已完成 v2-only ROS 集成与正式 Gate，详见 validation。
- 当前开发基线仍是 `Vector + flat SHA-256 v1`；`ContentIdentityDescriptor`、
  `VersionedContentDigest` 和 Merkle applier 只存在于 ROS-free 原型边界。
- 现有 `MapUpdate` 中的裸 SHA-256 身份不能区分 flat v1 与 Merkle v2。版本必须成为显式
  协议语义，未知版本必须 fail closed。
- C5a-C5c 不依赖内容身份内部结构，可以独立推进；C5d EdgeAggregator 必须等待本任务 Gate，
  且只能消费稳定的 versioned content identity，不能依赖 Patricia 内部节点。
- 正式证据要求包含 `3 x 300 秒` ROS 集成矩阵；C4.2 的 ROS-free 短测不能替代该矩阵。

## 3. 需求

### R1 v2-only 内容身份契约

- 正式 wire contract 的 `protocol_version` 必须提升为 v2，并显式表达 Merkle v2 descriptor；
  不能继续用裸 32-byte hash 猜测语义。
- v2 descriptor 必须提交影响根值的固定参数，至少包括 scheme、chunk edge、coordinate-key
  version 和 node-encoding version。
- 未知或不受支持的 descriptor/version 必须原子拒绝并产生可诊断结果，不得降级解释为 v1。
- 新生产节点不实现 v1/v2 双读、双写、能力协商或混合二进制兼容；所有参与节点统一重编译并
  使用 v2 schema。

### R2 生产端与接收端本地重算

- producer 必须从本地 committed canonical content 生成版本化内容身份，不能信任外部传入 root。
- receiver 必须在 candidate state 上本地重算 v2 root，并在 count、descriptor、base identity、
  result identity 和资源检查全部通过后原子提交；flat v1 oracle 只能在隔离的正确性测试中运行。
- keyframe、delta、revision-only、remove、epoch/session replacement 和 resync 的身份语义必须
  明确定义并覆盖测试。
- 任何校验失败均不得改变已提交的 snapshot、tree、revision、freshness 或 resync 状态。

### R3 发布边界与回退

- 本项目尚未正式发布，不要求旧 `MapUpdate.msg` 二进制与新节点共存；ROS graph 中的参与节点
  必须使用同一 v2 消息定义。
- 生产运行时只维护一条 v2 source/session 链，不实现 v1 fallback 配置或同链多版本输出。
- v1 仅保留为冻结的测试/性能基线与正确性 oracle，不得进入 v2 性能样本或正式运行路径。
- Gate 失败时通过不合并 v2 变更或回退对应提交恢复 v1 开发基线，不在运行时协议中增加为兼容
  而存在的状态机。
- C4 的 route、trust、QoS、freshness 和 source/session 语义保持不变；只扩展透明承载 v2
  `MapUpdate` 所必需的 schema/conversion，Relay 不解析 Merkle 内部结构。

### R4 资源准入

- 在分配或构建 candidate state 前，对 keyframe cells、live chunks、Merkle nodes、candidate
  bytes、delta mutations 和 resync envelope 建立可配置且有硬上限的 admission checks。
- 所有大小、计数和内存估算必须检查整数溢出；超限时原子拒绝并进入可恢复的 resync/诊断路径。
- 上限应由现有 `MapUpdateLimits` 和接收数据面预算扩展，不建立与生产限制相互矛盾的旁路配置。

### R5 Gate 与证据

- 先以确定性 correctness/conformance 验证 full-rebuild/incremental v2 root 一致性，并用隔离的
  flat v1 oracle 证明 canonical content 未改变，再运行生产链短筛选和正式矩阵。
- 正式矩阵至少覆盖冻结的 v1 基线与 v2 production、断链后的 keyframe/resync、拒绝路径，
  并记录端到端 apply、hash/tree 阶段、CPU、PSS/USS、队列/带宽、candidate bytes 和诊断计数。
- 正式资源证据为同一 workload、同一 candidate v2 构建身份下的 `3 x 300 秒` 独立 receiver
  运行，并包含 ASan/LSan/UBSan、严格 Memcheck 和 Heaptrack 复核；冻结 v1 对照必须单独标注
  其 commit/ELF，不能伪称与 v2 是同一构建。
- 不预设无法由业务约束证明的“CPU 必须低于 10%”等隐藏阈值；Gate 依据正确性、资源有界性、
  相对现有开发基线的量化收益与回归、以及变更集回退可行性给出 GO/NO-GO。
- 原始证据放入 profiling archive，可审计摘要、运行 manifest、hash 和最终报告进入 Git。

### R6 下游边界

- 本任务不实现 EdgeAggregator、跨来源融合或 Merkle proof 交换。
- C5d 只可消费 descriptor + content digest/root 的稳定边界，不得读取 chunk store 或 Patricia
  内部节点。
- 本任务不以第三方攻击防护、远程 inclusion proof 或内容寻址存储为目标；接收端重算属于
  数据一致性和失败原子性要求。

## 4. 验收标准

- [x] ROS-visible v2 schema 能无歧义表达 Merkle descriptor/digest，未知版本 fail closed；全部正式
  节点统一重编译，不提供旧消息二进制兼容。
- [x] producer、receiver 和 ROS conversion 对 v2 的 keyframe、delta、revision-only、remove、
  epoch/session replacement 与 resync 具有一致、确定且失败原子的语义。
- [x] receiver 在 candidate state 上本地重算身份；错误 base/root/descriptor/count、legacy/unknown
  protocol、超限资源和损坏 envelope 均被原子拒绝并产生明确诊断。
- [x] 正式运行路径为 v2-only；冻结 v1 仅用于独立 baseline/oracle，且没有进入 v2 性能样本。
- [x] 受影响的 ROS-free gtest、conversion tests、launch/integration tests 和隔离的 flat v1
  canonical-content oracle 全部通过。
- [x] `3 x 300 秒` 正式 ROS 矩阵完成，CPU、端到端耗时、PSS/USS、candidate/owned bytes、带宽、
  队列和 resync 指标具有完整 provenance；内存工具未发现业务泄漏或非法访问。
- [x] 最终报告给出 production integration GO/NO-GO；Gate 为 GO 时接受 v2-only 生产实现并
  解除 C5d 前置阻塞，Gate 为 NO-GO 时不合并或回退本任务提交，恢复冻结 v1 开发基线。
- [x] 权威文档、父工作流状态、运行入口与 profiling archive 索引同步更新。

验收中的“内存工具未发现业务泄漏”不等同于所有第三方 runtime 的严格工具计数为零。
本任务保留 Memcheck 原始 `gate_pass=false`，并以历史可复现的 `liblttng-ust` 768 B TLS
finding 作为显式例外；`business_memory_gate=pass` 仅覆盖该固定第三方栈。

## 5. 非目标

- 不重新比较 8/16/32；v2 正式候选固定使用 edge 16。
- 不替换 SHA-256 算法，也不引入第二种 Merkle 树结构。
- 不实现 C5d EdgeAggregator、聚合 contributor manifest 或多 Region 共享地图。
- 不实现旧、新 `MapUpdate.msg` 二进制共存、并行 v2 topic、桥接或运行时 downgrade。
- 不在 production v2 路径保留 flat v1 双算；flat v1 只允许作为隔离的 oracle/benchmark。

# C4.3 当前 wire 集成评估

## 1. 结论摘要

用户已确认当前是未发布的个人开发项目，不需要旧、新二进制共存。正式方案采用统一重编译的
v2-only `MapUpdate` schema，显式加入 versioned content identity 字段；`RoutedMapUpdate` 继续
透明包裹 `MapUpdate`，Relay 不解析 Merkle 内部结构。每个 source/session 只有一种固定 v2
descriptor，correlated keyframe/resync 只负责链路恢复，不负责 v1/v2 协商。

旧 v1 实现只作为冻结 baseline/oracle 保存，不进入生产运行时。Gate 失败通过不合并或回退本
任务提交恢复基线，不构建运行时 downgrade 状态机。

## 2. 现状证据

- `perception_interfaces/msg/MapUpdate.msg` 已有 `protocol_version`、`hash_algorithm` 和三个
  32-byte hash，但没有 content identity scheme 或 Merkle descriptor；SHA-256 原语无法区分
  flat v1 与 Merkle v2。
- `MapUpdateTypes.hpp` 的生产 `MapUpdate` 与 `CanonicalSnapshot` 都只保存裸 `Hash256`；
  `MapUpdateProducer`、`MapUpdateApplier` 和 `ProducerBaselineToken` 因此都默认 v1 语义。
- `MerklePrototypeProtocol.hpp` 已验证 `ContentIdentityDescriptor + VersionedContentDigest` 的
  ROS-free 结构，但刻意与生产 `MapUpdate` 分离，可作为正式类型晋升的起点。
- `RoutedMapUpdate.msg` 直接嵌套 `perception_interfaces/MapUpdate`；route source 先 decode，route
  receiver 再 decode/apply。因此内容身份仍由 C3/C4 map-update core 负责，C4 envelope 只需把
  versioned update 作为 payload 透明传输并继续校验 update hash。
- 直接 `RequestMapResync.srv` 与 routed `ResyncIntent/ResyncAck` 已具备 requester、source、
  revision、content hash 和 correlation barrier，是承载 receiver-supported/producer-selected
  identity descriptor 的最小控制点。
- 当前 `PerceptionLocalMapNode` 只有一个 `MapUpdate` publisher，`AsyncMapUpdateProducer` 只有一个
  committed baseline；它不是按 subscriber/route 维护多版本链。因此首版不能无代价地对不同
  receiver 同时双写 v1/v2 delta。

## 3. Schema 选项

### A. 原位扩展并统一重编译（已选）

- 在 `MapUpdate` 中显式携带 content identity scheme、chunk edge、coordinate-key version、
  node-encoding version，以及仍为 32-byte 的 base/result digest。
- 新 schema 只接受 Merkle v2，固定 edge 16 与已验证的 key/node encoding version；flat v1 不
  是正式 descriptor 的候选值。
- direct/routed resync 继续用 correlation-matched keyframe 建立或恢复 v2 链；不增加多版本支持
  集合和协商状态。
- 所有包统一重编译；新节点只运行 v2。

优点是只有一套生产 codec、applier、route 和诊断路径，单因素性能归因清晰。ROS 2 `.msg`
变更会改变类型身份，但当前项目没有旧二进制共存要求，因此不是阻塞项。

### B. 新建 `MapUpdateV2` 并行 topic/service

- 保留旧 `MapUpdate` 完全不变，新建 v2 消息、route envelope 和 resync 控制面，rollout 期间运行
  两套 topic 或桥接。

它能支持旧、新二进制共存，但会复制 conversion、QoS、路由、去重、resync 和 profiling 路径；
双发还会显著增加 CPU、内存和带宽，使 C4.2 的单因素收益难以解释。除非存在真实的滚动升级
部署约束，否则当前阶段不值得承担这组复杂度。

### C. 只提升 `protocol_version` 并隐式固定 v2

- 不增加 descriptor 字段，看到新 protocol version 就假定当前 Merkle 参数。

该方案无法独立版本化 coordinate key/node encoding，也无法在同一新 schema 内安全支持 v1
回滚；与 C4.2 已确认的 fail-closed contract 冲突，因此排除。

## 4. v2-only 运行边界

- `protocol_version=2` 与完整 descriptor 是 producer/source-session 的固定契约，不是 Relay 策略。
- receiver 只接受规范 v2 descriptor；未知 scheme、edge 或 encoding version 原子拒绝。
- correlated keyframe 建立或恢复 v2 链；已 Ready 的 receiver 遇到 descriptor drift 的 delta 必须
  原子拒绝并请求 keyframe/resync。
- 未来如果真实发布产生滚动升级需求，应另建迁移任务评估并行 topic/bridge，不能把兼容分支
  偷渡回当前 v2-only hot path。

## 5. 已确认范围

2026-08-15 用户确认直接采用 v2；项目尚未正式发布，不存在旧、新 `MapUpdate.msg` 二进制共存
或兼容要求。选择 A，排除并行消息/topic、桥接、双读、双写和运行时 downgrade。

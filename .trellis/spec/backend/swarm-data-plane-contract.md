# C4 通信数据面契约

## 1. Scope / Trigger

本规范适用于 `swarm_data_interfaces` 和 `swarm_data_plane` 的跨进程地图数据面。
它约束 C3 `MapUpdate` 进入 routed envelope、应用层 admission/resync 以及 ROS 端点
适配器的边界。生产传输由 ROS 2 RMW/DDS 提供；`LogicalLinkAdapter` 只能编入测试
target，不能成为生产可靠传输队列。

## 2. Signatures

ROS-free 核心接口：

```cpp
EnvelopeValidation validate_routed_map_update(
    const RoutedMapUpdate &, const DataPlaneLimits &);
IngressResult MapUpdateIngress::receive(
    const RoutedMapUpdate &, std::uint64_t local_receive_monotonic_ns);
bool MapUpdateIngress::expect_resync(std::string correlation_id);
ResyncAck RoutedResyncLedger::accept(
    const RoutedResyncIntent &, const SourceIdentity &, std::uint64_t current_revision);
AggregateValidationResult validate_aggregate_map_update(
    const AggregateMapUpdate &, const DataPlaneLimits &);
TrustDecision TrustValidator::validate(
    const RoutedMapUpdate &, const TrustEvidence &);
```

ROS 端点边界：

```cpp
bool encode_routed_map_update(
    const RoutedMapUpdate &, swarm_data_interfaces::msg::RoutedMapUpdate &, std::string &);
DecodeRoutedMapResult decode_routed_map_update(
    const swarm_data_interfaces::msg::RoutedMapUpdate &, ...);
```

C4 资源测量入口（仅 `BUILD_TESTING`）：

```text
run_c4_resource_profile.py --source <elf> --receiver <elf>
  [--workload-mode bounded|expanding|keyframe-replacement]
  [--storage-mode vector|chunked] [--chunk-edge 8|16|32]
  [--chunk-bucket-count <positive-int>] ...
run_c4_resource_matrix.py --runner <script> --source <elf> --receiver <elf>
  --output-dir <new-dir> --workspace-root <repo> --image-id sha256:<64-hex>
  [--scenario <name>]... [--formal]
```

## 3. Contracts

- `RoutedMapUpdate` 直接嵌套 C3 `perception_interfaces/MapUpdate`；source vehicle、mapper
  session、map epoch、base/new revision、update/content hash 和 payload 不得被转发节点改写。
- envelope 必须显式携带 `message_id`、producer/session、`sequence`、`correlation_id`、
  origin clock domain/session/time、validity budget、route epoch/hop/TTL、payload bytes/hash
  和 protocol version。bounded string/array 在分配前校验。
- ingress 按 producer/session、source/epoch、route epoch 和 sequence admission。相同 identity
  + hash 返回 `IgnoredDuplicate`；相同 identity + 不同 hash 返回 `RejectedConflict`；gap、旧
  route、过期或 TTL 耗尽保持最后合法状态并要求 resync。
- initial、gap、conflict 和 epoch change 进入 resync barrier。barrier 期间 delta 必须拒绝；
  只有 service 返回的 correlation 所关联 keyframe 才能原子恢复并清除 barrier。
- `RoutedResyncLedger` 以 requester session + client request id 幂等；重复请求返回同一
  correlation，service response 不携带大地图 payload，keyframe 走异步地图 topic。
- aggregate update 与 contributor manifest 使用同一个 aggregate revision；manifest 采用
  canonical SHA-256，校验失败不得部分提交。
- origin stamp 只作溯源；每跳使用本地 monotonic clock 扣减 validity budget。禁止通过重写
  `header.stamp` 改变排序或掩盖延迟。
- 生产 QoS 只通过标准 `rclcpp::QoS`：map 使用 Reliable/Volatile/KeepLast，state 使用
  Reliable/KeepLast(1) 并可配置 deadline/lifespan，diagnostic 使用有界 KeepLast。不得 include
  Fast DDS、Cyclone DDS、RTPS 或其他厂商私有类型。
- trust rejection 只产生有界诊断，不刷新 freshness、不改变 C3 地图或控制状态。
- C4 可视化 fixture 只使用一个 RViz 进程和四个独立 `MarkerArray` topic/group。地图 marker
  表示最后合法快照；gap、resync、correlation 和 TTL fault 必须在独立状态/时间线 marker
  中表达，不能把未知 delta 的体素直接涂成地图障碍物。
- 真实洞穴验收必须由 A1/A2 的正式 C1→C2→C3 链产生 `LocalMapUpdate`。RViz 默认显示的
  B 端远程地图必须由 `MapUpdateIngress` 已提交状态物化；A1/A2 source maps 只能作为默认
  关闭的可选对照层，洞穴真值和 source/receiver difference 只能作为独立非权威 oracle。
  动态 drop/gap/resync 正确性由 headless launch test 断言，GUI 目检只确认显示完整性。
- C4 资源 workload 只测量 receiver 的 routed decode、`MapUpdateIngress`、C3
  `MapUpdateApplier` 和每来源最后合法地图；source、RViz、Marker 和未来 C5
  `EdgeAggregator` 不计入 receiver 资源，因此结果不得表述为“地图聚合性能”。
- profile source/receiver、runner 和 smoke target 只能位于 `BUILD_TESTING`。正式 bounded
  矩阵固定为每场景 3 个独立、至少 300 秒的 plain receiver 样本；每轮必须记录唯一
  PID/starttime、统一 ELF SHA-256/build-id、完整业务守恒和每秒角色/内存采样。
- C4.1 已在不改变 wire、flat canonical SHA-256、revision 和原子 apply 的前提下评估
  `8/16/32` 分块不可变快照与 COW。edge 16 是三档中的折中研究候选，但成熟三维 replay
  的 copied-cell P95 未达到 `<5%`，短 A/B 的端到端 apply/PSS 也未优于 vector，因此 Gate B
  为 no-go，生产默认保持 `Vector`。C5 消费 `CanonicalCellView` 的有序 cursor，不得重新
  绑定 `std::vector<CanonicalCell>` 所有权或依赖 chunk/bucket 内部布局。

## 4. Validation & Error Matrix

| 条件 | 必须结果 |
| --- | --- |
| protocol/version、身份、hash、payload bytes 或 TTL 非法 | 在分配/状态 mutation 前拒绝 |
| 同 identity + 同 hash 重复 | `IgnoredDuplicate`，不刷新 freshness |
| 同 identity + 不同 hash | `RejectedConflict`，保持最后合法 revision，要求 resync |
| sequence gap、旧 route/epoch、过期 validity budget | 拒绝并进入 resync barrier |
| barrier 中无 correlation 的 delta/keyframe | 拒绝，不改变地图 |
| correlation 不匹配的 keyframe | 拒绝，不清除 barrier |
| 重复 resync intent | 返回已有 correlation，不增加 ledger 条目 |
| manifest 与 aggregate revision/hash 不一致 | 原子拒绝，禁止部分提交 |
| spoof/replay/old authority/expired/revoked/unknown producer | trust reject + bounded diagnostic，领域 fingerprint 不变 |
| 后续高 sequence 到达但中间 sequence 未接收 | 显示 gap 状态并进入 resync barrier，不声称物理丢包已被证明 |
| 没有后续消息且 freshness budget 到期 | 由 watchdog/timeout 报告静默过期；不得伪造新地图 revision |
| profile target 在 `BUILD_TESTING=OFF` 时仍可构建或安装 | 质量门失败；移回测试条件内 |
| formal run 少于 300 秒、不是 bounded/plain，或 matrix 不是恰好三轮 | 在启动 ROS 进程前拒绝 |
| matrix 混用 ELF/build-id、重复 PID/starttime、业务不守恒或采样不完整 | 保留原始目录并标记无效，不纳入基线 |
| 固定地图下任一正式样本 PSS/USS 斜率达到 `1024 KiB/min` | 不得下“无持续增长”结论，先定位无界状态 |
| storage A/B 混用 receiver/runner ELF、wire/hash 版本、workload 或采样窗口 | 证据不可归因，标记无效并重跑同身份对照 |
| edge 16 被配置为 chunked，但 Gate B 没有新的通过证据 | 允许作为测试/研究配置；不得据此修改生产默认 |

## 5. Good / Base / Bad Cases

- Good：sequence 1 keyframe 到达后丢弃 sequence 2，sequence 3 delta 被拒绝；关联 keyframe
  sequence 4 恢复 revision 3，最终 C3 状态为 `[1, 3]`。
- Good：同一 `client_request_id` 重试 resync，服务两次返回相同 correlation，地图只提交一次。
- Good：真实洞穴场景中 A2 receiver map 在 gap 期间冻结并经相关 keyframe 恢复，A1 receiver
  map 继续推进；默认 RViz 地图层不直接订阅 source-local OctoMap。
- Base：RMW/DDS 负责 Reliable history、重传、分片和 discovery；应用只记录有界 admission 与
  resync ledger。
- Good：独立 C4 workload 对 bounded、expanding、keyframe replacement 分别验证稳态、容量增长
  和候选替换峰值；正式结论只使用三轮 bounded 证据。
- Bad：在 C4 复制一份 MapUpdate applier、自己实现 ACK/NACK/重传，或让 relay 重写 source
  revision/hash/header stamp。
- Bad：复用洞穴/RViz fixture 测容量，或把多来源 receiver 的完整地图保存成本称为 C5
  `EdgeAggregator` 聚合算法成本。

## 6. Tests Required

- `TestDataPlaneCore`：协议/大小/TTL、重复冲突、gap/barrier、route、aggregate 原子性、trust
  拒绝及状态 fingerprint 不变。
- `TestRoutedMapConversions`：C3 envelope round-trip、字段漂移拒绝、resync correlation、
  manifest round-trip 和标准 QoS policy。
- `TestLogicalLinkAdapter`：固定 seed 轨迹、丢包/分区/队列边界、clock fault 和 payload/hash
  不变。
- `test_swarm_data_plane_closed_loop.py`：真实 ROS source → link fixture → receiver，断链后
  service correlation + 异步 keyframe 恢复；必须有 active test method 和 clean process exit。
- `test_c4_visualization.py`：单 RViz 配置对应的四个 MarkerArray scene 必须发布完整阶段集合；
  gap/TTL 状态为红色，correlated recovery 为绿色，并同时断言 sequence、revision、
  correlation 和 contributor/aggregate marker。
- `test_c4_cave_visual_validation.py`：使用真实 A1/A2 感知与本机地图链，断言 B 的 A2
  accepted map 经历 drop、gap、冻结和 correlated keyframe 恢复，A1 独立推进，A1/A2
  完成约 8 m 路径、B 完成约 3.5 m 路径，accepted OctoMap 非空且进程正常退出。
- C4 resource smoke：bounded、expanding、keyframe replacement 都必须产生有效报告并断言
  source/revision/hash/cell/bytes 守恒、无 rejection/duplicate/endpoint anomaly、进程正常退出。
- C4.1 storage conformance：vector 与 `8/16/32` chunked 短 A/B 使用相同 receiver/runner
  身份和 flat SHA-256；断言每组消息、revision、hash、cell count 守恒，并把
  candidate/hash/commit、copied cells/bucket entries 与 PSS/USS 分开报告。
- 生产隔离：用全新 `BUILD_TESTING=OFF` CMake 目录构建，并断言 target 列表中不存在
  `c4_resource_profile_*`；formal runner/matrix 的窗口和轮数错误必须在启动进程前拒绝。
- 正式矩阵复核：从 raw `analysis-summary.json` 重算 18 个样本的身份唯一性、ELF 一致性、
  300 点采样完整性、资源斜率与业务守恒，并校验 matrix 中保存的 summary SHA-256。
- launch 闭环至少重复三次；第二种 RMW 可用后，在不改领域代码的情况下用同一 fixture 复跑。

## 7. Wrong vs Correct

Wrong：

```cpp
// 通过 header stamp 或 topic 到达顺序推断 revision，并在 C4 自己重传。
if (message.header.stamp > last_stamp) {
    resend_with_ack_nack(message);
}
```

Correct：

```cpp
auto admission = ingress.receive(routed, local_monotonic_now_ns);
if (admission.resync_required) {
    request_idempotent_resync(routed.route.route_epoch);
}
// 仅 C3 MapUpdateApplier 修改地图；恢复 keyframe 必须匹配 correlation_id。
```

Wrong：

```text
洞穴 + RViz 总 RSS -> “C4 聚合器内存”，一次短 run -> “长期无增长”
```

Correct：

```text
独立 receiver PID + 3 x 300 s bounded raw evidence -> C4 多来源接收数据面结论
expanding/keyframe replacement -> 容量和瞬时峰值解释，不替代泄漏判定
```

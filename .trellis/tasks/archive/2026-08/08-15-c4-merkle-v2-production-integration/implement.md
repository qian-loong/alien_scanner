# C4.3 Merkle v2 生产集成：实施计划

## 0. 启动条件与基线

- [x] 用户审核 `prd.md`、`design.md` 和本实施清单。
- [x] 从 `phase/4-incremental-merkle-hash-v2` 当前已提交状态创建
  `phase/4-merkle-v2-production-integration`，记录 base/HEAD、Docker image 和工具版本。
- [x] 运行 Trellis `task.py validate`，然后 `task.py start`；inline 模式跳过 JSONL context gate。
- [x] 运行 `trellis-before-dev`，加载 `perception_interfaces`、`perception_map_update`、
  `perception_local_map`、`swarm_data_plane`、profiling 和 CMake/test 相关 spec。
- [x] 冻结 v1 对照 commit `6865c08b98d65fa2994d729fbb5bf06f9f316095` 与已有证据 hash。

## 1. 先冻结 v2 协议契约

- [x] 新增 `ContentIdentityDescriptor.msg`，把 `MapUpdate.msg` 直接提升为 protocol v2-only。
- [x] 更新 direct/routed resync message/service 的 versioned identity 字段。
- [x] 在 ROS-free types 中提升 descriptor/digest，固定 edge 16 和 encoding versions。
- [x] 把 update hash domain 提升到 v2 并提交 descriptor/base/result digest。
- [x] 先写 conversion/golden/unknown enum/zero digest tests，再修改实现使其通过。
- [x] 验证 `RoutedMapUpdate` 只透明承载 v2，不在 route envelope 复制 descriptor。

回滚点：本步仅改变 schema/types/conversion；若契约不收敛，整体撤销，不接 producer/receiver。

## 2. 晋升 Merkle 生产状态

- [x] 从 prototype 提取/重命名生产 `MerkleMapState`，复用 `CellSnapshotStore` 与
  `MerklePatriciaTree`，不保留第二套协议状态机。
- [x] 实现 keyframe build、delta apply、revision-only sharing 和 remove 清理所需 API。
- [x] 实现 keyframe/delta Merkle node/byte checked upper-bound 与实际 metrics 复核。
- [x] 扩展 `MapUpdateLimits` 与 ROS 参数，覆盖 live chunks/nodes、owned/candidate/peak bytes。
- [x] 添加 build/apply failure atomicity、负坐标、empty tree、last-cell delete、overflow 和 limit tests。

回滚点：生产 state 仍未接 wire，可恢复 C4.2 prototype 而不影响现有 v1 节点。

## 3. Producer v2-only

- [x] `CanonicalSnapshotAdapter` 删除 flat content hash，保留 geometry fingerprint 并更新计时/诊断。
- [x] `MapUpdateProducer` 同时维护 canonical baseline 与 committed `MerkleMapState`。
- [x] keyframe prepare full-build candidate；delta prepare 对 diff operations 做增量 candidate。
- [x] `PreparedUpdate` 携带 candidate state；`commit_published()` 原子提交 snapshot/state。
- [x] revision-only 复用 tree/root，publish failure 和 stale prepared result 不改变 baseline。
- [x] 更新 `AsyncMapUpdateProducer` 指标和节点诊断，不把 snapshot traversal/diff 计入 Merkle 收益。
- [x] 添加 producer keyframe/delta/resync/publish failure/coalescing/resource tests。

## 4. Receiver v2-only

- [x] `MapUpdateApplier` 固定 chunk-16 `MerkleMapState`，删除 production storage-mode 选择。
- [x] keyframe/delta 在 candidate 上本地重算 root，count/descriptor/root/resource 全通过后提交。
- [x] summary 复用 identity，remove 提交 zero tombstone，descriptor drift/unknown version fail closed。
- [x] `ReconstructedMap` 对外暴露 `VersionedContentDigest + CanonicalCellView`。
- [x] 保持 duplicate/gap/conflict/Removed/resync-required/freshness 的既有状态语义。
- [x] 更新 reference receiver、OctoMap adapter、diagnostics 和全部 core/storage/capacity tests。

## 5. ROS 与 C4 路由集成

- [x] 更新 local-map publisher、reference receiver、route source/receiver 和 routed conversions。
- [x] direct/routed resync ledger 比较并回传完整 v2 identity；correlation keyframe 恢复保持幂等。
- [x] 更新 route payload hash、DeliveryAck、fixture 和故障注入链中的字段/断言。
- [x] 验证 Relay 不解析/修改 descriptor，旧 route、TTL、duplicate、gap 和 resync 行为不回归。
- [x] 更新 launch/config 默认：正式地图更新节点只生成/接受 v2。

## 6. 自动化正确性与可视化回归

- [x] 运行 `perception_map_update` 全部 gtest 和 receiver rejection launch test。
- [x] 运行 `perception_local_map` closed-loop、full-ray scene 和异步 producer tests。
- [x] 运行 `swarm_data_plane` core/conversion/closed-loop/cave visualization tests。
- [x] 用 flat v1 oracle 对比 source、geometry、revision、canonical cells 和最终 OctoMap。
- [x] 运行现有 RViz replay/cave fixture，确认显示内容、阶段切换、topic/TF 与差异层不变；记录
  新截图时只作为 C4.3 回归证据，不复用旧人工验收结论冒充新结果。

## 7. 性能与内存 Gate

- [x] 先运行短 correctness/performance screening，确认 production v2 samples 不含 flat hash。
- [x] 保留 C4.2 同构建 flat-v1/Merkle-v2 算法 A/B，验证结果可复现。
- [x] 使用冻结 v1 commit 与 candidate v2 运行同参数端到端短对照，分别记录 ELF/source manifest。
- [x] 对 candidate v2 运行 3 次 300 秒独立 receiver ROS 矩阵；每个输出目录 create-once。
- [x] 报告 producer snapshot/diff 与 receiver store/Merkle 阶段，不能只报告 Merkle 微基准。
- [x] 运行 ASan/LSan/UBSan、严格 Memcheck、Heaptrack；区分工具开销与业务 retained bytes。
- [x] 聚合 CPU、apply latency、PSS/USS、chunks/nodes/candidate bytes、带宽、队列、resync 和最终
  revision，生成带 provenance/hash 的 GO/NO-GO 报告。
- [x] raw evidence 迁入忽略的 `profiling-archive/c4-merkle-v2-production-<date>/raw/`，Git 仅保留
  可审计摘要、manifest、运行脚本、hash 和关键图表/日志。

当前状态：实现分支的 schema、生产 producer/receiver、C4 route、短筛选、正式矩阵和内存工具
证据均已完成并写入 `validation.md`。严格 Memcheck 原始结果保留第三方 `liblttng-ust` 768 B
`possibly lost`，但该 finding 已由历史 C1/C2/C4 证据确认并列为已知 runtime 例外；业务
memory Gate 通过，C4.3 生产 Gate GO，C5d 前置阻塞解除。

## 8. 收尾

- [x] Gate GO（含已知第三方 runtime 例外）时更新父任务 C4.3/C5d 状态并记录回退结论；解除 C5d 前置阻塞。
- [x] 更新 `docs/` 中 map-state/C4 运行方式、schema 和性能结论。
- [x] 运行 `trellis-check`，修复 spec、lint、build、test、数据流和证据问题。
- [x] 运行最终串行验证和 change-set rollback dry-run。
- [x] 使用 `trellis-update-spec` 同步稳定的 v2 contract 与测量边界。
- [x] 已汇报 diff/验证，并在用户明确授权后提交实现与最终 Gate 文档；未自动 push/merge。

## 9. 主要风险文件

- `perception_interfaces/msg/MapUpdate.msg` 与 resync interfaces：全仓编译面。
- `MapUpdateTypes.hpp` / `ContentHasher.cpp`：canonical wire identity 与 golden hash。
- `MapUpdateProducer.*` / `AsyncMapUpdateProducer.*`：发布成功前 candidate 生命周期。
- `MapUpdateApplier.*` / `MerkleMapState.*`：store/tree/revision 原子提交与资源峰值。
- `MapUpdateConversions.cpp` / routed conversions：unknown enum、长度和分配前校验。
- C4 profiling runner：跨 commit provenance、PID/starttime 和 300 秒矩阵归因。

## 10. 验证入口（规划值）

实现时以容器 `/workspaces/alien-scanner/ws` 为工作区，使用独立 build/install/log 目录避免污染
既有证据。最终命令在实现后按实际 package/runner 参数固化，至少包括：

```bash
colcon build --symlink-install --packages-up-to \
  perception_interfaces perception_map_update perception_local_map \
  swarm_data_interfaces swarm_data_plane perception_profiling
colcon test --packages-select \
  perception_map_update perception_local_map swarm_data_plane perception_profiling
colcon test-result --verbose
```

内存工具和 `3 x 300 秒` runner 必须使用任务内最终保存的命令/manifest，不能仅凭此处占位命令
宣称通过。

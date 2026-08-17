# 增量 Merkle 内容哈希 v2：实施计划

> 状态：原型、短 Gate、内存工具和证据归档均已完成；生产 Vector/v1、ROS schema 和
> rollout 仍保持不变。

## 1. 规划与基线

- [x] 冻结 B 基线的 commit、ELF、chunk edge/bucket、workload 和阶段指标。
- [x] 完成需求访谈，确认本任务止于 opt-in 原型 + Gate；生产 schema/rollout 另立任务。
- [x] 研究三种 Merkle 结构及 coordinate key，选择 batch persistent Patricia + 192-bit key。
- [x] 完成 PRD convergence pass、技术设计和代码审核方案。
- [x] 用户审核规划产物并明确批准启动。

## 2. 身份与事务设计

- [x] 固定 leaf/internal/empty/content domain、canonical bytes、golden vectors 和 prototype descriptor。
- [x] 定义 keyframe、delta、revision-only、remove、epoch/resync 的 root 状态机。
- [x] 定义 candidate chunks/tree 同事务 commit、溢出检查和失败原子性；生产 transport quota 留待迁移任务。
- [x] 定义 correctness 双算模式与 v2-only 性能模式，禁止混淆证据。

## 3. ROS-free 原型

- [x] 先实现 192-bit coordinate key、full-rebuild Patricia 和结构/节点 estimator；不接 applier。
- [x] 实现 batch persistent leaf mutation，验证结果与 full rebuild 完全一致。
- [x] 为 `CellSnapshotStore` 增加只读 chunk visitor/touched-coordinate seam，不泄露可变内部状态。
- [x] 接入独立 prototype v2 applier harness，与 chunk 16 COW candidate 同事务提交；不修改默认
  Vector/v1、现有 `MapUpdate` 或 ROS schema。
- [x] 增加 full rebuild 与 incremental update 两条独立实现用于 conformance。
- [x] 增加 leaf/path/root/keyframe、dirty-union paths、node count、allocations 和 owned-bytes 指标。

## 4. 正确性与兼容验证

- [x] 添加 coordinate/domain/golden、插入删除、空树、边界和原子拒绝 gtest。
- [x] 对 deterministic 三维 replay 的 400 个 checkpoint 比较 full/incremental root。
- [x] 验证 prototype producer/applier wrapper、keyframe/resync 边界，以及现有 C4 route/C5 manifest 不会把 v2
  当作 v1 接受或转发。
- [x] 运行受影响包测试并确认默认 v1 代码路径未变；本原型未接入可视化接线。

## 5. 性能与内存 Gate

- [x] 增加独立 `perception_merkle_profile` workload，固定 chunk 16 并分开记录 flat hash 与 Merkle apply 阶段。
  运行参数为 `--output-dir <new-dir> --cells N --iterations N --warmup N --touched-chunks N`，输出
  `merkle-hash-benchmark.csv` 与 `merkle-hash-benchmark-summary.json`。
- [x] 同构建运行 B/C 短 A/B，报告阶段耗时、CPU、PSS/USS、节点和 keyframe 成本。
- [x] 验证 delta 成本随 touched chunks 而非 known cells 扩张。
- [x] 同时记录当前生产 A 影子参照；A/C 差异只做风险报告，不设置固定百分比硬门。
- [x] 短门通过后运行 Heaptrack、ASan/LSan/UBSan 和严格 Memcheck。
- [ ] 3 x 300 秒正式 ROS 矩阵：原型未接入 wire，留待生产迁移任务，未标记通过。
- [x] 输出 go/no-go 和下一步生产迁移建议：B/C delta apply 在 10k/100k/500k 下改善
  `20.5x / 189x / 858x`，算法 Gate GO；生产 rollout NO-GO，默认保持 Vector/v1；
  下一任务为 C4.3 生产集成，并作为 C5d EdgeAggregator 的前置 Gate。
- [x] 将完整 raw 迁入忽略的 `profiling-archive/c4-merkle-v2-20260815/raw/`，保留 52 个
  Git 可审计文件；迁移前后 905 文件、4,699,734 字节和 tree SHA-256 完全一致。

## 6. 完成流程

- [x] 运行 `trellis-check` 与最终 validation。
- [x] 更新 backend code-spec。
- [ ] 获得明确授权后提交；不自动 push 或切换生产默认。

## 7. 基线与风险文件

- B 基线源码提交固定为 `5875a41773b5f152e0bd51cb073387bb00d58cfd`。
- 首要变更区域：`perception_map_update` core、其 gtest，以及 `perception_profiling` harness。
- 高风险点：Patricia insert/delete canonicality、批量路径共享、candidate tree/store 原子提交、
  preflight 内存上界和 v1/v2 类型误用。
- 回滚点：Merkle core conformance、applier opt-in 接入、短 Gate 三段分别可独立移除；任何阶段
  失败都保持生产 `Vector + flat SHA-256`。

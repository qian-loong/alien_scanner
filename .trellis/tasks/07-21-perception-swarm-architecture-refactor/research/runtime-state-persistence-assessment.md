# Research: 运行状态持久化与重启恢复

- Query: Coordinator、EdgeAggregator、mapper 和 task state 是否需要持久化到本地存储？
- Scope: internal
- Date: 2026-07-21

## Findings

- 当前 Phase 3 的 mapper、merger、allocator 状态都在内存中；进程重启后依靠重新发布完整 OctoMap、诊断和任务重算恢复。
- D-003/D-009/D-012 已定义 source keyframe、contributor manifest 和两级 resync，能够在 Aggregator 或中央状态丢失后重新建立视图。
- D-015/D-024 规定 producer/pose/map 重启产生新 session/epoch，旧 delta、alignment 和任务不得跨 session 继承。
- D-018 的 Warm Standby 复制 committed control prefix，但没有要求同步完整地图 payload；C9b 的外部 quorum/fencing 需要保存 authority term，却不等于业务状态全部落盘。
- 要求 durable map/control log 会引入文件格式、原子落盘、损坏恢复、版本迁移、磁盘预算和写放大；同时还要证明恢复状态与当前 fleet session 一致。

## Implication

推荐 C1-C8 的权威运行状态默认 ephemeral：进程重启生成新 component/coordinator session 和相应 epoch，先 fail closed，再通过 registration、keyframe、alignment、topology、health 和 task resync 重建；不跨重启保留 lease 或 MotionIntent。C9a 复制 committed control prefix 以验证进程级接管，但不承诺同机掉电后的持久恢复；C9b 只要求外部 quorum/fencing 持久保存 authority term。持久化地图快照、控制日志和快速恢复可作为后续独立扩展，不应成为首条纵向链前提。

## Evidence

- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/prd.md:177`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/design.md:421`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/implement.md:61`
- `.trellis/tasks/07-21-perception-swarm-architecture-refactor/implement.md:110`

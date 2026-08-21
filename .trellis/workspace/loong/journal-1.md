# Journal - loong (Part 1)

> AI development session journal
> Started: 2026-07-21

---



## Session 1: Perception input and LiDAR fixtures

**Date**: 2026-07-25
**Task**: Perception input and LiDAR fixtures
**Branch**: `phase/4-perception-swarm-refactor`

### Summary

Completed the C1 perception input implementation and corrected deterministic LiDAR fixture geometry.

### Main Changes

- Added ROS-free perception contracts, ROS adapters, input node, interfaces, and deterministic fixtures.
- Corrected 2D/3D fixture geometry and expressed installation orientation through TF.
- Added the unified RViz scene and synchronized user, task, and quality documentation.
- Verified five ROS 2 packages with 60 passing tests and completed the final RViz inspection.


### Git Commits

| Hash | Message |
|------|---------|
| `508d3dc` | (see git log) |

### Testing

- Five ROS 2 packages built in Release mode; 60 tests passed with zero errors, failures, or skips.
- Final RViz inspection passed for flat 2D, +Z-tilted 2D, multi-line 3D, TF, and mixed display combinations.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 2: 完成 C1 外部性能与内存剖析

**Date**: 2026-07-26
**Task**: 完成 C1 外部性能与内存剖析
**Branch**: `phase/4-perception-swarm-refactor`

### Summary

完成外部 CPU、ROS tracing、堆、sanitizer、Valgrind 与三轮稳态内存分析，补齐可复现 runner、证据门、报告和长期复现文档。

### Main Changes

- Added a unified external profiling runner for perf, ROS tracing, Heaptrack,
  sanitizers, Valgrind, pidstat, smem, and smaps.
- Added fixed mixed-workload configuration, evidence validation, provenance,
  three-run trend analysis, and synthetic parser regression tests.
- Documented measured results, limitations, and reproducible build/run commands.

### Git Commits

| Hash | Message |
|------|---------|
| `101b79c` | phase4(step1): add perception resource profiling |

### Testing

- [OK] `bash -n scripts/profile-perception.sh`
- [OK] 11/11 synthetic parser regression tests
- [OK] Three historical plain-sample runs recomputed successfully
- [OK] `gpt-5.6-sol / xhigh` review completed with no outstanding findings

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 3: Complete C1 perception input

**Date**: 2026-07-27
**Task**: Complete C1 perception input
**Branch**: `phase/4-perception-swarm-refactor`

### Summary

Completed and validated C1 perception input, ray-evidence capability, deterministic YZ tunnel debug view, coverage gates, architecture planning, and source indexing; archived C1 after 88 passing tests and user RViz acceptance.

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `a61fb93` | (see git log) |
| `56274ea` | (see git log) |
| `f5aedf6` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 4: Complete C2 local observation map

**Date**: 2026-07-27
**Task**: Complete C2 local observation map
**Branch**: `phase/4-perception-swarm-refactor`

### Summary

Implemented and verified the authoritative vehicle-local occupancy map, continuous FullRay cave scene, pose/health fail-closed recovery, revision-locked reads, documentation, and RViz acceptance; Release build and 248 tests passed.

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `8dbdba5` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete

## Session 5: C1/C2 real-chain performance and memory retest (v3 executed end to end)

**Date**: 2026-08-03
**Task**: 08-01-c1-c2-perception-perf-memory-retest
**Branch**: `phase/4-perception-swarm-refactor`

### Summary

Trimmed the over-scoped retest plan to v3, then executed all measurement steps
with fresh acquisition: 3 scene-record rounds, replay-isolated baselines with
C2 stage decomposition, full sanitizer path coverage with a positive control,
and 30-minute cycle-replay leak runs for both product nodes. Report:
docs/perception-real-chain-retest.md.

### Main Changes

- Rewrote task artifacts (prd/design/implement) to v3: two-tier gates,
  sanitizer-as-path-coverage, stamp-shifted concat replay, 5 ACs.
- Upgraded cookbook: sanitizer hard rules, periodic-replay recipe (4a),
  two-tier validity gates, environment-vs-metric clarification.
- Productized scripts/restamp-concat-bag.py (--trim-s, QoS carry-over).
- Extended scripts/profile-graph.sh: replay --topics selection, replay-out
  recorder + count equivalence for C1, terminal-state skip, cycle gates for
  replay-loop, targeted ASan helper preload.
- Calibrated reconcile-graph-bags.py trailing allowance (1 -> 5, recovery
  stability gate consumes 0-3 startup frames).

### Key Results

- No business leaks on real paths: ASan/LSan zero reports (4 runs), positive
  control CAUGHT, Memcheck zero access errors with all loss in glibc dl-open.
- 30-min cycle runs: trough slope C2 +38.3 / C1 +32.0 KiB/min = 3.7%/3.1% of
  threshold; drift attributed via Heaptrack (F1: OcTree instances retained
  per epoch, Fast DDS pools).
- CPU ranking (contended scene): scanner 3.9% > C2 1.9% > C1 1.0%; system
  total PSS ~80 MB (6 nodes); C2 callback p99 2.60 ms (~2.6% of budget).
- F2 lesson: pose_timeout_s is a contract-fingerprint member - slowed replay
  must scale only arrival-clock watchdogs, never contract members.

### Git Commits

| Hash | Message |
|------|---------|
| `1197b35` | feat(profiling): real-chain C1/C2 retest with cycle-replay leak harness |

### Testing

- [OK] All measurement runs valid=true with blocking gates passed
- [OK] Positive control caught planted 4096 B leak (exit 23)
- [OK] bash -n on modified runners; tool validated on 3-copy trim bag

### Status

[OK] **Completed** (commit pending)

### Next Steps

- Follow-up finding F1: review epoch-retirement release path for OcTree/
  snapshot retention (~35 KiB/min drift, 27x below threshold)


## Session 5: 完成 C3 性能与内存验收收口

**Date**: 2026-08-09
**Task**: 完成 C3 性能与内存验收收口
**Branch**: `phase/4-c3-map-updates`

### Summary

完成 C3 profiling 数据面、分析器与回归测试收口；固化 3x300 秒矩阵、ASan/LSan、Memcheck 和容量边界证据；同步地图更新 QA、规范及父子任务状态，并归档 C3 任务。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `b4e26b1` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 6: Complete C4 communication data plane

**Date**: 2026-08-13
**Task**: Complete C4 communication data plane
**Branch**: `phase/4-c4-communication-data-plane`

### Summary

完成 C4 通信数据面、可视化验收、正式性能内存矩阵、sanitizer/memcheck、证据分层，并确认 C5 前实施分块不可变快照与 COW。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `add6ca3` | (see git log) |
| `1459593` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 7: Relocate C3 profiling evidence

**Date**: 2026-08-13
**Task**: Relocate C3 profiling evidence
**Branch**: `chore/profiling-evidence-layout`

### Summary

将 C3 872 个 profiling raw 迁移到 profiling-archive，保留 8 个摘要与历史路径，新增 relocation provenance、证据布局规范和 QA；已验证 872/872 ignore、文件数/字节/树摘要守恒，待 squash 合并到 main。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `26636e4` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 8: Complete incremental Merkle hash v2 evaluation

**Date**: 2026-08-15
**Task**: Complete incremental Merkle hash v2 evaluation
**Branch**: `phase/4-incremental-merkle-hash-v2`

### Summary

Completed chunk-16 incremental Merkle v2 prototype, conformance, performance-memory Gate, evidence relocation, and C4.3 rollout sequencing.

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `6865c08` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 9: 完成 C4.1 与 C4.3 收尾归档

**Date**: 2026-08-17
**Task**: 完成 C4.1 与 C4.3 收尾归档
**Branch**: `phase/4-merkle-v2-production-integration`

### Summary

C4.1 分块 COW Gate B no-go 与 C4.3 Merkle v2 production Gate GO 的状态、证据和父任务依赖已同步；C4.1 中间 estimator raw 已按完整性校验迁入 profiling archive；两个子任务已归档，C5 前置阻塞解除。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `5875a41` | (see git log) |
| `4033bd1` | (see git log) |
| `6c742cb` | (see git log) |
| `0ef09ee` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 10: 完成 C5a 拓扑基础

**Date**: 2026-08-17
**Task**: 完成 C5a 拓扑基础
**Branch**: `phase/5a-topology-foundation`

### Summary

实现稳定身份、成员生命周期、三逻辑图、link/route epoch 高水位与 ROS 转换；受影响包构建通过，411 项测试全部通过。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `7036e5e` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 11: 完成 C5b 角色与能力契约

**Date**: 2026-08-17
**Task**: 完成 C5b 角色与能力契约
**Branch**: `phase/5b-role-contract`

### Summary

实现 declared/effective capability、主角色与组合服务、全局 role_epoch、健康与预算门、quiesce/handoff/commit/rollback 状态机及 bounded ROS 转换；新增 15 项角色/转换测试，全量 428 tests 通过，并同步 backend spec 与 validation。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `047f852` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete


## Session 12: 完成 C5 拓扑与运行期收尾

**Date**: 2026-08-21
**Task**: 完成 C5 拓扑与运行期收尾
**Branch**: `phase/5d-edge-aggregator`

### Summary

完成并验证 C5a 拓扑基础、C5b 角色与能力契约、C5c Explorer 与纯 Relay 运行期、C5d EdgeAggregator 聚合运行期；归档 C5a-C5d 任务及证据，父任务进度更新为 C5 完成并转入 C6 多 Region 与任务生命周期。C5d 构建与全量 718 测试通过，N=5 聚合链、Relay failover、resync、aggregate receiver 和 RViz Marker 验收通过。保留既有无关 profiling/build 未跟踪产物。

### Main Changes

- Detailed change bullets were not supplied; see the summary above.

### Git Commits

| Hash | Message |
|------|---------|
| `7036e5e` | (see git log) |
| `047f852` | (see git log) |
| `c22eda5` | (see git log) |
| `e1eae39` | (see git log) |

### Testing

- Validation was not recorded for this session.

### Status

[OK] **Completed**

### Next Steps

- None - task complete

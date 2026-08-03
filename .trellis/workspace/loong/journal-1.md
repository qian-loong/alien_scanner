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

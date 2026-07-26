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

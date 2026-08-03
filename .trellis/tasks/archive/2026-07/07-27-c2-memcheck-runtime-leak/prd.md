# C2 Memcheck runtime leak attribution

## Goal

Attribute the Memcheck findings rooted in rcutils/glibc dynamic loading and
LTTng TLS without classifying them as C2 business leaks unless comparative
evidence establishes a C2-owned cause.

## Requirements

- Reproduce or isolate the 128-byte definite loss rooted at glibc
  `resize_scopes` through `rcutils_load_shared_library`, and the 384-byte
  possible loss rooted at glibc pthread TLS allocation through
  `liblttng-ust`.
- Compare the C2 target with a minimal ROS 2 control that exercises the
  relevant runtime loading and tracing paths. Preserve exact tool, library,
  ELF, stack, and exit provenance for every comparison.
- Treat suppressions as attribution artifacts, not proof that no leak exists.
  Any proposed suppression must match only the confirmed runtime/upstream
  stack and must not hide C2-owned allocations or invalid memory access.
- Keep the original Memcheck directory unchanged and do not modify C2
  occupancy, health, epoch/revision, FullRay, or visualization semantics.

## Acceptance Criteria

- [ ] Minimal-control comparison establishes whether each finding is owned by
  C2 code, ROS 2/rcutils, glibc dynamic loading, or LTTng runtime teardown;
  uncertain attribution remains explicitly unresolved.
- [ ] A runtime/upstream classification has a narrowly validated suppression
  or upstream reference/reproducer; a C2-owned classification creates a
  separate semantic fix task instead of changing C2 here.
- [ ] The final evidence keeps definite, indirect, possible, still-reachable,
  and invalid-access categories separate and does not cite the parent's
  throughput-invalid run as a valid C2 baseline.

## Notes

- Parent raw log:
  `/tmp/alien-c2-smoke-valgrind-memcheck-20260727/memcheck.89489.log`.

---

## 收口（2026-07-31）：**未解决，不再跟进**

未完成 rcutils/glibc dlopen 的 128 B definite loss 与 LTTng TLS 384 B possible
loss 的逐帧归因。

已确认的事实：

- 二者栈均位于 ROS/glibc 动态加载器与 LTTng 运行时，**未经过 C2 业务源码**。
- 退出期残留量在 bounded 与 expanding 下**完全相同**（455.54 K），说明与地图
  规模无关，是固定的退出期行为，不随负载增长。
- 交叉 ASan/LSan/Memcheck 后**不定性为 C2 业务泄漏**。

**为什么不再跟进**：量级固定且与业务无关，对 C2 的内存结论无影响；Valgrind 在
本环境又无法在真实负载下运行，进一步归因需换环境。

现状记录于 `docs/local-map-resource-profiling.md` §4.5 与 §8。

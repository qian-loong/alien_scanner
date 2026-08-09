# C3 validation evidence

This directory contains final, task-scoped validation logs for the C3 map-update implementation.
Generated build/install trees remain outside the repository; only command output and tool reports
needed to audit the result are retained here.

Repository retention is intentionally split: Git tracks this README, provenance, and aggregate
JSON. Raw logs, including sanitizer and capacity reports, remain local task evidence and are not
added to repository history. Their names, source identities, and hashes remain recorded below and
in `analysis-provenance.txt`.

The first full-dependency ASan build (`asan-build.log`) stopped when Docker Desktop became
unresponsive. It is retained as environment-failure evidence and is not a source failure. The
resource-limited rerun uses the main workspace install as an underlay, rebuilds only
`perception_map_update` and `perception_local_map`, and limits colcon to one package worker with
`MAKEFLAGS=-j2`.

Final memory-safety evidence:

- `asan-map-update-core.log`: 26/26 core tests passed with ASan/LSan enabled.
- `asan-local-map-adapters.log`: 5/5 canonical adapter and async producer tests passed.
- `memcheck-map-update-core.log`: 0 errors; all heap blocks freed.
- `memcheck-local-map-adapters.log`: 0 errors and 0 definite/indirect/possible
  leaks. OctoMap's process-lifetime static registries account for 2,840 bytes
  reported as `still reachable`.

The Release four-package baseline (`perception_interfaces`,
`perception_map_update`, `perception_local_map`, and `perception_profiling`)
completed 320 tests with zero errors, failures, or skips. The profiling package
contributed 24 passing tests. A later visualization-only follow-up added one
periodic MarkerArray regression; the unchanged production packages were not
rerun, while the full `perception_profiling` package completed 25 tests with
zero errors, failures, or skips. The expanding-capacity case reconstructed 1,812,520 known
cells with 1,020 changed cells. Its 25,511-byte sparse delta was smaller than
the matching 45,313,011-byte keyframe. The latest local measurements were:

```text
keyframe_prepare_ms=931
keyframe_encode_ns=762019493
keyframe_hash_ns=20588906
keyframe_apply_ms=297
delta_prepare_ms=199
delta_diff_ns=46369404
delta_encode_ns=462012
delta_hash_ns=37088
delta_apply_ms=178
```

These absolute timings record the local environment only. Cross-run variation
is approximately 30-50%, so they do not support a percentage improvement
claim.

The current deterministic replay/oracle drives the real C2
`LocalObservationMapper + OctoMapBackend`, acquires an exact read transaction for
every committed revision, and runs the production canonical adapter, producer,
and applier. Every checkpoint matches source identity, geometry, revision,
content hash, and complete cells. Repeated fixed input also matches revision,
update kind, content hash, and update hash. The RViz launch publishes side-by-side
oracle/reconstructed maps, a difference layer, and an OK diagnostic; its GUI
smoke completed without plugin/display errors. Standard Octomap topics remain
available for byte comparison, while MarkerArray is used for display because the
container's `octomap_rviz_plugins` has a ROS/system OctoMap ABI mismatch.
The visualization-only follow-up caches the one-shot replay result and republishes
the map/difference MarkerArrays at a configurable rate (1 Hz by default). A
volatile subscriber created after replay completion received multiple later
samples with increasing header stamps, proving display recovery does not rely
on transient-local history or rerun the oracle.

The final manual RViz acceptance on 2026-08-05 confirmed that disabling and
re-enabling the map MarkerArray display restores the visualization without
restarting the replay. `ros2 topic hz /map_update_replay/map_markers` measured
`1.000 Hz`; the replay diagnostic reported `match=true`, 59 matching checkpoints,
and zero missing, unexpected, or state-mismatched cells. RViz then exited cleanly,
with no plugin, display, or exception errors in the launch log.

The later C3 acceptance-view follow-up adds independently selectable resync and
epoch-reset RViz groups. Each group now displays one current stage at a fixed
position and advances through four cached stages every 2 seconds by default.
The scenario seeds are the first, middle, and final exact-revision snapshots;
the production producer/applier prove that dropped/gap and retired-epoch stages
retain the last valid receiver revision, while the recovery keyframe advances
to the final revision or establishes epoch 2 at revision 1. Stage advancement
only changes the cached stage index; it does not trigger an extra publication.
The default `visualization_publish_rate_hz=1.0` therefore remains fixed in both
baseline and acceptance topics. On 2026-08-06, the actual launch kept the node
and RViz processes alive with OpenGL 4.5 and no plugin/display/exception errors;
`ros2 topic hz` measured `1.000 Hz` for both
`/map_update_replay/map_markers` and `/map_update_replay/resync/markers`.
The profiling package contains 27 tests; the final package run completed with
zero errors, failures, or skips.

The four-package joint smoke was rerun after stabilizing the two affected launch
tests. `test_canonical_equivalence.py` now checks that each observation's
acquisition stamp has a matching authoritative PoseEstimate stamp from the same
odometry chain; `test_profile_integration.py` additionally checks the fixed
profile fixture's `stamp + 50 ms` lead-pose contract. Neither test infers
causality from cross-topic callback arrival order. Each test passed in three
independent consecutive runs, and the final four-package run completed with no
errors, failures, or skips. The workspace aggregate reported by
`colcon test-result --verbose` was 323 tests with zero errors, failures, or
skips.

## Frozen-version performance and memory matrix

The frozen implementation at
`7bb76643e8c800fa938406e87e42ee9923151d92` was measured with one
RelWithDebInfo profiling build and the same bounded 10 Hz workload in three
C3 modes. Each mode has three independent 300-second formal runs after an
approximately 800-revision warmup. Every run completed normally, passed the
raw-evidence analyzer, retained exact workload identity, and met its role,
count, and resource gates.

| C3 mode | Steady CPU mean | RSS peak KiB | PSS peak KiB | USS peak KiB | RSS slope KiB/min |
| --- | ---: | ---: | ---: | ---: | ---: |
| disabled | 11.88-12.50% | 62,648-63,032 | 44,356-44,665 | 40,636-41,104 | 0-289.7 |
| enabled | 66.91-67.54% | 102,392-104,516 | 81,638-83,935 | 77,996-80,080 | 0-448.0 |
| keyframe-only | 76.04-77.50% | 102,728-104,472 | 84,442-87,153 | 78,452-80,008 | 520.3-866.6 |

Each run processed 3,001 observations and 3,001-3,002 revisions. Enabled
runs published 3,803-3,805 updates, comprising 30 keyframes and 3,773-3,775
deltas, and converged to the latest revision after input stopped. Keyframe-only
runs published 3,804-3,805 keyframes, no deltas, and also converged. Disabled
runs produced no C3 updates, as required.

The three aggregates all report `suspected_sustained_growth=false` under the
1,024 KiB/min bounded-growth rule. The positive keyframe-only slopes remain an
observation item for a future long-duration soak; the current result means that
the configured 300-second formal window did not trigger the sustained-growth
gate, not that unlimited operation is proven to have zero growth.

Enabled materialization P95 was approximately 54 ms, and diff P95 was
approximately 2.6-2.8 ms. Enabled versus disabled CPU is a large effect above
this machine's 30-50% resolution boundary and therefore demonstrates the extra
cost of the current full-snapshot comparison path. Keyframe-only versus enabled
is only about a 15% difference and is reported without a directional
performance conclusion.

The raw directories are named
`final-bounded-20260808-{disabled,enabled,keyframe-only}-run{1,2,3}`. Their
capture-time `analysis-summary.json` files and the original `*-aggregate.json`
files were generated with the source state identified by
`source_diff_sha256=6d68747bbf94b0a7183522e0b8a63209d83779f5f1634d066aec44c5a4c4fc5b`.
The `*-aggregate-cpu.json` files are a later read-only reanalysis of those same
raw directories after tracee-only pidstat summaries were added. The current
analysis module SHA-256 is
`66e6ff4f0eeeb862ddf6fc82fea3e3c3eb5387ac629825afed56ae6beb671cae`;
all hashes and artifact relationships are recorded in
`analysis-provenance.txt`.

Short expanding smoke runs passed for disabled and keyframe-only modes. The
first enabled smoke recorded five revision-skew failures and is retained as
invalid evidence; an independent enabled rerun passed. Because the rerun did
not reproduce the skew, the first run is treated as a scheduling-jitter
observation rather than a stable capacity knee.

Final closeout verification on 2026-08-09 passed `bash -n` for the runner,
Python bytecode compilation, and all 46 analyzer regression tests (one
environment-conditional test skipped). The current analyzer independently
accepted all three 3-run formal aggregates. `perception_profiling` rebuilt
successfully and completed 27 package tests with zero errors, failures, or
skips.

Historical Phase 3 source-level bag assets remain unavailable. This is recorded
as an unavailable optional compatibility input, not as an unimplemented C3
function or a blocker for the exact-revision replay acceptance.

One cross-stage evidence gap remains intentionally open: shared-view alignment
admission and invalidation belong to the later aggregation path. C3 validates
source-local updates but does not claim that consumer-side integration is complete.

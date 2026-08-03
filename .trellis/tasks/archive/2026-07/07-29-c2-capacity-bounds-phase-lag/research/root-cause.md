# Research: C2 capacity-ramp `revision 700: production/oracle bounds mismatch` — root cause

- **Query**: why does production known-bounds `max_x` trail the oracle by up to 4 revisions at rev 700 but match exactly at rev 100..600?
- **Scope**: internal (code + raw evidence, read-only)
- **Raw dir**: `/tmp/alien-c2-capacity-ramp-20260729-v8` (container `alien-scanner-dev`)
- **Date**: 2026-07-29

---

## 1. Verdict (one line)

Production is **not** geometrically wrong: from ~revision 610 the `perception_local_map_node`
saturates one CPU core, its single-threaded executor can no longer drain the 20 Hz pose
subscription, and each observation is therefore projected with a pose that is **7 pose samples
(0.35 s / 0.175 m) stale**. 0.175 m ≥ 1 voxel (0.2 m grid, frontier moves 0.05 m/revision), so the
frontier voxel index — and hence `known_bounds.max_x` — sits exactly one voxel behind the oracle
from rev ~613 onwards. Checkpoints are every 100 revisions = exactly 5.000 m = exactly 25 voxels,
so **every** checkpoint lands on an exact voxel boundary and any non-zero pose lag is guaranteed to
surface there as a one-voxel `max_x` mismatch.

Classification: **production defect (node-level, `PerceptionLocalMapNode` + `OctoMapBackend::known_bounds`)**
— not an oracle model defect, not an analyzer join defect.

---

## 2. Geometry of the fixture (established, needed for the numbers)

From `ProfileScenario.hpp/.cpp` (`FrozenProfileConfig`) and `ProfileFixtureNode.cpp`:

| quantity | value |
|---|---|
| pose x | `x(seq) = 1.0 + 0.5 * (seq-1) * 0.1` → **+0.05 m per revision** |
| observation period | 100 ms (10 Hz); pose stream 20 Hz (`pose` at `t`, `lead_pose` at `t+50 ms`, `x+0.025`) |
| resolution | 0.2 m, lattice origin 0 |
| `body_from_sensor` | quaternion (0.5,0.5,0.5,0.5) → exact permutation matrix; scan plane is the **body YZ plane** |
| revision ↔ sequence | `revision r` = `sequence r+1` (first sample is `Unavailable`, oracle manifest `applied=5999, unavailable=1`) |

Consequence (verified in code, `project_observation` + `transform_point`): the linear part of
`map_from_sensor` has row0 = (0,0,1) with **exact** zeros, and the scan points have `z_sensor = 0`,
so **every ray endpoint has `x == pose.x` exactly**. The whole map is a stack of 1-voxel-thick
x-slabs; `max_x` is fully determined by `floor(x_pose / 0.2)`:

```
known_bounds.max_x = (floor(x_used / 0.2) + 1) * 0.2      # OctoMapBackend::known_bounds + voxel_bounds
```

A pure-Python replay of that formula with `x = 1.0 + 0.5*(r*1e8/1e9)` reproduces the **oracle
checkpoints bit-for-bit** (rev 100→6.2, 200→11.2, …, 600→31.2, 700→36.2) and reproduces
**production exactly for revisions 1..609**, including the irregular 3/4/5-revision step pattern
caused by IEEE rounding of `x/0.2` at exact multiples of 0.2 (e.g. first `max_x=1.8` at rev 12,
`2.6` at rev 29, `3.0` at rev 37 — production matches all of them).

---

## 3. Numeric evidence

### 3.1 The production/oracle divergence is a growing pose phase lag, not a rounding artifact

`states.csv` carries `changed_cell_count` per revision. It is **exactly 4520 on the revision where
the sensor plane enters a fresh x-slab and exactly 0 on every other revision** — i.e. it is a direct
readout of "which revision did the *used* pose cross a voxel boundary".

Fitting `x_used(r) = 1.0 + 0.05 r − 0.025 k` (k = pose samples of staleness) against the production
crossing set, per 50-revision window:

| revisions | best k | exact crossing matches |
|---|---|---|
| 1–600 (12 windows) | **k = 0** | 12/12, 13/13, … (perfect in every window) |
| 601–650 | mixed (0→7, transition) | 5/11 |
| 651–700 | 7 | 8/13 |
| **701–750** | **k = 7** | **12/12** |
| 751–761 | 7 | 2/3 |

k = 7 → `x_used = x_true − 0.175 m`, i.e. the newest pose the mapper had processed was **350 ms
older** than the observation it projected.

Check against the reported facts:
- rev 700: `x_true = 36.0`, `x_used = 35.825` → `floor(35.825/0.2) = 179` → `max_x = 36.0` (production, observed).
  Oracle uses `x = 36.0` → index 180 → `max_x = 36.2` (observed). **One voxel.**
- production reaches `max_x = 36.2` when `x_used ≥ 36.0` ⇒ `x_true ≥ 36.175` ⇒ `r ≥ 703.5` ⇒ **r = 704** (observed).

Per-revision index comparison (`prod_idx` from `states.csv`, `model_idx` = oracle formula):
lag = 0 for every revision ≤ 610; lag = exactly 1 voxel for every revision ≥ ~615 through 761.
First divergence: index 158 (`max_x` 31.8) — model/oracle at rev 612, production at rev 613.

Only `max_x` can diverge: `min_x` is frozen at 1.0 by the first applied sample (rev 1, x = 1.05) and
`min_y/max_y/min_z/max_z` are set by the 30 m no-return sector on the first scans and never grow —
which is exactly what the CSVs show.

### 3.2 The lag is caused by executor/CPU saturation of the mapper node

`states.csv` `state_sequence` counts every callback of the node (each of `on_pose`, `on_health`,
`on_observation` and the 4 Hz heartbeat ends in `publish_state`). Expected input rate = 20 pose +
20 health + 10 observation + 4 heartbeat ≈ 54–55 events/s.

| wall window | node callbacks/s | revision at end |
|---|---|---|
| 5–55 s | **55.0** (full rate) | 79 … 578 |
| 60–65 s | 52.5 | 628 |
| 65–70 s | 45.4 | 678 |
| 70–75 s | 39.8 | 728 |
| 75–78 s | **36.0** | 761 |

`pidstat.txt` for `perception_loca…` (%CPU): 21 → 27 → 37 → 51 → 67 → 85 → 91 → **99–101 %** — the
node grows linearly with map size and **pins one core from ≈ revision 570–600**, i.e. immediately
before the first frontier divergence at rev 612/613.

The observation path never starves (all 761 revisions present, `stamp_ns` matches
`10e9 + r*1e8` for every revision, revision cadence stays 100.0 ms ± 0.15 s total drift over 76 s),
so the ~18 events/s deficit falls on the pose/health subscriptions. `pose_subscription_` is
`QoS(10).reliable()`, so a slow reader keeps a sliding 10-deep window of the newest samples and the
mapper's `pose_history` newest entry falls behind by up to 10 samples — measured: 7.

`pose_for_observation_locked()` (LocalObservationMapper.cpp:713-741) then legally picks the newest
pose with `stamp ≤ observation.origin_stamp` (freshness budget 1 s, so a 0.35 s-old pose is
accepted without any diagnostic — `diagnostics.csv` contains a single startup row and nothing else).

### 3.3 What consumes the CPU (attribution, estimated)

Two O(N-known-cells) code paths run per callback / per revision:

1. `OctoMapBackend::known_bounds()` (OctoMapBackend.cpp:62-80) **linearly scans the whole
   `std::set<VoxelIndex> known_cells`** (795 520 cells at rev 700) to recompute min/max. It is
   called from `LocalObservationMapper::state()` (LocalObservationMapper.cpp:1263) — i.e. from
   `PerceptionLocalMapNode::publish_state()` on **every** pose/health/observation callback and every
   heartbeat (~55 Hz) — and from both `acquire_read_transaction()` overloads (lines 1282, 1308,
   once more per revision).
2. `publish_octomap()` → `OctoMapSnapshotBridge::materialize` → `octomap_msgs::binaryMapToMsg`
   serialises the **entire** tree once per revision (`snapshots.csv`: 761 snapshots, 366 KB each at
   the end → ~3.6 MB/s).

Consistency check: fitting `%CPU ≈ a + b·N` on (117 k cells, ~24 %) and (682 k cells, ~100 %) gives
b ≈ 0.134 %/1000 cells ⇒ ~1.35 ns per known cell per second of runtime. Divided over the ~65
full-set traversals per second that path 1 performs, that is ~20 ns per cell per traversal — the
right order for an RB-tree walk over a 50 MB `std::set`. Path 1 is therefore the dominant term,
path 2 a clear second. (Exact split is measurable with the
`PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS` build: `StatePublication` vs
`SnapshotTotal` vs `MapperApply`; this run has no stage CSV.)

---

## 4. Hypotheses from the brief — verdicts

- **H1 (voxel-boundary rounding, oracle "analytic model")** — *rejected as stated, but boundary
  sensitivity is the amplifier.* The oracle is **not** analytic: `ProfileOracle::run` drives the same
  `LocalObservationMapper` + `OctoMapBackend` in a serial loop (ProfileOracle.cpp:58-65, 280-367),
  so both sides use identical `floor((x−origin)/res)` code. What is true: because 100 revisions =
  5.000 m = 25 voxels exactly, **every** checkpoint frontier sits on an exact voxel boundary, so a
  pose lag as small as one 0.025 m pose sample is guaranteed to show up as a 1-voxel `max_x`
  mismatch at a checkpoint. That is why the symptom is a clean 0.2 m offset. (Signature: after
  rev 690 production's slab crossings are perfectly 4-revision-spaced, whereas the exact-grid model
  jitters 3/5 — because a 7×0.025 m offset never lands on a 0.2 boundary.)
- **H2 (max-range endpoint semantics)** — *rejected.* Range at the frontier is unrelated: the scan
  plane is perpendicular to +X, so `max_x` comes from the ray **origin** x, not from any hit/no-return
  endpoint distance (all endpoints share `x == pose.x` exactly). Tunnel radii are 2.0/2.5 m vs a
  30 m sensor range — nowhere near the max-range boundary. The 30 m no-return sector only sets
  `max_y/max_z` (7.8 / 31.4), which match at every checkpoint.
- **H3 (analyzer join picks a lagging production row)** — *rejected.* `_validate_oracle_join`
  (scripts/lib/local_map_profile_analysis.py:181-203) joins strictly on `revision`, and
  `_state_by_revision` (lines 125-149) **raises** if two rows with the same revision disagree on
  bounds. Verified in the raw data: every revision 1..761 has ≥1 state row, no revision carries two
  different `max_x` values, and `stamp_ns` matches the revision for all 761 revisions. The join is
  correct and the compared row is the right one.
- **New / accepted hypothesis (H4)**: CPU-saturation-induced pose staleness. Verified above.

---

## 5. Classification and minimal fix location (not implemented)

**Classification: production defect** — specifically a *production node performance defect that
breaks oracle equivalence*, not a mapping-math defect. The oracle checkpoints describe the correct
result of the frozen input sequence; production silently degrades its input freshness under load.

Minimal correct fix location, in priority order:

1. `ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp` →
   `OctoMapBackend::known_bounds()` (line 62) and `OctoMapBackend::Impl` (line 13) /
   `apply()`'s `apply_cell` lambda (line 184).
   *Minimal change:* maintain incremental `min`/`max` `VoxelIndex` in `Impl`, updated O(1) per newly
   inserted cell in `apply_cell`, so `known_bounds()` becomes O(1) instead of O(known_cells). This
   removes the ~65 full-set scans per second that dominate the node's CPU and is behaviour-preserving
   (identical bounds values; `known_cells` is insert-only in this backend).
2. If still saturated: `ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp`
   → `on_pose` / `on_health` (lines 523-536, 501-521) call `publish_state()` on every input message
   (~40 Hz). Rate-limiting state publication to the heartbeat + per-revision would cut it further.
   Third-order: `publish_octomap()` (line 580) full-tree serialisation on every revision.

Neither `ProfileOracle` nor `local_map_profile_analysis.py` needs to change to fix the mismatch;
changing them would only hide a real capacity finding. (If the C2 gate is *meant* to characterise
capacity rather than assert equivalence, the alternative product decision is to declare the
pose-freshness degradation an expected outcome and record it, but that is a scope decision, not a
correctness fix.)

---

## 6. Does the mechanism explain rev 100..600 matching? (required)

Yes, exactly and quantitatively:

- For revisions 1–600 the node ran at the **full 55 callbacks/s input rate** (measured) with CPU
  below saturation, so every pose message was processed before the next observation callback;
  `pose_for_observation_locked` always returned the pose with `stamp == observation.origin_stamp`
  and `x = x_true`. The lag fit gives **k = 0 in all twelve 50-revision windows up to rev 600**, and
  every slab-crossing revision matches the exact-grid model, including its irregular 3/4/5 spacing.
- Therefore checkpoints at rev 100, 200, 300, 400, 500, 600 reproduce the oracle bounds exactly
  (6.2 / 11.2 / 16.2 / 21.2 / 26.2 / 31.2 — confirmed in `oracle_checkpoints.csv` and `states.csv`).
- CPU reached 99–101 % at ≈ rev 570–600; the first frontier divergence is rev 612/613 (lag ≈ 1 pose
  sample), growing to k = 7 by rev ~700. The rev 600 checkpoint therefore still passes and the
  **rev 700 checkpoint is the first one taken after the lag exceeded one voxel** — matching the
  analyzer's failure message exactly.

---

## 7. Key file references

| File | Why |
|---|---|
| `ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp:62` | `known_bounds()` O(N) scan (root cost) |
| `ws/src/alien_perception/perception_local_map/src/OctoMapBackend.cpp:160` | `apply()` / `apply_cell` — where `known_cells` grows |
| `ws/src/alien_perception/perception_local_map/src/LocalObservationMapper.cpp:713` | `pose_for_observation_locked` — newest pose ≤ obs stamp (uses stale pose legally) |
| `ws/src/alien_perception/perception_local_map/src/LocalObservationMapper.cpp:396` | `project_observation` — endpoint x ≡ pose x |
| `ws/src/alien_perception/perception_local_map/src/LocalObservationMapper.cpp:1250` | `state()` → `known_bounds()` per publish |
| `ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp:299` | pose subscription `QoS(10).reliable()`, single-threaded executor |
| `ws/src/alien_perception/perception_local_map/src/PerceptionLocalMapNode.cpp:612` | `publish_state()` called from every callback |
| `ws/src/alien_perception/perception_local_map/src/MapTypes.cpp:126,162` | `quantize_point` / `voxel_bounds` — the `floor(x/res)` and `(max+1)*res` conventions |
| `ws/src/alien_perception/perception_profiling/src/ProfileOracle.cpp:280` | oracle replays the same mapper serially (no timing) |
| `ws/src/alien_perception/perception_profiling/src/ProfileScenario.cpp:152` | `x_for_sequence` — 0.05 m/revision |
| `ws/src/alien_perception/perception_profiling/src/ProfileFixtureNode.cpp:112` | 50 ms tick: pose, lead pose, observation ordering |
| `scripts/lib/local_map_profile_analysis.py:181` | `_validate_oracle_join` — the failing assertion |

## 8. Caveats

- The CPU attribution between `known_bounds()` and octomap serialisation is an **estimate** from the
  linear `%CPU` vs known-cell fit; it is not directly measured in this run (no stage-latency CSV).
  A tracepoint-enabled rerun (`StatePublication` vs `SnapshotTotal`) would confirm the split.
- The pose lag k = 7 is inferred from slab-crossing revisions (`changed_cell_count`), which is an
  exact observable, but the run contains no direct log of "which pose stamp was used per
  observation". A one-line debug field (used pose stamp in `LocalMapState`) would make this directly
  observable in future runs.
- No file outside this research directory was modified; no ROS workload was executed.

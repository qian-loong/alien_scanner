# Quality Guidelines

> Code quality standards for backend development.

---

## Overview

<!--
Document your project's quality standards here.

Questions to answer:
- What patterns are forbidden?
- What linting rules do you enforce?
- What are your testing requirements?
- What code review standards apply?
-->

(To be filled by the team)

## ROS 2 CMake Superbuild Contracts

### 1. Scope / Trigger

These rules apply when the repository root CMake project adds multiple
`ament_cmake` packages as subdirectories for CLion indexing.

### 2. Signatures

- Package discovery: `colcon list --topological-order --paths-only`.
- Package selection: `colcon list --packages-up-to <pkg> ...` so selected
  packages include recursive workspace build dependencies.
- Directory selection: discover package names below each configured source
  root with `colcon list --base-paths <root> --names-only`, then pass the
  merged package-name set through the full-workspace `--packages-up-to` query.
- Exported library target names stay package-scoped, for example
  `perception_core::perception_core`.

### 3. Contracts

- The root CMake must consume colcon's topological order; path sorting is not a
  valid dependency resolver.
- Package manifest change detection may use `GLOB_RECURSE CONFIGURE_DEPENDS`,
  but the glob result must not replace colcon package discovery or impose a
  fixed source-tree depth.
- A package that can be linked by another source package defines a build-tree
  alias with the same name as its installed exported target.
- Internal consumers use a local target when it exists and call
  `find_package()` only for standalone package builds.
- ROSIDL consumers use a local `<pkg>__rosidl_typesupport_cpp` target in a
  superbuild; installed package targets are the standalone fallback.
- Hand-written headers use `BUILD_INTERFACE` source paths. ROSIDL-generated
  headers use the current build generation directory.
- With the current CMake 3.8 package baseline, hand-written public headers are
  attached to their implementation target with `target_sources(... PRIVATE ...)`.
  Public API propagation remains the responsibility of PUBLIC include paths,
  install rules, and ament exports.

### 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| `colcon` is unavailable | Root CMake stops with a container/setup error |
| Selected package name is unknown | Root CMake stops with colcon's package error |
| Selected package root is missing or outside `ws/src` | Root CMake stops and identifies the invalid root |
| Selected package root contains no discoverable package | Root CMake stops instead of silently loading nothing |
| A hand-written public header is opened directly in CLion | CMake File API associates it with the real implementation target |
| Adapter is configured after local Core | Adapter compile commands contain the Core source include path |
| Local and installed ROSIDL targets are both visible | Consumer links the local target only; no duplicate RPATH cycle |
| Clean standalone colcon build | Config files are under `share/<pkg>/cmake`; no hand-written `lib/cmake/<pkg>` config |

### 5. Good / Base / Bad Cases

- Good: a source root is resolved to package names first, then those names go
  through the full-workspace dependency closure and topological query.
- Good: `perception_core` is added before `perception_adapters`, and Adapter
  links `perception_core::perception_core` without `${perception_core_INCLUDE_DIRS}`.
- Base: an external ROS package is resolved by `find_package()` from the ROS
  installation prefix.
- Bad: a directory path is passed directly to `--packages-up-to`, a fixed-depth
  glob limits package manifests, or glob ordering replaces colcon ordering.

### 6. Tests Required

- Configure a selected downstream package and assert dependency-first log order.
- Configure a package root and assert all packages below it are discovered,
  then assert dependencies outside that root are still loaded first.
- Configure missing, out-of-tree, and package-empty roots and assert each fails
  with a root-specific diagnostic.
- Inspect CMake's glob verification input and assert manifest watching uses a
  recursive expression instead of a fixed number of source-tree levels.
- Inspect `compile_commands.json` for source include paths and absence of the
  corresponding install include path.
- Inspect CMake File API target sources and assert each hand-written public
  header belongs to its real library target.
- Configure all workspace packages and assert no local/install ROSIDL RPATH
  cycle warning.
- Perform a clean colcon build and test; assert exported target names and
  `colcon test-result` reports zero failures.

### 7. Wrong vs Correct

Wrong:

```cmake
list(SORT _package_xmls)
find_package(perception_core REQUIRED)
target_include_directories(perception_adapters PUBLIC
  $<BUILD_INTERFACE:${perception_core_INCLUDE_DIRS}>)
```

Correct:

```cmake
if(NOT TARGET perception_core::perception_core)
  find_package(perception_core REQUIRED)
endif()
target_link_libraries(perception_adapters
  PUBLIC perception_core::perception_core)
target_sources(perception_adapters
  PRIVATE
  include/perception_adapters/laser_scan_adapter.hpp)
```

Directory selection must also preserve the two-phase colcon boundary.

Wrong:

```cmake
file(GLOB _package_xmls
  "${ROS2_WS_DIR}/src/*/package.xml"
  "${ROS2_WS_DIR}/src/*/*/package.xml")
list(APPEND _colcon_command --packages-up-to ${ROS2_PACKAGE_ROOTS})
```

Correct:

```cmake
# First query: colcon list --base-paths <root> --names-only
# Final query: colcon list --base-paths <ws/src>
#              --packages-up-to <resolved names> --topological-order
file(GLOB_RECURSE _package_manifests CONFIGURE_DEPENDS
  "${ROS2_WS_DIR}/src/*/package.xml")
```

## ROS 2 Launch and Integration Test Contracts

### 1. Scope / Trigger

These rules apply to ROS 2 packages that install Python launch files or use
`launch_testing` against C++ nodes.

### 2. Signatures

- A launch module exports `generate_launch_description()` and returns a
  `launch.LaunchDescription`.
- An active launch test class derives from `unittest.TestCase` and uses
  `setUpClass` / `tearDownClass` for its shared `rclpy` node.

### 3. Contracts

- `.launch.py` files use `launch_ros.actions.Node` and are installed with the
  package's `launch` directory.
- Packages shipping these launch files declare runtime dependencies on
  `launch` and `launch_ros`.
- Python sensor publishers use `QoSProfile(depth=10)` with
  `ReliabilityPolicy.BEST_EFFORT` to match C++ `rclcpp::SensorDataQoS()`.
- A launch test must report at least one executed test method; a process exit
  code of zero with `NO TESTS RAN` is a test failure, not a pass.
- Pose input mode is explicit: `pose_input_type=odometry` consumes
  `nav_msgs/Odometry`; `pose_input_type=tf` consumes `tf2_msgs/TFMessage` and
  selects `tf_child_frame` before conversion.
- A required pose is usable only when freshness, expected frame, and minimum
  quality checks all pass. Invalid quality includes NaN and infinity.
- Producer SessionID belongs to one process lifetime. A respawned process must
  publish a new SessionID; rejecting the old SessionID belongs to the consumer.

### 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| Launch module cannot be imported | Python syntax/import failure |
| Test class is not discoverable by `unittest` | Fix the test; do not accept an empty test run |
| Publisher QoS is reliable while input is best effort | Use explicit best-effort QoS |
| Launch process starts but a node exits immediately | Fail the smoke test and inspect node parameters |
| `pose_input_type` is unknown or TF child frame is empty | Node startup fails with an actionable parameter error |
| Pose is stale, has the wrong frame, or quality is below threshold | Health becomes unavailable immediately and reports the failed boundary |
| Producer process is respawned | New observations carry a different SessionID |

### 5. Good / Base / Bad Cases

- Good: `class TestInput(unittest.TestCase)` with `setUpClass`, active assertions,
  and an output containing `Ran N tests` where `N > 0`.
- Base: static launch composition with `Node` actions and no business logic in
  the launch module.
- Bad: pytest-only `setup_class` methods passed to
  `launch_testing.launch_test`, which can produce a successful process with no
  executed tests.
- Bad: session restart is approximated by two concurrent nodes, or a 100-frame
  performance test counts repeated timestamps as distinct frames.

### 6. Tests Required

- Parse every `.launch.py` and launch-test module with Python `ast` or the ROS
  launch loader.
- Run each public launch entry in an isolated `ROS_DOMAIN_ID` and assert that
  expected nodes start without frame/parameter rejection logs.
- Run package tests and inspect the launch-test output for executed test counts.
- Exercise Odometry and TF pose inputs, including stale, wrong-frame,
  low-quality, and reset-epoch paths.
- Inject a real process failure with launch respawn and assert the SessionID
  changes after restart.
- Performance baselines use more than 100 unique frames, report loss plus
  average/P95/max latency, and separately report conversion and health-gate
  timings.

### 7. Wrong vs Correct

Wrong:

```python
class TestInput:
    def setup_class(cls):
        ...
```

Correct:

```python
class TestInput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ...
```

## External Resource Profiling Contracts

### 1. Scope / Trigger

These rules apply when a ROS 2 node is measured with external CPU, tracing,
heap, sanitizer, or process-memory tools. They prevent an empty workload, a
different process, or a truncated profiler run from being reported as a valid
baseline.

### 2. Signatures

- Unified runner: `scripts/profile-perception.sh <mode> <install-prefix>
  <new-output-dir> <duration-seconds>`.
- Supported mode names are explicit and closed; an unknown mode or an existing
  output directory is an error.
- Repeated memory analysis consumes exactly three normally completed,
  independently captured run directories:
  `scripts/analyze-perception-profile.py <run-a> <run-b> <run-c>`.

### 3. Contracts

- Record the exact installed target ELF, SHA-256, build ID, source revision,
  image, tool versions, RMW, affinity, cgroup, and role-specific PIDs in every
  run manifest. Require the host image identity in `sha256:<64 hex>` form and
  hash the runner, workload config, and analysis script themselves.
- Treat launcher, profiler/tool, tracee, fixture, preflight sink, measurement
  sink, and samplers as distinct roles. Validate each real PID from `/proc`,
  and never infer the tracee from an unrelated wrapper PID. Poll until wrapper
  `exec()` identity and the isolated PGID have stabilized before recording a
  role.
- Give created process trees isolated process groups. Before group signaling,
  reject the runner's own PGID. During the formal window, check every required
  PID, `/proc/<pid>/stat` starttime, and zombie state at least once per second.
- Prove workload before measurement with a preflight sink. Start a fresh
  measurement sink, wait until every sensor ID has arrived, and count only CSV
  rows between recorded `t0` and `t1` offsets. Require the declared exact
  publisher/subscriber cardinality so a domain collision cannot add foreign
  endpoints unnoticed.
- A perf attach starts disabled and becomes valid only after enable and disable
  control FIFO acknowledgements. Freeze and re-check the target TID set, keep
  profiler and workload windows nested, and require non-empty workspace symbols
  plus an explicit unknown/lost-sample gate.
- Heaptrack, Valgrind, sanitizers, and ROS tracing must use their tool-specific
  PID and normal-stop semantics. A forced stop, missing report section, missing
  trace category, or ambiguous target makes the run invalid; a complete
  Memcheck report with a configured finding exit code remains valid evidence of
  a finding.
- `normal_completion=true` requires reaching the full stop/finalization state
  with no forced signal escalation, PID/starttime/PGID mismatch, premature role
  exit, or unexpected role exit code. Keep this independent from `valid`: a
  normally finalized but incomplete report can be invalid without pretending
  that the process state machine failed.
- Require exactly one Heaptrack, Massif, or Memcheck primary artifact and prove
  its target/runtime/summary contract before parsing metrics. Repeated memory
  analysis requires three distinct 300-second `plain-sample` directories whose
  `(tracee_pid, t0_monotonic_ns, t1_monotonic_ns)` evidence identities are
  pairwise distinct, and recomputes workload and expected pidstat/smem sample
  completeness from raw data rather than trusting copied summary flags.
- Write role exit codes, `valid`, `normal_completion`, actual duration,
  per-source workload counts, invalid reasons, and SHA-256 for every raw
  artifact. A report may cite only runs where both validity flags are true.

### 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| Target ELF, PID role, or runtime parameter is ambiguous | Stop and mark the run invalid |
| Package prefix resolves outside the requested profiling install | Stop before starting workload |
| Host image ID is missing, lacks the `sha256:` prefix, or has the wrong length | Stop before starting workload |
| Fixture, target, sink, profiler, or sampler exits during `t0-t1` | Mark invalid even if a partial artifact exists |
| PID starttime changes or a required process becomes a zombie | Mark invalid as PID reuse/premature exit |
| Measurement sink never observes every required sensor ID | Do not start the profiling window |
| Topic endpoint cardinality differs from the fixed workload | Reject the run as a domain collision or foreign endpoint |
| Same-window total or per-sensor count is below its declared threshold | Mark invalid |
| perf enable/disable acknowledgement, samples, symbols, or lost-sample evidence is missing | Mark the perf run invalid |
| Trace cannot filter callback/take/publish by the target vpid | Mark the trace run invalid |
| Heap/Memcheck/Massif summary is incomplete or the tool was force-killed | Mark invalid and rerun |
| Memcheck exits with the dedicated finding code and has a complete matching summary | Keep a valid run and record the finding |
| The same plain-sample evidence is copied under different run directories | Reject it as duplicate evidence |
| Fewer than three valid steady-state memory runs are available | Do not issue a sustained-growth conclusion |

### 5. Good / Base / Bad Cases

- Good: perf is attached to a verified target after preflight, both control ACKs
  surround the counted workload, every role survives the window, and the final
  manifest plus hashes are complete.
- Base: unsupported hardware PMU events are reported as unavailable while an
  independently valid software-event run remains usable.
- Bad: using wall-clock sleep plus a topic publisher count without proving the
  target PID, profiler state, or same-window output.
- Bad: accepting a report written after SIGKILL, treating Memcheck's finding
  exit code as tool failure, or inferring a leak from one RSS curve.

### 6. Tests Required

- Run `bash -n` and a short valid smoke for every runner mode.
- Fault-inject target, fixture, sink, profiler, and sampler termination during
  the formal window; assert `valid=false`, non-normal role exit evidence, and a
  specific invalid reason.
- Delay wrapper `exec()`/`setsid()` in a startup probe; assert the runner waits
  for the target identity and isolated PGID instead of recording the wrapper or
  its parent process group.
- Pass a non-sanitized ELF to sanitizer modes and assert rejection before the
  formal window.
- For perf, assert both ACKs, frozen TIDs, a non-zero workspace sample count,
  bounded unknown ratio, explicit lost-sample count, and same-window workload.
- For tracing and memory tools, assert required event/report sections and
  normal tool finalization before accepting the run.
- Recompute three steady windows from raw pidstat/smem/smaps data and assert the
  documented means, percentiles, slopes, sample counts, and aggregate growth
  decision.
- Run synthetic positive and negative parser fixtures for perf control/lost
  samples, Heaptrack quantities, Massif stack-aware peak detail, Memcheck target
  identity, duplicate plain paths or evidence identities, low workload, and
  incomplete sampler output.

### 7. Wrong vs Correct

Wrong:

```bash
sleep "${DURATION}"
kill -KILL "${profiler_pid}"
echo "valid=true"
```

Correct:

```bash
verify_roles_and_workload
perf_control enable   # require ACK before t0
monitor_formal_window # PID, starttime, zombie, workload roles
perf_control disable  # require ACK after t1
finalize_tool_normally
validate_reports_and_hashes
```

## Perception Fixture LiDAR Geometry Contracts

### 1. Scope / Trigger

These rules apply when changing `perception_fixtures` scene geometry, ROS
parameters, public launch files, or observation round-trip tests.

### 2. Signatures

- 2D default profile: 181 samples from `-pi/2` through `+pi/2` in the
  sensor-frame XY plane; `scan_point_count >= 2` remains configurable and the
  publisher computes the increment from the actual count.
- `FixtureSceneConfig` owns the 2D count, angle bounds, and range bounds. The
  publisher copies all five values, plus the derived increment, from the
  validated config instead of maintaining message-side literals.
- 3D cloud: `cloud_azimuth_sample_count`, `cloud_range_m`, and
  `elevation_angles_rad`.
- Default 3D profile: 16 elevations from `-15` to `+15` degrees in 2-degree
  increments, with 360 azimuth samples per channel.
- Visualization launch: `fixture_visualization.launch.py` publishes
  `/fixture/scan/flat`, `/fixture/scan/tilted`, and `/fixture/points`; it
  accepts `show_rviz` and `tilted_scan_pitch_rad`.

### 3. Contracts

- `LaserScan` geometry always follows the ROS sensor-frame XY convention;
  installation tilt belongs in TF.
- 3D azimuth zero points along +X, positive azimuth rotates toward +Y, and
  positive elevation points toward +Z.
- A 3D frame contains complete azimuth rings for each elevation channel, not
  a spiral. Overall 3D sensor installation rotation belongs in TF; the
  visualization's zero-rotation transform is only the standard baseline.
- Point order is elevation-channel first, then increasing azimuth within each
  channel.
- The algorithm library owns the default elevation table; publishers and
  launch files must not duplicate it.
- The generic fixture is deterministic test input, not a replay of the legacy
  `drone_scanner::FakeLidar` vertical-ring geometry.
- The default 181-beam range profile retains the legacy float-arithmetic
  values and is locked by representative exact golden samples, not only by
  comparing two generated frames.
- Scan construction rejects counts below two, non-finite angle/range bounds,
  `angle_min >= angle_max`, `range_min < 0`, and `range_min >= range_max`.
- Debug-return injection is a fixed 360-beam `[-pi, pi)` profile with
  `[0.1, 10]` m range. It first generates the local-X/Y ellipse with half axes
  3 m and 4 m, then writes `+inf` at indices `[255,285]`, then writes NaN,
  `-inf`, below-min, and above-max at `44/136/180/316`.
- The debug launch derives fixture and frozen descriptor parameters from one
  Python constant group. Its `map -> debug_scan_link` `Ry(+pi/2)` transform
  maps local +Z to map +X and the scan plane to map YZ; the default runtime
  fixture is 2D-only.
- Visualization uses one RViz config with `base_link` as Fixed Frame. Flat 2D,
  tilted 2D, multi-line 3D, and TF displays remain independently selectable;
  mixed views are display combinations, not separate RViz configs.
- Both 2D publishers emit the same sensor-frame XY `LaserScan` geometry.
  `base_link -> fixture_scan_tilted_link` owns the installation pitch.
- The visualization default pitch is `-0.5235987756` rad (`-30` degrees), so
  the tilted sensor's local +X axis points toward `base_link` +Z.
- RViz sensor displays use Best Effort and Volatile QoS to match
  `rclcpp::SensorDataQoS()`. Check the settled DDS endpoint QoS: RViz can
  briefly create a default Reliable subscription while loading a config.

### 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| Empty elevation array | Reject with `std::invalid_argument` |
| Non-finite elevation | Reject with `std::invalid_argument` |
| Elevation outside `[-pi/2,+pi/2]` | Reject with `std::invalid_argument` |
| Azimuth sample count below 1 | Normalize to 1 |
| Cloud range below 0.1 m | Normalize to 0.1 m |
| Explicit ROS elevation array | Replace, not append to, the default profile |
| Tilted 2D installation | Keep raw scan in XY; apply pitch only in static TF |
| Tilted 3D installation | Keep raw elevation rings in the sensor frame; apply the complete installation rotation in TF |
| Settled RViz subscription is Reliable | Fix the display QoS; Best Effort publishers cannot send to it |
| GUI is unavailable during smoke | Launch with `show_rviz:=false`; do not skip publisher and TF checks |
| Debug injection uses any other count, angle, or range layout | Reject construction instead of shifting fixed branch/invalid indices |

### 5. Good / Base / Bad Cases

- Good: a three-channel test profile verifies negative, zero, and positive Z
  through the full ROS adapter round trip.
- Good: one visualization launch composes three publishers and three static
  transforms, while one RViz config exposes each sensor as a checkbox.
- Base: public launch files omit elevation parameters and consume the library
  default profile.
- Bad: encoding sensor installation pitch in `LaserScan`/`PointCloud2` or
  reintroducing a single YZ ring as the generic 3D fixture.
- Bad: adding a second mixed RViz config or changing scan coordinates to make
  a tilted display without TF.

### 6. Tests Required

- Gtest the complete default elevation table, total point count, deterministic
  coordinates, range formula, intensity, azimuth order, signed elevation, and
  invalid profiles.
- Launch-test a compact explicit profile and assert XYZ, intensity, and order
  after `PointCloud2 -> LidarObservation` conversion.
- Assert 2D angle metadata and the middle zero-angle sample after the ROS
  round trip.
- Smoke-test every public fixture launch and reject unknown-parameter or node
  startup errors.
- Smoke-test the visualization launch with `show_rviz:=false` and assert three
  publisher nodes, three sensor topics, and three static transforms exist.
- Assert the default tilted transform maps sensor-local +X to a positive
  `base_link` Z component; checking only that the transform exists is not
  sufficient to lock the installation direction.
- Assert the legacy 181-beam exact golden values, all 360 debug directions,
  the ellipse equation, the exact branch/invalid index sets, disjoint sets,
  and non-repeated first/last directions.
- Launch-test the debug scene at `beam_stride=1`; assert Marker point-count
  relations and subscribe to `/tf_static` to verify all three transformed
  basis vectors. Inject Cloud3D hit-only directly on the authoritative
  observation topic instead of adding it to the default debug fixture.
- Start real RViz once, assert settled subscriptions are Best Effort/Volatile,
  and visually check nonblank flat 2D, tilted 2D, multi-line 3D, and TF output.

### 7. Wrong vs Correct

Wrong:

```cpp
config.elevation_angles_rad.reserve(values.size());
for(double value : values) {
    config.elevation_angles_rad.push_back(static_cast<float>(value));
}
```

Correct:

```cpp
config.elevation_angles_rad.clear();
config.elevation_angles_rad.reserve(values.size());
for(double value : values) {
    config.elevation_angles_rad.push_back(static_cast<float>(value));
}
```

Wrong:

```python
# Tilting the generated LaserScan changes its message-frame contract.
parameters=[{"mode": "2d", "scan_frame": "base_link"}]
```

Correct:

```python
# The message remains in its sensor frame; TF expresses installation pitch.
parameters=[{"mode": "2d", "scan_frame": "fixture_scan_tilted_link"}]
# static_transform_publisher: base_link -> fixture_scan_tilted_link, pitch=-0.5235987756
```

---

## Forbidden Patterns

<!-- Patterns that should never be used and why -->

(To be filled by the team)

---

## Required Patterns

<!-- Patterns that must always be used -->

(To be filled by the team)

---

## Testing Requirements

<!-- What level of testing is expected -->

(To be filled by the team)

---

## Code Review Checklist

<!-- What reviewers should check -->

(To be filled by the team)

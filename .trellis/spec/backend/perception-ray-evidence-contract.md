# Perception Ray Evidence Contract

## 1. Scope / Trigger

Apply this contract whenever LiDAR descriptors, observations, mapper input
requirements, ROS messages, sensor YAML, or adapters change how occupied hits
or free-space rays may be consumed. C1 declares and propagates evidence; only a
downstream mapper such as C2 may turn that evidence into occupancy updates.

## 2. Signatures

```cpp
enum class RayEvidenceCapability : std::uint8_t {
    HitOnly = 0,
    HitRay = 1,
    FullRay = 2,
};

constexpr bool provides_at_least(
    RayEvidenceCapability actual,
    RayEvidenceCapability required) noexcept;

constexpr bool is_valid_ray_evidence(
    RayEvidenceCapability capability) noexcept;

enum class RayReturnKind : std::uint8_t { Hit, NoReturn, Invalid };

RayReturnKind Scan2D::return_kind(std::size_t index) const;

// ROS boundary: explicit decode or throw; never cast the wire value.
LidarObservation to_core_observation(
    const perception_interfaces::msg::LidarObservation& message);

// Accepted batches always replace all four marker categories.
visualization_msgs::msg::MarkerArray make_markers(
    const perception_interfaces::msg::LidarObservation& message,
    const RayEvidenceDebugGeometry& geometry);
```

`SensorDescriptor::ray_evidence` is frozen with the sensor inventory.
`LidarObservation::ray_evidence` carries the same capability for each batch.
`SensorRequirement::minimum_ray_evidence` declares the mapper boundary.

## 3. Contracts

- The capability order is closed and monotonic:
  `HitOnly < HitRay < FullRay`.
- `is_valid_ray_evidence()` is the single closed-set check. Capability
  comparisons validate both operands first; an unknown underlying value never
  satisfies any requirement, including `HitOnly`.
- `HitOnly` permits occupied hit endpoints only.
- `HitRay` additionally permits origin-to-hit free-space rays.
- `FullRay` additionally permits explicit no-return rays through the declared
  maximum reliable range.
- `Scan2D` keeps the native `ranges` payload. Classification is queried on
  demand: an in-range finite value is `Hit`, positive infinity is `NoReturn`,
  and NaN, negative infinity, or an out-of-range finite value is `Invalid`.
- Payload classification and consumption permission are separate. A positive
  infinity in a `HitOnly` or `HitRay` scan is not permission to write a
  maximum-range free-space ray.
- `sensor.<id>.ray_evidence`, `minimum_lidar_ray_evidence`, and
  `degraded_lidar_ray_evidence` accept only `hit_only`, `hit_ray`, or
  `full_ray`. All default to `hit_only`.
- ROSIDL values remain numerically aligned with the C++ enum: 0, 1, and 2.
  `HealthState.has_free_space_hit_rays` means at least `HitRay` is active;
  `has_full_no_return_rays` means at least `FullRay` is active.
- The current `PointCloud2` schema contains XYZ endpoints and optional
  intensity only. It must reject descriptors above `HitOnly` in both
  `validate()` and direct `convert()` calls. Direct conversion also runs the
  complete type, frame, FLOAT32 field-layout, row/point-step, and data-length
  validation before reading bytes; malformed storage never reaches point
  extraction.
- A `HitRay` or `FullRay` `LaserScan` is accepted only when range limits,
  horizontal FOV, and angular resolution match the frozen descriptor within
  adapter tolerance. Basic non-finite or inconsistent scan metadata is
  rejected for every capability.
- `LaserScanAdapter::convert()` runs the same complete validation internally;
  direct library calls cannot bypass type, frame, metadata, ranges, or optional
  intensity-array checks.
- ROS callbacks call an internally validating adapter `convert()` exactly once.
  They catch `std::invalid_argument` at the callback boundary, reject the batch,
  and use throttled diagnostics. Calling `validate()` and then `convert()` is a
  redundant double validation and still requires an exception boundary.
- Inventory freeze is the producer-side pre-publication boundary. A
  successfully published observation is the authoritative batch and carries
  its own capability, frame, timestamp, session, and payload metadata. A
  consumer traces sensor/session identity but does not require a second hidden
  descriptor topic to reinterpret that batch.
- `RayEvidenceDebugNode` consumes only the authoritative ROS
  `LidarObservation`. It decodes `ray_evidence` and `data_type` with closed
  switches. Unknown values, empty `frame_id` / `sensor_id` / `clock_domain`, a
  zero `session_boot_time_ns`, invalid 2D metadata, mismatched optional-array
  lengths, or payload from the other data type reject the complete batch.
  Cloud3D must be `HitOnly`, must not retain any Scan2D arrays or scalar
  metadata, and contributes only finite XYZ endpoints; it never creates a free
  ray.
- Debug markers use the stable namespaces `ray_evidence/hit_endpoints`,
  `ray_evidence/hit_free`, `ray_evidence/no_return_free`, and
  `ray_evidence/invalid`. The marker ID is stable for a sensor. Every accepted
  batch publishes all four `ADD` markers, including empty point lists, so a
  capability downgrade replaces categories emitted by the previous batch.
  Every marker has a positive finite lifetime so output disappears after the
  producer stops. A rejected batch produces no new geometry.

## 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| Unknown ray-evidence parameter value | Node startup fails with the accepted closed set in the diagnostic |
| Unknown C++ capability operand such as `3` or `255` | `is_valid_ray_evidence()` and `provides_at_least()` return false |
| Active descriptor capability is below a mapper requirement | That sensor does not satisfy the requirement |
| `PointCloud2` declares `HitRay` or `FullRay` | `validate()` rejects it and direct `convert()` throws `std::invalid_argument` |
| Direct `PointCloud2Adapter::convert()` receives malformed fields, steps, or data length | Throw `std::invalid_argument` before point extraction |
| `LaserScan` range metadata is non-finite or `range_min >= range_max` | Reject the complete batch |
| `LaserScan` angle bounds are non-finite or increment is non-positive | Reject the complete batch |
| Direct `LaserScanAdapter::convert()` input fails validation | Throw `std::invalid_argument` without returning an observation |
| A ROS sensor callback receives adapter-invalid input | Catch `std::invalid_argument`, reject once, and emit a throttled diagnostic; do not pre-run the same validation |
| `HitRay`/`FullRay` scan metadata drifts from the frozen descriptor | Reject the complete batch and do not activate the sensor |
| Descriptor capability changes after inventory freeze | Reconnect is rejected as descriptor drift |
| `Scan2D::return_kind()` index is outside `ranges` | Throw `std::out_of_range` |
| Debug observation has ray-evidence value `3` / `255` or an unknown `data_type` | Reject the complete batch; publish no new geometry |
| Debug observation has empty provenance, zero boot session, invalid 2D metadata, or cross-type arrays/scalars | Reject the complete batch; publish no new geometry |
| Debug Cloud3D declares `HitRay` / `FullRay` or contains a non-finite endpoint | Reject the complete batch; never draw origin-to-point free rays |
| An accepted batch no longer contains a previously rendered evidence category | Publish the same namespace and sensor ID with `ADD` and an empty point list |
| No further accepted batch arrives | Finite marker lifetime removes the last rendered geometry |

## 5. Good / Base / Bad Cases

- Good: a 2D fixture declares `full_ray`, publishes matching frozen scan
  metadata, and health reports both free-space capability flags as true.
- Good: a debug sensor changes from `FullRay` to `HitOnly`; all four markers
  retain their namespace/ID, while the two free-ray markers are replaced by
  empty `ADD` messages with the same finite lifetime.
- Base: a 3D XYZ cloud declares `hit_only`; the debug consumer accepts only its
  finite endpoints and leaves both free-ray markers empty.
- Bad: a mapper infers `FullRay` from `LaserScan` message type or positive
  infinity without checking the descriptor and batch capability.
- Bad: a cloud descriptor claims `HitRay` because the producer internally knew
  the sensor origin but did not encode ray direction/range in the public
  payload.
- Bad: a debug consumer casts wire integers to the capability enum, ignores
  Cloud3D scan scalar residue, or publishes only non-empty marker categories;
  each can grant unknown authority or leave stale geometry visible.
- Bad: a ROS callback runs `validate()`, calls the self-validating `convert()`
  again, and omits the `std::invalid_argument` catch.

## 6. Tests Required

- Core tests assert enum ordering, conservative defaults, descriptor equality,
  unknown values `3/255`, all `Scan2D` classification boundaries, and mapper
  minimum/degraded gates.
- Adapter tests call both `validate()` and `convert()` for non-`HitOnly` and
  malformed-layout `PointCloud2`, reject range/FOV/resolution drift on both sides of each frozen
  `LaserScan` boundary, and call `LaserScanAdapter::convert()` directly for
  invalid type, frame, metadata, and arrays.
- Session tests assert that changing only `ray_evidence` after freeze is still
  descriptor drift.
- Launch tests cover invalid minimum, degraded, and per-sensor parameter values;
  `HitOnly` failing `HitRay`; `HitRay` satisfying degraded `HitRay` but failing
  minimum `FullRay`; and ROS observation/health field mappings.
- Debug geometry tests cover all three capabilities, every 2D return kind,
  deterministic `beam_stride`, unknown capabilities, and Cloud3D HitOnly with
  finite endpoints and no free segments.
- Debug launch tests inject wire capabilities `3` and `255`, unknown data type,
  both directions of cross-type payload, Cloud3D scan scalar residue, empty
  clock domain, zero boot session, invalid 2D metadata, high-capability cloud,
  and non-finite cloud data. Assert that none produces a marker geometry batch.
- Marker launch tests assert the four exact namespaces, stable sensor IDs,
  frame/stamp/color/type, positive finite lifetime, and a `FullRay` to
  `HitOnly` update where empty `ADD` markers replace both old free-ray
  categories. Graph assertions reject legacy `scan_returns`, FakeLidar,
  OctoMapBuilder, occupancy, and OctoMap endpoints.
- The final gate builds and tests `perception_core`, `perception_interfaces`,
  `perception_adapters`, `perception_input_node`, and `perception_fixtures` in
  an isolated Release prefix, then requires zero `colcon test-result` failures
  and a clean `git diff --check`.

## 7. Wrong vs Correct

Wrong:

```cpp
// The payload type or value is not sufficient evidence.
if (std::isinf(scan.ranges[i])) {
    write_free_space_to(scan.range_max_m);
}
```

Correct:

```cpp
if (observation.ray_evidence == RayEvidenceCapability::FullRay
    && scan.return_kind(i) == RayReturnKind::NoReturn) {
    write_free_space_to(scan.range_max_m);
}
```

The consumer treats a successfully published observation as the authoritative
batch and keeps sensor/session provenance. This C1 contract does not itself
perform the occupancy write.

Wrong:

```cpp
if (adapter.validate(message, descriptor).valid) {
    publish(adapter.convert(message, descriptor, session)); // validates twice
}
// Only non-empty categories are published, so an old free ray can remain.
if (!geometry.hit_free_segments.empty()) {
    markers.markers.push_back(make_hit_free_marker(geometry));
}
```

Correct:

```cpp
try {
    publish(adapter.convert(message, descriptor, session));
} catch (const std::invalid_argument& error) {
    warn_throttled(error.what());
    return;
}

// Use the same namespace/id even when points is empty; lifetime is finite.
markers.markers.push_back(make_hit_free_marker(geometry.hit_free_segments));
```

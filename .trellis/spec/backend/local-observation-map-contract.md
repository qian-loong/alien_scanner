# Local Observation Map Contract

## 1. Scope / Trigger

Apply this contract whenever C2 local-map inputs, health recovery, pose/session
handling, occupancy backends, map state, or revision-locked query APIs change.
C2 is the only authoritative vehicle-local occupancy writer. Visualization-only
OctoMap output is not a query or synchronization contract.

## 2. Signatures

```cpp
bool MapperHealthGate::recovery_stable() const noexcept;

InputResult LocalObservationMapper::submit_pose(
    const Perception::PoseEstimate& pose,
    std::int64_t receive_monotonic_ns);

InputResult LocalObservationMapper::submit_observation(
    const Perception::LidarObservation& observation,
    const SensorExtrinsicSample& extrinsic,
    std::int64_t receive_monotonic_ns);

AcquireResult LocalObservationMapper::acquire_read_transaction();
AcquireResult LocalObservationMapper::acquire_read_transaction(
    const CommitReceipt& receipt);
```

The ROS boundary consumes C1 `LidarObservation`, `PoseEstimate`, and
`HealthState`, and publishes authoritative `LocalMapState` plus a
visualization-only `octomap_msgs/Octomap` snapshot.

## 3. Contracts

- `map_epoch` identifies one spatial continuity chain. `revision` starts at
  zero in a new epoch and advances only after an atomic backend mutation with
  accepted occupancy evidence.
- A required pose is matched by acquisition stamp in the same clock domain.
  Wrong frame, invalid quality, retired lineage, duplicate/rollback stamp, or
  stale receive age makes pose authorization unavailable immediately.
- A pose fault clears the pose history that could authorize later observations
  but preserves active/retired lineage fences and stamp high-water marks. A
  low-quality pose is unusable data, not evidence of a spatial jump, and must
  not itself advance the map epoch.
- When pose is required, the configured expected pose frame equals the map
  geometry frame. Reject an inconsistent configuration at construction.
- `HealthState::Degraded` is insufficient by itself to authorize a write.
  After `Unavailable`, `recovery_stable()` must satisfy the configured
  consecutive-sample gate before any new revision may commit. A stable,
  intentionally degraded capability combination may still commit.
- A contract-bound new pose or sensor lineage retires the previous lineage.
  Replays from retired lineages are rejected without repeatedly resetting the
  map. Validation-only relays follow the same active/retired rule.
- Descriptor identity, contract fingerprint, frame, capability, extrinsic,
  lattice representability, and whole-batch evidence validation all precede
  backend mutation. Rejection is atomic and never advances revision or
  freshness.
- A `MapReadTransaction` pins one exact `(mapper_session, map_epoch, revision)`
  view. Writers and epoch-reset faults wait until that transaction closes;
  old receipts become `Superseded` after the epoch changes.
- Occupancy evidence follows
  [Perception Ray Evidence Contract](./perception-ray-evidence-contract.md).
  C2 never consumes legacy `scan_returns`, raw `LaserScan`, or cave truth.

## 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| Required pose frame differs from map frame | Construction fails with an actionable configuration error |
| Pose frame/quality is invalid, or stamp is duplicate/rollback | Clear usable pose history, enter `Unavailable`, preserve lineage fences, and reject |
| Observation arrives before recovery is stable | Return `InputStatus::Unavailable`; do not mutate, advance revision, or refresh provenance |
| Stable degraded capability combination satisfies the contract | Observation may commit; publish the exact effective capability combination |
| New contract-bound lineage arrives | Retire the prior lineage and activate the new one |
| Retired lineage replays | Reject without another epoch reset |
| Backend mutation throws or reports an invalid result | Enter backend-fault handling atomically; advance epoch with revision reset and expose no partial write |
| Exact-revision read is open while a writer/fault arrives | Writer waits; reader retains the pinned view until close |
| Receipt belongs to an old epoch | Return `AcquireStatus::Superseded` |
| Observation has no usable evidence | Return `NoEvidence`; do not advance revision or map freshness |

## 5. Good / Base / Bad Cases

- Good: three required healthy recovery samples arrive after a pose fault; only
  the first observation after the third sample advances revision.
- Good: a stable degraded `HitRay` combination that satisfies the configured
  degraded contract commits and advertises the degraded combination ID.
- Base: heartbeats continue after the last scan while revision and provenance
  remain fixed and map freshness expires.
- Bad: treating a recovery bridge reported as `Degraded` as permission to
  write before the consecutive-sample gate is stable.
- Bad: rejecting a rollback pose but retaining the previous pose history, which
  lets the next observation reuse stale authorization.
- Bad: decoding the visualization OctoMap snapshot to answer an authoritative
  revision-locked query.

## 6. Tests Required

- Mapper-health tests cover every state transition and explicitly assert that
  recovery is unstable until the configured consecutive sample count is met.
- Pose tests cover wrong frame, non-finite/low quality, stale receive age,
  duplicate/rollback stamps, discontinuity, new lineage, retired replay, and
  the distinction between unusable pose and a confirmed spatial reset.
- Observation tests assert no backend mutation, revision, provenance, or
  freshness change for every rejected/unavailable/no-evidence path.
- Backend conformance tests run identical atomic apply/query/bounds cases for
  each registered backend and inject mutation faults.
- Transaction tests hold an exact-revision reader while an asynchronous writer
  or reset fault waits, then assert old-view stability, epoch advance,
  `Superseded` receipt, and unknown state in the new empty epoch.
- Launch tests assert one authoritative state/map publisher, C1/C2 fingerprint
  agreement, monotonic revisions, end-of-trajectory drain behavior, and the
  absence of `scan_returns`, legacy builders, and a second mapper.
- Final validation uses an isolated Release prefix and sequential executor,
  then requires zero `colcon test-result` errors/failures/skips.

## 7. Wrong vs Correct

Wrong:

```cpp
if (health_gate.current_state() != HealthState::Unavailable) {
    backend.apply(batch);  // Recovery may still be pending.
}

if (invalid_pose) {
    return InputStatus::Rejected;  // Old pose history still authorizes scans.
}
```

Correct:

```cpp
if (health_gate.current_state() == HealthState::Unavailable
    || !health_gate.recovery_stable()) {
    return InputStatus::Unavailable;
}

if (invalid_pose) {
    pose_history.clear();
    current_health = HealthState::Unavailable;
    return InputStatus::Rejected;  // Keep lineage/high-water replay fences.
}
```

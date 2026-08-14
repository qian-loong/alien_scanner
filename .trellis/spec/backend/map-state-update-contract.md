# Map State Update Contract

## 1. Scope / Trigger

Apply this contract whenever source-local occupancy snapshots, keyframes,
deltas, revision admission, resynchronization, the C2-to-C3 asynchronous
producer, or the replay visualization publisher changes. C3 replicates the
final occupancy state above C2; it does not replace C2 as the authoritative
local-map writer or implement C4 routing, backpressure, or link retry.

## 2. Signatures

```cpp
PreparedUpdate MapUpdateProducer::prepare(
    const CanonicalSnapshot & target,
    std::uint64_t observed_coalesced_receipt_count = 0U) const;
PreparedUpdate MapUpdateProducer::prepare(
    std::shared_ptr<const CanonicalSnapshot> target,
    std::uint64_t observed_coalesced_receipt_count = 0U) const;
bool MapUpdateProducer::commit_published(const PreparedUpdate & prepared);
const std::shared_ptr<const CanonicalSnapshot> &
MapUpdateProducer::baseline() const noexcept;

bool MapUpdateApplier::admit_source(const SourceIdentity & source);
ApplyUpdateResult MapUpdateApplier::apply(const MapUpdate & update);

struct CellStorageConfig {
    CellStorageMode mode = CellStorageMode::Vector;
    std::uint32_t chunk_edge = 16U;
    std::size_t bucket_count = 256U;
};
MapUpdateApplier::MapUpdateApplier(
    MapUpdateLimits limits = {}, CellStorageConfig storage = {});
CanonicalCellCursor CanonicalCellView::cursor() const;
std::vector<CanonicalCell> CanonicalCellView::materialize() const;
CellStoreResult CellSnapshotStore::estimate_apply_upper_bound(
    const std::vector<DeltaOperation> & operations) const noexcept;
CellStoreResult CellSnapshotStore::apply(
    const std::vector<DeltaOperation> & operations);

ResyncResponse ResyncRequestLedger::accept(
    const ResyncRequest & request,
    const SourceIdentity & current_source,
    std::uint64_t current_revision);

bool AsyncMapUpdateProducer::enqueue(const CommitReceipt & receipt);
bool AsyncMapUpdateProducer::request_keyframe(std::string correlation_id);

MapUpdateReplayRun MapUpdateReplayOracle::run(
    const MapUpdateReplayOptions & options = {}) const;
ReplayComparison MapUpdateReplayOracle::compare(
    const CanonicalSnapshot & oracle,
    const ReconstructedMap & reconstructed,
    std::size_t max_difference_samples = 4096U);

double visualization_publish_rate_hz = 1.0;
```

The ROS boundary is `perception_interfaces/msg/MapUpdate.msg` plus
`perception_interfaces/srv/RequestMapResync.srv`. The service response carries
only acceptance, correlation, current-source identity/revision, and a bounded
diagnostic; the keyframe remains an asynchronous `MapUpdate` publication.

## 3. Contracts

- Canonical state is a strictly sorted sparse vector of known `Free` or
  `Occupied` cells. Missing cells are `Unknown`; a delta uses
  `RemoveToUnknown` to delete a known cell.
- `ReconstructedMap::cells` is a storage-independent `CanonicalCellView`, not
  an owned vector contract. Production consumers stream through `cursor()` or
  `for_each()`; `materialize()` is reserved for explicit compatibility, test,
  or serialization boundaries and must not be cached as a second authoritative
  map.
- `Vector` and `Chunked` storage modes implement the same apply state machine.
  `Vector` remains the production default because the C4.1 Gate B was no-go.
  Edge 16 with 256 deterministic buckets is the retained research candidate,
  not a claim that chunked storage is currently faster or production-enabled.
- A chunked committed snapshot, its buckets, and chunks are immutable. Delta
  apply shallow-copies the fixed directory and clones only touched buckets and
  chunks; unchanged chunks retain shared object identity. Revision-only delta
  shares every chunk. Candidate construction must not allocate a complete
  canonical cell vector.
- Chunk coordinates use mathematical floor division for negative voxel indices.
  A chunked canonical cursor performs a deterministic global `(x,y,z)` merge;
  bucket order, chunk-coordinate order, and local chunk order are not protocol
  v1 canonical order by themselves.
- Protocol v1 `content_hash` remains flat SHA-256 over the complete canonical
  stream for both storage modes. Chunked COW reduces candidate copying but does
  not remove the `O(N)` receiver hash traversal; Merkle identity is a separate
  wire/hash-version task.
- Protocol v1 orders a chain by `(vehicle_id, mapper_session, map_epoch,
  revision)`. ROS `header.stamp`, correlation IDs, and transport arrival time
  never participate in ordering or hashes.
- Geometry, content, and update hashes are distinct SHA-256 identities.
  `content_hash` describes reconstructed occupancy and therefore excludes the
  revision. A revision-only delta has an empty operation list, advances the
  revision, and preserves the content hash.
- The asynchronous hot path moves the adapter result into one immutable shared
  snapshot. `PreparedUpdate::target_snapshot` and the committed producer
  baseline retain that same object; prepare and commit must not deep-copy its
  cell vector. The const-reference overload is a convenience boundary for
  existing callers and creates an owned copy before delegating.
- SHA-256 input bytes remain the versioned canonical stream even when the EVP
  implementation batches tiny field writes. Buffering is an implementation
  detail and must be guarded by a fixed golden hash whose input crosses the
  buffer boundary.
- A delta may span coalesced C2 revisions, but its `base_revision` must exactly
  equal the receiver revision and its `base_content_hash` must equal the
  receiver content hash. `revision_span` reports the span; it is not an event
  log for skipped intermediate revisions.
- Producer state advances only after the publication callback succeeds.
  `PreparedUpdate.expected_baseline` is a fixed-size token containing source,
  revision, and content hash. `commit_published()` rejects a stale prepared
  result so out-of-order publish acknowledgements cannot roll the baseline
  backward.
- C2 observation callbacks only enqueue a bounded receipt. Exact-revision
  acquisition, full-map materialization, sorting, diffing, hashing, and payload
  encoding run on the single worker. The worker retains at most one pending and
  one in-flight receipt, coalesces pending work latest-wins, and joins on
  shutdown.
- Envelope versions, enums, hashes, counts, payload size, string size, cell
  count, operation count, retained snapshots, peak apply memory, revision span,
  delta chain, and resync ledger are admitted against `MapUpdateLimits` before
  allocation or mutation. Rejection is atomic and does not refresh freshness.
- Chunked peak admission includes the committed live snapshot, fixed candidate
  directory, copied bucket entries, cloned/new chunk payload, decoded payload
  and operations, and canonical-cursor scratch. The store exposes checked upper
  bounds before mutation and actual owned/shared metrics after apply; sharing
  never removes the still-live committed map from the capacity calculation.
- `Remove` is a terminal tombstone for its admitted source chain. Malformed,
  stale, or conflicting same-chain updates cannot change `Removed` into
  `ResyncRequired`; only explicit admission of a newer source chain can build a
  new baseline.
- A resync ledger is FIFO-bounded and idempotent by requester session plus
  client request ID. Conflicting duplicates, unknown reason enum values, stale
  target identities, and invalid exact revisions are rejected without
  consuming capacity. A failed asynchronous keyframe publication releases its
  correlation so the request can be retried.
- Event-driven keyframes cover initial baseline, admitted resync, lost producer
  base, resource/chain fallback, and source epoch changes. Periodic keyframes
  default to disabled; late join must complete through resync rather than wait
  for a periodic full map.
- `CanonicalSnapshot` and exact-revision acquisition are production functions.
  `MapUpdateReplayOracle` is a profiling/acceptance fixture: it drives the real
  C2 mapper/backend and reuses the production adapter, producer, and applier. It
  must not become a dependency of the production mapper, receiver, or safety
  path, and it must not implement a second codec or apply algorithm.
- Every C2 input with an `Applied` receipt is a replay checkpoint. A gate-time
  `Unavailable` input has no committed revision and is counted for diagnostics,
  but is not fabricated into a checkpoint. At every checkpoint, the replay
  compares source, geometry, revision, content hash, and complete canonical
  cells; fixed input repeats also compare update kind and update hash.
- Historical bag files are optional compatibility inputs, not an authority
  oracle and not a runtime dependency. When unavailable, a deterministic C2
  exact-transaction dataset can fully validate the revision-aware C3 chain.
- The replay visualization publishes standard oracle/reconstructed Octomap
  topics for machine comparison plus bounded deterministic MarkerArray views
  and a difference layer. Marker visualization is permitted when the deployed
  Octomap RViz plugin has an ABI mismatch; it does not replace byte/cell
  equivalence assertions.
- Replay/oracle calculation, acceptance-scenario construction, and
  Octomap/diagnostic publication remain one-shot. The fixture retains at most
  the first three exact-revision canonical snapshots and feeds those snapshots
  through the production `MapUpdateProducer` and `MapUpdateApplier` to build
  deterministic resync-recovery and epoch-reset playback stages. The retained
  snapshots are the first, middle, and final advancing checkpoints, so the
  visualization exposes meaningful revision/map growth without retaining the
  complete snapshot history or implementing a second protocol state machine.
- The visualization fixture caches the final baseline map/difference,
  resync-recovery, and epoch-reset MarkerArrays and republishes those arrays at
  `visualization_publish_rate_hz` (default `1.0`) so an RViz display cleared by
  disable/enable receives a fresh sample. Republishing may refresh Marker header
  stamps, but must not recompute replay/scenarios, change revision/hash/checkpoint
  results, or publish production `MapUpdate` traffic.
- Baseline, resync, and epoch-reset views use distinct topics, Marker namespaces,
  and TF frames. Each acceptance view publishes exactly one current stage at a
  fixed spatial location; `scenario_step_period_s` advances the cached stage,
  without publishing immediately, while `visualization_publish_rate_hz` is the
  only publication timer for the cached views. This keeps every visualization
  topic at a fixed configured rate even when a stage changes. Their RViz Display
  Groups can be toggled independently.
  The display checkbox controls subscription/rendering only; launch parameters
  control whether a test scenario is generated.
- Frozen-version resource acceptance uses the repository local-map profiling
  runner with one build/workload across disabled, enabled, and keyframe-only
  modes, three independent 300-second runs per mode. Enabled/keyframe-only
  runs must drain to the latest exact C2 revision after input stops. CPU and
  absolute latency follow the repository measurement-boundary wording; the
  matrix does not introduce a sub-10% relative gate.
- Exported CMake targets are `perception_map_update::core`, `::ros`,
  `::parameters`, and `::octomap`. Only `::core` is ROS-free; parameter
  declaration remains in the separate `::parameters` target.

## 4. Validation & Error Matrix

| Condition | Required result |
| --- | --- |
| Duplicate revision and committed update hash | Return `IgnoredDuplicate`; do not refresh state/freshness |
| Same identity/revision with a different update hash | Return `RejectedConflict`, retain the last map, require resync unless already removed |
| Delta base is stale/future or base hash differs | Reject atomically, retain the last valid revision, require resync unless already removed |
| Unknown protocol, encoding, hash, kind, operation, cell, or resync-reason enum | Reject before mutation; an invalid resync reason consumes no ledger entry |
| Unknown `CellStorageMode`, zero chunk edge, or zero bucket count | Throw `std::invalid_argument` during store/applier construction before any source is admitted |
| Negative voxel index lies on or beside a chunk boundary | Use floor division and produce a local coordinate in `[0, chunk_edge)`; never use C++ truncation toward zero |
| Chunked estimate or apply sees unsorted/duplicate/invalid operations or checked-size overflow | Return a failed `CellStoreResult`; keep the committed snapshot and sharing identities unchanged |
| Candidate count/hash/resource check fails after chunk construction | Discard the candidate atomically; retain the last legal revision, cells, hash, and freshness |
| Declared count/bytes disagree with payload or exceed a configured limit | Reject before decode allocation or candidate-state mutation |
| Shared snapshot passed to `prepare()` is null | Return `RejectedInvalidSnapshot`; do not create an update or change the baseline |
| C2 receipt is superseded | Fall forward to the current exact transaction or discard; never combine old metadata with current cells |
| Publish callback fails | Keep producer baseline unchanged and release a pending resync correlation for retry |
| A stale prepared publication is committed after a newer one | `commit_published()` returns false; baseline and chain length remain at the newer state |
| Same-chain update arrives after `Remove` | Reject while preserving `ReceiverState::Removed` and the tombstone fence |
| Receiver detects initial/gap/conflict/local-state failure | Keep the last valid map and issue an idempotent short resync request; recover only from an admitted keyframe |
| Replay input is unavailable before commit | Count it as an input result; do not create a revision checkpoint |
| Any replay checkpoint differs in identity, geometry, revision, hash, or cells | Mark the run failed and publish an error diagnostic with bounded difference samples |
| Resync storyboard drops a delta and applies a later future-base delta | Return `RejectedGap`, enter `ResyncRequired`, and preserve the baseline map until a correlated keyframe restores the latest snapshot |
| Epoch storyboard admits a newer epoch then receives an old-epoch delta | Return `RejectedAdmission`, preserve the retired map while awaiting a keyframe, then atomically establish the new epoch at revision 1 |
| `visualization_publish_rate_hz` is non-finite, non-positive, or produces an invalid timer period | Reject node construction with `std::invalid_argument` |
| `scenario_step_period_s` is non-finite, non-positive, or produces an invalid timer period | Reject node construction with `std::invalid_argument` |

## 5. Good / Base / Bad Cases

- Good: C2 revisions 12, 13, and 14 are coalesced into one delta with base 11,
  new revision 14, and an explicit span. A receiver at revision 11 applies it
  and reconstructs the same canonical snapshot as a revision-14 keyframe.
- Good: occupancy probabilities change while canonical cells do not. C2 still
  advances its revision, and C3 emits an empty revision-only delta without a
  false gap or content-hash change.
- Good: the worker moves one exact adapter snapshot into shared immutable
  ownership; prepare, publish acknowledgement, and the committed baseline all
  refer to the same object.
- Base: delta production is disabled. The same envelope, resource admission,
  atomic keyframe replacement, and resync path remain valid.
- Base: `MapUpdateApplier(limits)` uses `Vector`; callers opt into chunked
  storage only through an explicit `CellStorageConfig` for conformance or
  profiling.
- Good: a chunked delta changes two chunks, preserves the pointer identity of
  every untouched chunk, hashes the cursor stream to the same v1 digest as a
  vector applier, and commits only after all checks pass.
- Good: replay runs once, caches the final MarkerArrays, and republishes only
  those arrays at 1 Hz; an RViz display toggled back on recovers within one
  period without changing the oracle diagnostic.
- Good: the resync and epoch-reset storyboards reuse three bounded snapshots and
  the production producer/applier, publish one dynamic current stage on isolated
  topics, and remain independently selectable in one RViz window.
- Bad: committing prepared update A after newer prepared update B and blindly
  assigning A's target as the producer baseline.
- Bad: treating a malformed update after a tombstone as a resync transition,
  which allows a same-chain keyframe to resurrect removed state.
- Bad: computing canonical traversal, diff, or SHA-256 in the C2 mutation
  callback, or using an unbounded receipt queue to preserve every revision.
- Bad: changing only `PreparedUpdate::target_snapshot` to `shared_ptr` while
  leaving optional/value checks or baseline copies in the implementation.
- Bad: rerunning `MapUpdateReplayOracle::run()` on every visualization timer or
  periodically republishing production `MapUpdate` messages to refresh RViz.
- Bad: retaining every replay snapshot for a visual storyboard, or hand-coding
  gap/epoch state transitions in the ROS visualization node.
- Bad: iterating buckets directly as canonical order, or calling
  `materialize()` in every callback and retaining that vector beside the
  authoritative store.

## 6. Tests Required

- Golden codec/hash tests assert deterministic big-endian bytes, normalized
  floating-point handling, strict ordering, truncation/trailing rejection, and
  checked count/size arithmetic. At least one fixed SHA-256 input must cross
  the digest batching boundary.
- Core tests cover added, removed, flipped, revision-only, multi-revision span,
  duplicate, stale, gap, hash conflict, source admission, tombstone, unknown
  enums, all resource limits, stale publish commit, null shared input, shared
  target/baseline object identity, and bounded resync FIFO.
- Storage tests cover negative floor division, chunk boundaries, invalid
  layouts, immutable untouched-chunk identity, touched bucket/chunk accounting,
  cursor global order, flat-hash golden equivalence, and checked peak-admission
  bounds. Vector/chunked conformance covers every `ApplyUpdateStatus` and every
  three-dimensional replay checkpoint.
- Layout estimator tests compare predicted metrics with a real
  `CellSnapshotStore::apply()` for every row. A storage A/B uses one
  RelWithDebInfo receiver/runner identity and keeps flat SHA-256, wire input,
  workload, and sampling rules fixed.
- Backend conformance tests materialize the same exact C2 transaction through
  both registered local-map backends and assert identical canonical snapshots.
- Async producer tests assert one pending/one in-flight behavior, latest-wins
  coalescing, superseded fall-forward, publish-before-commit, failed-resync
  retry, and shutdown drain/join.
- Direct ROS closed-loop tests assert late-join service-to-topic recovery,
  duplicate/gap/corruption rejection, exact reconstructed revision/content,
  bounded diagnostics, and a non-empty RViz OctoMap smoke path.
- Replay tests drive the real C2 mapper/backend, compare every committed
  revision through the production producer/applier, repeat fixed input for
  deterministic revision/kind/hash results, and bound difference samples.
- Replay launch testing asserts byte-identical final Octomap messages, empty
  difference markers, non-empty side-by-side map markers, isolated resync/epoch
  Marker namespaces, fixed scene frames, a complete dynamic stage cycle, and OK diagnostics. Core
  tests assert that rejected gap/old-epoch updates preserve the prior map and
  that keyframes recover the intended revision/epoch. A volatile subscriber
  created after replay completion must receive at least two later samples from
  every MarkerArray topic with increasing baseline header stamps, proving
  display recovery does not rely on transient-local history. A real RViz
  process smoke remains a manual integration check.
- Final validation runs the four affected packages (`perception_interfaces`,
  `perception_map_update`, `perception_local_map`, and `perception_profiling`)
  with zero test failures or skips, plus ASan/LSan and Memcheck on core and C2
  adapter/async-producer tests.
  Third-party static registries may be reported as `still reachable`, but
  definite, indirect, or possible business leaks and memory errors are zero.
- The frozen profiling matrix runs disabled, enabled, and keyframe-only modes
  three times for 300 seconds with one RelWithDebInfo build and identical
  bounded 10 Hz input. Assert graph/parameter agreement, update-chain and
  counter conservation, bounded pending/in-flight state, drain convergence,
  three distinct evidence identities, CPU/RSS/PSS/USS summaries, and no
  threshold-triggering sustained memory growth.

## 7. Wrong vs Correct

Wrong:

```cpp
// Deep-copies the full map once into PreparedUpdate and again into baseline_.
PreparedUpdate prepared{status, update, target_snapshot_by_value};
baseline_ = prepared.target_snapshot;

TestOnlyMapUpdateProducer replay_producer; // A second protocol implementation drifts.
timer = create_wall_timer(period, [this] { replay_oracle.run(); });

if (reject_update) {
    state_ = ReceiverState::ResyncRequired; // Revives a removed chain later.
}
```

Correct:

```cpp
auto target = std::make_shared<const CanonicalSnapshot>(
    std::move(materialized_snapshot));
PreparedUpdate prepared = producer.prepare(target);

if (prepared.expected_baseline != current_baseline_token()) {
    return false;
}
baseline_ = prepared.target_snapshot;

MapUpdateProducer replay_producer(limits); // Replay reuses production behavior.
MapUpdateApplier replay_applier(limits);
timer = create_wall_timer(period, [this] { publish_cached_marker_arrays(); });

if (state_ != ReceiverState::Removed && require_resync) {
    state_ = ReceiverState::ResyncRequired;
}
```

Wrong:

```cpp
// Recreates and retains a second complete map in a hot consumer.
cached_cells_ = reconstructed.cells.materialize();
for (const auto & bucket : chunked_snapshot.buckets) {
    hash_bucket_order(bucket); // Bucket order is not canonical v1 order.
}
```

Correct:

```cpp
auto cursor = reconstructed.cells.cursor();
while (!cursor.done()) {
    consume_canonical_cell(cursor.value());
    cursor.advance();
}

CellStorageConfig research_storage{
    CellStorageMode::Chunked, 16U, 256U};
MapUpdateApplier research_applier(limits, research_storage);
```

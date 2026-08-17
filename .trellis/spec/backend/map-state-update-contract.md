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
explicit MapUpdateApplier::MapUpdateApplier(MapUpdateLimits limits = {});
CanonicalCellCursor CanonicalCellView::cursor() const;
std::vector<CanonicalCell> CanonicalCellView::materialize() const;
CellStoreResult CellSnapshotStore::estimate_apply_upper_bound(
    const std::vector<DeltaOperation> & operations) const noexcept;
CellStoreResult CellSnapshotStore::apply(
    const std::vector<DeltaOperation> & operations);
void CellSnapshotStore::for_each_chunk(const ChunkVisitor & visitor) const;
bool CellSnapshotStore::copy_chunk(
    const ChunkCoordinate & coordinate,
    std::vector<CanonicalCell> & cells) const;

struct ContentIdentityDescriptor {
    ContentIdentityScheme scheme = ContentIdentityScheme::MerklePatriciaSha256V2;
    std::uint32_t chunk_edge = 16U;
    std::uint16_t coordinate_key_version = 1U;
    std::uint16_t node_encoding_version = 1U;
};
MerkleMapStateResult MerkleMapState::build(
    const SourceIdentity & source,
    const Hash256 & geometry_fingerprint,
    const std::vector<CanonicalCell> & cells,
    CellStorageConfig storage = {CellStorageMode::Chunked, 16U, 256U},
    ContentIdentityDescriptor descriptor = {},
    MapUpdateLimits limits = {});
MerkleMapStateResult MerkleMapState::apply(
    const std::vector<DeltaOperation> & operations) const;
MerkleTreeResult MerklePatriciaTree::full_rebuild(
    const SourceIdentity & source,
    const Hash256 & geometry_fingerprint,
    const CellSnapshotStore & store,
    ContentIdentityDescriptor descriptor = {});
MerkleTreeResult MerklePatriciaTree::apply(
    const std::vector<MerkleChunkMutation> & mutations) const;
MerklePrototypeResult MerklePrototypeApplier::apply(
    const std::vector<DeltaOperation> & operations) const;
MerklePrototypeTransition MerklePrototypeProtocol::verify_delta(
    const MerklePrototypeApplier & committed,
    std::uint64_t committed_revision,
    const MerklePrototypeUpdate & update);

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

C4.3 fixes that ROS boundary to protocol v2-only. `MapUpdate.protocol_version`
is `2`, `canonical_encoding_version` is `1`, and the only accepted hash
algorithm is SHA-256. `content_identity` must be
`{scheme=MerklePatriciaSha256V2, chunk_edge=16, coordinate_key_version=1,
node_encoding_version=1}`. The `base_content_hash` and `content_hash` fields
carry the versioned base/result Merkle digests; the descriptor is never inferred
from a bare 32-byte value. Direct and routed resync messages carry the same
descriptor plus digest, so a descriptor drift or unknown enum fails closed.

## 3. Contracts

- Canonical state is a strictly sorted sparse vector of known `Free` or
  `Occupied` cells. Missing cells are `Unknown`; a delta uses
  `RemoveToUnknown` to delete a known cell.
- `ReconstructedMap::cells` is a storage-independent `CanonicalCellView`, not
  an owned vector contract. Production consumers stream through `cursor()` or
  `for_each()`; `materialize()` is reserved for explicit compatibility, test,
  or serialization boundaries and must not be cached as a second authoritative
  map.
- `CellSnapshotStore` still exposes `Vector` and `Chunked` conformance modes,
  but the production `MapUpdateProducer` and `MapUpdateApplier` are now v2-only
  and use immutable chunked storage with edge 16 and 256 deterministic buckets.
  Flat v1/vector remains available only to isolated correctness or benchmark
  oracles; it is not a production runtime fallback.
- A chunked committed snapshot, its buckets, and chunks are immutable. Delta
  apply shallow-copies the fixed directory and clones only touched buckets and
  chunks; unchanged chunks retain shared object identity. Revision-only delta
  shares every chunk. Candidate construction must not allocate a complete
  canonical cell vector.
- Production Merkle code may use `CellSnapshotStore::for_each_chunk()` or
  `copy_chunk()` to read deterministic const chunk contents. These visitors
  never expose bucket/chunk mutability or replace the storage-independent
  `CanonicalCellView` consumer boundary.
- The production v2 identity is a persistent Merkle Patricia trie over edge-16
  chunks. Its collision-free 192-bit key is signed `x/y/z` int64 with each sign
  bit flipped and each axis written big-endian. Empty, leaf, branch, outer
  content, and prototype-update hashes use distinct length-prefixed domains.
  Leaves retain only key, digest, and cell count; canonical chunk cells remain
  owned by `CellSnapshotStore` and are not duplicated in the tree.
- A v2 descriptor is part of the content and update identity. Scheme, chunk
  edge, coordinate-key version, and node-encoding version must all match. A
  bare `HashAlgorithm::Sha256` value cannot identify flat v1 versus Merkle v2,
  and a v1 base must never continue a v2 delta chain.
- `MerkleMapState` copies the immutable committed store/tree handles, builds
  touched chunks and persistent trie paths, validates the locally recomputed
  result root, then publishes one candidate containing both. The historical
  `MerklePrototypeApplier` name is only a compatibility alias used by isolated
  tests/benchmarks; it is not a second production state machine. Any operation,
  descriptor, base, count/overflow, or root failure discards the candidate and
  cannot mutate the committed store, root, or revision.
- Initial/resync keyframes still rebuild store and tree in O(N). Existing-leaf
  delta work is proportional to touched chunks and their shared Patricia paths,
  not the complete known-cell stream. Revision-only deltas allocate no tree
  nodes and preserve the content root. An empty keyframe has a non-zero
  canonical root; `Remove` uses a zero tombstone and no candidate map.
- Merkle correctness mode may compute flat v1 and full-rebuild v2 oracles.
  Merkle performance mode must not compute any flat keyframe/hash/apply work;
  flat performance mode must not compute Merkle work. Evidence directories are
  create-once and each run records executable SHA/build ID, image SHA,
  PID/starttime, source hashes, command, compiler, PSS/USS, and raw smaps.
- C4.3 promotes the ROS-visible contract to protocol v2-only. `MapUpdate`
  carries a `ContentIdentityDescriptor` and the base/result Merkle digests;
  conversions and direct/routed resync messages reject unknown descriptors
  before allocation. Relays treat the nested update as opaque and do not inspect
  Patricia nodes. The implementation is complete on the integration branch,
  but production acceptance remains pending the separately required 3 x
  300-second resource matrix, sanitizer/memory-tool evidence, and rollback
  review. Until that Gate is recorded as GO, C5d must remain blocked.
- Chunk coordinates use mathematical floor division for negative voxel indices.
  A chunked canonical cursor performs a deterministic global `(x,y,z)` merge;
  bucket order, chunk-coordinate order, and local chunk order are not protocol
  v1 canonical order by themselves.
- Flat protocol-v1 `content_hash` remains a frozen SHA-256 oracle over the
  complete canonical stream for both storage modes. It is not read or written
  by production v2 nodes. Production `content_hash` is the locally recomputed
  v2 Merkle root, whose descriptor is carried in the same update and resync
  identity; the receiver never trusts a sender-provided root.
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
| Merkle descriptor is unknown, has a non-16 edge, or differs between envelope/base/result | Reject before candidate commit; do not reinterpret the 32-byte digest as flat v1 |
| Merkle delta base descriptor/root/revision, source, or geometry differs from committed state | Reject atomically and retain the committed store/tree pair |
| Producer result root or prototype update hash is tampered | Recompute locally, reject the candidate, and retain the committed store/tree pair |
| Revision-only Merkle delta has no operations | Reuse the committed tree/root and allocate zero candidate tree bytes |
| Merkle remove targets the valid committed chain | Commit a zero tombstone with no candidate map; never equate it with the non-zero empty-keyframe root |
| Profiling output directory already exists or run identities differ | Fail the run/aggregate; never overwrite or combine the evidence |
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
- Base: the production `MapUpdateApplier(limits)` uses fixed chunk-16 Merkle
  state; `CellStorageConfig{Vector,...}` is reserved for isolated flat-v1
  conformance and benchmark code.
- Good: a chunked delta changes two chunks, preserves the pointer identity of
  every untouched chunk, hashes the cursor stream to the same v1 digest as a
  vector applier, and commits only after all checks pass.
- Good: a v2 receiver starts from its local committed tree, applies four
  touched chunks, recomputes leaf/branch/content hashes, and accepts only when
  its result equals the versioned producer root.
- Base: a flat-v1 oracle may be linked by unit/profiling code, but production
  ROS nodes publish and verify only the v2 descriptor/root contract.
- Bad: trusting a producer-provided Merkle root, using a bucket hash as the trie
  key, accepting an unknown descriptor as v1, or computing flat SHA-256 in a
  v2 production/profile run.
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
- Merkle tests cover signed-coordinate golden keys, fixed empty/single-leaf
  roots, odd leaf counts, insert/replace/delete, deleting the last cell,
  revision-only zero allocation, duplicate/invalid mutations, and at least 400
  deterministic three-dimensional checkpoints where incremental roots equal an
  independent full rebuild.
- Production protocol tests assert locally recomputed keyframe/delta roots,
  keyframe/resync replacement, revision-only, zero remove tombstone, update-hash
  determinism, wrong base/revision/source/geometry, descriptor drift, unknown
  kind, tampering, and failure atomicity. The compatibility prototype tests
  cover the same pure algorithm vectors, while performance evidence asserts
  zero flat columns in v2-only runs and zero Merkle columns in flat-only oracle
  runs.
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

MapUpdateApplier production_applier(limits); // fixed chunk-16 Merkle v2
CellStorageConfig oracle_storage{CellStorageMode::Vector, 16U, 256U};
```

Wrong:

```cpp
// Same SHA-256 enum, but the identity structure is silently changed.
update.hash_algorithm = HashAlgorithm::Sha256;
committed_root = update.result_root; // Trusts the sender and skips local apply.
```

Correct:

```cpp
ContentIdentityDescriptor descriptor{
    ContentIdentityScheme::MerklePatriciaSha256V2, 16U, 1U, 1U};
auto verified = MerklePrototypeProtocol::verify_delta(
    committed_prototype, committed_revision, prototype_update);
if (!verified) {
    return; // Committed store/tree remain unchanged.
}
auto next = std::move(verified.candidate);
```

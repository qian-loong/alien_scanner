#include "perception_map_update/MerkleMapState.hpp"

#include <chrono>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        using Clock = std::chrono::steady_clock;
        constexpr std::size_t kMaxPersistentNodesPerTouchedChunk =
                kMerkleCoordinateKeyBytes * 8U + 1U;

        std::uint64_t elapsed_ns(Clock::time_point begin, Clock::time_point end) noexcept
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count());
        }

        bool checked_add(std::size_t left, std::size_t right, std::size_t & result) noexcept
        {
            if(left > std::numeric_limits<std::size_t>::max() - right) {
                return false;
            }
            result = left + right;
            return true;
        }

        bool checked_mul(std::size_t left, std::size_t right, std::size_t & result) noexcept
        {
            if(left != 0U && right > std::numeric_limits<std::size_t>::max() / left) {
                return false;
            }
            result = left * right;
            return true;
        }

        bool live_node_upper_bound(std::size_t chunks, std::size_t & nodes) noexcept
        {
            if(chunks == 0U) {
                nodes = 0U;
                return true;
            }
            std::size_t doubled = 0U;
            return checked_mul(chunks, 2U, doubled)
                   && (nodes = doubled - 1U, true);
        }

        bool valid_storage(
                const CellStorageConfig & storage,
                const ContentIdentityDescriptor & descriptor) noexcept
        {
            return descriptor.valid()
                   && storage.mode == CellStorageMode::Chunked
                   && storage.chunk_edge == kMerkleChunkEdge
                   && storage.bucket_count == kMerkleChunkBucketCount
                   && storage.chunk_edge == descriptor.chunk_edge;
        }

        bool valid_limits(const MapUpdateLimits & limits) noexcept
        {
            return limits.max_known_cells > 0U
                   && limits.max_delta_operations > 0U
                   && limits.max_live_chunks > 0U
                   && limits.max_merkle_nodes > 0U
                   && limits.max_merkle_owned_bytes > 0U
                   && limits.max_candidate_merkle_bytes > 0U
                   && limits.max_peak_apply_bytes > 0U;
        }

        bool count_distinct_chunks(
                const std::vector<CanonicalCell> & cells,
                std::size_t & count,
                std::string & diagnostic)
        {
            count = 0U;
            ChunkCoordinate previous {};
            bool has_previous = false;
            for(const auto & cell : cells) {
                const auto address = locate_chunk(cell.index, kMerkleChunkEdge);
                if(!address) {
                    diagnostic = address.diagnostic;
                    return false;
                }
                if(!has_previous || address.address.chunk != previous) {
                    if(count == std::numeric_limits<std::size_t>::max()) {
                        diagnostic = "Merkle live chunk count overflow";
                        return false;
                    }
                    ++count;
                    previous = address.address.chunk;
                    has_previous = true;
                }
            }
            return true;
        }

        bool count_touched_chunks(
                const std::vector<DeltaOperation> & operations,
                std::size_t & count,
                std::string & diagnostic)
        {
            count = 0U;
            ChunkCoordinate previous {};
            bool has_previous = false;
            for(const auto & operation : operations) {
                const auto address = locate_chunk(operation.index, kMerkleChunkEdge);
                if(!address) {
                    diagnostic = address.diagnostic;
                    return false;
                }
                if(!has_previous || address.address.chunk != previous) {
                    if(count == std::numeric_limits<std::size_t>::max()) {
                        diagnostic = "Merkle touched chunk count overflow";
                        return false;
                    }
                    ++count;
                    previous = address.address.chunk;
                    has_previous = true;
                }
            }
            return true;
        }

        bool reject_resource(
                MerkleMapStateResult & result,
                const char * diagnostic)
        {
            result.resource_limit = true;
            result.diagnostic = diagnostic;
            return false;
        }

    }// namespace

    MerkleMapState::MerkleMapState(
            CellSnapshotStore store,
            std::shared_ptr<MerklePatriciaTree> tree,
            MapUpdateLimits limits)
            : store_(std::move(store))
            , tree_(std::move(tree))
            , identity_(tree_->versioned_digest())
            , limits_(std::move(limits))
    {
    }

    MerkleMapStateResult MerkleMapState::build(
            const SourceIdentity & source,
            const Hash256 & geometry_fingerprint,
            const std::vector<CanonicalCell> & cells,
            CellStorageConfig storage,
            ContentIdentityDescriptor descriptor,
            MapUpdateLimits limits)
    {
        MerkleMapStateResult result;
        if(!valid_storage(storage, descriptor)) {
            result.diagnostic = "Merkle map state requires the canonical chunk-16 layout";
            return result;
        }
        if(!valid_limits(limits)) {
            result.diagnostic = "Merkle map state limits must be positive";
            return result;
        }
        if(cells.size() > limits.max_known_cells) {
            reject_resource(result, "Merkle keyframe cell count exceeds configured limit");
            return result;
        }

        try {
            CellSnapshotStore store(storage);
            std::size_t chunk_count = 0U;
            if(!count_distinct_chunks(cells, chunk_count, result.diagnostic)) {
                return result;
            }
            const auto store_estimate = store.estimate_replace_upper_bound(
                    cells.size(), chunk_count);
            if(!store_estimate.success) {
                result.diagnostic = store_estimate.diagnostic;
                return result;
            }
            std::size_t node_count = 0U;
            std::size_t merkle_bytes = 0U;
            if(!live_node_upper_bound(chunk_count, node_count)
               || !MerklePatriciaTree::estimate_node_bytes(node_count, merkle_bytes)) {
                reject_resource(result, "Merkle keyframe resource estimate overflow");
                return result;
            }
            if(chunk_count > limits.max_live_chunks
               || node_count > limits.max_merkle_nodes
               || merkle_bytes > limits.max_merkle_owned_bytes
               || merkle_bytes > limits.max_candidate_merkle_bytes) {
                reject_resource(result, "Merkle keyframe exceeds configured chunk or tree limit");
                return result;
            }
            if(!checked_add(
                       store_estimate.metrics.candidate_owned_bytes,
                       merkle_bytes,
                       result.estimated_peak_bytes)
               || result.estimated_peak_bytes > limits.max_peak_apply_bytes) {
                reject_resource(result, "Merkle keyframe exceeds configured peak apply bytes");
                return result;
            }

            const auto storage_begin = Clock::now();
            const auto stored = store.replace(cells);
            const auto storage_end = Clock::now();
            result.timings.storage_ns = elapsed_ns(storage_begin, storage_end);
            if(!stored.success) {
                result.diagnostic = stored.diagnostic;
                return result;
            }

            const auto merkle_begin = Clock::now();
            const auto tree = MerklePatriciaTree::full_rebuild(
                    source, geometry_fingerprint, store, descriptor);
            const auto merkle_end = Clock::now();
            result.timings.merkle_ns = elapsed_ns(merkle_begin, merkle_end);
            if(!tree) {
                result.diagnostic = tree.diagnostic;
                return result;
            }

            result.storage_metrics = store.metrics();
            result.merkle_metrics = tree.metrics;
            if(result.storage_metrics.total_chunks > limits.max_live_chunks
               || result.merkle_metrics.node_count > limits.max_merkle_nodes
               || result.merkle_metrics.owned_bytes > limits.max_merkle_owned_bytes
               || result.merkle_metrics.candidate_owned_bytes
                          > limits.max_candidate_merkle_bytes
               || !checked_add(
                       result.storage_metrics.committed_live_bytes,
                       result.merkle_metrics.owned_bytes,
                       result.actual_peak_bytes)
               || result.actual_peak_bytes > limits.max_peak_apply_bytes) {
                reject_resource(result, "Merkle keyframe actual resources exceed configured limit");
                return result;
            }

            const auto commit_begin = Clock::now();
            result.candidate = std::shared_ptr<MerkleMapState>(
                    new MerkleMapState(std::move(store), tree.tree, std::move(limits)));
            const auto commit_end = Clock::now();
            result.timings.commit_ns = elapsed_ns(commit_begin, commit_end);
            result.success = true;
            return result;
        }
        catch(const std::exception & error) {
            result.diagnostic = error.what();
            return result;
        }
    }

    MerkleMapStateResult MerkleMapState::apply(
            const std::vector<DeltaOperation> & operations) const
    {
        MerkleMapStateResult result;
        if(!tree_) {
            result.diagnostic = "Merkle map state has no tree";
            return result;
        }
        if(operations.size() > limits_.max_delta_operations) {
            reject_resource(result, "Merkle delta operation count exceeds configured limit");
            return result;
        }

        try {
            const auto store_estimate = store_.estimate_apply_upper_bound(operations);
            if(!store_estimate.success) {
                result.diagnostic = store_estimate.diagnostic;
                return result;
            }
            std::size_t touched_chunks = 0U;
            if(!count_touched_chunks(operations, touched_chunks, result.diagnostic)) {
                return result;
            }
            std::size_t live_chunk_upper = 0U;
            std::size_t live_node_upper = 0U;
            std::size_t candidate_node_upper = 0U;
            std::size_t candidate_merkle_bytes = 0U;
            if(!checked_add(
                       static_cast<std::size_t>(store_.metrics().total_chunks),
                       touched_chunks,
                       live_chunk_upper)
               || !live_node_upper_bound(live_chunk_upper, live_node_upper)
               || !checked_mul(
                       touched_chunks,
                       kMaxPersistentNodesPerTouchedChunk,
                       candidate_node_upper)
               || !MerklePatriciaTree::estimate_node_bytes(
                       candidate_node_upper, candidate_merkle_bytes)) {
                reject_resource(result, "Merkle delta resource estimate overflow");
                return result;
            }
            if(live_chunk_upper > limits_.max_live_chunks
               || live_node_upper > limits_.max_merkle_nodes
               || candidate_merkle_bytes > limits_.max_candidate_merkle_bytes) {
                reject_resource(result, "Merkle delta exceeds configured chunk or tree limit");
                return result;
            }
            std::size_t committed_bytes = 0U;
            std::size_t candidate_bytes = 0U;
            if(!checked_add(
                       store_estimate.metrics.committed_live_bytes,
                       tree_->metrics().owned_bytes,
                       committed_bytes)
               || !checked_add(
                       store_estimate.metrics.candidate_owned_bytes,
                       candidate_merkle_bytes,
                       candidate_bytes)
               || !checked_add(
                       committed_bytes,
                       candidate_bytes,
                       result.estimated_peak_bytes)
               || result.estimated_peak_bytes > limits_.max_peak_apply_bytes) {
                reject_resource(result, "Merkle delta exceeds configured peak apply bytes");
                return result;
            }

            auto candidate_store = store_;
            const auto storage_begin = Clock::now();
            const auto stored = candidate_store.apply(operations);
            const auto storage_end = Clock::now();
            result.timings.storage_ns = elapsed_ns(storage_begin, storage_end);
            if(!stored.success) {
                result.diagnostic = stored.diagnostic;
                return result;
            }

            const auto mutation_begin = Clock::now();
            std::set<ChunkCoordinate> touched;
            for(const auto & operation : operations) {
                const auto address = locate_chunk(operation.index, kMerkleChunkEdge);
                if(!address) {
                    result.diagnostic = address.diagnostic;
                    return result;
                }
                touched.insert(address.address.chunk);
            }
            std::vector<MerkleChunkMutation> mutations;
            mutations.reserve(touched.size());
            for(const auto & coordinate : touched) {
                std::vector<CanonicalCell> cells;
                if(candidate_store.copy_chunk(coordinate, cells)) {
                    mutations.push_back({coordinate, std::move(cells), false});
                }
                else {
                    mutations.push_back({coordinate, {}, true});
                }
            }
            const auto mutation_end = Clock::now();
            result.timings.mutation_build_ns = elapsed_ns(mutation_begin, mutation_end);

            const auto merkle_begin = Clock::now();
            const auto tree_candidate = tree_->apply(mutations);
            const auto merkle_end = Clock::now();
            result.timings.merkle_ns = elapsed_ns(merkle_begin, merkle_end);
            if(!tree_candidate) {
                result.diagnostic = tree_candidate.diagnostic;
                return result;
            }

            result.storage_metrics = stored.metrics;
            result.merkle_metrics = tree_candidate.metrics;
            std::size_t actual_committed = 0U;
            std::size_t actual_candidate = 0U;
            if(result.storage_metrics.total_chunks > limits_.max_live_chunks
               || result.merkle_metrics.node_count > limits_.max_merkle_nodes
               || result.merkle_metrics.owned_bytes > limits_.max_merkle_owned_bytes
               || result.merkle_metrics.candidate_owned_bytes
                          > limits_.max_candidate_merkle_bytes
               || !checked_add(
                       result.storage_metrics.committed_live_bytes,
                       tree_->metrics().owned_bytes,
                       actual_committed)
               || !checked_add(
                       result.storage_metrics.candidate_owned_bytes,
                       result.merkle_metrics.candidate_owned_bytes,
                       actual_candidate)
               || !checked_add(
                       actual_committed,
                       actual_candidate,
                       result.actual_peak_bytes)
               || result.actual_peak_bytes > limits_.max_peak_apply_bytes) {
                reject_resource(result, "Merkle delta actual resources exceed configured limit");
                return result;
            }

            const auto commit_begin = Clock::now();
            result.candidate = std::shared_ptr<MerkleMapState>(
                    new MerkleMapState(
                            std::move(candidate_store),
                            tree_candidate.tree,
                            limits_));
            const auto commit_end = Clock::now();
            result.timings.commit_ns = elapsed_ns(commit_begin, commit_end);
            result.success = true;
            return result;
        }
        catch(const std::exception & error) {
            result.diagnostic = error.what();
            return result;
        }
    }

    CanonicalCellView MerkleMapState::cells() const
    {
        return store_.view();
    }

    const VersionedContentDigest & MerkleMapState::identity() const noexcept
    {
        return identity_;
    }

    const CellSnapshotStore & MerkleMapState::store() const noexcept
    {
        return store_;
    }

    const MerklePatriciaTree & MerkleMapState::tree() const noexcept
    {
        return *tree_;
    }

}// namespace PerceptionMapUpdate

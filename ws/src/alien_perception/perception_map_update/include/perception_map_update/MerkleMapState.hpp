#ifndef PERCEPTION_MAP_UPDATE_MERKLE_MAP_STATE_HPP
#define PERCEPTION_MAP_UPDATE_MERKLE_MAP_STATE_HPP

#include "perception_map_update/CellSnapshotStore.hpp"
#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MerklePatricia.hpp"

#include <memory>
#include <string>
#include <vector>

namespace PerceptionMapUpdate {

    struct MerkleMapTimings {
        std::uint64_t storage_ns = 0U;
        std::uint64_t mutation_build_ns = 0U;
        std::uint64_t merkle_ns = 0U;
        std::uint64_t commit_ns = 0U;
    };

    struct MerkleMapStateResult {
        bool success = false;
        bool resource_limit = false;
        std::string diagnostic;
        std::shared_ptr<class MerkleMapState> candidate;
        CellStorageMetrics storage_metrics;
        MerkleTreeMetrics merkle_metrics;
        std::size_t estimated_peak_bytes = 0U;
        std::size_t actual_peak_bytes = 0U;
        MerkleMapTimings timings;

        explicit operator bool() const noexcept { return success; }
    };

    class MerkleMapState
    {
    public:
        static MerkleMapStateResult build(
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                const std::vector<CanonicalCell> & cells,
                CellStorageConfig storage = {
                        CellStorageMode::Chunked,
                        kMerkleChunkEdge,
                        kMerkleChunkBucketCount},
                ContentIdentityDescriptor descriptor = {},
                MapUpdateLimits limits = {});

        MerkleMapStateResult apply(
                const std::vector<DeltaOperation> & operations) const;

        CanonicalCellView cells() const;
        const VersionedContentDigest & identity() const noexcept;
        const CellSnapshotStore & store() const noexcept;
        const MerklePatriciaTree & tree() const noexcept;

    private:
        MerkleMapState(
                CellSnapshotStore store,
                std::shared_ptr<MerklePatriciaTree> tree,
                MapUpdateLimits limits);

        CellSnapshotStore store_;
        std::shared_ptr<MerklePatriciaTree> tree_;
        VersionedContentDigest identity_;
        MapUpdateLimits limits_;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MERKLE_MAP_STATE_HPP

#ifndef PERCEPTION_MAP_UPDATE_MERKLE_PATRICIA_HPP
#define PERCEPTION_MAP_UPDATE_MERKLE_PATRICIA_HPP

#include "perception_map_update/CellSnapshotStore.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"
#include "perception_map_update/SpatialChunkLayout.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace PerceptionMapUpdate {

    constexpr std::size_t kMerkleCoordinateKeyBytes = 24U;

    struct MerkleCoordinateKey {
        std::array<std::uint8_t, kMerkleCoordinateKeyBytes> bytes {};

        bool operator==(const MerkleCoordinateKey & other) const noexcept;
        bool operator!=(const MerkleCoordinateKey & other) const noexcept;
        bool operator<(const MerkleCoordinateKey & other) const noexcept;
    };

    // The trie key is the signed lexicographic chunk coordinate, encoded as
    // sign-bit-flipped big-endian int64 values. It is not a bucket hash.
    MerkleCoordinateKey merkle_coordinate_key(const ChunkCoordinate & coordinate) noexcept;
    MerkleCoordinateKey merkle_coordinate_key(const VoxelIndex & coordinate) noexcept;

    struct MerkleChunkMutation {
        ChunkCoordinate coordinate;
        std::vector<CanonicalCell> cells;
        bool remove = false;
    };

    struct MerkleTreeMetrics {
        std::size_t leaf_count = 0U;
        std::size_t branch_count = 0U;
        std::size_t node_count = 0U;
        std::size_t total_cell_count = 0U;
        // Reachable node bytes for this tree. Canonical chunk cells remain in
        // CellSnapshotStore and are not duplicated in Merkle leaves.
        std::size_t owned_bytes = 0U;
        // Nodes allocated while constructing this candidate, excluding reused
        // nodes from the committed tree.
        std::size_t allocated_nodes = 0U;
        // Node objects allocated for this candidate.
        // This excludes persistent nodes shared from the committed tree.
        std::size_t candidate_owned_bytes = 0U;
        std::size_t leaf_hashes = 0U;
        std::size_t branch_hashes = 0U;
        std::size_t path_nodes_rebuilt = 0U;
        std::uint64_t leaf_hash_ns = 0U;
        std::uint64_t branch_hash_ns = 0U;
        std::uint64_t content_hash_ns = 0U;
    };

    class MerklePatriciaTree;

    struct MerkleTreeResult {
        bool success = false;
        std::string diagnostic;
        std::shared_ptr<MerklePatriciaTree> tree;
        MerkleTreeMetrics metrics;

        explicit operator bool() const noexcept { return success; }
    };

    class MerklePatriciaTree
    {
    public:
        static MerkleTreeResult full_rebuild(
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                const std::vector<CanonicalCell> & cells,
                ContentIdentityDescriptor descriptor = {});
        static MerkleTreeResult full_rebuild(
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                const CanonicalCellView & cells,
                ContentIdentityDescriptor descriptor = {});
        static MerkleTreeResult full_rebuild(
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                const CellSnapshotStore & store,
                ContentIdentityDescriptor descriptor = {});

        MerkleTreeResult apply(const std::vector<MerkleChunkMutation> & mutations) const;

        const ContentIdentityDescriptor & descriptor() const noexcept;
        const SourceIdentity & source() const noexcept;
        const Hash256 & geometry_fingerprint() const noexcept;
        const Hash256 & trie_root() const noexcept;
        Hash256 content_root() const noexcept;
        VersionedContentDigest versioned_digest() const noexcept;
        const MerkleTreeMetrics & metrics() const noexcept;

        std::size_t leaf_count() const noexcept;
        std::size_t total_cell_count() const noexcept;
        bool empty() const noexcept;
        static bool estimate_node_bytes(
                std::size_t node_count,
                std::size_t & bytes) noexcept;

    private:
        struct State;

        explicit MerklePatriciaTree(std::shared_ptr<const State> state);
        std::shared_ptr<const State> state_;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MERKLE_PATRICIA_HPP

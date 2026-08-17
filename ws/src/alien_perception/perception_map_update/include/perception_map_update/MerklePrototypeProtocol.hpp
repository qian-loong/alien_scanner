#ifndef PERCEPTION_MAP_UPDATE_MERKLE_PROTOTYPE_PROTOCOL_HPP
#define PERCEPTION_MAP_UPDATE_MERKLE_PROTOTYPE_PROTOCOL_HPP

#include "perception_map_update/MerklePrototypeApplier.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace PerceptionMapUpdate {

    enum class MerklePrototypeUpdateKind : std::uint8_t
    {
        Keyframe = 1,
        Delta = 2,
        Remove = 3
    };

    struct MerklePrototypeUpdate {
        MerklePrototypeUpdateKind kind = MerklePrototypeUpdateKind::Keyframe;
        SourceIdentity source;
        Hash256 geometry_fingerprint {};
        std::uint64_t base_revision = 0U;
        std::uint64_t result_revision = 0U;
        ContentIdentityDescriptor descriptor;
        VersionedContentDigest base_content;
        VersionedContentDigest result_content;
        std::vector<CanonicalCell> keyframe_cells;
        std::vector<DeltaOperation> delta_operations;
        Hash256 update_hash {};
    };

    struct MerklePrototypeTransition {
        bool success = false;
        std::string diagnostic;
        MerklePrototypeUpdate update;
        std::shared_ptr<MerklePrototypeApplier> candidate;
        bool removed = false;

        explicit operator bool() const noexcept { return success; }
    };

    // ROS-free protocol harness for v2 correctness and rollout research. It is
    // deliberately separate from the production MapUpdate v1 wire schema.
    class MerklePrototypeProtocol
    {
    public:
        static MerklePrototypeTransition prepare_keyframe(
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                std::uint64_t result_revision,
                const std::vector<CanonicalCell> & cells,
                CellStorageConfig storage = {CellStorageMode::Chunked, 16U, 256U},
                ContentIdentityDescriptor descriptor = {});

        static MerklePrototypeTransition prepare_delta(
                const MerklePrototypeApplier & base,
                std::uint64_t base_revision,
                std::uint64_t result_revision,
                const std::vector<DeltaOperation> & operations);

        static MerklePrototypeTransition prepare_remove(
                const MerklePrototypeApplier & base,
                std::uint64_t base_revision,
                std::uint64_t result_revision);

        static MerklePrototypeTransition verify_keyframe(
                const MerklePrototypeUpdate & update,
                CellStorageConfig storage = {CellStorageMode::Chunked, 16U, 256U});

        static MerklePrototypeTransition verify_delta(
                const MerklePrototypeApplier & committed,
                std::uint64_t committed_revision,
                const MerklePrototypeUpdate & update);

        static MerklePrototypeTransition verify_remove(
                const MerklePrototypeApplier & committed,
                std::uint64_t committed_revision,
                const MerklePrototypeUpdate & update);

        static Hash256 update_hash(const MerklePrototypeUpdate & update);
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MERKLE_PROTOTYPE_PROTOCOL_HPP

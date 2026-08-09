#ifndef PERCEPTION_MAP_UPDATE_MAP_UPDATE_TYPES_HPP
#define PERCEPTION_MAP_UPDATE_MAP_UPDATE_TYPES_HPP

#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace PerceptionMapUpdate {

    constexpr std::uint16_t kProtocolVersion          = 1U;
    constexpr std::uint16_t kCanonicalEncodingVersion = 1U;
    constexpr std::size_t   kSha256Bytes              = 32U;

    using Hash256 = std::array<std::uint8_t, kSha256Bytes>;

    enum class HashAlgorithm : std::uint8_t
    {
        Sha256 = 1
    };

    enum class CellState : std::uint8_t
    {
        Free     = 1,
        Occupied = 2
    };

    enum class DeltaOperationKind : std::uint8_t
    {
        UpsertFree      = 1,
        UpsertOccupied  = 2,
        RemoveToUnknown = 3
    };

    enum class UpdateKind : std::uint8_t
    {
        Keyframe = 1,
        Delta    = 2,
        Summary  = 3,
        Remove   = 4
    };

    struct Point3d {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        bool operator==(const Point3d & other) const noexcept;
    };

    struct VoxelIndex {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        bool operator==(const VoxelIndex & other) const noexcept;
        bool operator!=(const VoxelIndex & other) const noexcept;
        bool operator<(const VoxelIndex & other) const noexcept;
    };

    struct SourceIdentity {
        std::string           vehicle_id;
        Perception::SessionID mapper_session {0U, 0U};
        std::uint64_t         map_epoch = 0U;

        bool operator==(const SourceIdentity & other) const noexcept;
        bool operator!=(const SourceIdentity & other) const noexcept;
        bool operator<(const SourceIdentity & other) const noexcept;
    };

    struct MapGeometry {
        double      resolution_m = 0.0;
        Point3d     lattice_origin;
        std::string frame_id;

        bool operator==(const MapGeometry & other) const noexcept;
    };

    struct RevisionProvenance {
        std::string           sensor_id;
        Perception::SessionID sensor_session {0U, 0U};
        Perception::Timestamp observation_stamp {0};
        std::string           clock_domain;
        std::uint32_t         changed_cell_count = 0U;

        bool operator==(const RevisionProvenance & other) const noexcept;
    };

    struct CanonicalCell {
        VoxelIndex index;
        CellState  state = CellState::Free;

        bool operator==(const CanonicalCell & other) const noexcept;
    };

    struct DeltaOperation {
        VoxelIndex         index;
        DeltaOperationKind kind = DeltaOperationKind::RemoveToUnknown;

        bool operator==(const DeltaOperation & other) const noexcept;
    };

    struct CanonicalSnapshot {
        SourceIdentity              source;
        MapGeometry                geometry;
        std::uint64_t              revision = 0U;
        RevisionProvenance         latest_commit;
        std::vector<CanonicalCell> cells;
        Hash256                    geometry_fingerprint {};
        Hash256                    content_hash {};
    };

    struct MapUpdate {
        std::uint16_t protocol_version           = kProtocolVersion;
        std::uint16_t canonical_encoding_version = kCanonicalEncodingVersion;
        HashAlgorithm hash_algorithm             = HashAlgorithm::Sha256;
        UpdateKind    kind                       = UpdateKind::Keyframe;
        SourceIdentity source;
        MapGeometry   geometry;
        std::uint64_t base_revision  = 0U;
        std::uint64_t new_revision   = 0U;
        std::uint64_t revision_span  = 0U;
        std::uint64_t observed_coalesced_receipt_count = 0U;
        Hash256       geometry_fingerprint {};
        Hash256       base_content_hash {};
        Hash256       content_hash {};
        Hash256       update_hash {};
        RevisionProvenance latest_commit;
        std::uint64_t known_cell_count       = 0U;
        std::uint64_t operation_count        = 0U;
        std::uint64_t canonical_payload_bytes = 0U;
        std::string   correlation_id;
        std::vector<std::uint8_t> payload;
    };

    struct ValidationResult {
        bool        valid = false;
        std::string diagnostic;

        explicit operator bool() const noexcept { return valid; }
    };

    bool is_zero_hash(const Hash256 & hash) noexcept;
    std::string hash_to_hex(const Hash256 & hash);

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MAP_UPDATE_TYPES_HPP

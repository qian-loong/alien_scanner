#ifndef PERCEPTION_LOCAL_MAP_CANONICAL_SNAPSHOT_ADAPTER_HPP
#define PERCEPTION_LOCAL_MAP_CANONICAL_SNAPSHOT_ADAPTER_HPP

#include "perception_local_map/LocalObservationMapper.hpp"
#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <optional>
#include <string>

namespace PerceptionLocalMap {

    enum class CanonicalSnapshotStatus : std::uint8_t
    {
        Ready,
        TransactionClosed,
        InvalidMetadata,
        InvalidCell,
        ResourceLimit,
        BackendFault
    };

    struct CanonicalSnapshotTiming {
        std::int64_t traversal_duration_ns = 0;
        std::int64_t canonicalize_duration_ns = 0;
        std::int64_t content_hash_duration_ns = 0;
    };

    struct CanonicalSnapshotResult {
        CanonicalSnapshotStatus status = CanonicalSnapshotStatus::InvalidMetadata;
        std::optional<PerceptionMapUpdate::CanonicalSnapshot> snapshot;
        std::string diagnostic;
        CanonicalSnapshotTiming timing;
    };

    class CanonicalSnapshotAdapter
    {
    public:
        explicit CanonicalSnapshotAdapter(
                PerceptionMapUpdate::MapUpdateLimits limits = {});

        CanonicalSnapshotResult materialize(const MapReadTransaction & transaction) const;

    private:
        PerceptionMapUpdate::MapUpdateLimits limits_;
    };

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_CANONICAL_SNAPSHOT_ADAPTER_HPP

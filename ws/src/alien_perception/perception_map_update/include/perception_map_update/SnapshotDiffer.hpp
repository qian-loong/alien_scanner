#ifndef PERCEPTION_MAP_UPDATE_SNAPSHOT_DIFFER_HPP
#define PERCEPTION_MAP_UPDATE_SNAPSHOT_DIFFER_HPP

#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

namespace PerceptionMapUpdate {

    struct DiffResult {
        bool                        success = false;
        std::vector<DeltaOperation> operations;
        std::string                 diagnostic;
    };

    class SnapshotDiffer
    {
    public:
        static DiffResult compare(
                const CanonicalSnapshot & base,
                const CanonicalSnapshot & target,
                const MapUpdateLimits & limits);
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_SNAPSHOT_DIFFER_HPP

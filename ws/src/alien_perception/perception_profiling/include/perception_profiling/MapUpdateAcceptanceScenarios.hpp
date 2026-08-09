#ifndef PERCEPTION_PROFILING_MAP_UPDATE_ACCEPTANCE_SCENARIOS_HPP
#define PERCEPTION_PROFILING_MAP_UPDATE_ACCEPTANCE_SCENARIOS_HPP

#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace PerceptionProfiling {

    enum class MapUpdateAcceptanceScenarioKind : std::uint8_t
    {
        ResyncRecovery,
        EpochReset
    };

    enum class MapUpdateAcceptanceStageKind : std::uint8_t
    {
        ReadyBaseline,
        DeltaDropped,
        GapRejected,
        ResyncRecovered,
        EpochBaseline,
        NewEpochAdmitted,
        OldEpochRejected,
        EpochRecovered
    };

    struct MapUpdateAcceptanceStage {
        MapUpdateAcceptanceStageKind kind =
                MapUpdateAcceptanceStageKind::ReadyBaseline;
        PerceptionMapUpdate::ReceiverState receiver_state =
                PerceptionMapUpdate::ReceiverState::Empty;
        std::optional<PerceptionMapUpdate::ApplyUpdateStatus> apply_status;
        std::uint64_t attempted_base_revision = 0U;
        std::uint64_t attempted_new_revision = 0U;
        std::optional<PerceptionMapUpdate::ReconstructedMap> receiver_map;
        std::string correlation_id;
        std::string diagnostic;
    };

    struct MapUpdateAcceptanceRun {
        MapUpdateAcceptanceScenarioKind kind =
                MapUpdateAcceptanceScenarioKind::ResyncRecovery;
        bool passed = false;
        std::string diagnostic;
        std::vector<MapUpdateAcceptanceStage> stages;
    };

    class MapUpdateAcceptanceScenarios
    {
    public:
        static MapUpdateAcceptanceRun resync_recovery(
                const std::vector<PerceptionMapUpdate::CanonicalSnapshot> & snapshots,
                const PerceptionMapUpdate::MapUpdateLimits & limits = {});

        static MapUpdateAcceptanceRun epoch_reset(
                const std::vector<PerceptionMapUpdate::CanonicalSnapshot> & snapshots,
                const PerceptionMapUpdate::MapUpdateLimits & limits = {});
    };

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_MAP_UPDATE_ACCEPTANCE_SCENARIOS_HPP

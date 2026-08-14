#ifndef PERCEPTION_PROFILING_MAP_UPDATE_REPLAY_ORACLE_HPP
#define PERCEPTION_PROFILING_MAP_UPDATE_REPLAY_ORACLE_HPP

#include "perception_profiling/ProfileScenario.hpp"

#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace PerceptionProfiling {

    struct ReplayComparison {
        bool matches = false;
        std::uint64_t missing_cell_count = 0U;
        std::uint64_t unexpected_cell_count = 0U;
        std::uint64_t state_mismatch_count = 0U;
        std::vector<PerceptionMapUpdate::CanonicalCell> missing_cell_samples;
        std::vector<PerceptionMapUpdate::CanonicalCell> unexpected_cell_samples;
        std::vector<PerceptionMapUpdate::VoxelIndex> state_mismatch_samples;
        std::string diagnostic;
    };

    struct ReplayCheckpoint {
        std::uint64_t sequence = 0U;
        std::int64_t observation_stamp_ns = 0;
        std::uint64_t revision = 0U;
        PerceptionMapUpdate::UpdateKind update_kind =
                PerceptionMapUpdate::UpdateKind::Keyframe;
        PerceptionMapUpdate::ApplyUpdateStatus apply_status =
                PerceptionMapUpdate::ApplyUpdateStatus::RejectedInvalid;
        std::uint64_t source_cell_count = 0U;
        std::uint64_t reconstructed_cell_count = 0U;
        std::uint64_t operation_count = 0U;
        std::uint64_t payload_bytes = 0U;
        PerceptionMapUpdate::Hash256 content_hash {};
        PerceptionMapUpdate::Hash256 update_hash {};
        ReplayComparison comparison;
    };

    struct MapUpdateReplayOptions {
        using SnapshotTransitionObserver = std::function<void(
                const PerceptionMapUpdate::CanonicalSnapshot * base,
                const PerceptionMapUpdate::CanonicalSnapshot & target,
                const std::vector<PerceptionMapUpdate::DeltaOperation> & operations,
                const PerceptionMapUpdate::MapUpdate & update)>;

        std::uint64_t sequence_count = 60U;
        std::size_t max_difference_samples = 4096U;
        PerceptionMapUpdate::MapUpdateLimits limits;
        SnapshotTransitionObserver snapshot_transition_observer;
    };

    struct MapUpdateReplayRun {
        ScenarioMode mode = ScenarioMode::Canonical;
        std::uint64_t accepted_input_count = 0U;
        std::uint64_t applied_input_count = 0U;
        std::uint64_t no_evidence_input_count = 0U;
        std::uint64_t unavailable_input_count = 0U;
        std::uint64_t rejected_input_count = 0U;
        std::uint64_t backend_fault_input_count = 0U;
        std::uint64_t keyframe_count = 0U;
        std::uint64_t delta_count = 0U;
        std::vector<ReplayCheckpoint> checkpoints;
        std::vector<PerceptionMapUpdate::CanonicalSnapshot> acceptance_snapshots;
        std::optional<PerceptionMapUpdate::CanonicalSnapshot> final_oracle;
        std::optional<PerceptionMapUpdate::ReconstructedMap> final_reconstructed;
        ReplayComparison final_comparison;

        bool all_checkpoints_match() const noexcept;
    };

    class MapUpdateReplayOracle
    {
    public:
        explicit MapUpdateReplayOracle(ProfileScenario scenario);

        MapUpdateReplayRun run(const MapUpdateReplayOptions & options = {}) const;

        static ReplayComparison compare(
                const PerceptionMapUpdate::CanonicalSnapshot & oracle,
                const PerceptionMapUpdate::ReconstructedMap & reconstructed,
                std::size_t max_difference_samples = 4096U);

    private:
        ProfileScenario scenario_;
    };

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_MAP_UPDATE_REPLAY_ORACLE_HPP

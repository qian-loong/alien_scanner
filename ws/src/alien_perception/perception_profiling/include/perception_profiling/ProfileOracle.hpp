#ifndef PERCEPTION_PROFILING_PROFILE_ORACLE_HPP
#define PERCEPTION_PROFILING_PROFILE_ORACLE_HPP

#include "perception_profiling/ProfileScenario.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace PerceptionProfiling {

    struct VoxelCounts {
        std::uint64_t known = 0U;
        std::uint64_t free = 0U;
        std::uint64_t occupied = 0U;

        bool operator==(const VoxelCounts & other) const noexcept;
        bool operator!=(const VoxelCounts & other) const noexcept;
    };

    struct OracleCheckpoint {
        std::uint64_t sequence = 0U;
        std::int64_t observation_stamp_ns = 0;
        std::string payload_digest;
        std::uint64_t map_epoch = 0U;
        std::uint64_t revision = 0U;
        VoxelCounts counts;
        std::optional<PerceptionLocalMap::AxisAlignedBounds> bounds;
        std::string contract_fingerprint;
    };

    struct MilestoneCrossing {
        std::uint64_t threshold = 0U;
        std::uint64_t sequence = 0U;
        std::int64_t observation_stamp_ns = 0;
        std::uint64_t revision = 0U;
        std::uint64_t known = 0U;
    };

    enum class CapacityStatus : std::uint8_t
    {
        Covered,
        NotReached,
        CrossingWithoutPostWindow
    };

    struct RevisionBucket {
        std::uint64_t first = 0U;
        std::uint64_t last = 0U;

        std::uint64_t sample_count() const noexcept;
    };

    struct CapacityPlan {
        CapacityStatus status = CapacityStatus::NotReached;
        std::optional<std::uint64_t> crossing_revision;
        std::optional<std::uint64_t> required_end_revision;
        std::optional<RevisionBucket> pre;
        std::optional<RevisionBucket> crossing;
        std::optional<RevisionBucket> post;
    };

    struct OracleOptions {
        std::uint64_t sequence_count = 0U;
        std::uint64_t checkpoint_period_revisions = 100U;
        std::uint64_t bounded_plateau_revisions = 600U;
        std::uint64_t capacity_max_revision = 6000U;
        std::vector<std::uint64_t> milestone_thresholds {
                100'000U, 250'000U, 500'000U, 750'000U};
    };

    struct OracleRun {
        ScenarioMode mode = ScenarioMode::Canonical;
        std::uint64_t accepted = 0U;
        std::uint64_t applied = 0U;
        std::uint64_t no_evidence = 0U;
        std::uint64_t rejected = 0U;
        std::uint64_t unavailable = 0U;
        std::uint64_t backend_fault = 0U;
        std::uint64_t final_revision = 0U;
        std::uint64_t final_map_epoch = 0U;
        std::optional<std::uint64_t> plateau_start_revision;
        std::vector<OracleCheckpoint> checkpoints;
        std::vector<MilestoneCrossing> milestones;
        CapacityPlan capacity;
    };

    struct ProductionStateProjection {
        std::int64_t receipt_monotonic_ns = 0;
        std::uint64_t state_sequence = 0U;
        std::uint64_t map_epoch = 0U;
        std::uint64_t revision = 0U;
        std::int64_t observation_stamp_ns = 0;
        std::string contract_fingerprint;
        std::optional<PerceptionLocalMap::AxisAlignedBounds> bounds;
        std::uint32_t changed_cell_count = 0U;
    };

    struct JoinResult {
        bool matches = false;
        std::string diagnostic;
    };

    class ProfileOracle
    {
    public:
        explicit ProfileOracle(ProfileScenario scenario);

        OracleRun run(const OracleOptions & options) const;
        static CapacityPlan classify_capacity(
                std::optional<std::uint64_t> crossing_revision,
                std::uint64_t maximum_revision);
        static JoinResult validate_join(
                const ProductionStateProjection & production,
                const OracleCheckpoint & oracle);

    private:
        ProfileScenario scenario_;
    };

    std::string capacity_status_name(CapacityStatus status);

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_PROFILE_ORACLE_HPP

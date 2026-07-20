#ifndef SWARM_CONTROLLER_OCTOMAPMERGER_HPP
#define SWARM_CONTROLLER_OCTOMAPMERGER_HPP

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>

namespace SwarmController {

    struct OctoMapMergerConfig {
        double      resolution {0.1};
        std::size_t max_voxels_per_source {5'000'000U};
        std::size_t max_global_voxels {10'000'000U};

        // Test-only fault injection. Production configurations leave this empty.
        std::function<void()> commit_failure_hook;
    };

    enum class SourceUpdateStatus {
        AcceptedChanged,
        AcceptedUnchanged,
        Invalid,
    };

    enum class SourceUpdateFailureStage {
        None,
        InvalidInput,
        Normalize,
        Resource,
        DeltaPreflight,
        Commit,
    };

    struct SourceUpdateStageTiming {
        double normalize_seconds {};
        double snapshot_compare_seconds {};
        double delta_preflight_seconds {};
        double contribution_tree_apply_seconds {};
        double update_inner_occupancy_seconds {};
        double source_commit_seconds {};
    };

    struct SourceUpdateResult {
        SourceUpdateStatus status {SourceUpdateStatus::Invalid};
        SourceUpdateFailureStage failure_stage {SourceUpdateFailureStage::None};
        bool               global_changed {false};
        std::uint64_t      source_revision {0U};
        std::uint64_t      global_revision {0U};
        std::size_t        added_keys {0U};
        std::size_t        removed_keys {0U};
        std::size_t        flipped_keys {0U};
        SourceUpdateStageTiming timing {};
        std::string        reason;

        bool accepted() const
        {
            return status != SourceUpdateStatus::Invalid;
        }
    };

    class OctoMapMerger
    {
    public:
        explicit OctoMapMerger(const OctoMapMergerConfig & config);

        SourceUpdateResult updateSource(
                const std::string & source_id, const octomap::OcTree & source);
        SourceUpdateResult removeSource(const std::string & source_id);

        const octomap::OcTree & tree() const;
        std::uint64_t           sourceRevision() const;
        std::uint64_t           globalRevision() const;
        std::size_t             sourceCount() const;
        std::size_t             sourceVoxelCount(const std::string & source_id) const;
        std::size_t             totalSourceVoxelCount() const;
        std::size_t             knownCount() const;
        std::size_t             freeCount() const;
        std::size_t             occupiedCount() const;

    private:
        enum class VoxelState : std::uint8_t {
            Free,
            Occupied,
        };

        enum class DerivedState : std::uint8_t {
            Unknown,
            Free,
            Occupied,
        };

        struct VoxelRecord {
            octomap::OcTreeKey key {};
            VoxelState         state {VoxelState::Free};
        };

        struct ContributionCounts {
            std::uint32_t free_sources {0U};
            std::uint32_t occupied_sources {0U};
        };

        struct PendingDelta {
            octomap::OcTreeKey  key {};
            ContributionCounts  counts {};
            DerivedState        old_global {DerivedState::Unknown};
            DerivedState        new_global {DerivedState::Unknown};
        };

        using SourceSnapshot = std::vector<VoxelRecord>;
        using SourceMap = std::unordered_map<std::string, SourceSnapshot>;
        using ContributionMap = std::unordered_map<
                octomap::OcTreeKey, ContributionCounts, octomap::OcTreeKey::KeyHash>;

        SourceUpdateResult invalidResult(
                const std::string & reason, SourceUpdateFailureStage failure_stage,
                const SourceUpdateStageTiming & timing = {}) const;
        bool normalizeSource(
                const octomap::OcTree & source, SourceSnapshot & snapshot,
                std::string & reason, SourceUpdateFailureStage & failure_stage) const;
        SourceUpdateResult replaceSource(
                const std::string & source_id, SourceSnapshot snapshot, bool remove_entry,
                SourceUpdateStageTiming timing = {});

        static bool keyLess(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs);
        static bool recordsEqual(const SourceSnapshot & lhs, const SourceSnapshot & rhs);
        static DerivedState derivedState(const ContributionCounts & counts);
        static bool checkedAdd(std::uint64_t lhs, std::uint64_t rhs, std::uint64_t & result);
        static bool checkedMultiply(std::uint64_t lhs, std::uint64_t rhs, std::uint64_t & result);

        OctoMapMergerConfig                                      config_;
        octomap::OcTree                                          tree_;
        SourceMap                                                 sources_;
        ContributionMap                                          contributions_;
        std::uint64_t                                            source_revision_ {0U};
        std::uint64_t                                            global_revision_ {0U};
        std::size_t                                              total_source_voxels_ {0U};
        std::size_t                                              occupied_count_ {0U};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_OCTOMAPMERGER_HPP

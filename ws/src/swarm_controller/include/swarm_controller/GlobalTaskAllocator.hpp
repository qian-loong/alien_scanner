#ifndef SWARM_CONTROLLER_GLOBALTASKALLOCATOR_HPP
#define SWARM_CONTROLLER_GLOBALTASKALLOCATOR_HPP

#include "swarm_controller/ExplorationTask.hpp"
#include "swarm_controller/GlobalFrontierDetector.hpp"
#include "swarm_controller/KnownFreePathChecker.hpp"
#include "swarm_controller/Pose3D.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace SwarmController {

    enum class CoordinationMode {
        LocalFallback,
        Coordinated,
    };

    struct GlobalTaskAllocatorConfig {
        std::size_t min_persistence_updates {3U};
        double      min_persistence_seconds {2.0};
        std::size_t missed_update_grace {2U};
        float       track_match_distance {1.0F};
        float       min_direction_similarity {0.5F};
        std::size_t max_tracks {128U};
        std::size_t max_eligible_edges {4096U};
        std::size_t max_assignment_dimension {64U};

        float  max_assignment_distance {8.0F};
        float  first_hop_distance {1.0F};
        float  entry_backward_margin {0.1F};
        std::int64_t information_gain_scale {100};
        std::int64_t distance_cost_scale {100};
        std::int64_t heading_cost_scale {50};
        std::int64_t owner_bonus {2'000};
        std::int64_t switch_margin {250};

        std::size_t activation_min_regions {2U};
        std::size_t activation_min_drones {2U};
        std::size_t activation_updates {2U};
        double      activation_seconds {1.0};
        double      deactivation_grace_seconds {2.0};

        float  min_owner_progress {0.30F};
        double no_progress_timeout_seconds {35.0};
        double failed_task_cooldown_seconds {5.0};
        float  target_update_threshold {0.30F};
        BodyEnvelopeConfig body_envelope {};
    };

    struct DroneAllocationState {
        std::string id;
        Pose3D      pose {};
        const octomap::OcTree * local_map {nullptr};
        bool        odom_fresh {false};
        bool        local_map_fresh {false};
    };

    struct GlobalAllocationInput {
        std::vector<FrontierRegion> regions;
        std::vector<DroneAllocationState> drones;
        std::uint64_t global_update_sequence {};
        double        monotonic_time_seconds {};
        bool          healthy {false};
    };

    struct TrackedFrontierRegion {
        std::uint64_t task_id {};
        FrontierRegion region;
        bool stable {false};
        std::size_t persistence_updates {};
        std::size_t missed_updates {};
    };

    struct DroneTaskAssignment {
        std::string         drone_id;
        ExplorationTaskMode mode {ExplorationTaskMode::LocalFallback};
        std::uint64_t       task_id {};
        std::uint64_t       revision {1U};
        Point3f             target {};
        std::string         reason;
    };

    enum class GlobalAllocationStatus {
        Accepted,
        InvalidInput,
        ResourceLimit,
    };

    struct GlobalAllocationResult {
        GlobalAllocationStatus status {GlobalAllocationStatus::InvalidInput};
        CoordinationMode       coordination_mode {CoordinationMode::LocalFallback};
        std::vector<TrackedFrontierRegion> tracks;
        std::vector<DroneTaskAssignment> assignments;
        std::size_t eligible_edges {};
        std::size_t matching_cardinality {};
        std::string reason;

        bool accepted() const
        {
            return status == GlobalAllocationStatus::Accepted;
        }
    };

    class GlobalTaskAllocator
    {
    public:
        explicit GlobalTaskAllocator(GlobalTaskAllocatorConfig config = {});

        GlobalAllocationResult update(const GlobalAllocationInput & input);
        const GlobalTaskAllocatorConfig & config() const;

    private:
        struct Track {
            std::uint64_t id {};
            FrontierRegion region;
            std::size_t persistence_updates {};
            std::size_t missed_updates {};
            double first_seen_time {};
            double last_seen_time {};
            bool stable {false};
        };

        struct DroneRuntime {
            std::optional<Pose3D> entry_pose;
            float max_entry_forward_progress {};
            DroneTaskAssignment last_assignment;
            bool has_assignment {false};
            Pose3D progress_start_pose {};
            Point3f progress_direction {};
            Point3f progress_target {};
            double progress_start_time {};
            std::uint64_t progress_task_id {};
            std::uint64_t failed_task_id {};
            double failed_until {};
        };

        struct EligibleEdge;

        GlobalAllocationResult fallbackResult(
                const std::vector<DroneAllocationState> & drones,
                GlobalAllocationStatus status, const std::string & reason);
        bool updateTracks(const GlobalAllocationInput & input, std::string & reason);
        std::vector<TrackedFrontierRegion> trackedRegions() const;
        bool pairEligible(
                const DroneAllocationState & drone, const Track & track,
                DroneRuntime & runtime, Point3f & first_hop) const;
        std::int64_t edgeUtility(
                const DroneAllocationState & drone, const Track & track,
                const DroneRuntime & runtime) const;
        DroneTaskAssignment semanticAssignment(
                const std::string & drone_id, ExplorationTaskMode mode,
                std::uint64_t task_id, const Point3f & target,
                const std::string & reason);
        void updateProgress(
                const DroneAllocationState & drone, const DroneTaskAssignment & assignment,
                double now_seconds);

        GlobalTaskAllocatorConfig config_;
        KnownFreePathChecker      path_checker_;
        std::vector<Track>        tracks_;
        std::unordered_map<std::string, DroneRuntime> drone_runtime_;
        std::uint64_t next_task_id_ {1U};
        std::uint64_t last_global_update_sequence_ {};
        CoordinationMode coordination_mode_ {CoordinationMode::LocalFallback};
        std::size_t activation_update_count_ {};
        double activation_start_time_ {-1.0};
        double deactivation_start_time_ {-1.0};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_GLOBALTASKALLOCATOR_HPP

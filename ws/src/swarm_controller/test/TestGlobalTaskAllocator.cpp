#include "swarm_controller/GlobalTaskAllocator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>

namespace SwarmController {

    namespace {

        std::shared_ptr<octomap::OcTree> freeMap()
        {
            auto tree = std::make_shared<octomap::OcTree>(0.1);
            const octomap::OcTreeKey minimum =
                    tree->coordToKey(-1.0, -3.0, 0.5);
            const octomap::OcTreeKey maximum =
                    tree->coordToKey(4.0, 3.0, 2.5);
            for(std::uint32_t x = minimum[0]; x <= maximum[0]; ++x) {
                for(std::uint32_t y = minimum[1]; y <= maximum[1]; ++y) {
                    for(std::uint32_t z = minimum[2]; z <= maximum[2]; ++z) {
                        tree->updateNode(
                                octomap::OcTreeKey {
                                        static_cast<octomap::key_type>(x),
                                        static_cast<octomap::key_type>(y),
                                        static_cast<octomap::key_type>(z)},
                                false, true);
                    }
                }
            }
            tree->updateInnerOccupancy();
            return tree;
        }

        FrontierRegion region(
                const std::int64_t key, const float x, const float y,
                const float direction_y)
        {
            FrontierRegion result;
            result.stable_key = FrontierColumnKey {key, key};
            result.columns = {
                    FrontierColumnKey {key, key}, FrontierColumnKey {key + 1, key},
                    FrontierColumnKey {key, key + 1}, FrontierColumnKey {key + 1, key + 1},
            };
            result.representative = Point3f {x, y, 1.5F};
            result.unknown_direction = Point3f {1.0F, direction_y, 0.0F};
            result.information_gain = 100.0F;
            result.area = 1.0F;
            result.horizontal_span = 1.0F;
            result.direction_consistency = 1.0F;
            return result;
        }

        DroneAllocationState drone(
                const std::string & id, const float x, const float y,
                const std::shared_ptr<octomap::OcTree> & map)
        {
            DroneAllocationState state;
            state.id = id;
            state.pose.position = Point3f {x, y, 1.5F};
            state.pose.yaw = 0.0F;
            state.local_map = map.get();
            state.odom_fresh = true;
            state.local_map_fresh = true;
            return state;
        }

        GlobalTaskAllocatorConfig immediateConfig()
        {
            GlobalTaskAllocatorConfig config;
            config.min_persistence_updates = 1U;
            config.min_persistence_seconds = 0.0;
            config.activation_updates = 1U;
            config.activation_seconds = 0.0;
            config.deactivation_grace_seconds = 0.0;
            config.first_hop_distance = 0.5F;
            config.body_envelope.robot_radius = 0.05F;
            config.body_envelope.robot_half_height = 0.05F;
            config.body_envelope.safety_margin = 0.05F;
            config.body_envelope.vertical_margin = 0.05F;
            return config;
        }

        std::map<std::string, DroneTaskAssignment> byDrone(
                const GlobalAllocationResult & result)
        {
            std::map<std::string, DroneTaskAssignment> indexed;
            for(const auto & assignment : result.assignments) {
                indexed.emplace(assignment.drone_id, assignment);
            }
            return indexed;
        }

    }// namespace

    TEST(GlobalTaskAllocatorTest, RejectsInvalidConfiguration)
    {
        auto config = immediateConfig();
        config.activation_min_regions = 1U;
        EXPECT_THROW({ GlobalTaskAllocator allocator {config}; }, std::invalid_argument);
    }

    TEST(GlobalTaskAllocatorTest, SingleRegionKeepsEveryDroneInLocalFallback)
    {
        const auto map = freeMap();
        KnownFreePathChecker checker(immediateConfig().body_envelope);
        ASSERT_TRUE(checker.checkBody(*map, Point3f {0.0F, -0.8F, 1.5F}).safe());
        ASSERT_TRUE(checker.checkBody(*map, Point3f {0.5F, -0.8F, 1.5F}).safe());
        GlobalTaskAllocator allocator(immediateConfig());
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, 0.0F, 0.0F)};
        input.drones = {drone("drone_0", 0.0F, -0.5F, map), drone("drone_1", 0.0F, 0.5F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        const auto result = allocator.update(input);
        ASSERT_TRUE(result.accepted());
        EXPECT_EQ(result.coordination_mode, CoordinationMode::LocalFallback);
        for(const auto & assignment : result.assignments) {
            EXPECT_EQ(assignment.mode, ExplorationTaskMode::LocalFallback);
        }
    }

    TEST(GlobalTaskAllocatorTest, TwoRegionsActivateUniqueAssignments)
    {
        const auto map = freeMap();
        GlobalTaskAllocator allocator(immediateConfig());
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        const auto result = allocator.update(input);
        ASSERT_GE(result.eligible_edges, 2U);
        ASSERT_EQ(result.coordination_mode, CoordinationMode::Coordinated);
        ASSERT_EQ(result.matching_cardinality, 2U);
        const auto indexed = byDrone(result);
        ASSERT_EQ(indexed.size(), 2U);
        EXPECT_EQ(indexed.at("drone_0").mode, ExplorationTaskMode::Assigned);
        EXPECT_EQ(indexed.at("drone_1").mode, ExplorationTaskMode::Assigned);
        EXPECT_NE(indexed.at("drone_0").task_id, indexed.at("drone_1").task_id);
    }

    TEST(GlobalTaskAllocatorTest, ExtraEligibleDroneStandsBy)
    {
        const auto map = freeMap();
        GlobalTaskAllocator allocator(immediateConfig());
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {
                drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map),
                drone("drone_2", 0.0F, 0.0F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        const auto allocation = allocator.update(input);
        ASSERT_GE(allocation.eligible_edges, 2U);
        const auto indexed = byDrone(allocation);
        const auto standby_count = std::count_if(
                indexed.begin(), indexed.end(), [](const auto & entry) {
                    return entry.second.mode == ExplorationTaskMode::Standby;
                });
        EXPECT_EQ(standby_count, 1);
    }

    TEST(GlobalTaskAllocatorTest, DroneWithoutEligibleEdgeContinuesLocalFallback)
    {
        const auto map = freeMap();
        GlobalTaskAllocator allocator(immediateConfig());
        auto unavailable = drone("drone_2", 0.0F, 0.0F, map);
        unavailable.local_map = nullptr;
        unavailable.local_map_fresh = false;
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {
                drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map),
                unavailable};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        const auto indexed = byDrone(allocator.update(input));
        EXPECT_EQ(indexed.at("drone_2").mode, ExplorationTaskMode::LocalFallback);
    }

    TEST(GlobalTaskAllocatorTest, PersistenceAndInputOrderAreDeterministic)
    {
        const auto map = freeMap();
        auto config = immediateConfig();
        config.min_persistence_updates = 2U;
        config.min_persistence_seconds = 1.0;
        GlobalAllocationInput first;
        first.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        first.drones = {drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map)};
        first.global_update_sequence = 1U;
        first.monotonic_time_seconds = 0.0;
        first.healthy = true;

        GlobalTaskAllocator ordered(config);
        EXPECT_EQ(ordered.update(first).coordination_mode, CoordinationMode::LocalFallback);
        auto second = first;
        second.global_update_sequence = 2U;
        second.monotonic_time_seconds = 1.0;
        const auto ordered_result = byDrone(ordered.update(second));

        GlobalTaskAllocator reversed(config);
        std::reverse(first.regions.begin(), first.regions.end());
        std::reverse(first.drones.begin(), first.drones.end());
        EXPECT_EQ(reversed.update(first).coordination_mode, CoordinationMode::LocalFallback);
        auto reversed_second = first;
        reversed_second.global_update_sequence = 2U;
        reversed_second.monotonic_time_seconds = 1.0;
        const auto reversed_result = byDrone(reversed.update(reversed_second));

        ASSERT_EQ(ordered_result.size(), reversed_result.size());
        for(const auto & [id, assignment] : ordered_result) {
            EXPECT_EQ(assignment.mode, reversed_result.at(id).mode);
            EXPECT_EQ(assignment.task_id, reversed_result.at(id).task_id);
        }
    }

    TEST(GlobalTaskAllocatorTest, UnhealthyInputExplicitlyFallsBack)
    {
        const auto map = freeMap();
        GlobalTaskAllocator allocator(immediateConfig());
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = false;
        const auto result = allocator.update(input);
        EXPECT_EQ(result.coordination_mode, CoordinationMode::LocalFallback);
        for(const auto & assignment : result.assignments) {
            EXPECT_EQ(assignment.mode, ExplorationTaskMode::LocalFallback);
        }
    }

    TEST(GlobalTaskAllocatorTest, StaleOdometryDoesNotLatchDefaultEntryPose)
    {
        const auto map = freeMap();
        GlobalTaskAllocator allocator(immediateConfig());
        GlobalAllocationInput input;
        input.regions = {
                region(10, 3.0F, -1.0F, -1.0F),
                region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {
                drone("drone_0", 0.0F, 0.0F, map),
                drone("drone_1", 0.0F, 0.0F, map)};
        for(auto & state : input.drones) {
            state.odom_fresh = false;
        }
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        EXPECT_EQ(
                allocator.update(input).coordination_mode,
                CoordinationMode::LocalFallback);

        input.drones = {
                drone("drone_0", 2.0F, -0.8F, map),
                drone("drone_1", 2.0F, 0.8F, map)};
        for(auto & state : input.drones) {
            state.pose.yaw = 3.14159265358979323846F;
        }
        input.global_update_sequence = 2U;
        input.monotonic_time_seconds = 1.0;
        const auto result = allocator.update(input);

        EXPECT_EQ(result.eligible_edges, 0U);
        EXPECT_EQ(result.coordination_mode, CoordinationMode::LocalFallback);
    }

    TEST(GlobalTaskAllocatorTest, AdvancedNoRetreatPlaneRejectsRegionsBehindProgress)
    {
        const auto map = freeMap();
        auto config = immediateConfig();
        config.missed_update_grace = 0U;
        GlobalTaskAllocator allocator(config);
        GlobalAllocationInput input;
        input.regions = {
                region(10, 3.0F, -1.0F, -1.0F),
                region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {
                drone("drone_0", 0.0F, -0.8F, map),
                drone("drone_1", 0.0F, 0.8F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        ASSERT_EQ(
                allocator.update(input).coordination_mode,
                CoordinationMode::Coordinated);

        input.regions = {
                region(50, 1.5F, -1.0F, -1.0F),
                region(70, 1.5F, 1.0F, 1.0F)};
        input.drones = {
                drone("drone_0", 2.5F, -0.8F, map),
                drone("drone_1", 2.5F, 0.8F, map)};
        input.global_update_sequence = 2U;
        input.monotonic_time_seconds = 1.0;
        const auto result = allocator.update(input);

        EXPECT_EQ(result.eligible_edges, 0U);
        EXPECT_EQ(result.coordination_mode, CoordinationMode::LocalFallback);
    }

    TEST(GlobalTaskAllocatorTest, ResourceLimitDoesNotCommitPartialTracks)
    {
        const auto map = freeMap();
        auto config = immediateConfig();
        config.max_tracks = 1U;
        GlobalTaskAllocator allocator(config);
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;

        const auto rejected = allocator.update(input);
        EXPECT_EQ(rejected.status, GlobalAllocationStatus::ResourceLimit);
        EXPECT_TRUE(rejected.tracks.empty());

        input.regions = {region(10, 3.0F, -1.0F, -1.0F)};
        input.global_update_sequence = 2U;
        const auto accepted = allocator.update(input);
        ASSERT_TRUE(accepted.accepted());
        ASSERT_EQ(accepted.tracks.size(), 1U);
        EXPECT_EQ(accepted.tracks.front().task_id, 1U);
    }

    TEST(GlobalTaskAllocatorTest, SwitchMarginKeepsOwnersAcrossSmallUtilityCrossing)
    {
        const auto map = freeMap();
        auto config = immediateConfig();
        config.owner_bonus = 0;
        config.switch_margin = 500;
        GlobalTaskAllocator allocator(config);
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {drone("drone_0", 0.0F, -0.2F, map), drone("drone_1", 0.0F, 0.2F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        const auto first = byDrone(allocator.update(input));
        ASSERT_EQ(first.at("drone_0").mode, ExplorationTaskMode::Assigned);
        ASSERT_EQ(first.at("drone_1").mode, ExplorationTaskMode::Assigned);

        input.drones = {drone("drone_0", 0.0F, 0.2F, map), drone("drone_1", 0.0F, -0.2F, map)};
        input.global_update_sequence = 2U;
        input.monotonic_time_seconds = 1.0;
        const auto second = byDrone(allocator.update(input));

        EXPECT_EQ(second.at("drone_0").task_id, first.at("drone_0").task_id);
        EXPECT_EQ(second.at("drone_1").task_id, first.at("drone_1").task_id);
    }

    TEST(GlobalTaskAllocatorTest, TargetUpdateRefreshesProgressDirectionWithoutExtendingTimeout)
    {
        const auto map = freeMap();
        auto config = immediateConfig();
        config.no_progress_timeout_seconds = 2.0;
        config.failed_task_cooldown_seconds = 10.0;
        GlobalTaskAllocator allocator(config);
        GlobalAllocationInput input;
        input.regions = {region(10, 3.0F, -1.0F, -1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones = {drone("drone_0", 0.0F, -0.8F, map), drone("drone_1", 0.0F, 0.8F, map)};
        input.global_update_sequence = 1U;
        input.monotonic_time_seconds = 0.0;
        input.healthy = true;
        const auto initial = byDrone(allocator.update(input));
        const auto initial_task = initial.at("drone_0").task_id;
        ASSERT_NE(initial_task, 0U);

        input.regions = {region(10, 0.0F, 2.0F, 1.0F), region(30, 3.0F, 1.0F, 1.0F)};
        input.drones[0] = drone("drone_0", 0.0F, -0.4F, map);
        input.drones[1] = drone("drone_1", 0.4F, 0.8F, map);
        input.global_update_sequence = 2U;
        input.monotonic_time_seconds = 1.0;
        EXPECT_EQ(byDrone(allocator.update(input)).at("drone_0").task_id, initial_task);

        input.drones[0] = drone("drone_0", 0.0F, 0.0F, map);
        input.drones[1] = drone("drone_1", 0.8F, 0.8F, map);
        input.global_update_sequence = 3U;
        input.monotonic_time_seconds = 1.5;
        EXPECT_EQ(byDrone(allocator.update(input)).at("drone_0").task_id, initial_task);

        input.global_update_sequence = 4U;
        input.monotonic_time_seconds = 3.1;
        EXPECT_EQ(byDrone(allocator.update(input)).at("drone_0").task_id, initial_task);
        input.global_update_sequence = 5U;
        input.monotonic_time_seconds = 3.2;
        EXPECT_EQ(byDrone(allocator.update(input)).at("drone_0").task_id, initial_task);
    }

}// namespace SwarmController

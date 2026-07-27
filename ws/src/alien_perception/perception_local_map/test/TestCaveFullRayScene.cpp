#include "cave_world/TreeCaveField.hpp"
#include "drone_scanner/FakeLidar.hpp"
#include "drone_scanner/LaserScanProjection.hpp"
#include "drone_scanner/Pose3D.hpp"
#include "perception_core/health/MapperContractFingerprint.hpp"
#include "perception_local_map/LocalObservationMapper.hpp"
#include "perception_local_map/OctoMapBackend.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace PerceptionLocalMap::Test {
    namespace {

        struct SceneConfig {
            CaveWorld::TreeCaveFieldConfig cave;
            double start_x {};
            double end_x {};
            double y {};
            double z {};
            std::size_t beam_count {};
            double scan_rate_hz {};
            double range_min_m {};
            double range_max_m {};
            Eigen::Quaterniond body_from_scan = Eigen::Quaterniond::Identity();
            std::string map_frame;
            std::string body_frame;
            std::string scan_frame;
            std::string sensor_id;
            std::string pose_source_id;
            std::string clock_domain;
            double pose_timeout_s {};
            double minimum_pose_quality {};
            std::size_t recovery_stability_samples {};
            double resolution_m {};
            Eigen::Vector3d lattice_origin = Eigen::Vector3d::Zero();
        };

        SceneConfig load_scene_config()
        {
            const YAML::Node root = YAML::LoadFile(CAVE_FULL_RAY_SCENE_CONFIG_PATH);
            const auto cave = root["cave"];
            const auto tree = cave["tree"];
            const auto trajectory = root["trajectory"];
            const auto scan = root["scan"];
            const auto frames = root["frames"];
            const auto contract = root["sensor_contract"];
            const auto map = root["map"];

            SceneConfig config;
            config.cave.approach_length = tree["approach_length_m"].as<float>();
            config.cave.base_radius = cave["base_radius_m"].as<float>();
            config.cave.n_segments = cave["n_segments"].as<int>();
            config.cave.density = cave["density"].as<int>();
            config.cave.noise_scale = cave["noise_scale"].as<float>();
            config.cave.seed = cave["seed"].as<std::uint32_t>();
            config.cave.loop_yaw = tree["loop_yaw_rad"].as<float>();
            config.cave.loop_direct_length = tree["loop_direct_length_m"].as<float>();
            config.cave.loop_bulge = tree["loop_bulge_m"].as<float>();
            config.cave.exit1_length = tree["exit1_length_m"].as<float>();
            config.cave.right_yaw = tree["right_yaw_rad"].as<float>();
            config.cave.right_corridor_length = tree["right_corridor_length_m"].as<float>();
            config.cave.exit_yaw_spread = tree["exit_yaw_spread_rad"].as<float>();
            config.cave.exit_arm_length = tree["exit_arm_length_m"].as<float>();
            config.cave.vertical_step = tree["vertical_step_rad"].as<float>();
            config.cave.asymmetry = tree["asymmetry"].as<float>();
            config.cave.chamber_on_approach = tree["chamber_on_approach"].as<bool>();
            config.cave.chamber_at = tree["chamber_at"].as<float>();
            config.cave.chamber_scale = tree["chamber_scale"].as<float>();

            const auto start = trajectory["start_m"];
            const auto end = trajectory["end_m"];
            config.start_x = start[0].as<double>();
            config.end_x = end[0].as<double>();
            config.y = start[1].as<double>();
            config.z = start[2].as<double>();
            config.beam_count = scan["beam_count"].as<std::size_t>();
            config.scan_rate_hz = scan["rate_hz"].as<double>();
            config.range_min_m = scan["range_min_m"].as<double>();
            config.range_max_m = scan["range_max_m"].as<double>();
            const auto quaternion = scan["body_from_scan_quaternion_xyzw"];
            config.body_from_scan = Eigen::Quaterniond(
                    quaternion[3].as<double>(), quaternion[0].as<double>(),
                    quaternion[1].as<double>(), quaternion[2].as<double>());
            config.map_frame = frames["map"].as<std::string>();
            config.body_frame = frames["body"].as<std::string>();
            config.scan_frame = frames["scan"].as<std::string>();
            config.sensor_id = contract["sensor_id"].as<std::string>();
            config.pose_source_id = contract["pose_source_id"].as<std::string>();
            config.clock_domain = contract["clock_domain"].as<std::string>();
            config.pose_timeout_s = contract["pose_timeout_s"].as<double>();
            config.minimum_pose_quality = contract["minimum_pose_quality"].as<double>();
            config.recovery_stability_samples =
                    contract["recovery_stability_samples"].as<std::size_t>();
            config.resolution_m = map["resolution_m"].as<double>();
            const auto origin = map["lattice_origin_m"];
            config.lattice_origin = Eigen::Vector3d(
                    origin[0].as<double>(), origin[1].as<double>(), origin[2].as<double>());
            return config;
        }

        MapperConfig mapper_config(
                const SceneConfig & scene,
                const DroneScanner::ProjectedLaserScan & metadata)
        {
            MapperConfig config;
            config.vehicle_id = "cave-scene-vehicle";
            config.mapper_session = {100, 7};
            config.geometry = {
                    scene.resolution_m,
                    {scene.lattice_origin.x(), scene.lattice_origin.y(), scene.lattice_origin.z()},
                    scene.map_frame};
            config.body_frame = scene.body_frame;
            config.sensor_descriptors = {
                    Perception::SensorDescriptor {
                            Perception::SensorID {scene.sensor_id},
                            Perception::SensorType::LIDAR_2D,
                            scene.scan_frame,
                            Eigen::Vector3d::Zero(),
                            scene.body_from_scan,
                            Perception::FieldOfView {
                                    metadata.angle_min_rad, metadata.angle_max_rad, 0.0, 0.0},
                            metadata.angle_increment_rad,
                            scene.range_min_m,
                            scene.range_max_m,
                            Perception::RayEvidenceCapability::FullRay}};
            config.input_contract.requires_pose = true;
            config.input_contract.expected_pose_frame = scene.map_frame;
            config.input_contract.pose_freshness_threshold =
                    Perception::Duration::from_seconds(scene.pose_timeout_s);
            config.input_contract.minimum_pose_quality = scene.minimum_pose_quality;
            config.input_contract.recovery_stability_samples = scene.recovery_stability_samples;
            const Perception::SensorRequirement requirement {
                    Perception::SensorType::LIDAR_2D,
                    1,
                    {},
                    Perception::RayEvidenceCapability::FullRay};
            config.input_contract.minimum_viable = {requirement};
            config.input_contract.degraded_combinations = {
                    Perception::DegradedCombination {{requirement}, "canonical FullRay lidar"}};
            config.sensor_validity_ns = 60'000'000'000LL;
            config.health_validity_ns = 60'000'000'000LL;
            config.pose_receive_validity_ns = 60'000'000'000LL;
            config.map_freshness_ns = 60'000'000'000LL;
            return config;
        }

        MapPoint cell_center(const VoxelIndex & index, const MapGeometry & geometry)
        {
            return {
                    geometry.lattice_origin.x
                            + (static_cast<double>(index.x) + 0.5) * geometry.resolution_m,
                    geometry.lattice_origin.y
                            + (static_cast<double>(index.y) + 0.5) * geometry.resolution_m,
                    geometry.lattice_origin.z
                            + (static_cast<double>(index.z) + 0.5) * geometry.resolution_m};
        }

    }// namespace

    TEST(CaveFullRayScene, CanonicalTrajectoryBuildsQueryableThreeStateMap)
    {
        const SceneConfig scene = load_scene_config();
        auto field = std::make_shared<CaveWorld::TreeCaveField>(scene.cave);
        DroneScanner::FakeLidar lidar(
                field,
                {static_cast<int>(scene.beam_count), static_cast<float>(scene.range_max_m), 0.0F, 0U, 0.0F});
        DroneScanner::LaserScanProjection projection(
                {scene.beam_count, static_cast<float>(scene.range_min_m),
                 static_cast<float>(scene.range_max_m), scene.scan_rate_hz});
        const auto metadata = projection.metadata();
        const MapperConfig config = mapper_config(scene, metadata);
        LocalObservationMapper mapper(
                config,
                [](const MapGeometry & geometry) {
                    return std::make_unique<OctoMapBackend>(geometry);
                });

        const Perception::SessionID producer_session {5, 6};
        ASSERT_TRUE(mapper.submit_health(
                {"perception_input", producer_session,
                 Perception::MapperContractFingerprint::compute(
                         config.sensor_descriptors, config.input_contract),
                 Perception::HealthState::Healthy, 0, 60'000'000'000LL}));

        const SensorExtrinsicSample extrinsic {
                Perception::SensorID {scene.sensor_id}, producer_session,
                scene.scan_frame, Eigen::Vector3d::Zero(), scene.body_from_scan};
        std::size_t applied_count = 0U;
        std::optional<MapPoint> occupied_oracle;
        constexpr std::size_t kPoseCount = 11U;

        for(std::size_t index = 0; index < kPoseCount; ++index) {
            const double fraction = static_cast<double>(index) / static_cast<double>(kPoseCount - 1U);
            const double x = scene.start_x + fraction * (scene.end_x - scene.start_x);
            const std::int64_t stamp_ns = static_cast<std::int64_t>(index + 1U) * 100'000'000LL;
            const std::int64_t receive_ns = stamp_ns;
            Perception::PoseEstimate pose {
                    Perception::SourceID {scene.pose_source_id}, producer_session, scene.map_frame,
                    scene.clock_domain, Perception::Timestamp {stamp_ns},
                    Eigen::Vector3d(x, scene.y, scene.z), Eigen::Quaterniond::Identity(),
                    std::nullopt, 1.0, Perception::Duration {0}, 0};
            ASSERT_EQ(mapper.submit_pose(pose, receive_ns).status, InputStatus::Accepted);

            DroneScanner::Pose3D lidar_pose;
            lidar_pose.x = static_cast<float>(x);
            lidar_pose.y = static_cast<float>(scene.y);
            lidar_pose.z = static_cast<float>(scene.z);
            const auto returns = lidar.scanReturns(lidar_pose);
            const auto projected = projection.project(returns);
            ASSERT_TRUE(projected.has_value());
            ASSERT_EQ(projected->ranges.size(), scene.beam_count);

            if(index == kPoseCount / 2U) {
                const std::size_t positive_body_y_index = scene.beam_count / 2U;
                ASSERT_TRUE(std::isfinite(projected->ranges[positive_body_y_index]));
                occupied_oracle = MapPoint {
                        x,
                        scene.y + projected->ranges[positive_body_y_index],
                        scene.z};
            }

            Perception::LidarObservation observation {
                    Perception::SensorID {scene.sensor_id}, producer_session,
                    scene.scan_frame, scene.clock_domain, Perception::Timestamp {stamp_ns},
                    Perception::Scan2D {
                            projected->angle_min_rad, projected->angle_max_rad,
                            projected->angle_increment_rad, projected->range_min_m,
                            projected->range_max_m, std::move(projected->ranges), {}},
                    Perception::RayEvidenceCapability::FullRay};
            const auto result = mapper.submit_observation(observation, extrinsic, receive_ns + 1);
            if(result.status == InputStatus::Applied) {
                ++applied_count;
            } else {
                EXPECT_EQ(result.status, InputStatus::Unavailable) << result.diagnostic;
            }
        }

        EXPECT_GE(applied_count, kPoseCount - scene.recovery_stability_samples);
        auto acquired = mapper.acquire_read_transaction();
        ASSERT_EQ(acquired.status, AcquireStatus::Ready);
        ASSERT_TRUE(acquired.transaction.has_value());
        auto & transaction = acquired.transaction.value();
        const auto map_metadata = transaction.metadata();
        ASSERT_TRUE(map_metadata.has_value());
        ASSERT_TRUE(map_metadata->known_bounds.has_value());
        EXPECT_GE(
                map_metadata->known_bounds->maximum.x - map_metadata->known_bounds->minimum.x,
                9.0);

        const MapPoint free_point {6.0, 0.0, 1.5};
        const MapPoint unknown_point {6.0, 0.0, 10.0};
        ASSERT_TRUE(occupied_oracle.has_value());
        const auto occupied_index = quantize_point(occupied_oracle.value(), map_metadata->geometry);
        ASSERT_EQ(occupied_index.status, QueryStatus::Ok);
        const MapPoint occupied_point = cell_center(occupied_index.index, map_metadata->geometry);
        EXPECT_EQ(transaction.query(free_point).state, OccupancyState::Free);
        EXPECT_EQ(transaction.query(occupied_point).state, OccupancyState::Occupied);
        EXPECT_EQ(transaction.query(unknown_point).state, OccupancyState::Unknown);

        const auto cross_section = transaction.query_region(
                {{5.8, -4.0, -2.0}, {6.2, 4.0, 5.0}});
        ASSERT_EQ(cross_section.status, QueryStatus::Ok);
        std::size_t occupied_cells = 0U;
        std::size_t free_cells = 0U;
        for(const auto & cell : cross_section.cells) {
            occupied_cells += cell.state == OccupancyState::Occupied ? 1U : 0U;
            free_cells += cell.state == OccupancyState::Free ? 1U : 0U;
        }
        EXPECT_GT(occupied_cells, 0U);
        EXPECT_GT(free_cells, 0U);
    }

}// namespace PerceptionLocalMap::Test

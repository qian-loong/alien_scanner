#ifndef PERCEPTION_PROFILING_PROFILE_SCENARIO_HPP
#define PERCEPTION_PROFILING_PROFILE_SCENARIO_HPP

#include "perception_core/health/MapperContractFingerprint.hpp"
#include "perception_core/observation/lidar_observation.hpp"
#include "perception_core/observation/pose_estimate.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"
#include "perception_local_map/LocalObservationMapper.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace PerceptionProfiling {

    enum class ScenarioMode : std::uint8_t
    {
        Canonical,
        Bounded,
        Expanding
    };

    struct FrozenProfileConfig {
        static constexpr std::size_t kBeamCount = 360U;
        static constexpr std::int64_t kObservationPeriodNs = 100'000'000LL;
        static constexpr std::int64_t kPoseLeadNs = 50'000'000LL;

        std::string map_frame = "map";
        std::string body_frame = "base_link";
        std::string sensor_frame = "profile_scan_link";
        std::string sensor_id = "profile_scan";
        std::string pose_source_id = "profile_odom";
        std::string producer_source_id = "perception_profile_fixture";
        std::string clock_domain = "profile_clock";
        std::string vehicle_id = "profile-vehicle";
        Perception::SessionID producer_session {10'000'000'000ULL, 0xC2F1U};
        std::int64_t first_observation_stamp_ns = 10'000'000'000LL;
        double range_min_m = 0.1;
        double range_max_m = 30.0;
        double resolution_m = 0.2;
        double start_x_m = 1.0;
        double end_x_m = 11.0;
        double speed_mps = 0.5;
        double body_y_m = 0.0;
        double body_z_m = 1.5;
        double tunnel_y_radius_m = 2.5;
        double tunnel_z_radius_m = 2.0;
        std::size_t no_return_first_index = 255U;
        std::size_t no_return_last_index = 285U;
        Eigen::Quaterniond body_from_sensor = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
    };

    struct ScenarioSample {
        std::uint64_t sequence = 0U;
        Perception::PoseEstimate observation_pose;
        Perception::PoseEstimate lead_pose;
        Perception::LidarObservation observation;
        std::string payload_digest;
    };

    class ProfileScenario
    {
    public:
        explicit ProfileScenario(ScenarioMode mode);

        ScenarioMode mode() const noexcept;
        const FrozenProfileConfig & config() const noexcept;
        std::optional<std::uint64_t> canonical_sequence_limit() const noexcept;
        ScenarioSample sample(std::uint64_t sequence) const;
        std::optional<std::uint64_t> sequence_for_stamp(
                Perception::Timestamp stamp) const noexcept;

        Perception::SensorDescriptor sensor_descriptor() const;
        Perception::MapperInputContract input_contract() const;
        Perception::MapperContractFingerprint contract_fingerprint() const;
        PerceptionLocalMap::MapperConfig mapper_config(
                Perception::SessionID mapper_session) const;
        PerceptionLocalMap::SensorExtrinsicSample extrinsic() const;

        static ScenarioMode parse_mode(const std::string & value);
        static std::string mode_name(ScenarioMode mode);
        static std::string payload_digest(
                const Perception::LidarObservation & observation);

    private:
        struct CanonicalAssets;

        ScenarioMode mode_;
        FrozenProfileConfig config_;
        std::shared_ptr<const CanonicalAssets> canonical_assets_;

        double x_for_sequence(std::uint64_t sequence) const;
        Perception::PoseEstimate make_pose(
                std::uint64_t sequence,
                std::int64_t stamp_ns,
                double x_m) const;
    };

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_PROFILE_SCENARIO_HPP

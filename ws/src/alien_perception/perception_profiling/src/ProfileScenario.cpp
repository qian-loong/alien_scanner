#include "perception_profiling/ProfileScenario.hpp"

#include "cave_world/TreeCaveField.hpp"
#include "drone_scanner/FakeLidar.hpp"
#include "drone_scanner/LaserScanProjection.hpp"
#include "perception_core/observation/ray_evidence_capability.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace PerceptionProfiling {
    struct ProfileScenario::CanonicalAssets {
        static CaveWorld::TreeCaveFieldConfig cave_config()
        {
            CaveWorld::TreeCaveFieldConfig config;
            config.approach_length = 12.0F;
            config.base_radius = 2.5F;
            config.n_segments = 200;
            config.density = 400;
            config.noise_scale = 0.4F;
            config.seed = 42U;
            config.loop_yaw = 0.50F;
            config.loop_direct_length = 16.0F;
            config.loop_bulge = 12.0F;
            config.exit1_length = 14.0F;
            config.right_yaw = -0.12F;
            config.right_corridor_length = 10.0F;
            config.exit_yaw_spread = 0.35F;
            config.exit_arm_length = 14.0F;
            config.vertical_step = -0.10F;
            config.asymmetry = 0.22F;
            config.chamber_on_approach = false;
            config.chamber_at = 0.55F;
            config.chamber_scale = 2.2F;
            return config;
        }

        CanonicalAssets()
            : field(std::make_shared<CaveWorld::TreeCaveField>(cave_config()))
            , lidar(field, DroneScanner::FakeLidarConfig {360, 30.0F, 0.0F, 0U, 0.0F})
            , projection(DroneScanner::LaserScanProjectionConfig {
                      FrozenProfileConfig::kBeamCount, 0.1F, 30.0F, 10.0})
        {}

        std::shared_ptr<CaveWorld::TreeCaveField> field;
        DroneScanner::FakeLidar lidar;
        DroneScanner::LaserScanProjection projection;
    };

    namespace {

        constexpr double kPi = 3.141592653589793238462643383279502884;

        class Fnv1a64Writer
        {
        public:
            void append_u8(std::uint8_t value) noexcept
            {
                hash_ ^= value;
                hash_ *= 1099511628211ULL;
            }

            template<typename Integer>
            void append_integer(Integer value) noexcept
            {
                using Unsigned = std::make_unsigned_t<Integer>;
                const auto bits = static_cast<Unsigned>(value);
                for(std::size_t index = 0U; index < sizeof(bits); ++index) {
                    append_u8(static_cast<std::uint8_t>(bits >> (index * 8U)));
                }
            }

            void append_string(const std::string & value) noexcept
            {
                append_integer<std::uint64_t>(value.size());
                for(const unsigned char byte : value) {
                    append_u8(byte);
                }
            }

            void append_double(double value) noexcept
            {
                std::uint64_t bits = 0U;
                static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
                std::memcpy(&bits, &value, sizeof(bits));
                append_integer(bits);
            }

            void append_float(float value) noexcept
            {
                std::uint32_t bits = 0U;
                static_assert(sizeof(bits) == sizeof(value), "unexpected float width");
                std::memcpy(&bits, &value, sizeof(bits));
                append_integer(bits);
            }

            std::string hex_digest() const
            {
                std::ostringstream stream;
                stream << std::hex << std::setfill('0') << std::setw(16) << hash_;
                return stream.str();
            }

        private:
            std::uint64_t hash_ = 14695981039346656037ULL;
        };

        double scan_angle(std::size_t index)
        {
            return -kPi + static_cast<double>(index)
                                  * (2.0 * kPi / FrozenProfileConfig::kBeamCount);
        }

    }// namespace

    ProfileScenario::ProfileScenario(ScenarioMode mode)
        : mode_(mode)
    {
        if(config_.no_return_first_index > config_.no_return_last_index
           || config_.no_return_last_index >= FrozenProfileConfig::kBeamCount) {
            throw std::logic_error("frozen no-return sector is invalid");
        }
        if(mode_ == ScenarioMode::Canonical) {
            canonical_assets_ = std::make_shared<CanonicalAssets>();
        }
    }

    ScenarioMode ProfileScenario::mode() const noexcept
    {
        return mode_;
    }

    const FrozenProfileConfig & ProfileScenario::config() const noexcept
    {
        return config_;
    }

    std::optional<std::uint64_t> ProfileScenario::canonical_sequence_limit() const noexcept
    {
        if(mode_ == ScenarioMode::Canonical) {
            return 200U;
        }
        return std::nullopt;
    }

    double ProfileScenario::x_for_sequence(std::uint64_t sequence) const
    {
        if(sequence == 0U) {
            throw std::out_of_range("profile sequence is one-based");
        }
        const double elapsed_s = static_cast<double>(sequence - 1U)
                                 * static_cast<double>(FrozenProfileConfig::kObservationPeriodNs)
                                 / 1e9;
        const double distance_m = config_.speed_mps * elapsed_s;
        if(mode_ == ScenarioMode::Expanding) {
            return config_.start_x_m + distance_m;
        }
        if(mode_ == ScenarioMode::Canonical) {
            return std::min(config_.start_x_m + distance_m, config_.end_x_m);
        }

        const double span_m = config_.end_x_m - config_.start_x_m;
        const double phase_m = std::fmod(distance_m, 2.0 * span_m);
        return phase_m <= span_m
                       ? config_.start_x_m + phase_m
                       : config_.end_x_m - (phase_m - span_m);
    }

    Perception::PoseEstimate ProfileScenario::make_pose(
            std::uint64_t,
            std::int64_t stamp_ns,
            double x_m) const
    {
        return {
                Perception::SourceID {config_.pose_source_id}, config_.producer_session,
                config_.map_frame, config_.clock_domain,
                Perception::Timestamp::from_nanoseconds(stamp_ns),
                Eigen::Vector3d(x_m, config_.body_y_m, config_.body_z_m),
                Eigen::Quaterniond::Identity(), std::nullopt, 1.0,
                Perception::Duration::from_nanoseconds(0), 0U};
    }

    ScenarioSample ProfileScenario::sample(std::uint64_t sequence) const
    {
        if(sequence == 0U
           || (canonical_sequence_limit().has_value()
               && sequence > canonical_sequence_limit().value())) {
            throw std::out_of_range("profile sequence is outside the scenario");
        }
        if(sequence - 1U
           > static_cast<std::uint64_t>(
                   (std::numeric_limits<std::int64_t>::max()
                    - config_.first_observation_stamp_ns)
                   / FrozenProfileConfig::kObservationPeriodNs)) {
            throw std::overflow_error("profile sequence stamp overflows int64");
        }

        const std::int64_t stamp_ns = config_.first_observation_stamp_ns
                                      + static_cast<std::int64_t>(sequence - 1U)
                                                * FrozenProfileConfig::kObservationPeriodNs;
        const double x_m = x_for_sequence(sequence);
        const double next_x_m = x_for_sequence(sequence + 1U);
        ScenarioSample result;
        result.sequence = sequence;
        result.observation_pose = make_pose(sequence, stamp_ns, x_m);
        result.lead_pose = make_pose(
                sequence, stamp_ns + FrozenProfileConfig::kPoseLeadNs,
                0.5 * (x_m + next_x_m));

        Perception::Scan2D scan;
        if(mode_ == ScenarioMode::Canonical) {
            DroneScanner::Pose3D pose;
            pose.x = static_cast<float>(x_m);
            pose.y = static_cast<float>(config_.body_y_m);
            pose.z = static_cast<float>(config_.body_z_m);
            const auto projected = canonical_assets_->projection.project(
                    canonical_assets_->lidar.scanReturns(pose));
            if(!projected.has_value()) {
                throw std::runtime_error("canonical TreeCave scan projection failed");
            }
            scan.angle_min_rad = projected->angle_min_rad;
            scan.angle_max_rad = projected->angle_max_rad;
            scan.angle_increment_rad = projected->angle_increment_rad;
            scan.range_min_m = projected->range_min_m;
            scan.range_max_m = projected->range_max_m;
            scan.ranges = projected->ranges;
        }
        else {
            scan.angle_min_rad = scan_angle(0U);
            scan.angle_increment_rad = 2.0 * kPi / FrozenProfileConfig::kBeamCount;
            scan.angle_max_rad = scan_angle(FrozenProfileConfig::kBeamCount - 1U);
            scan.range_min_m = config_.range_min_m;
            scan.range_max_m = config_.range_max_m;
            scan.ranges.reserve(FrozenProfileConfig::kBeamCount);
            for(std::size_t index = 0U; index < FrozenProfileConfig::kBeamCount; ++index) {
                if(index >= config_.no_return_first_index
                   && index <= config_.no_return_last_index) {
                    scan.ranges.push_back(std::numeric_limits<float>::infinity());
                    continue;
                }
                const double angle = scan_angle(index);
                const double cosine = std::cos(angle);
                const double sine = std::sin(angle);
                const double inverse_range_squared =
                        cosine * cosine
                                / (config_.tunnel_y_radius_m * config_.tunnel_y_radius_m)
                        + sine * sine
                                  / (config_.tunnel_z_radius_m * config_.tunnel_z_radius_m);
                scan.ranges.push_back(
                        static_cast<float>(1.0 / std::sqrt(inverse_range_squared)));
            }
        }

        result.observation = {
                Perception::SensorID {config_.sensor_id}, config_.producer_session,
                config_.sensor_frame, config_.clock_domain,
                Perception::Timestamp::from_nanoseconds(stamp_ns), std::move(scan),
                Perception::RayEvidenceCapability::FullRay};
        result.payload_digest = payload_digest(result.observation);
        return result;
    }

    std::optional<std::uint64_t> ProfileScenario::sequence_for_stamp(
            Perception::Timestamp stamp) const noexcept
    {
        if(stamp.nanoseconds < config_.first_observation_stamp_ns) {
            return std::nullopt;
        }
        const auto delta = stamp.nanoseconds - config_.first_observation_stamp_ns;
        if(delta % FrozenProfileConfig::kObservationPeriodNs != 0) {
            return std::nullopt;
        }
        const auto sequence = static_cast<std::uint64_t>(
                                      delta / FrozenProfileConfig::kObservationPeriodNs)
                              + 1U;
        if(canonical_sequence_limit().has_value()
           && sequence > canonical_sequence_limit().value()) {
            return std::nullopt;
        }
        return sequence;
    }

    Perception::SensorDescriptor ProfileScenario::sensor_descriptor() const
    {
        const double increment = 2.0 * kPi / FrozenProfileConfig::kBeamCount;
        return {
                Perception::SensorID {config_.sensor_id},
                Perception::SensorType::LIDAR_2D,
                config_.sensor_frame,
                Eigen::Vector3d::Zero(),
                config_.body_from_sensor,
                Perception::FieldOfView {
                        -kPi,
                        -kPi + static_cast<double>(FrozenProfileConfig::kBeamCount - 1U)
                                       * increment,
                        0.0,
                        0.0},
                increment,
                config_.range_min_m,
                config_.range_max_m,
                Perception::RayEvidenceCapability::FullRay};
    }

    Perception::MapperInputContract ProfileScenario::input_contract() const
    {
        Perception::MapperInputContract contract;
        const Perception::SensorRequirement requirement {
                Perception::SensorType::LIDAR_2D,
                1U,
                {},
                Perception::RayEvidenceCapability::FullRay};
        contract.minimum_viable = {requirement};
        contract.degraded_combinations = {
                Perception::DegradedCombination {{requirement}, "profile FullRay lidar"}};
        contract.requires_pose = true;
        contract.pose_freshness_threshold = Perception::Duration::from_seconds(1.0);
        contract.expected_pose_frame = config_.map_frame;
        contract.minimum_pose_quality = 0.0;
        contract.recovery_stability_samples = 3U;
        return contract;
    }

    Perception::MapperContractFingerprint ProfileScenario::contract_fingerprint() const
    {
        return Perception::MapperContractFingerprint::compute(
                {sensor_descriptor()}, input_contract());
    }

    PerceptionLocalMap::MapperConfig ProfileScenario::mapper_config(
            Perception::SessionID mapper_session) const
    {
        PerceptionLocalMap::MapperConfig config;
        config.vehicle_id = config_.vehicle_id;
        config.mapper_session = mapper_session;
        config.geometry = {
                config_.resolution_m, {0.0, 0.0, 0.0}, config_.map_frame};
        config.body_frame = config_.body_frame;
        config.sensor_descriptors = {sensor_descriptor()};
        config.input_contract = input_contract();
        config.sensor_validity_ns = 2'000'000'000LL;
        config.health_validity_ns = 2'000'000'000LL;
        config.pose_receive_validity_ns = 2'000'000'000LL;
        config.map_freshness_ns = 2'000'000'000LL;
        config.pose_history_limit = 128U;
        return config;
    }

    PerceptionLocalMap::SensorExtrinsicSample ProfileScenario::extrinsic() const
    {
        return {
                Perception::SensorID {config_.sensor_id}, config_.producer_session,
                config_.sensor_frame, Eigen::Vector3d::Zero(), config_.body_from_sensor};
    }

    ScenarioMode ProfileScenario::parse_mode(const std::string & value)
    {
        if(value == "canonical") {
            return ScenarioMode::Canonical;
        }
        if(value == "bounded") {
            return ScenarioMode::Bounded;
        }
        if(value == "expanding") {
            return ScenarioMode::Expanding;
        }
        throw std::invalid_argument("scenario mode must be canonical, bounded, or expanding");
    }

    std::string ProfileScenario::mode_name(ScenarioMode mode)
    {
        switch(mode) {
            case ScenarioMode::Canonical:
                return "canonical";
            case ScenarioMode::Bounded:
                return "bounded";
            case ScenarioMode::Expanding:
                return "expanding";
        }
        throw std::invalid_argument("unknown scenario mode");
    }

    std::string ProfileScenario::payload_digest(
            const Perception::LidarObservation & observation)
    {
        Fnv1a64Writer writer;
        writer.append_string("alien-scanner/profile-observation/fnv1a64-v1");
        writer.append_string(observation.sensor_id.value);
        writer.append_integer(observation.session_id.boot_time_ns);
        writer.append_integer(observation.session_id.random_suffix);
        writer.append_string(observation.frame_id);
        writer.append_string(observation.clock_domain);
        writer.append_integer(observation.origin_stamp.nanoseconds);
        writer.append_integer(static_cast<std::uint8_t>(observation.ray_evidence));
        if(observation.is_2d()) {
            writer.append_u8(0U);
            const auto & scan = observation.as_scan_2d();
            writer.append_double(scan.angle_min_rad);
            writer.append_double(scan.angle_max_rad);
            writer.append_double(scan.angle_increment_rad);
            writer.append_double(scan.range_min_m);
            writer.append_double(scan.range_max_m);
            writer.append_integer<std::uint64_t>(scan.ranges.size());
            for(const float range : scan.ranges) {
                writer.append_float(range);
            }
            writer.append_integer<std::uint64_t>(scan.intensities.size());
            for(const float intensity : scan.intensities) {
                writer.append_float(intensity);
            }
        }
        else {
            writer.append_u8(1U);
            const auto & cloud = observation.as_cloud_3d();
            writer.append_integer<std::uint64_t>(cloud.points.size());
            for(const auto & point : cloud.points) {
                writer.append_float(point.x);
                writer.append_float(point.y);
                writer.append_float(point.z);
                writer.append_float(point.intensity);
            }
        }
        return writer.hex_digest();
    }

}// namespace PerceptionProfiling

#include "perception_local_map/LocalObservationMapper.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace PerceptionLocalMap {

    namespace {

        int health_rank(Perception::HealthState state) noexcept
        {
            switch(state) {
                case Perception::HealthState::Unavailable:
                    return 0;
                case Perception::HealthState::Degraded:
                    return 1;
                case Perception::HealthState::Healthy:
                    return 2;
            }
            return 0;
        }

        Perception::HealthState minimum_health(
                Perception::HealthState left,
                Perception::HealthState right) noexcept
        {
            return health_rank(left) <= health_rank(right) ? left : right;
        }

        bool valid_session(const Perception::SessionID & session) noexcept
        {
            return session.boot_time_ns != 0U;
        }

        bool valid_health(Perception::HealthState state) noexcept
        {
            switch(state) {
                case Perception::HealthState::Healthy:
                case Perception::HealthState::Degraded:
                case Perception::HealthState::Unavailable:
                    return true;
            }
            return false;
        }

        bool valid_alignment_status(AlignmentStatus status) noexcept
        {
            switch(status) {
                case AlignmentStatus::Candidate:
                case AlignmentStatus::Committed:
                case AlignmentStatus::Degraded:
                case AlignmentStatus::Revoked:
                    return true;
            }
            return false;
        }

        bool valid_sensor_type(Perception::SensorType type) noexcept
        {
            switch(type) {
                case Perception::SensorType::LIDAR_2D:
                case Perception::SensorType::LIDAR_3D:
                    return true;
            }
            return false;
        }

        bool valid_descriptor(const Perception::SensorDescriptor & descriptor)
        {
            const auto & fov = descriptor.fov;
            return !descriptor.sensor_id.value.empty() && !descriptor.frame_id.empty()
                   && valid_sensor_type(descriptor.type)
                   && Perception::is_valid_ray_evidence(descriptor.ray_evidence)
                   && descriptor.mounting_position.allFinite()
                   && descriptor.mounting_orientation.coeffs().allFinite()
                   && descriptor.mounting_orientation.norm() > 1e-12
                   && std::isfinite(fov.horizontal_min_rad)
                   && std::isfinite(fov.horizontal_max_rad)
                   && fov.horizontal_min_rad <= fov.horizontal_max_rad
                   && std::isfinite(fov.vertical_min_rad)
                   && std::isfinite(fov.vertical_max_rad)
                   && fov.vertical_min_rad <= fov.vertical_max_rad
                   && std::isfinite(descriptor.angular_resolution_rad)
                   && descriptor.angular_resolution_rad > 0.0
                   && std::isfinite(descriptor.range_min_m)
                   && std::isfinite(descriptor.range_max_m)
                   && descriptor.range_min_m >= 0.0
                   && descriptor.range_min_m < descriptor.range_max_m
                   && (descriptor.type != Perception::SensorType::LIDAR_3D
                       || descriptor.ray_evidence
                                  == Perception::RayEvidenceCapability::HitOnly);
        }

        bool valid_requirement(
                const Perception::SensorRequirement & requirement,
                const std::vector<Perception::SensorDescriptor> & descriptors)
        {
            if(!valid_sensor_type(requirement.type) || requirement.min_count == 0U
               || !Perception::is_valid_ray_evidence(
                       requirement.minimum_ray_evidence)) {
                return false;
            }
            std::set<std::string> specific_ids;
            if(requirement.specific_sensors.has_value()) {
                if(requirement.specific_sensors->empty()) {
                    return false;
                }
                for(const auto & id : requirement.specific_sensors.value()) {
                    if(id.value.empty() || !specific_ids.insert(id.value).second) {
                        return false;
                    }
                }
                if(requirement.min_count > specific_ids.size()) {
                    return false;
                }
            }
            const auto available = std::count_if(
                    descriptors.begin(), descriptors.end(),
                    [&](const auto & descriptor) {
                        return descriptor.type == requirement.type
                               && Perception::provides_at_least(
                                       descriptor.ray_evidence,
                                       requirement.minimum_ray_evidence)
                               && (specific_ids.empty()
                                   || specific_ids.count(descriptor.sensor_id.value) != 0U);
                    });
            return static_cast<std::size_t>(available) >= requirement.min_count;
        }

        bool valid_contract(
                const Perception::MapperInputContract & contract,
                const std::vector<Perception::SensorDescriptor> & descriptors)
        {
            const auto valid_requirements = [&](const auto & requirements) {
                return !requirements.empty()
                       && std::all_of(
                               requirements.begin(), requirements.end(),
                               [&](const auto & requirement) {
                                   return valid_requirement(requirement, descriptors);
                               });
            };
            if(!valid_requirements(contract.minimum_viable)
               || contract.recovery_stability_samples == 0U
               || !std::isfinite(contract.minimum_pose_quality)
               || contract.minimum_pose_quality < 0.0
               || contract.minimum_pose_quality > 1.0
               || (contract.requires_pose
                   && contract.pose_freshness_threshold.nanoseconds <= 0)) {
                return false;
            }
            return std::all_of(
                    contract.degraded_combinations.begin(),
                    contract.degraded_combinations.end(),
                    [&](const auto & combination) {
                        return valid_requirements(combination.requirements);
                    });
        }

        bool same_geometry(
                const MapGeometry & left,
                const MapGeometry & right) noexcept
        {
            return left.resolution_m == right.resolution_m
                   && left.lattice_origin.x == right.lattice_origin.x
                   && left.lattice_origin.y == right.lattice_origin.y
                   && left.lattice_origin.z == right.lattice_origin.z
                   && left.frame_id == right.frame_id;
        }

        std::string session_key(const Perception::SessionID & session)
        {
            return std::to_string(session.boot_time_ns) + "/"
                   + std::to_string(session.random_suffix);
        }

        std::string pose_session_key(
                const Perception::SourceID & source,
                const Perception::SessionID & session)
        {
            return source.value + "@" + session_key(session);
        }

        bool valid_pose_payload(const Perception::PoseEstimate & pose)
        {
            const bool covariance_valid = !pose.covariance.has_value()
                                          || pose.covariance->allFinite();
            return !pose.source_id.value.empty() && valid_session(pose.session_id)
                   && !pose.frame_id.empty() && !pose.clock_domain.empty()
                   && pose.stamp.nanoseconds >= 0 && pose.position.allFinite()
                   && std::isfinite(pose.orientation.w()) && std::isfinite(pose.orientation.x())
                   && std::isfinite(pose.orientation.y()) && std::isfinite(pose.orientation.z())
                   && pose.orientation.norm() > 1e-12 && std::isfinite(pose.quality)
                   && pose.quality >= 0.0 && pose.quality <= 1.0
                   && pose.freshness.nanoseconds >= 0 && covariance_valid;
        }

        Eigen::Isometry3d pose_transform(const Perception::PoseEstimate & pose)
        {
            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            result.translation() = pose.position;
            result.linear() = pose.orientation.normalized().toRotationMatrix();
            return result;
        }

        bool requirement_satisfied(
                const Perception::SensorRequirement & requirement,
                const std::vector<Perception::SensorDescriptor> & active)
        {
            std::size_t count = 0;
            for(const auto & descriptor : active) {
                if(descriptor.type != requirement.type
                   || !Perception::provides_at_least(
                           descriptor.ray_evidence, requirement.minimum_ray_evidence)) {
                    continue;
                }
                if(requirement.specific_sensors.has_value()
                   && std::find(
                              requirement.specific_sensors->begin(),
                              requirement.specific_sensors->end(), descriptor.sensor_id)
                              == requirement.specific_sensors->end()) {
                    continue;
                }
                ++count;
            }
            return count >= requirement.min_count;
        }

        bool requirements_satisfied(
                const std::vector<Perception::SensorRequirement> & requirements,
                const std::vector<Perception::SensorDescriptor> & active,
                bool pose_usable,
                bool pose_required)
        {
            if(pose_required && !pose_usable) {
                return false;
            }
            return std::all_of(
                    requirements.begin(), requirements.end(),
                    [&](const auto & requirement) {
                        return requirement_satisfied(requirement, active);
                    });
        }

        std::uint32_t bounded_u32(std::size_t value) noexcept
        {
            return static_cast<std::uint32_t>(std::min<std::size_t>(
                    value, std::numeric_limits<std::uint32_t>::max()));
        }

        bool finite_scan_metadata(const Perception::Scan2D & scan)
        {
            return std::isfinite(scan.angle_min_rad) && std::isfinite(scan.angle_max_rad)
                   && std::isfinite(scan.angle_increment_rad)
                   && std::isfinite(scan.range_min_m) && std::isfinite(scan.range_max_m)
                   && scan.angle_min_rad <= scan.angle_max_rad
                   && scan.angle_increment_rad > 0.0 && scan.range_min_m >= 0.0
                   && scan.range_min_m < scan.range_max_m && !scan.ranges.empty()
                   && (scan.intensities.empty() || scan.intensities.size() == scan.ranges.size());
        }

        bool near(double left, double right, double tolerance = 1e-5)
        {
            return std::abs(left - right) <= tolerance;
        }

        struct ProjectionResult {
            bool          valid = false;
            EvidenceBatch batch;
            std::string   diagnostic;
        };

        bool add_index(
                std::vector<VoxelIndex> & output,
                const MapPoint & point,
                const MapGeometry & geometry,
                std::string & diagnostic)
        {
            const auto indexed = quantize_point(point, geometry);
            if(indexed.status != QueryStatus::Ok) {
                diagnostic = indexed.status == QueryStatus::OutOfRange
                                     ? "evidence is outside canonical lattice range"
                                     : "evidence contains invalid coordinates";
                return false;
            }
            output.push_back(indexed.index);
            return true;
        }

        bool add_free_ray(
                EvidenceBatch & batch,
                const MapPoint & origin,
                const MapPoint & endpoint,
                const VoxelIndex & endpoint_index,
                const MapGeometry & geometry,
                std::string & diagnostic)
        {
            const auto origin_index = quantize_point(origin, geometry);
            if(origin_index.status != QueryStatus::Ok) {
                diagnostic = "ray origin is outside canonical lattice range";
                return false;
            }

            VoxelIndex current = origin_index.index;
            if(current == endpoint_index) {
                return true;
            }
            const double direction[3] {
                    endpoint.x - origin.x,
                    endpoint.y - origin.y,
                    endpoint.z - origin.z};
            const double origin_coordinates[3] {origin.x, origin.y, origin.z};
            const double lattice_origins[3] {
                    geometry.lattice_origin.x,
                    geometry.lattice_origin.y,
                    geometry.lattice_origin.z};
            std::int64_t * current_axes[3] {&current.x, &current.y, &current.z};
            int steps[3] {0, 0, 0};
            double maximum_t[3] {
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity()};
            double delta_t[3] {
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity()};
            for(std::size_t axis = 0; axis < 3U; ++axis) {
                if(direction[axis] == 0.0) {
                    continue;
                }
                steps[axis] = direction[axis] > 0.0 ? 1 : -1;
                const double boundary_index = static_cast<double>(*current_axes[axis])
                                              + (steps[axis] > 0 ? 1.0 : 0.0);
                const double boundary = lattice_origins[axis]
                                        + boundary_index * geometry.resolution_m;
                maximum_t[axis] = (boundary - origin_coordinates[axis])
                                  / direction[axis];
                delta_t[axis] = geometry.resolution_m / std::abs(direction[axis]);
            }

            const std::uint64_t maximum_steps =
                    static_cast<std::uint64_t>(std::abs(endpoint_index.x - current.x))
                    + static_cast<std::uint64_t>(std::abs(endpoint_index.y - current.y))
                    + static_cast<std::uint64_t>(std::abs(endpoint_index.z - current.z))
                    + 1U;
            std::uint64_t traversed = 0;
            while(current != endpoint_index && traversed < maximum_steps) {
                batch.free_cells.push_back(current);
                const double next_t = std::min({maximum_t[0], maximum_t[1], maximum_t[2]});
                if(!std::isfinite(next_t)) {
                    diagnostic = "ray traversal cannot reach endpoint voxel";
                    return false;
                }
                for(std::size_t axis = 0; axis < 3U; ++axis) {
                    if(maximum_t[axis] == next_t) {
                        *current_axes[axis] += steps[axis];
                        maximum_t[axis] += delta_t[axis];
                    }
                }
                if(!is_representable_voxel(current, geometry)) {
                    diagnostic = "ray crosses outside canonical lattice range";
                    return false;
                }
                ++traversed;
            }
            if(current != endpoint_index) {
                diagnostic = "ray traversal exceeded canonical voxel bound";
                return false;
            }
            return true;
        }

        MapPoint transform_point(
                const Eigen::Isometry3d & transform,
                const Eigen::Vector3d & point)
        {
            const Eigen::Vector3d transformed = transform * point;
            return {transformed.x(), transformed.y(), transformed.z()};
        }

        ProjectionResult project_observation(
                const Perception::LidarObservation & observation,
                const Perception::SensorDescriptor & descriptor,
                const Eigen::Isometry3d & map_from_sensor,
                const MapGeometry & geometry)
        {
            ProjectionResult result;
            if(observation.sensor_id != descriptor.sensor_id
               || observation.frame_id != descriptor.frame_id
               || observation.clock_domain.empty() || observation.origin_stamp.nanoseconds < 0
               || !Perception::is_valid_ray_evidence(observation.ray_evidence)
               || observation.ray_evidence != descriptor.ray_evidence) {
                result.diagnostic = "observation identity or capability does not match frozen descriptor";
                return result;
            }

            const MapPoint origin = transform_point(map_from_sensor, Eigen::Vector3d::Zero());
            if(observation.is_2d()) {
                if(descriptor.type != Perception::SensorType::LIDAR_2D) {
                    result.diagnostic = "2D payload does not match sensor type";
                    return result;
                }
                const auto & scan = observation.as_scan_2d();
                if(!finite_scan_metadata(scan)
                   || !near(scan.angle_min_rad, descriptor.fov.horizontal_min_rad)
                   || !near(scan.angle_max_rad, descriptor.fov.horizontal_max_rad)
                   || !near(scan.angle_increment_rad, descriptor.angular_resolution_rad)
                   || !near(scan.range_min_m, descriptor.range_min_m)
                   || !near(scan.range_max_m, descriptor.range_max_m)) {
                    result.diagnostic = "2D metadata drifted from frozen descriptor";
                    return result;
                }
                const double expected_max = scan.angle_min_rad
                                            + static_cast<double>(scan.ranges.size() - 1U)
                                                      * scan.angle_increment_rad;
                if(!near(expected_max, scan.angle_max_rad, 1e-4)) {
                    result.diagnostic = "2D angular layout is inconsistent";
                    return result;
                }

                for(std::size_t index = 0; index < scan.ranges.size(); ++index) {
                    const auto kind = scan.return_kind(index);
                    if(kind == Perception::RayReturnKind::Invalid) {
                        continue;
                    }
                    const double range = kind == Perception::RayReturnKind::NoReturn
                                                 ? scan.range_max_m
                                                 : static_cast<double>(scan.ranges[index]);
                    const double angle = scan.angle_min_rad
                                         + static_cast<double>(index) * scan.angle_increment_rad;
                    const MapPoint endpoint = transform_point(
                            map_from_sensor,
                            Eigen::Vector3d(range * std::cos(angle), range * std::sin(angle), 0.0));
                    const auto endpoint_index = quantize_point(endpoint, geometry);
                    if(endpoint_index.status != QueryStatus::Ok) {
                        result.diagnostic = "2D endpoint is outside canonical lattice range";
                        return result;
                    }

                    if(kind == Perception::RayReturnKind::Hit) {
                        result.batch.occupied_cells.push_back(endpoint_index.index);
                        if(Perception::provides_at_least(
                                   observation.ray_evidence,
                                   Perception::RayEvidenceCapability::HitRay)
                           && !add_free_ray(
                                   result.batch, origin, endpoint, endpoint_index.index,
                                   geometry, result.diagnostic)) {
                            return result;
                        }
                    }
                    else if(observation.ray_evidence
                            == Perception::RayEvidenceCapability::FullRay) {
                        if(!add_free_ray(
                                   result.batch, origin, endpoint, endpoint_index.index,
                                   geometry, result.diagnostic)) {
                            return result;
                        }
                    }
                }
            }
            else {
                if(descriptor.type != Perception::SensorType::LIDAR_3D
                   || observation.ray_evidence != Perception::RayEvidenceCapability::HitOnly) {
                    result.diagnostic = "Cloud3D supports HitOnly endpoints only";
                    return result;
                }
                for(const auto & point : observation.as_cloud_3d().points) {
                    if(!std::isfinite(point.x) || !std::isfinite(point.y)
                       || !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
                        result.diagnostic = "Cloud3D contains non-finite data";
                        return result;
                    }
                    const double range = Eigen::Vector3d(point.x, point.y, point.z).norm();
                    if(!std::isfinite(range) || range < descriptor.range_min_m
                       || range > descriptor.range_max_m) {
                        result.diagnostic = "Cloud3D endpoint is outside frozen range";
                        return result;
                    }
                    if(!add_index(
                               result.batch.occupied_cells,
                               transform_point(
                                       map_from_sensor, Eigen::Vector3d(point.x, point.y, point.z)),
                               geometry, result.diagnostic)) {
                        return result;
                    }
                }
            }

            result.valid = true;
            return result;
        }

    }// namespace

    struct SharedMapStorage {
        mutable std::shared_mutex mutex;
        std::unique_ptr<ILocalOccupancyBackend> backend;
        std::shared_ptr<std::atomic_bool> fence = std::make_shared<std::atomic_bool>(true);
    };

    struct MapReadTransaction::Impl {
        Impl(
                std::shared_ptr<SharedMapStorage> storage_value,
                std::shared_lock<std::shared_mutex> lock_value,
                const ILocalOccupancyBackend * backend_value,
                std::shared_ptr<std::atomic_bool> fence_value,
                MapReadMetadata metadata_value)
            : storage(std::move(storage_value))
            , lock(std::move(lock_value))
            , backend(backend_value)
            , fence(std::move(fence_value))
            , metadata(std::move(metadata_value))
        {}

        std::shared_ptr<SharedMapStorage>   storage;
        std::shared_lock<std::shared_mutex> lock;
        const ILocalOccupancyBackend *      backend = nullptr;
        std::shared_ptr<std::atomic_bool>   fence;
        MapReadMetadata                    metadata;
        bool                               closed = false;
    };

    MapReadTransaction::MapReadTransaction() noexcept = default;
    MapReadTransaction::MapReadTransaction(std::unique_ptr<Impl> impl) noexcept
        : impl_(std::move(impl))
    {}
    MapReadTransaction::~MapReadTransaction() { close(); }
    MapReadTransaction::MapReadTransaction(MapReadTransaction && other) noexcept = default;
    MapReadTransaction & MapReadTransaction::operator=(MapReadTransaction && other) noexcept = default;

    bool MapReadTransaction::is_open() const noexcept
    {
        return impl_ && !impl_->closed && impl_->backend != nullptr && impl_->fence
               && impl_->fence->load();
    }

    std::optional<MapReadMetadata> MapReadTransaction::metadata() const
    {
        return is_open() ? std::optional<MapReadMetadata>(impl_->metadata) : std::nullopt;
    }

    MapQueryResult MapReadTransaction::query(const MapPoint & point) const
    {
        if(!is_open()) {
            return {QueryStatus::TransactionClosed, OccupancyState::Unknown};
        }
        return impl_->backend->query(point);
    }

    RegionQueryResult MapReadTransaction::query_region(const AxisAlignedBounds & bounds) const
    {
        if(!is_open()) {
            return {QueryStatus::TransactionClosed, {}};
        }
        return impl_->backend->query_region(bounds);
    }

    bool MapReadTransaction::for_each_known_cell(
            const std::function<void(OccupancyCell)> & visitor) const
    {
        if(!is_open() || !visitor) {
            return false;
        }
        return impl_->backend->for_each_known_cell(visitor);
    }

    void MapReadTransaction::close() noexcept
    {
        if(!impl_ || impl_->closed) {
            return;
        }
        impl_->closed = true;
        impl_->backend = nullptr;
        if(impl_->lock.owns_lock()) {
            impl_->lock.unlock();
        }
    }

    const ILocalOccupancyBackend * MapReadTransaction::backend_for_snapshot() const noexcept
    {
        return is_open() ? impl_->backend : nullptr;
    }

    struct LocalObservationMapper::Impl {
        struct PoseRecord {
            Perception::PoseEstimate pose;
            std::int64_t              receive_monotonic_ns = 0;
        };

        struct PoseLineage {
            Perception::SourceID    source_id;
            Perception::SessionID   session {0, 0};
            std::uint64_t           reset_epoch = 0;
            std::string             frame_id;
            Perception::Timestamp   high_water_stamp {0};
            Perception::PoseEstimate last_pose;
            bool                    has_usable_pose = false;
        };

        struct SensorRuntime {
            Perception::SensorDescriptor descriptor;
            std::optional<Perception::SessionID> active_session;
            std::unordered_set<std::string>      retired_sessions;
            Perception::Timestamp                high_water_stamp {0};
            std::int64_t                         receive_monotonic_ns = 0;
            bool                                 active_for_gate = false;
        };

        Impl(MapperConfig value, BackendFactory factory_value)
            : config(std::move(value))
            , factory(std::move(factory_value))
            , fingerprint(Perception::MapperContractFingerprint::compute(
                      config.sensor_descriptors, config.input_contract))
            , extrinsics(
                      config.sensor_descriptors,
                      config.extrinsic_position_tolerance_m,
                      config.extrinsic_orientation_tolerance_rad)
            , health_gate(std::make_unique<Perception::MapperHealthGate>(config.input_contract))
            , identity {config.vehicle_id, config.mapper_session, 1, 0}
        {
            storage = std::make_shared<SharedMapStorage>();
            storage->backend = factory(config.geometry);
            if(!storage->backend
               || !storage->backend->capabilities().has_required_capabilities()
               || !same_geometry(storage->backend->geometry(), config.geometry)) {
                throw std::invalid_argument("backend does not provide required local-map capabilities");
            }
            for(const auto & descriptor : config.sensor_descriptors) {
                SensorRuntime runtime;
                runtime.descriptor = descriptor;
                if(!Perception::is_valid_ray_evidence(descriptor.ray_evidence)
                   || !sensors.emplace(
                               descriptor.sensor_id.value, std::move(runtime)).second) {
                    throw std::invalid_argument("mapper requires unique descriptors with closed capabilities");
                }
            }
        }

        MapperConfig config;
        BackendFactory factory;
        Perception::MapperContractFingerprint fingerprint;
        SensorExtrinsicRegistry extrinsics;
        std::unique_ptr<Perception::MapperHealthGate> health_gate;
        std::shared_ptr<SharedMapStorage> storage;
        MapIdentity identity;
        std::unordered_map<std::string, SensorRuntime> sensors;
        std::deque<PoseRecord> pose_history;
        std::optional<PoseLineage> active_pose;
        std::unordered_set<std::string> retired_pose_sessions;
        std::optional<UpstreamHealth> upstream_health;
        std::optional<std::string> health_source_id;
        std::unordered_set<std::string> retired_health_sessions;
        Perception::HealthState current_health = Perception::HealthState::Unavailable;
        EffectiveMapCapabilities effective_capabilities;
        std::optional<CommitProvenance> last_commit;
        std::int64_t last_commit_monotonic_ns = 0;
        std::optional<AlignmentReference> alignment;
        void make_unavailable_locked()
        {
            current_health = Perception::HealthState::Unavailable;
            effective_capabilities.satisfied_combination_id.clear();
        }

        bool expire_sensor_inputs_locked(std::int64_t now)
        {
            bool expired = false;
            for(auto & entry : sensors) {
                if(entry.second.active_for_gate
                   && (now < entry.second.receive_monotonic_ns
                       || now - entry.second.receive_monotonic_ns
                                  > config.sensor_validity_ns)) {
                    entry.second.active_for_gate = false;
                    expired = true;
                }
            }
            return expired;
        }

        bool upstream_matches_locked(
                std::int64_t now,
                const std::optional<Perception::SessionID> & required_session = std::nullopt) const
        {
            if(!upstream_health.has_value() || !upstream_health->contract_fingerprint.is_well_formed()
               || upstream_health->contract_fingerprint.schema_version != fingerprint.schema_version
               || upstream_health->contract_fingerprint.hex_digest != fingerprint.hex_digest
               || upstream_health->producer_source_id.empty()
               || !valid_session(upstream_health->producer_session)
               || upstream_health->validity_budget_ns <= 0
               || now < upstream_health->receive_monotonic_ns
               || now - upstream_health->receive_monotonic_ns
                          > std::min(config.health_validity_ns, upstream_health->validity_budget_ns)) {
                return false;
            }
            return !required_session.has_value()
                   || upstream_health->producer_session == required_session.value();
        }

        std::optional<PoseRecord> pose_for_observation_locked(
                const Perception::LidarObservation & observation,
                std::int64_t now) const
        {
            if(!config.input_contract.requires_pose) {
                return PoseRecord {
                        Perception::PoseEstimate {
                                Perception::SourceID {"identity"}, config.mapper_session,
                                config.geometry.frame_id, observation.clock_domain,
                                observation.origin_stamp, Eigen::Vector3d::Zero(),
                                Eigen::Quaterniond::Identity(), std::nullopt, 1.0,
                                Perception::Duration {0}, 0},
                        now};
            }
            for(auto iterator = pose_history.rbegin(); iterator != pose_history.rend(); ++iterator) {
                if(iterator->pose.clock_domain != observation.clock_domain
                   || iterator->pose.stamp > observation.origin_stamp
                   || observation.origin_stamp.nanoseconds - iterator->pose.stamp.nanoseconds
                              > config.input_contract.pose_freshness_threshold.nanoseconds
                   || now < iterator->receive_monotonic_ns
                   || now - iterator->receive_monotonic_ns > config.pose_receive_validity_ns
                   || !std::isfinite(iterator->pose.quality)
                   || iterator->pose.quality < config.input_contract.minimum_pose_quality) {
                    continue;
                }
                return *iterator;
            }
            return std::nullopt;
        }

        std::vector<Perception::SensorDescriptor> active_descriptors_locked() const
        {
            std::vector<Perception::SensorDescriptor> result;
            for(const auto & entry : sensors) {
                if(entry.second.active_for_gate) {
                    result.push_back(entry.second.descriptor);
                }
            }
            return result;
        }

        std::optional<Perception::PoseEstimate> current_pose_for_gate_locked(std::int64_t now) const
        {
            if(!config.input_contract.requires_pose) {
                return std::nullopt;
            }
            if(pose_history.empty() || now < pose_history.back().receive_monotonic_ns
               || now - pose_history.back().receive_monotonic_ns > config.pose_receive_validity_ns) {
                return std::nullopt;
            }
            auto pose = pose_history.back().pose;
            pose.freshness = Perception::Duration::from_nanoseconds(
                    now - pose_history.back().receive_monotonic_ns);
            return pose;
        }

        void update_effective_capabilities_locked(bool pose_usable)
        {
            effective_capabilities = {};
            const auto active = active_descriptors_locked();
            for(const auto & descriptor : active) {
                ++effective_capabilities.active_sensor_count;
                effective_capabilities.has_active_ray_evidence = true;
                if(static_cast<std::uint8_t>(descriptor.ray_evidence)
                   > static_cast<std::uint8_t>(
                           effective_capabilities.maximum_active_ray_evidence)) {
                    effective_capabilities.maximum_active_ray_evidence = descriptor.ray_evidence;
                }
                auto * count = &effective_capabilities.active_2d_hit_only_sensor_count;
                if(descriptor.type == Perception::SensorType::LIDAR_2D) {
                    if(descriptor.ray_evidence == Perception::RayEvidenceCapability::HitRay) {
                        count = &effective_capabilities.active_2d_hit_ray_sensor_count;
                    }
                    else if(descriptor.ray_evidence == Perception::RayEvidenceCapability::FullRay) {
                        count = &effective_capabilities.active_2d_full_ray_sensor_count;
                    }
                }
                else {
                    count = &effective_capabilities.active_3d_hit_only_sensor_count;
                    if(descriptor.ray_evidence == Perception::RayEvidenceCapability::HitRay) {
                        count = &effective_capabilities.active_3d_hit_ray_sensor_count;
                    }
                    else if(descriptor.ray_evidence == Perception::RayEvidenceCapability::FullRay) {
                        count = &effective_capabilities.active_3d_full_ray_sensor_count;
                    }
                }
                ++(*count);
            }

            if(requirements_satisfied(
                       config.input_contract.minimum_viable, active, pose_usable,
                       config.input_contract.requires_pose)) {
                effective_capabilities.satisfied_combination_id = "minimum";
            }
            else {
                for(std::size_t index = 0;
                    index < config.input_contract.degraded_combinations.size(); ++index) {
                    if(requirements_satisfied(
                               config.input_contract.degraded_combinations[index].requirements,
                               active, pose_usable, config.input_contract.requires_pose)) {
                        effective_capabilities.satisfied_combination_id =
                                "degraded/" + std::to_string(index);
                        break;
                    }
                }
            }
        }

        void evaluate_health_locked(std::int64_t now)
        {
            expire_sensor_inputs_locked(now);
            std::vector<Perception::SensorDescriptor> descriptors;
            std::vector<Perception::SensorHealth> health;
            descriptors.reserve(sensors.size());
            health.reserve(sensors.size());
            for(const auto & entry : sensors) {
                descriptors.push_back(entry.second.descriptor);
                health.push_back(
                        {entry.second.descriptor.sensor_id,
                         entry.second.active_for_gate
                                 ? Perception::SensorHealthStatus::Active
                                 : Perception::SensorHealthStatus::Stale,
                         Perception::Timestamp::from_nanoseconds(
                                 entry.second.receive_monotonic_ns)});
            }
            const auto pose = current_pose_for_gate_locked(now);
            const auto local = health_gate->evaluate(descriptors, health, pose);
            const bool pose_usable = !config.input_contract.requires_pose
                                     || health_gate->current_capability().has_usable_pose;
            update_effective_capabilities_locked(pose_usable);
            current_health = upstream_matches_locked(now) && storage->backend
                                     && storage->fence->load()
                                     ? minimum_health(local, upstream_health->state)
                                     : Perception::HealthState::Unavailable;
            if(current_health == Perception::HealthState::Unavailable) {
                effective_capabilities.satisfied_combination_id.clear();
            }
        }

        void invalidate_pose_locked(std::int64_t now)
        {
            pose_history.clear();
            if(config.input_contract.requires_pose) {
                evaluate_health_locked(now);
            }
        }

        void reset_map_locked()
        {
            storage->fence->store(false);
            if(alignment.has_value()) {
                alignment->status = AlignmentStatus::Revoked;
            }
            storage->backend.reset();
            ++identity.map_epoch;
            identity.revision = 0;
            last_commit.reset();
            last_commit_monotonic_ns = 0;
            pose_history.clear();
            for(auto & entry : sensors) {
                entry.second.active_for_gate = false;
            }
            effective_capabilities = {};
            health_gate = std::make_unique<Perception::MapperHealthGate>(config.input_contract);
            current_health = Perception::HealthState::Unavailable;
            storage->fence = std::make_shared<std::atomic_bool>(true);
            try {
                storage->backend = factory(config.geometry);
                if(!storage->backend
                   || !storage->backend->capabilities().has_required_capabilities()
                   || !same_geometry(storage->backend->geometry(), config.geometry)) {
                    throw std::runtime_error("replacement backend lacks required capabilities");
                }
            }
            catch(...) {
                storage->backend.reset();
                storage->fence->store(false);
            }
        }

        void backend_fault_locked()
        {
            reset_map_locked();
        }
    };

    LocalObservationMapper::LocalObservationMapper(
            MapperConfig config,
            BackendFactory backend_factory)
    {
        if(config.vehicle_id.empty() || !valid_session(config.mapper_session)
           || !is_valid_geometry(config.geometry) || config.body_frame.empty()
           || config.sensor_descriptors.empty() || !backend_factory
           || !std::all_of(
                   config.sensor_descriptors.begin(), config.sensor_descriptors.end(),
                   valid_descriptor)
           || !valid_contract(config.input_contract, config.sensor_descriptors)
           || config.sensor_validity_ns <= 0 || config.health_validity_ns <= 0
           || config.pose_receive_validity_ns <= 0 || config.map_freshness_ns <= 0
           || config.pose_history_limit == 0U
           || (config.input_contract.requires_pose
               && (!config.input_contract.expected_pose_frame.has_value()
                   || config.input_contract.expected_pose_frame.value()
                              != config.geometry.frame_id))
           || !std::isfinite(config.position_jump_threshold_m)
           || config.position_jump_threshold_m < 0.0
           || !std::isfinite(config.orientation_jump_threshold_rad)
           || config.orientation_jump_threshold_rad < 0.0) {
            throw std::invalid_argument("invalid local mapper configuration");
        }
        impl_ = std::make_unique<Impl>(std::move(config), std::move(backend_factory));
    }

    LocalObservationMapper::~LocalObservationMapper()
    {
        if(impl_) {
            impl_->storage->fence->store(false);
        }
    }

    bool LocalObservationMapper::submit_health(const UpstreamHealth & health)
    {
        std::unique_lock<std::shared_mutex> lock(impl_->storage->mutex);
        const bool valid = !health.producer_source_id.empty()
                           && valid_session(health.producer_session)
                           && valid_health(health.state)
                           && health.contract_fingerprint.is_well_formed()
                           && health.contract_fingerprint.schema_version
                                      == impl_->fingerprint.schema_version
                           && health.contract_fingerprint.hex_digest
                                      == impl_->fingerprint.hex_digest
                           && health.receive_monotonic_ns >= 0
                           && health.validity_budget_ns > 0;
        if(impl_->retired_health_sessions.count(session_key(health.producer_session)) != 0U) {
            return false;
        }
        if(impl_->upstream_health.has_value()
           && health.producer_session != impl_->upstream_health->producer_session
           && health.producer_session < impl_->upstream_health->producer_session) {
            return false;
        }
        if(!valid || (impl_->health_source_id.has_value()
                      && impl_->health_source_id.value() != health.producer_source_id)) {
            if(impl_->upstream_health.has_value()
               && impl_->upstream_health->producer_session == health.producer_session) {
                impl_->upstream_health.reset();
                impl_->make_unavailable_locked();
            }
            return false;
        }
        if(impl_->upstream_health.has_value()
           && impl_->upstream_health->producer_session != health.producer_session) {
            impl_->retired_health_sessions.insert(
                    session_key(impl_->upstream_health->producer_session));
        }
        impl_->health_source_id = health.producer_source_id;
        impl_->upstream_health = health;
        impl_->evaluate_health_locked(health.receive_monotonic_ns);
        return true;
    }

    InputResult LocalObservationMapper::submit_pose(
            const Perception::PoseEstimate & pose,
            std::int64_t receive_monotonic_ns)
    {
        if(receive_monotonic_ns < 0) {
            return {InputStatus::Rejected, std::nullopt, "invalid pose payload or frame"};
        }
        std::unique_lock<std::shared_mutex> lock(impl_->storage->mutex);
        if(!valid_pose_payload(pose) || pose.frame_id != impl_->config.geometry.frame_id) {
            impl_->invalidate_pose_locked(receive_monotonic_ns);
            return {InputStatus::Rejected, std::nullopt, "invalid pose payload or frame"};
        }
        const auto incoming_key = pose_session_key(pose.source_id, pose.session_id);
        if(impl_->retired_pose_sessions.count(incoming_key) != 0U) {
            impl_->invalidate_pose_locked(receive_monotonic_ns);
            return {InputStatus::Rejected, std::nullopt, "retired pose session replay"};
        }

        const bool pose_usable = pose.quality >= impl_->config.input_contract.minimum_pose_quality;
        bool discontinuity = false;
        if(impl_->active_pose.has_value()) {
            const bool same_session = impl_->active_pose->source_id == pose.source_id
                                      && impl_->active_pose->session == pose.session_id;
            if(!same_session) {
                if(!pose_usable) {
                    impl_->invalidate_pose_locked(receive_monotonic_ns);
                    return {InputStatus::Unavailable, std::nullopt,
                            "new pose session is not usable"};
                }
                if(!impl_->upstream_matches_locked(receive_monotonic_ns, pose.session_id)) {
                    impl_->invalidate_pose_locked(receive_monotonic_ns);
                    return {InputStatus::Unavailable, std::nullopt, "new pose session is not contract-bound"};
                }
                impl_->retired_pose_sessions.insert(pose_session_key(
                        impl_->active_pose->source_id, impl_->active_pose->session));
                discontinuity = true;
            }
            else {
                if(pose.reset_epoch < impl_->active_pose->reset_epoch
                   || pose.stamp <= impl_->active_pose->high_water_stamp
                   || pose.frame_id != impl_->active_pose->frame_id) {
                    impl_->invalidate_pose_locked(receive_monotonic_ns);
                    return {InputStatus::Rejected, std::nullopt, "pose rollback, replay, or frame drift"};
                }
                if(!pose_usable) {
                    impl_->active_pose->reset_epoch = pose.reset_epoch;
                    impl_->active_pose->high_water_stamp = pose.stamp;
                    impl_->invalidate_pose_locked(receive_monotonic_ns);
                    return {InputStatus::Accepted, std::nullopt, {}};
                }
                if(impl_->active_pose->has_usable_pose) {
                    const double position_jump =
                            (pose.position - impl_->active_pose->last_pose.position).norm();
                    const double orientation_jump = pose.orientation.normalized().angularDistance(
                            impl_->active_pose->last_pose.orientation.normalized());
                    discontinuity = pose.reset_epoch
                                            > impl_->active_pose->last_pose.reset_epoch
                                    || position_jump > impl_->config.position_jump_threshold_m
                                    || orientation_jump
                                               > impl_->config.orientation_jump_threshold_rad;
                }
                if(discontinuity
                   && !impl_->upstream_matches_locked(receive_monotonic_ns, pose.session_id)) {
                    impl_->invalidate_pose_locked(receive_monotonic_ns);
                    return {InputStatus::Unavailable, std::nullopt, "pose discontinuity is not contract-bound"};
                }
            }
        }
        else if(!pose_usable) {
            impl_->active_pose = Impl::PoseLineage {
                    pose.source_id, pose.session_id, pose.reset_epoch, pose.frame_id,
                    pose.stamp, pose, false};
            impl_->invalidate_pose_locked(receive_monotonic_ns);
            return {InputStatus::Accepted, std::nullopt, {}};
        }

        if(discontinuity) {
            impl_->reset_map_locked();
        }
        impl_->active_pose = Impl::PoseLineage {
                pose.source_id, pose.session_id, pose.reset_epoch, pose.frame_id,
                pose.stamp, pose, true};
        impl_->pose_history.push_back({pose, receive_monotonic_ns});
        while(impl_->pose_history.size() > impl_->config.pose_history_limit) {
            impl_->pose_history.pop_front();
        }
        impl_->evaluate_health_locked(receive_monotonic_ns);
        return {InputStatus::Accepted, std::nullopt, {}};
    }

    InputResult LocalObservationMapper::submit_observation(
            const Perception::LidarObservation & observation,
            const SensorExtrinsicSample & extrinsic,
            std::int64_t receive_monotonic_ns)
    {
        if(receive_monotonic_ns < 0 || !valid_session(observation.session_id)) {
            return {InputStatus::Rejected, std::nullopt, "invalid observation receive/session"};
        }
        std::unique_lock<std::shared_mutex> lock(impl_->storage->mutex);
        auto sensor = impl_->sensors.find(observation.sensor_id.value);
        if(sensor == impl_->sensors.end()) {
            return {InputStatus::Rejected, std::nullopt, "unknown sensor"};
        }
        auto & runtime = sensor->second;
        const auto incoming_session_key = session_key(observation.session_id);
        if(runtime.retired_sessions.count(incoming_session_key) != 0U) {
            return {InputStatus::Rejected, std::nullopt, "retired sensor session replay"};
        }
        if(runtime.active_session.has_value()) {
            if(runtime.active_session.value() == observation.session_id) {
                if(observation.origin_stamp <= runtime.high_water_stamp) {
                    return {InputStatus::Rejected, std::nullopt, "observation replay or rollback"};
                }
            }
            else {
                if(!impl_->upstream_matches_locked(receive_monotonic_ns, observation.session_id)) {
                    return {InputStatus::Unavailable, std::nullopt, "sensor session is not contract-bound"};
                }
                runtime.retired_sessions.insert(session_key(runtime.active_session.value()));
            }
        }
        else if(!impl_->upstream_matches_locked(receive_monotonic_ns, observation.session_id)) {
            return {InputStatus::Unavailable, std::nullopt, "observation is not contract-bound"};
        }

        if(extrinsic.sensor_id != observation.sensor_id
           || extrinsic.sensor_session != observation.session_id) {
            return {InputStatus::Rejected, std::nullopt, "extrinsic provenance mismatch"};
        }
        const auto frozen_extrinsic = impl_->extrinsics.validate_and_freeze(extrinsic);
        if(frozen_extrinsic.status != ExtrinsicStatus::Accepted) {
            runtime.active_for_gate = false;
            impl_->evaluate_health_locked(receive_monotonic_ns);
            return {InputStatus::Rejected, std::nullopt, frozen_extrinsic.diagnostic};
        }
        const auto pose_record = impl_->pose_for_observation_locked(observation, receive_monotonic_ns);
        if(!pose_record.has_value()) {
            impl_->make_unavailable_locked();
            return {InputStatus::Unavailable, std::nullopt, "no same-clock usable pose"};
        }
        const auto projection = project_observation(
                observation, runtime.descriptor,
                pose_transform(pose_record->pose) * frozen_extrinsic.body_from_sensor,
                impl_->config.geometry);
        if(!projection.valid) {
            return {InputStatus::Rejected, std::nullopt, projection.diagnostic};
        }

        runtime.active_session = observation.session_id;
        runtime.high_water_stamp = observation.origin_stamp;
        runtime.receive_monotonic_ns = receive_monotonic_ns;
        runtime.active_for_gate = true;
        impl_->evaluate_health_locked(receive_monotonic_ns);
        if(impl_->current_health == Perception::HealthState::Unavailable) {
            return {InputStatus::Unavailable, std::nullopt, "mapper input health is unavailable"};
        }
        if(!impl_->health_gate->recovery_stable()) {
            return {InputStatus::Unavailable, std::nullopt, "mapper recovery stability gate"};
        }
        if(!impl_->storage->backend) {
            return {InputStatus::BackendFault, std::nullopt, "backend is unavailable"};
        }

        ApplyResult applied;
        try {
            applied = impl_->storage->backend->apply(projection.batch);
        }
        catch(const std::exception & error) {
            const std::string diagnostic = error.what();
            impl_->backend_fault_locked();
            return {InputStatus::BackendFault, std::nullopt, diagnostic};
        }
        catch(...) {
            impl_->backend_fault_locked();
            return {InputStatus::BackendFault, std::nullopt, "unknown backend mutation fault"};
        }
        switch(applied.status) {
            case ApplyStatus::Rejected:
                if(applied.changed_cell_count != 0U) {
                    impl_->backend_fault_locked();
                    return {InputStatus::BackendFault, std::nullopt,
                            "backend rejected after reporting mutation"};
                }
                return {InputStatus::Rejected, std::nullopt, applied.diagnostic};
            case ApplyStatus::NoEvidence:
                if(applied.changed_cell_count != 0U
                   || !projection.batch.free_cells.empty()
                   || !projection.batch.occupied_cells.empty()) {
                    impl_->backend_fault_locked();
                    return {InputStatus::BackendFault, std::nullopt,
                            "backend returned contradictory NoEvidence"};
                }
                return {InputStatus::NoEvidence, std::nullopt, {}};
            case ApplyStatus::Applied:
                if(projection.batch.free_cells.empty()
                   && projection.batch.occupied_cells.empty()) {
                    impl_->backend_fault_locked();
                    return {InputStatus::BackendFault, std::nullopt,
                            "backend applied an empty evidence batch"};
                }
                break;
            default:
                impl_->backend_fault_locked();
                return {InputStatus::BackendFault, std::nullopt,
                        "backend returned unknown apply status"};
        }

        ++impl_->identity.revision;
        impl_->last_commit_monotonic_ns = receive_monotonic_ns;
        impl_->last_commit = CommitProvenance {
                observation.sensor_id, observation.session_id, observation.origin_stamp,
                observation.clock_domain, bounded_u32(applied.changed_cell_count)};
        CommitReceipt receipt {
                impl_->identity.mapper_session, impl_->identity.map_epoch,
                impl_->identity.revision};
        return {InputStatus::Applied, receipt, {}};
    }

    bool LocalObservationMapper::mark_sensor_unavailable(
            const Perception::SensorID & sensor_id,
            std::int64_t monotonic_now_ns)
    {
        if(monotonic_now_ns < 0) {
            return false;
        }
        std::unique_lock<std::shared_mutex> lock(impl_->storage->mutex);
        const auto sensor = impl_->sensors.find(sensor_id.value);
        if(sensor == impl_->sensors.end()) {
            return false;
        }
        sensor->second.active_for_gate = false;
        impl_->evaluate_health_locked(monotonic_now_ns);
        return true;
    }

    bool LocalObservationMapper::submit_alignment(const AlignmentReference & reference)
    {
        std::unique_lock<std::shared_mutex> lock(impl_->storage->mutex);
        if(reference.vehicle_id != impl_->identity.vehicle_id
           || reference.mapper_session != impl_->identity.mapper_session
           || reference.map_epoch != impl_->identity.map_epoch
           || reference.provider_id.empty() || !valid_session(reference.provider_session)
           || !valid_alignment_status(reference.status)
           || (impl_->alignment.has_value()
               && (reference.alignment_epoch < impl_->alignment->alignment_epoch
                   || (reference.alignment_epoch == impl_->alignment->alignment_epoch
                       && reference.alignment_revision
                                  <= impl_->alignment->alignment_revision)))) {
            return false;
        }
        impl_->alignment = reference;
        return true;
    }

    void LocalObservationMapper::tick(std::int64_t monotonic_now_ns)
    {
        std::unique_lock<std::shared_mutex> lock(impl_->storage->mutex);
        bool expired = impl_->expire_sensor_inputs_locked(monotonic_now_ns);
        if(impl_->upstream_health.has_value()
           && !impl_->upstream_matches_locked(monotonic_now_ns)) {
            expired = true;
        }
        if(!impl_->pose_history.empty()
           && (monotonic_now_ns < impl_->pose_history.back().receive_monotonic_ns
               || monotonic_now_ns - impl_->pose_history.back().receive_monotonic_ns
                          > impl_->config.pose_receive_validity_ns)) {
            expired = true;
        }
        if(expired) {
            impl_->evaluate_health_locked(monotonic_now_ns);
        }
    }

    LocalMapState LocalObservationMapper::state(std::int64_t monotonic_now_ns) const
    {
        std::shared_lock<std::shared_mutex> lock(impl_->storage->mutex);
        LocalMapState result;
        result.identity = impl_->identity;
        result.geometry = impl_->config.geometry;
        result.health = impl_->current_health;
        result.contract_fingerprint = impl_->fingerprint;
        result.effective_capabilities = impl_->effective_capabilities;
        result.last_commit = impl_->last_commit;
        result.alignment = impl_->alignment;
        if(impl_->storage->backend) {
            result.backend_capabilities = impl_->storage->backend->capabilities();
            result.known_bounds = impl_->storage->backend->known_bounds();
        }
        if(impl_->last_commit.has_value() && monotonic_now_ns >= impl_->last_commit_monotonic_ns) {
            const auto age = monotonic_now_ns - impl_->last_commit_monotonic_ns;
            result.map_fresh = age <= impl_->config.map_freshness_ns;
            result.validity_remaining_ns = result.map_fresh
                                                   ? impl_->config.map_freshness_ns - age
                                                   : 0;
        }
        return result;
    }

    AcquireResult LocalObservationMapper::acquire_read_transaction()
    {
        std::shared_lock<std::shared_mutex> lock(impl_->storage->mutex);
        if(!impl_->storage->backend || !impl_->storage->fence->load()) {
            return {AcquireStatus::Unavailable, std::nullopt};
        }
        MapReadMetadata metadata {
                impl_->identity, impl_->config.geometry, impl_->storage->backend->known_bounds()};
        auto transaction_impl = std::make_unique<MapReadTransaction::Impl>(
                impl_->storage, std::move(lock), impl_->storage->backend.get(),
                impl_->storage->fence, std::move(metadata));
        return {AcquireStatus::Ready,
                std::optional<MapReadTransaction>(
                        MapReadTransaction(std::move(transaction_impl)))};
    }

    AcquireResult LocalObservationMapper::acquire_read_transaction(
            const CommitReceipt & receipt)
    {
        std::shared_lock<std::shared_mutex> lock(impl_->storage->mutex);
        if(receipt.mapper_session != impl_->identity.mapper_session
           || receipt.map_epoch != impl_->identity.map_epoch
           || receipt.revision != impl_->identity.revision) {
            return {AcquireStatus::Superseded, std::nullopt};
        }
        if(!impl_->storage->backend || !impl_->storage->fence->load()) {
            return {AcquireStatus::Unavailable, std::nullopt};
        }
        MapReadMetadata metadata {
                impl_->identity, impl_->config.geometry, impl_->storage->backend->known_bounds()};
        auto transaction_impl = std::make_unique<MapReadTransaction::Impl>(
                impl_->storage, std::move(lock), impl_->storage->backend.get(),
                impl_->storage->fence, std::move(metadata));
        return {AcquireStatus::Ready,
                std::optional<MapReadTransaction>(
                        MapReadTransaction(std::move(transaction_impl)))};
    }

}// namespace PerceptionLocalMap

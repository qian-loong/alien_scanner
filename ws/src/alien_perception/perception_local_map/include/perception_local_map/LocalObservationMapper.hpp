#ifndef PERCEPTION_LOCAL_MAP_LOCAL_OBSERVATION_MAPPER_HPP
#define PERCEPTION_LOCAL_MAP_LOCAL_OBSERVATION_MAPPER_HPP

#include "perception_core/health/MapperContractFingerprint.hpp"
#include "perception_core/health/mapper_health_gate.hpp"
#include "perception_core/observation/lidar_observation.hpp"
#include "perception_core/observation/pose_estimate.hpp"
#include "perception_local_map/OccupancyBackend.hpp"
#include "perception_local_map/SensorExtrinsicRegistry.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace PerceptionLocalMap {

    class OctoMapSnapshotBridge;

    struct MapperConfig {
        std::string                                vehicle_id;
        Perception::SessionID                      mapper_session {0, 0};
        MapGeometry                                geometry;
        std::string                                body_frame;
        std::vector<Perception::SensorDescriptor>  sensor_descriptors;
        Perception::MapperInputContract            input_contract;
        std::int64_t sensor_validity_ns       = 1'000'000'000LL;
        std::int64_t health_validity_ns       = 1'000'000'000LL;
        std::int64_t pose_receive_validity_ns = 1'000'000'000LL;
        std::int64_t map_freshness_ns         = 2'000'000'000LL;
        std::size_t  pose_history_limit       = 128;
        double       position_jump_threshold_m      = 2.0;
        double       orientation_jump_threshold_rad = 0.7853981633974483;
        double       extrinsic_position_tolerance_m = 1e-4;
        double       extrinsic_orientation_tolerance_rad = 1e-4;
    };

    struct UpstreamHealth {
        std::string                           producer_source_id;
        Perception::SessionID                 producer_session {0, 0};
        Perception::MapperContractFingerprint contract_fingerprint;
        Perception::HealthState               state = Perception::HealthState::Unavailable;
        std::int64_t                          receive_monotonic_ns = 0;
        std::int64_t                          validity_budget_ns = 0;
    };

    enum class InputStatus : std::uint8_t
    {
        Accepted,
        Applied,
        NoEvidence,
        Rejected,
        Unavailable,
        BackendFault
    };

    struct InputResult {
        InputStatus                  status = InputStatus::Rejected;
        std::optional<CommitReceipt> receipt;
        std::string                  diagnostic;
    };

    struct MapReadMetadata {
        MapIdentity                       identity;
        MapGeometry                       geometry;
        std::optional<AxisAlignedBounds>   known_bounds;
        std::optional<CommitProvenance>    last_commit;
        Perception::MapperContractFingerprint contract_fingerprint;
    };

    class MapReadTransaction
    {
    public:
        MapReadTransaction() noexcept;
        ~MapReadTransaction();
        MapReadTransaction(MapReadTransaction && other) noexcept;
        MapReadTransaction & operator=(MapReadTransaction && other) noexcept;
        MapReadTransaction(const MapReadTransaction &) = delete;
        MapReadTransaction & operator=(const MapReadTransaction &) = delete;

        bool is_open() const noexcept;
        std::optional<MapReadMetadata> metadata() const;
        MapQueryResult query(const MapPoint & point) const;
        RegionQueryResult query_region(const AxisAlignedBounds & bounds) const;
        bool for_each_known_cell(const std::function<void(OccupancyCell)> & visitor) const;
        void close() noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        explicit MapReadTransaction(std::unique_ptr<Impl> impl) noexcept;
        const ILocalOccupancyBackend * backend_for_snapshot() const noexcept;
        friend class LocalObservationMapper;
        friend class OctoMapSnapshotBridge;
    };

    enum class AcquireStatus : std::uint8_t
    {
        Ready,
        Superseded,
        Unavailable
    };

    struct AcquireResult {
        AcquireStatus                      status = AcquireStatus::Unavailable;
        std::optional<MapReadTransaction> transaction;
    };

    class LocalObservationMapper
    {
    public:
        LocalObservationMapper(MapperConfig config, BackendFactory backend_factory);
        ~LocalObservationMapper();

        LocalObservationMapper(const LocalObservationMapper &) = delete;
        LocalObservationMapper & operator=(const LocalObservationMapper &) = delete;

        bool submit_health(const UpstreamHealth & health);
        InputResult submit_pose(
                const Perception::PoseEstimate & pose,
                std::int64_t receive_monotonic_ns);
        InputResult submit_observation(
                const Perception::LidarObservation & observation,
                const SensorExtrinsicSample & extrinsic,
                std::int64_t receive_monotonic_ns);
        bool mark_sensor_unavailable(
                const Perception::SensorID & sensor_id,
                std::int64_t monotonic_now_ns);
        bool submit_alignment(const AlignmentReference & reference);
        void tick(std::int64_t monotonic_now_ns);

        LocalMapState state(std::int64_t monotonic_now_ns) const;
        AcquireResult acquire_read_transaction();
        AcquireResult acquire_read_transaction(const CommitReceipt & receipt);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_LOCAL_OBSERVATION_MAPPER_HPP

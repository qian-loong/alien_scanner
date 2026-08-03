#include "DeterministicVoxelBackend.hpp"

#include "perception_core/health/MapperContractFingerprint.hpp"
#include "perception_fixtures/FixtureScene.hpp"
#include "perception_local_map/LocalObservationMapper.hpp"
#include "perception_local_map/OctoMapBackend.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <functional>
#include <future>
#include <limits>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <sys/resource.h>

namespace PerceptionLocalMap::Test {

    static_assert(!std::is_copy_constructible<MapReadTransaction>::value);
    static_assert(!std::is_copy_assignable<MapReadTransaction>::value);
    static_assert(std::is_same<
                  decltype(std::declval<const MapReadTransaction &>().query(MapPoint {})),
                  MapQueryResult>::value);

    namespace {

        MapGeometry geometry(double resolution = 1.0)
        {
            return {resolution, {0.0, 0.0, 0.0}, "map"};
        }

        void run_backend_conformance(
                const std::function<std::unique_ptr<ILocalOccupancyBackend>(const MapGeometry &)> & factory)
        {
            auto backend = factory(geometry());
            ASSERT_TRUE(backend->capabilities().has_required_capabilities());
            EXPECT_FALSE(backend->known_bounds().has_value());
            EXPECT_EQ(backend->query({0.5, 0.5, 0.5}).state, OccupancyState::Unknown);
            EXPECT_EQ(backend->query({NAN, 0.0, 0.0}).status, QueryStatus::Invalid);
            EXPECT_EQ(backend->query({40'000.5, 0.5, 0.5}).status, QueryStatus::OutOfRange);

            const auto negative = quantize_point({-0.001, 0.0, 0.0}, geometry());
            ASSERT_EQ(negative.status, QueryStatus::Ok);
            EXPECT_EQ(negative.index, (VoxelIndex {-1, 0, 0}));
            EXPECT_EQ(quantize_point({0.0, 0.0, 0.0}, geometry()).index, (VoxelIndex {0, 0, 0}));

            const auto applied = backend->apply(
                    EvidenceBatch {{VoxelIndex {-1, 0, 0}, VoxelIndex {1, 0, 0}},
                                   {VoxelIndex {1, 0, 0}, VoxelIndex {2, 0, 0}}});
            EXPECT_EQ(applied.status, ApplyStatus::Applied);
            EXPECT_EQ(applied.changed_cell_count, 3U);
            EXPECT_EQ(backend->query({-0.5, 0.5, 0.5}).state, OccupancyState::Free);
            EXPECT_EQ(backend->query({1.5, 0.5, 0.5}).state, OccupancyState::Occupied);
            EXPECT_EQ(backend->query({2.5, 0.5, 0.5}).state, OccupancyState::Occupied);

            const auto region = backend->query_region({{-1.0, 0.0, 0.0}, {3.0, 1.0, 1.0}});
            ASSERT_EQ(region.status, QueryStatus::Ok);
            ASSERT_EQ(region.cells.size(), 3U);
            EXPECT_LT(region.cells[0].index, region.cells[1].index);
            EXPECT_LT(region.cells[1].index, region.cells[2].index);

            const auto snapshot = region.cells;
            std::vector<OccupancyCell> visited;
            ASSERT_TRUE(backend->for_each_known_cell(
                    [&](OccupancyCell cell) { visited.push_back(std::move(cell)); }));
            ASSERT_EQ(visited.size(), snapshot.size());
            for(std::size_t index = 0; index < snapshot.size(); ++index) {
                EXPECT_EQ(visited[index].index, snapshot[index].index);
                EXPECT_EQ(visited[index].state, snapshot[index].state);
            }
            const auto half_open = backend->query_region(
                    {{-1.0, 0.0, 0.0}, {2.0, 1.0, 1.0}});
            ASSERT_EQ(half_open.status, QueryStatus::Ok);
            ASSERT_EQ(half_open.cells.size(), 2U);
            EXPECT_EQ(half_open.cells.back().index, (VoxelIndex {1, 0, 0}));
            EXPECT_EQ(backend->apply({{}, {}}).status, ApplyStatus::NoEvidence);
            EXPECT_EQ(
                    backend->apply(
                            {{VoxelIndex {std::numeric_limits<std::int64_t>::max(), 0, 0}}, {}})
                            .status,
                    ApplyStatus::Rejected);
            const auto after_rejection = backend->query_region(
                    {{-1.0, 0.0, 0.0}, {3.0, 1.0, 1.0}});
            ASSERT_EQ(after_rejection.status, QueryStatus::Ok);
            ASSERT_EQ(after_rejection.cells.size(), snapshot.size());
            for(std::size_t index = 0; index < snapshot.size(); ++index) {
                EXPECT_EQ(after_rejection.cells[index].index, snapshot[index].index);
                EXPECT_EQ(after_rejection.cells[index].state, snapshot[index].state);
            }

            backend->reset(geometry(0.5));
            EXPECT_FALSE(backend->known_bounds().has_value());
            EXPECT_EQ(backend->query({1.5, 0.5, 0.5}).state, OccupancyState::Unknown);
            EXPECT_EQ(
                    backend->query_region({{0.0, 0.0, 0.0}, {0.0, 1.0, 1.0}}).status,
                    QueryStatus::Invalid);

            backend->reset(geometry());
            EXPECT_EQ(
                    backend->query({-32'767.5, 0.5, 0.5}).status,
                    QueryStatus::Ok);
            EXPECT_EQ(
                    backend->query({32'767.5, 0.5, 0.5}).status,
                    QueryStatus::Ok);
            EXPECT_EQ(
                    backend->query({-32'768.5, 0.5, 0.5}).status,
                    QueryStatus::OutOfRange);
            EXPECT_EQ(
                    backend->query({32'768.5, 0.5, 0.5}).status,
                    QueryStatus::OutOfRange);
            const auto boundary = backend->apply(
                    {{}, {VoxelIndex {-32'768, 0, 0}, VoxelIndex {32'767, 0, 0}}});
            EXPECT_EQ(boundary.status, ApplyStatus::Applied);
            EXPECT_EQ(boundary.changed_cell_count, 2U);
            EXPECT_EQ(
                    backend->apply({{}, {VoxelIndex {-32'769, 0, 0}}}).status,
                    ApplyStatus::Rejected);
            EXPECT_EQ(
                    backend->apply({{}, {VoxelIndex {32'768, 0, 0}}}).status,
                    ApplyStatus::Rejected);
        }

        std::optional<AxisAlignedBounds> full_scan_known_bounds(
                const ILocalOccupancyBackend & backend)
        {
            std::optional<VoxelIndex> minimum;
            std::optional<VoxelIndex> maximum;
            const bool scanned = backend.for_each_known_cell([&](OccupancyCell cell) {
                if(!minimum.has_value()) {
                    minimum = cell.index;
                    maximum = cell.index;
                    return;
                }
                minimum->x = std::min(minimum->x, cell.index.x);
                minimum->y = std::min(minimum->y, cell.index.y);
                minimum->z = std::min(minimum->z, cell.index.z);
                maximum->x = std::max(maximum->x, cell.index.x);
                maximum->y = std::max(maximum->y, cell.index.y);
                maximum->z = std::max(maximum->z, cell.index.z);
            });
            if(!scanned || !minimum.has_value()) {
                return std::nullopt;
            }
            return voxel_bounds(
                    *minimum,
                    {maximum->x + 1, maximum->y + 1, maximum->z + 1}, backend.geometry());
        }

        void expect_known_bounds_match_full_scan(const ILocalOccupancyBackend & backend)
        {
            const auto cached = backend.known_bounds();
            const auto scanned = full_scan_known_bounds(backend);
            ASSERT_EQ(cached.has_value(), scanned.has_value());
            if(!cached.has_value()) {
                return;
            }
            EXPECT_EQ(cached->minimum.x, scanned->minimum.x);
            EXPECT_EQ(cached->minimum.y, scanned->minimum.y);
            EXPECT_EQ(cached->minimum.z, scanned->minimum.z);
            EXPECT_EQ(cached->maximum.x, scanned->maximum.x);
            EXPECT_EQ(cached->maximum.y, scanned->maximum.y);
            EXPECT_EQ(cached->maximum.z, scanned->maximum.z);
        }

        Perception::SensorDescriptor descriptor(
                Perception::RayEvidenceCapability capability,
                Perception::SensorType type = Perception::SensorType::LIDAR_2D)
        {
            return {
                    Perception::SensorID {"front"}, type, "front_link",
                    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                    Perception::FieldOfView {0.0, 0.0, 0.0, 0.0}, 1.0, 0.1, 3.0,
                    capability};
        }

        MapperConfig mapper_config(
                Perception::RayEvidenceCapability capability,
                bool requires_pose = false)
        {
            MapperConfig config;
            config.vehicle_id = "vehicle-1";
            config.mapper_session = {100, 7};
            config.geometry = geometry();
            config.body_frame = "base_link";
            config.sensor_descriptors = {descriptor(capability)};
            config.input_contract.requires_pose = requires_pose;
            config.input_contract.expected_pose_frame = requires_pose
                                                                  ? std::optional<std::string>("map")
                                                                  : std::nullopt;
            config.input_contract.pose_freshness_threshold = Perception::Duration::from_seconds(1.0);
            config.input_contract.recovery_stability_samples = 1;
            config.input_contract.minimum_viable = {
                    Perception::SensorRequirement {
                            Perception::SensorType::LIDAR_2D, 1, {}, capability}};
            config.input_contract.degraded_combinations = {
                    Perception::DegradedCombination {
                            {Perception::SensorRequirement {
                                    Perception::SensorType::LIDAR_2D, 1, {},
                                    Perception::RayEvidenceCapability::HitOnly}},
                            "one lidar"}};
            config.position_jump_threshold_m = 100.0;
            return config;
        }

        UpstreamHealth health_for(const MapperConfig & config, std::int64_t now)
        {
            return {
                    "perception_input", {1, 2},
                    Perception::MapperContractFingerprint::compute(
                            config.sensor_descriptors, config.input_contract),
                    Perception::HealthState::Healthy, now, 1'000'000'000};
        }

        SensorExtrinsicSample extrinsic()
        {
            return {
                    Perception::SensorID {"front"}, {1, 2}, "front_link",
                    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
        }

        Perception::LidarObservation scan_observation(
                Perception::RayEvidenceCapability capability,
                Perception::Timestamp stamp,
                float range)
        {
            return {
                    Perception::SensorID {"front"}, {1, 2}, "front_link", "vehicle_clock",
                    stamp,
                    Perception::Scan2D {0.0, 0.0, 1.0, 0.1, 3.0, {range}, {}},
                    capability};
        }

        std::unique_ptr<LocalObservationMapper> make_mapper(const MapperConfig & config)
        {
            return std::make_unique<LocalObservationMapper>(
                    config,
                    [](const MapGeometry & map_geometry) {
                        return std::make_unique<DeterministicVoxelBackend>(map_geometry);
                    });
        }

        class ThrowOnceBackend final : public ILocalOccupancyBackend
        {
        public:
            ThrowOnceBackend(const MapGeometry & value, bool should_throw)
                : delegate_(value)
                , should_throw_(should_throw)
            {}

            MapGeometry geometry() const noexcept override { return delegate_.geometry(); }
            BackendCapabilities capabilities() const noexcept override { return delegate_.capabilities(); }
            std::optional<AxisAlignedBounds> known_bounds() const override { return delegate_.known_bounds(); }
            MapQueryResult query(const MapPoint & point) const override { return delegate_.query(point); }
            RegionQueryResult query_region(const AxisAlignedBounds & bounds) const override
            {
                return delegate_.query_region(bounds);
            }
            bool for_each_known_cell(
                    const std::function<void(OccupancyCell)> & visitor) const override
            {
                return delegate_.for_each_known_cell(visitor);
            }
            void reset(const MapGeometry & value) override { delegate_.reset(value); }
            ApplyResult apply(const EvidenceBatch & batch) override
            {
                if(should_throw_) {
                    should_throw_ = false;
                    static_cast<void>(delegate_.apply(batch));
                    throw std::runtime_error("injected mutation fault");
                }
                return delegate_.apply(batch);
            }

        private:
            DeterministicVoxelBackend delegate_;
            bool should_throw_;
        };

        class ControlledFaultBackend final : public ILocalOccupancyBackend
        {
        public:
            ControlledFaultBackend(
                    const MapGeometry & value,
                    std::shared_ptr<std::atomic_bool> throw_next)
                : delegate_(value)
                , throw_next_(std::move(throw_next))
            {}

            MapGeometry geometry() const noexcept override { return delegate_.geometry(); }
            BackendCapabilities capabilities() const noexcept override { return delegate_.capabilities(); }
            std::optional<AxisAlignedBounds> known_bounds() const override { return delegate_.known_bounds(); }
            MapQueryResult query(const MapPoint & point) const override { return delegate_.query(point); }
            RegionQueryResult query_region(const AxisAlignedBounds & bounds) const override
            {
                return delegate_.query_region(bounds);
            }
            bool for_each_known_cell(
                    const std::function<void(OccupancyCell)> & visitor) const override
            {
                return delegate_.for_each_known_cell(visitor);
            }
            void reset(const MapGeometry & value) override { delegate_.reset(value); }
            ApplyResult apply(const EvidenceBatch & batch) override
            {
                const auto result = delegate_.apply(batch);
                if(throw_next_->exchange(false)) {
                    throw std::runtime_error("controlled mutation fault");
                }
                return result;
            }

        private:
            DeterministicVoxelBackend delegate_;
            std::shared_ptr<std::atomic_bool> throw_next_;
        };

        enum class InvalidApplyMode : std::uint8_t
        {
            UnknownStatus,
            ContradictoryNoEvidence
        };

        class InvalidApplyResultBackend final : public ILocalOccupancyBackend
        {
        public:
            InvalidApplyResultBackend(
                    const MapGeometry & value,
                    InvalidApplyMode mode)
                : delegate_(value)
                , mode_(mode)
            {}

            MapGeometry geometry() const noexcept override { return delegate_.geometry(); }
            BackendCapabilities capabilities() const noexcept override { return delegate_.capabilities(); }
            std::optional<AxisAlignedBounds> known_bounds() const override { return delegate_.known_bounds(); }
            MapQueryResult query(const MapPoint & point) const override { return delegate_.query(point); }
            RegionQueryResult query_region(const AxisAlignedBounds & bounds) const override
            {
                return delegate_.query_region(bounds);
            }
            bool for_each_known_cell(
                    const std::function<void(OccupancyCell)> & visitor) const override
            {
                return delegate_.for_each_known_cell(visitor);
            }
            void reset(const MapGeometry & value) override { delegate_.reset(value); }
            ApplyResult apply(const EvidenceBatch &) override
            {
                if(mode_ == InvalidApplyMode::ContradictoryNoEvidence) {
                    return {ApplyStatus::NoEvidence, 0U, {}};
                }
                return {static_cast<ApplyStatus>(0xFFU), 0U, {}};
            }

        private:
            DeterministicVoxelBackend delegate_;
            InvalidApplyMode mode_;
        };

    }// namespace

    TEST(BackendConformance, DeterministicVoxelBackend)
    {
        run_backend_conformance([](const MapGeometry & value) {
            return std::make_unique<DeterministicVoxelBackend>(value);
        });
    }

    TEST(BackendConformance, OctoMapBackend)
    {
        run_backend_conformance([](const MapGeometry & value) {
            return std::make_unique<OctoMapBackend>(value);
        });
    }

    TEST(BackendConformance, OctoMapChangedCountTracksVisibleClassification)
    {
        OctoMapBackend backend(geometry());
        const VoxelIndex cell {0, 0, 0};
        ASSERT_EQ(backend.apply({{}, {cell}}).changed_cell_count, 1U);
        ASSERT_EQ(backend.query({0.5, 0.5, 0.5}).state, OccupancyState::Occupied);
        const auto weakened = backend.apply({{cell}, {}});
        ASSERT_EQ(weakened.status, ApplyStatus::Applied);
        EXPECT_EQ(backend.query({0.5, 0.5, 0.5}).state, OccupancyState::Occupied);
        EXPECT_EQ(weakened.changed_cell_count, 0U);
    }

    TEST(BackendConformance, OctoMapKnownBoundsMatchFullScan)
    {
        OctoMapBackend backend(geometry());
        EXPECT_FALSE(backend.known_bounds().has_value());
        expect_known_bounds_match_full_scan(backend);

        const std::vector<EvidenceBatch> rounds {
                EvidenceBatch {{}, {VoxelIndex {0, 0, 0}}},
                EvidenceBatch {{VoxelIndex {-3, -4, -5}}, {VoxelIndex {2, 1, 4}}},
                EvidenceBatch {{VoxelIndex {-3, -4, -5}, VoxelIndex {0, 0, 0}}, {}},
                EvidenceBatch {{VoxelIndex {7, -9, 3}}, {VoxelIndex {-11, 6, -2}}},
                EvidenceBatch {{VoxelIndex {1, 1, 1}}, {VoxelIndex {1, 1, 1}}},
                EvidenceBatch {{VoxelIndex {-11, 6, -2}}, {VoxelIndex {7, -9, 3}}}};
        for(const auto & round : rounds) {
            ASSERT_EQ(backend.apply(round).status, ApplyStatus::Applied);
            expect_known_bounds_match_full_scan(backend);
        }

        const auto extremes = backend.known_bounds();
        ASSERT_TRUE(extremes.has_value());
        EXPECT_EQ(
                backend.apply({{}, {VoxelIndex {std::numeric_limits<std::int64_t>::max(), 0, 0}}})
                        .status,
                ApplyStatus::Rejected);
        const auto after_rejection = backend.known_bounds();
        ASSERT_TRUE(after_rejection.has_value());
        EXPECT_EQ(after_rejection->minimum.x, extremes->minimum.x);
        EXPECT_EQ(after_rejection->maximum.x, extremes->maximum.x);
        expect_known_bounds_match_full_scan(backend);

        backend.reset(geometry(0.5));
        EXPECT_FALSE(backend.known_bounds().has_value());
        expect_known_bounds_match_full_scan(backend);
        ASSERT_EQ(backend.apply({{}, {VoxelIndex {-2, 3, -4}}}).status, ApplyStatus::Applied);
        expect_known_bounds_match_full_scan(backend);
        ASSERT_EQ(backend.apply({{VoxelIndex {5, -6, 7}}, {}}).status, ApplyStatus::Applied);
        expect_known_bounds_match_full_scan(backend);
    }

    TEST(EvidenceMatrix, HitOnlyDoesNotInventFreeSpace)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        const auto result = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {1}, 2.0F),
                extrinsic(), 20);
        ASSERT_EQ(result.status, InputStatus::Applied);
        auto transaction = mapper->acquire_read_transaction(result.receipt.value());
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(transaction.transaction->query({2.5, 0.5, 0.5}).state, OccupancyState::Occupied);
        EXPECT_EQ(transaction.transaction->query({1.5, 0.5, 0.5}).state, OccupancyState::Unknown);
    }

    TEST(EvidenceMatrix, HitRayWritesFreeBeforeHitOnly)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitRay);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        const auto result = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitRay,
                        Perception::Timestamp {1}, 2.0F),
                extrinsic(), 20);
        ASSERT_EQ(result.status, InputStatus::Applied);
        auto transaction = mapper->acquire_read_transaction();
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(transaction.transaction->query({0.5, 0.5, 0.5}).state, OccupancyState::Free);
        EXPECT_EQ(transaction.transaction->query({1.5, 0.5, 0.5}).state, OccupancyState::Free);
        EXPECT_EQ(transaction.transaction->query({2.5, 0.5, 0.5}).state, OccupancyState::Occupied);
    }

    TEST(EvidenceMatrix, FullRayNoReturnKeepsBoundaryUnknown)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::FullRay);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        const auto result = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::FullRay,
                        Perception::Timestamp {1}, std::numeric_limits<float>::infinity()),
                extrinsic(), 20);
        ASSERT_EQ(result.status, InputStatus::Applied);
        auto transaction = mapper->acquire_read_transaction();
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(transaction.transaction->query({2.5, 0.5, 0.5}).state, OccupancyState::Free);
        EXPECT_EQ(transaction.transaction->query({3.5, 0.5, 0.5}).state, OccupancyState::Unknown);
    }

    TEST(EvidenceMatrix, CanonicalDdaIncludesShortGrazingVoxelCrossing)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitRay);
        const double direction_x = 3.0;
        const double direction_y = 2.832;
        const double angle = std::atan2(direction_y, direction_x);
        const float range = static_cast<float>(std::hypot(direction_x, direction_y));
        auto & frozen = config.sensor_descriptors.front();
        frozen.fov.horizontal_min_rad = angle;
        frozen.fov.horizontal_max_rad = angle;
        frozen.angular_resolution_rad = 1.0;
        frozen.range_max_m = 5.0;
        frozen.mounting_position = Eigen::Vector3d(0.1, 0.1, 0.0);

        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        auto sample = extrinsic();
        sample.translation = frozen.mounting_position;
        Perception::LidarObservation observation {
                Perception::SensorID {"front"}, {1, 2}, "front_link", "vehicle_clock",
                Perception::Timestamp {1},
                Perception::Scan2D {angle, angle, 1.0, 0.1, 5.0, {range}, {}},
                Perception::RayEvidenceCapability::HitRay};
        const auto applied = mapper->submit_observation(observation, sample, 20);
        ASSERT_EQ(applied.status, InputStatus::Applied);
        auto transaction = mapper->acquire_read_transaction(applied.receipt.value());
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(
                transaction.transaction->query({1.5, 0.5, 0.5}).state,
                OccupancyState::Free);
    }

    TEST(EvidenceMatrix, Cloud3DIsHitOnlyAndRejectsWholeOutOfRangeBatch)
    {
        auto config = mapper_config(
                Perception::RayEvidenceCapability::HitOnly);
        config.sensor_descriptors.front() = descriptor(
                Perception::RayEvidenceCapability::HitOnly,
                Perception::SensorType::LIDAR_3D);
        config.input_contract.minimum_viable.front().type =
                Perception::SensorType::LIDAR_3D;
        config.input_contract.degraded_combinations.front().requirements.front().type =
                Perception::SensorType::LIDAR_3D;
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));

        Perception::LidarObservation valid {
                Perception::SensorID {"front"}, {1, 2}, "front_link", "vehicle_clock",
                Perception::Timestamp {1},
                Perception::Cloud3D {{{2.0F, 0.0F, 0.0F, 1.0F}}},
                Perception::RayEvidenceCapability::HitOnly};
        const auto applied = mapper->submit_observation(valid, extrinsic(), 20);
        ASSERT_EQ(applied.status, InputStatus::Applied);
        auto transaction = mapper->acquire_read_transaction(applied.receipt.value());
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(transaction.transaction->query({2.5, 0.5, 0.5}).state, OccupancyState::Occupied);
        EXPECT_EQ(transaction.transaction->query({1.5, 0.5, 0.5}).state, OccupancyState::Unknown);
        transaction.transaction->close();

        valid.origin_stamp = Perception::Timestamp {2};
        std::get<Perception::Cloud3D>(valid.data).points.front().x = 4.0F;
        EXPECT_EQ(
                mapper->submit_observation(valid, extrinsic(), 21).status,
                InputStatus::Rejected);
        EXPECT_EQ(mapper->state(21).identity.revision, 1U);
    }

    TEST(MapperConfiguration, RejectsCloudCapabilityAboveHitOnlyAndBackendGeometryDrift)
    {
        auto cloud_config = mapper_config(
                Perception::RayEvidenceCapability::FullRay);
        cloud_config.sensor_descriptors.front().type = Perception::SensorType::LIDAR_3D;
        cloud_config.input_contract.minimum_viable.front().type =
                Perception::SensorType::LIDAR_3D;
        EXPECT_THROW(make_mapper(cloud_config), std::invalid_argument);

        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        EXPECT_THROW(
                LocalObservationMapper(
                        config,
                        [](const MapGeometry & value) {
                            auto changed = value;
                            changed.resolution_m *= 0.5;
                            return std::make_unique<DeterministicVoxelBackend>(changed);
                        }),
                std::invalid_argument);

        auto pose_frame_drift = mapper_config(
                Perception::RayEvidenceCapability::HitOnly, true);
        pose_frame_drift.input_contract.expected_pose_frame = "odom";
        EXPECT_THROW(make_mapper(pose_frame_drift), std::invalid_argument);
    }

    TEST(MapperRevision, ReplayNoEvidenceAndSupersededReceiptAreDistinct)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        const auto first = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {1}, 2.0F),
                extrinsic(), 20);
        ASSERT_EQ(first.status, InputStatus::Applied);
        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {1}, 2.0F),
                              extrinsic(), 21)
                        .status,
                InputStatus::Rejected);
        const auto second = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {2}, 2.0F),
                extrinsic(), 22);
        ASSERT_EQ(second.status, InputStatus::Applied);
        EXPECT_EQ(first.receipt->revision + 1U, second.receipt->revision);
        EXPECT_EQ(mapper->state(22).last_commit->changed_cell_count, 0U);
        const auto revision = second.receipt->revision;
        const auto no_evidence = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {3},
                        std::numeric_limits<float>::quiet_NaN()),
                extrinsic(), 23);
        EXPECT_EQ(no_evidence.status, InputStatus::NoEvidence);
        EXPECT_EQ(mapper->state(23).identity.revision, revision);
        EXPECT_EQ(mapper->state(23).last_commit->observation_stamp.nanoseconds, 2);
        EXPECT_EQ(
                mapper->acquire_read_transaction(first.receipt.value()).status,
                AcquireStatus::Superseded);

        auto transaction = mapper->acquire_read_transaction(second.receipt.value());
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        MapReadTransaction moved = std::move(transaction.transaction.value());
        EXPECT_TRUE(moved.is_open());
        EXPECT_FALSE(transaction.transaction->is_open());
        moved.close();
        EXPECT_EQ(moved.query({2.5, 0.5, 0.5}).status, QueryStatus::TransactionClosed);
    }

    TEST(PoseEpoch, ResetRevokesAlignmentAndClearsOldOccupancy)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly, true);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        Perception::PoseEstimate pose {
                Perception::SourceID {"odom"}, {1, 2}, "map", "vehicle_clock",
                Perception::Timestamp {1}, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), std::nullopt, 1.0,
                Perception::Duration {0}, 0};
        EXPECT_EQ(mapper->submit_pose(pose, 11).status, InputStatus::Accepted);
        const auto first = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {1}, 2.0F),
                extrinsic(), 12);
        ASSERT_EQ(first.status, InputStatus::Applied);
        const auto before = mapper->state(12);
        ASSERT_TRUE(mapper->submit_alignment(
                {"vehicle-1", config.mapper_session, before.identity.map_epoch,
                 "fixture", {9, 1}, 1, 1, AlignmentStatus::Committed}));

        pose.stamp = Perception::Timestamp {2};
        pose.reset_epoch = 1;
        EXPECT_EQ(mapper->submit_pose(pose, 13).status, InputStatus::Accepted);
        const auto after = mapper->state(13);
        EXPECT_EQ(after.identity.map_epoch, before.identity.map_epoch + 1U);
        EXPECT_EQ(after.identity.revision, 0U);
        ASSERT_TRUE(after.alignment.has_value());
        EXPECT_EQ(after.alignment->status, AlignmentStatus::Revoked);
        auto transaction = mapper->acquire_read_transaction();
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(transaction.transaction->query({2.5, 0.5, 0.5}).state, OccupancyState::Unknown);
    }

    TEST(BackendFault, MutationExceptionClosesChainAndRecoversOnlyIntoEmptyEpoch)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto creations = std::make_shared<std::size_t>(0U);
        LocalObservationMapper mapper(
                config,
                [creations](const MapGeometry & value) {
                    const bool should_throw = (*creations)++ == 0U;
                    return std::make_unique<ThrowOnceBackend>(value, should_throw);
                });
        ASSERT_TRUE(mapper.submit_health(health_for(config, 10)));
        const auto result = mapper.submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {1}, 2.0F),
                extrinsic(), 20);
        EXPECT_EQ(result.status, InputStatus::BackendFault);
        const auto state = mapper.state(20);
        EXPECT_EQ(state.identity.map_epoch, 2U);
        EXPECT_EQ(state.identity.revision, 0U);
        EXPECT_FALSE(state.known_bounds.has_value());
        EXPECT_EQ(state.health, Perception::HealthState::Unavailable);
        EXPECT_TRUE(state.effective_capabilities.satisfied_combination_id.empty());
        EXPECT_EQ(state.effective_capabilities.active_sensor_count, 0U);
        EXPECT_TRUE(state.effective_capabilities.is_consistent());
        auto transaction = mapper.acquire_read_transaction();
        ASSERT_EQ(transaction.status, AcquireStatus::Ready);
        EXPECT_EQ(transaction.transaction->query({2.5, 0.5, 0.5}).state, OccupancyState::Unknown);
    }

    TEST(BackendFault, InvalidApplyResultsCloseChain)
    {
        for(const auto mode : {
                    InvalidApplyMode::UnknownStatus,
                    InvalidApplyMode::ContradictoryNoEvidence}) {
            const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
            LocalObservationMapper mapper(
                    config,
                    [mode](const MapGeometry & value) {
                        return std::make_unique<InvalidApplyResultBackend>(value, mode);
                    });
            ASSERT_TRUE(mapper.submit_health(health_for(config, 10)));
            const auto result = mapper.submit_observation(
                    scan_observation(
                            Perception::RayEvidenceCapability::HitOnly,
                            Perception::Timestamp {1}, 2.0F),
                    extrinsic(), 20);
            EXPECT_EQ(result.status, InputStatus::BackendFault);
            const auto state = mapper.state(20);
            EXPECT_EQ(state.identity.map_epoch, 2U);
            EXPECT_EQ(state.identity.revision, 0U);
            EXPECT_EQ(state.health, Perception::HealthState::Unavailable);
            EXPECT_FALSE(state.known_bounds.has_value());
            EXPECT_TRUE(state.effective_capabilities.satisfied_combination_id.empty());
            EXPECT_EQ(state.effective_capabilities.active_sensor_count, 0U);
        }
    }

    TEST(ReadTransaction, OutlivingMapperIsRevokedWithoutDanglingBackendOrMutex)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        MapReadTransaction transaction;
        {
            auto mapper = make_mapper(config);
            ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
            const auto applied = mapper->submit_observation(
                    scan_observation(
                            Perception::RayEvidenceCapability::HitOnly,
                            Perception::Timestamp {1}, 2.0F),
                    extrinsic(), 20);
            ASSERT_EQ(applied.status, InputStatus::Applied);
            auto acquired = mapper->acquire_read_transaction();
            ASSERT_EQ(acquired.status, AcquireStatus::Ready);
            transaction = std::move(acquired.transaction.value());
            EXPECT_TRUE(transaction.is_open());
        }
        EXPECT_FALSE(transaction.is_open());
        EXPECT_EQ(transaction.query({2.5, 0.5, 0.5}).status, QueryStatus::TransactionClosed);
        transaction.close();
    }

    TEST(ReadTransaction, MutationFaultWaitsForReaderThenRevokesOldRevision)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        const auto throw_next = std::make_shared<std::atomic_bool>(false);
        LocalObservationMapper mapper(
                config,
                [throw_next](const MapGeometry & value) {
                    return std::make_unique<ControlledFaultBackend>(value, throw_next);
                });
        ASSERT_TRUE(mapper.submit_health(health_for(config, 10)));
        const auto first = mapper.submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {1}, 2.0F),
                extrinsic(), 20);
        ASSERT_EQ(first.status, InputStatus::Applied);
        auto acquired = mapper.acquire_read_transaction(first.receipt.value());
        ASSERT_EQ(acquired.status, AcquireStatus::Ready);
        ASSERT_TRUE(acquired.transaction.has_value());

        throw_next->store(true);
        std::atomic_bool worker_started {false};
        auto fault = std::async(std::launch::async, [&]() {
            worker_started.store(true);
            return mapper.submit_observation(
                    scan_observation(
                            Perception::RayEvidenceCapability::HitOnly,
                            Perception::Timestamp {2}, 2.0F),
                    extrinsic(), 21);
        });
        while(!worker_started.load()) {
            std::this_thread::yield();
        }
        EXPECT_EQ(fault.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
        EXPECT_EQ(
                acquired.transaction->query({2.5, 0.5, 0.5}).state,
                OccupancyState::Occupied);

        acquired.transaction->close();
        const auto fault_result = fault.get();
        EXPECT_EQ(fault_result.status, InputStatus::BackendFault);
        EXPECT_FALSE(acquired.transaction->is_open());
        const auto state = mapper.state(22);
        EXPECT_EQ(state.identity.map_epoch, first.receipt->map_epoch + 1U);
        EXPECT_EQ(state.identity.revision, 0U);
        EXPECT_EQ(
                mapper.acquire_read_transaction(first.receipt.value()).status,
                AcquireStatus::Superseded);
        auto empty_epoch = mapper.acquire_read_transaction();
        ASSERT_EQ(empty_epoch.status, AcquireStatus::Ready);
        EXPECT_EQ(
                empty_epoch.transaction->query({2.5, 0.5, 0.5}).state,
                OccupancyState::Unknown);
    }

    TEST(HealthBoundary, FingerprintMismatchAndExpiryNeverAdvanceRevisionOrFreshness)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto mapper = make_mapper(config);
        auto mismatched = health_for(config, 10);
        mismatched.contract_fingerprint.hex_digest[0] =
                mismatched.contract_fingerprint.hex_digest[0] == '0' ? '1' : '0';
        EXPECT_FALSE(mapper->submit_health(mismatched));
        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {1}, 2.0F),
                              extrinsic(), 20)
                        .status,
                InputStatus::Unavailable);
        EXPECT_EQ(mapper->state(20).identity.revision, 0U);

        ASSERT_TRUE(mapper->submit_health(health_for(config, 30)));
        const auto applied = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {2}, 2.0F),
                extrinsic(), 40);
        ASSERT_EQ(applied.status, InputStatus::Applied);
        const auto revision = mapper->state(40).identity.revision;
        mapper->tick(2'000'000'100LL);
        const auto expired = mapper->state(2'000'000'100LL);
        EXPECT_EQ(expired.health, Perception::HealthState::Unavailable);
        EXPECT_EQ(expired.identity.revision, revision);
        EXPECT_FALSE(expired.map_fresh);
        EXPECT_EQ(expired.validity_remaining_ns, 0);
    }

    TEST(HealthBoundary, RecoverySamplesBlockRevisionUntilStable)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        config.input_contract.recovery_stability_samples = 3;
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));

        for(std::int64_t stamp = 1; stamp <= 2; ++stamp) {
            const auto result = mapper->submit_observation(
                    scan_observation(
                            Perception::RayEvidenceCapability::HitOnly,
                            Perception::Timestamp {stamp}, 2.0F),
                    extrinsic(), 20 + stamp);
            EXPECT_EQ(result.status, InputStatus::Unavailable);
            EXPECT_EQ(mapper->state(20 + stamp).identity.revision, 0U);
        }
        const auto stable = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {3}, 2.0F),
                extrinsic(), 23);
        EXPECT_EQ(stable.status, InputStatus::Applied);
        EXPECT_EQ(mapper->state(23).identity.revision, 1U);
    }

    TEST(PoseBoundary, CrossClockDomainObservationFailsClosed)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly, true);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        Perception::PoseEstimate pose {
                Perception::SourceID {"odom"}, {1, 2}, "map", "pose_clock",
                Perception::Timestamp {1}, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), std::nullopt, 1.0,
                Perception::Duration {0}, 0};
        ASSERT_EQ(mapper->submit_pose(pose, 11).status, InputStatus::Accepted);
        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {2}, 2.0F),
                              extrinsic(), 12)
                        .status,
                InputStatus::Unavailable);
        EXPECT_EQ(mapper->state(12).identity.revision, 0U);
    }

    TEST(PoseBoundary, ReplayInvalidatesPoseHistoryWithoutAdvancingEpoch)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly, true);
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        Perception::PoseEstimate pose {
                Perception::SourceID {"odom"}, {1, 2}, "map", "vehicle_clock",
                Perception::Timestamp {1}, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), std::nullopt, 1.0,
                Perception::Duration {0}, 0};
        ASSERT_EQ(mapper->submit_pose(pose, 11).status, InputStatus::Accepted);
        ASSERT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {1}, 2.0F),
                              extrinsic(), 12)
                        .status,
                InputStatus::Applied);
        const auto before = mapper->state(12);

        EXPECT_EQ(mapper->submit_pose(pose, 13).status, InputStatus::Rejected);
        const auto rejected = mapper->state(13);
        EXPECT_EQ(rejected.health, Perception::HealthState::Unavailable);
        EXPECT_EQ(rejected.identity.map_epoch, before.identity.map_epoch);
        EXPECT_EQ(rejected.identity.revision, before.identity.revision);
        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {2}, 2.0F),
                              extrinsic(), 14)
                        .status,
                InputStatus::Unavailable);

        pose.stamp = Perception::Timestamp {2};
        ASSERT_EQ(mapper->submit_pose(pose, 15).status, InputStatus::Accepted);
        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {2}, 2.0F),
                              extrinsic(), 16)
                        .status,
                InputStatus::Applied);
    }

    TEST(PoseBoundary, LowQualityJumpDoesNotAdvanceEpochOrRemainSelectable)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly, true);
        config.input_contract.minimum_pose_quality = 0.8;
        config.position_jump_threshold_m = 1.0;
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        Perception::PoseEstimate pose {
                Perception::SourceID {"odom"}, {1, 2}, "map", "vehicle_clock",
                Perception::Timestamp {1}, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), std::nullopt, 1.0,
                Perception::Duration {0}, 0};
        ASSERT_EQ(mapper->submit_pose(pose, 11).status, InputStatus::Accepted);
        const auto initial_epoch = mapper->state(11).identity.map_epoch;

        pose.stamp = Perception::Timestamp {2};
        pose.position = Eigen::Vector3d(100.0, 0.0, 0.0);
        pose.quality = 0.1;
        EXPECT_EQ(mapper->submit_pose(pose, 12).status, InputStatus::Accepted);
        EXPECT_EQ(mapper->state(12).identity.map_epoch, initial_epoch);
        EXPECT_EQ(mapper->state(12).health, Perception::HealthState::Unavailable);

        pose.stamp = Perception::Timestamp {3};
        pose.position = Eigen::Vector3d::Zero();
        pose.quality = 1.0;
        ASSERT_EQ(mapper->submit_pose(pose, 13).status, InputStatus::Accepted);
        EXPECT_EQ(mapper->state(13).identity.map_epoch, initial_epoch);
        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {3}, 2.0F),
                              extrinsic(), 14)
                        .status,
                InputStatus::Applied);
    }

    TEST(PoseBoundary, LowQualityHistoricalPoseCannotBeSelectedAfterNewPoseRecovers)
    {
        auto config = mapper_config(
                Perception::RayEvidenceCapability::HitOnly, true);
        config.input_contract.minimum_pose_quality = 0.8;
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        Perception::PoseEstimate pose {
                Perception::SourceID {"odom"}, {1, 2}, "map", "vehicle_clock",
                Perception::Timestamp {1}, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), std::nullopt, 0.1,
                Perception::Duration {0}, 0};
        ASSERT_EQ(mapper->submit_pose(pose, 11).status, InputStatus::Accepted);
        pose.stamp = Perception::Timestamp {3};
        pose.quality = 1.0;
        ASSERT_EQ(mapper->submit_pose(pose, 12).status, InputStatus::Accepted);

        const auto result = mapper->submit_observation(
                scan_observation(
                        Perception::RayEvidenceCapability::HitOnly,
                        Perception::Timestamp {2}, 2.0F),
                extrinsic(), 13);
        EXPECT_EQ(result.status, InputStatus::Unavailable);
        const auto state = mapper->state(13);
        EXPECT_EQ(state.identity.revision, 0U);
        EXPECT_EQ(state.health, Perception::HealthState::Unavailable);
        EXPECT_TRUE(state.effective_capabilities.satisfied_combination_id.empty());
    }

    TEST(HealthBoundary, SensorReceiveAgeExpiresDuringAnotherInputWithoutTick)
    {
        auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto rear = descriptor(Perception::RayEvidenceCapability::HitOnly);
        rear.sensor_id = Perception::SensorID {"rear"};
        rear.frame_id = "rear_link";
        config.sensor_descriptors.push_back(rear);
        config.input_contract.minimum_viable.front().min_count = 2;
        config.input_contract.degraded_combinations.front().requirements.front().min_count = 2;
        config.sensor_validity_ns = 5;
        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 1)));

        EXPECT_EQ(
                mapper->submit_observation(
                              scan_observation(
                                      Perception::RayEvidenceCapability::HitOnly,
                                      Perception::Timestamp {1}, 2.0F),
                              extrinsic(), 10)
                        .status,
                InputStatus::Unavailable);
        auto rear_observation = scan_observation(
                Perception::RayEvidenceCapability::HitOnly,
                Perception::Timestamp {1}, 2.0F);
        rear_observation.sensor_id = Perception::SensorID {"rear"};
        rear_observation.frame_id = "rear_link";
        auto rear_extrinsic = extrinsic();
        rear_extrinsic.sensor_id = Perception::SensorID {"rear"};
        rear_extrinsic.frame_id = "rear_link";
        EXPECT_EQ(
                mapper->submit_observation(rear_observation, rear_extrinsic, 20).status,
                InputStatus::Unavailable);
        const auto state = mapper->state(20);
        EXPECT_EQ(state.identity.revision, 0U);
        EXPECT_EQ(state.effective_capabilities.active_sensor_count, 1U);
        EXPECT_TRUE(state.effective_capabilities.satisfied_combination_id.empty());
    }

    TEST(HealthBoundary, OlderProducerSessionCannotRetireCurrentHealthSession)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto mapper = make_mapper(config);
        auto current = health_for(config, 10);
        current.producer_session = {20, 1};
        ASSERT_TRUE(mapper->submit_health(current));
        auto older = current;
        older.producer_session = {10, 9};
        older.receive_monotonic_ns = 11;
        EXPECT_FALSE(mapper->submit_health(older));
        current.receive_monotonic_ns = 12;
        EXPECT_TRUE(mapper->submit_health(current));
    }

    TEST(AlignmentBoundary, UnknownStatusFailsClosed)
    {
        const auto config = mapper_config(Perception::RayEvidenceCapability::HitOnly);
        auto mapper = make_mapper(config);
        const auto state = mapper->state(0);
        EXPECT_FALSE(mapper->submit_alignment(
                {config.vehicle_id, config.mapper_session, state.identity.map_epoch,
                 "provider", {9, 1}, 1, 1,
                 static_cast<AlignmentStatus>(255)}));
    }

    TEST(CapabilityBoundary, ContradictoryCountsAndMaximumFailClosed)
    {
        EffectiveMapCapabilities capabilities;
        EXPECT_TRUE(capabilities.is_consistent());
        capabilities.active_sensor_count = 1;
        capabilities.has_active_ray_evidence = true;
        capabilities.active_2d_hit_ray_sensor_count = 1;
        EXPECT_FALSE(capabilities.is_consistent());
        capabilities.maximum_active_ray_evidence =
                Perception::RayEvidenceCapability::HitRay;
        EXPECT_TRUE(capabilities.is_consistent());
        capabilities.active_2d_hit_ray_sensor_count = 0;
        capabilities.active_3d_hit_ray_sensor_count = 1;
        EXPECT_FALSE(capabilities.is_consistent());
    }

    TEST(ResourceBaseline, ReportsFixedC1FixtureCellsWallTimeAndPeakRss)
    {
        Perception::Fixtures::FixtureSceneConfig scene_config;
        scene_config.scan_point_count = 360;
        scene_config.scan_angle_min_rad = -M_PI;
        scene_config.scan_angle_max_rad = M_PI - (2.0 * M_PI / 360.0);
        scene_config.scan_range_min_m = 0.1;
        scene_config.scan_range_max_m = 10.0;
        scene_config.inject_debug_returns = true;
        const Perception::Fixtures::FixtureScene scene(scene_config);

        auto config = mapper_config(Perception::RayEvidenceCapability::FullRay);
        config.geometry.resolution_m = 0.2;
        auto & frozen = config.sensor_descriptors.front();
        frozen.fov.horizontal_min_rad = scene.config().scan_angle_min_rad;
        frozen.fov.horizontal_max_rad = scene.config().scan_angle_max_rad;
        frozen.angular_resolution_rad = scene.config().scan_angle_increment_rad();
        frozen.range_min_m = scene.config().scan_range_min_m;
        frozen.range_max_m = scene.config().scan_range_max_m;
        frozen.mounting_orientation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());

        auto mapper = make_mapper(config);
        ASSERT_TRUE(mapper->submit_health(health_for(config, 10)));
        const auto start = std::chrono::steady_clock::now();
        Perception::LidarObservation observation {
                Perception::SensorID {"front"}, {1, 2}, "front_link", "vehicle_clock",
                Perception::Timestamp {1},
                Perception::Scan2D {
                        scene.config().scan_angle_min_rad,
                        scene.config().scan_angle_max_rad,
                        scene.config().scan_angle_increment_rad(),
                        scene.config().scan_range_min_m,
                        scene.config().scan_range_max_m,
                        scene.scan_ranges(), {}},
                Perception::RayEvidenceCapability::FullRay};
        auto sample = extrinsic();
        sample.orientation = frozen.mounting_orientation;
        const auto applied = mapper->submit_observation(observation, sample, 20);
        ASSERT_EQ(applied.status, InputStatus::Applied);
        auto acquired = mapper->acquire_read_transaction(applied.receipt.value());
        ASSERT_EQ(acquired.status, AcquireStatus::Ready);
        ASSERT_TRUE(acquired.transaction->metadata()->known_bounds.has_value());
        const auto cells = acquired.transaction->query_region(
                acquired.transaction->metadata()->known_bounds.value());
        ASSERT_EQ(cells.status, QueryStatus::Ok);
        const auto occupied_count = std::count_if(
                cells.cells.begin(), cells.cells.end(),
                [](const auto & cell) { return cell.state == OccupancyState::Occupied; });
        const auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start).count();
        rusage usage {};
        ASSERT_EQ(getrusage(RUSAGE_SELF, &usage), 0);
        std::cout << "[BASELINE] c2_fixed_c1_fixture wall_us=" << wall_us
                  << " peak_rss_kib=" << usage.ru_maxrss
                  << " known_cells=" << cells.cells.size()
                  << " occupied_cells=" << occupied_count << std::endl;
        EXPECT_GT(cells.cells.size(), 0U);
        EXPECT_GT(occupied_count, 0U);
    }

}// namespace PerceptionLocalMap::Test

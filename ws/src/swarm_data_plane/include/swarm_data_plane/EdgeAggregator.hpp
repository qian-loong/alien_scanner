#ifndef SWARM_DATA_PLANE_EDGE_AGGREGATOR_HPP
#define SWARM_DATA_PLANE_EDGE_AGGREGATOR_HPP

#include "perception_map_update/MapUpdateProducer.hpp"
#include "swarm_data_plane/AggregateContract.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace SwarmDataPlane {

    struct EdgeAggregatorLimits {
        DataPlaneLimits data_plane;
        PerceptionMapUpdate::MapUpdateLimits map_update;
        std::size_t max_contributors = 64U;
        std::size_t max_aggregate_cells = 3'000'000U;
    };

    struct EdgeAggregatorConfig {
        PerceptionMapUpdate::SourceIdentity aggregate_source;
        ProducerIdentity aggregate_producer;
        std::uint64_t route_epoch = 1U;
        std::uint16_t ttl_hops = 64U;
        std::uint64_t validity_budget_ns = 1'000'000'000U;
        std::string origin_clock_domain = "steady-sim";
        Perception::SessionID origin_clock_session {1U, 1U};
        std::string sensor_id = "edge-aggregator";
        Perception::SessionID sensor_session {1U, 2U};
    };

    enum class EdgeAggregatorStatus : std::uint8_t
    {
        Aggregated,
        IgnoredDuplicate,
        RejectedAdmission,
        RejectedEnvelope,
        RejectedGeometry,
        RejectedContributor,
        RejectedResync,
        RejectedResource,
        RejectedInternal
    };

    struct EdgeAggregatorResult {
        EdgeAggregatorStatus status = EdgeAggregatorStatus::RejectedInternal;
        std::optional<AggregateMapUpdate> update;
        bool state_changed = false;
        std::string diagnostic;

        explicit operator bool() const noexcept
        {
            return status == EdgeAggregatorStatus::Aggregated && update.has_value();
        }
    };

    struct EdgeContributorSnapshot {
        PerceptionMapUpdate::SourceIdentity source;
        std::uint64_t revision = 0U;
        PerceptionMapUpdate::Hash256 content_hash {};
        PerceptionMapUpdate::VersionedContentDigest content_identity;
        std::size_t cell_count = 0U;
        bool active = false;
        bool resync_required = false;
    };

    class EdgeAggregator
    {
    public:
        EdgeAggregator(
                EdgeAggregatorConfig config,
                EdgeAggregatorLimits limits = {});
        ~EdgeAggregator();

        EdgeAggregator(const EdgeAggregator &) = delete;
        EdgeAggregator & operator=(const EdgeAggregator &) = delete;
        EdgeAggregator(EdgeAggregator &&) noexcept = default;
        EdgeAggregator & operator=(EdgeAggregator &&) noexcept = default;

        EdgeAggregatorResult receive(
                const RoutedMapUpdate & message,
                std::uint64_t local_receive_monotonic_ns);
        bool expect_resync(
                const PerceptionMapUpdate::SourceIdentity & source,
                std::string correlation_id);

        const EdgeAggregatorConfig & config() const noexcept;
        const EdgeAggregatorLimits & limits() const noexcept;
        std::uint64_t aggregate_revision() const noexcept;
        const std::optional<AggregateMapUpdate> & last_update() const noexcept;
        std::vector<EdgeContributorSnapshot> contributors() const;

    private:
        struct ContributorState;

        ContributorState * find_contributor(const std::string & vehicle_id) noexcept;
        const ContributorState * find_contributor(
                const std::string & vehicle_id) const noexcept;
        EdgeAggregatorResult reject(
                EdgeAggregatorStatus status,
                std::string diagnostic) const;
        EdgeAggregatorResult build_aggregate(
                std::uint64_t receive_time_ns,
                const ContributorState & candidate,
                std::optional<std::size_t> replacement_index);

        EdgeAggregatorConfig config_;
        EdgeAggregatorLimits limits_;
        PerceptionMapUpdate::MapUpdateProducer producer_;
        std::vector<std::unique_ptr<ContributorState>> contributors_;
        std::optional<PerceptionMapUpdate::MapGeometry> geometry_;
        std::uint64_t aggregate_revision_ = 0U;
        std::uint64_t aggregate_sequence_ = 0U;
        std::optional<AggregateMapUpdate> last_update_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_EDGE_AGGREGATOR_HPP

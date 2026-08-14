#ifndef SWARM_DATA_PLANE_MAP_UPDATE_INGRESS_HPP
#define SWARM_DATA_PLANE_MAP_UPDATE_INGRESS_HPP

#include "perception_map_update/MapUpdateApplier.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"

#include <deque>
#include <optional>
#include <vector>

namespace SwarmDataPlane {

    enum class IngressStatus : std::uint8_t
    {
        AppliedKeyframe,
        AppliedDelta,
        AppliedRemove,
        AcceptedSummary,
        IgnoredDuplicate,
        RejectedAdmission,
        RejectedStale,
        RejectedGap,
        RejectedConflict,
        RejectedInvalid,
        RejectedExpired,
        RejectedRoute,
        RejectedResourceLimit
    };

    struct IngressResult {
        IngressStatus status = IngressStatus::RejectedInvalid;
        bool state_changed = false;
        bool freshness_refreshed = false;
        bool resync_required = false;
        std::string diagnostic;
    };

    class MapUpdateIngress
    {
    public:
        explicit MapUpdateIngress(
                DataPlaneLimits data_plane_limits = {},
                PerceptionMapUpdate::MapUpdateLimits map_update_limits = {},
                PerceptionMapUpdate::CellStorageConfig cell_storage = {});

        bool admit_producer(const ProducerIdentity & producer);
        bool admit_source(const PerceptionMapUpdate::SourceIdentity & source);
        bool require_resync();
        bool expect_resync(std::string correlation_id);
        IngressResult receive(
                const RoutedMapUpdate & message,
                std::uint64_t local_receive_monotonic_ns);

        const std::optional<ProducerIdentity> & expected_producer() const noexcept;
        std::optional<std::uint64_t> current_route_epoch() const noexcept;
        std::optional<std::uint64_t> last_fresh_receive_monotonic_ns() const noexcept;
        const PerceptionMapUpdate::MapUpdateApplier & map_applier() const noexcept;

    private:
        struct Receipt {
            std::string message_id;
            PerceptionMapUpdate::Hash256 payload_hash {};
        };

        const Receipt * find_receipt(const std::string & message_id) const noexcept;
        void remember_receipt(const RoutedMapUpdate & message);
        bool is_retired(const ProducerIdentity & producer) const noexcept;
        IngressResult reject(
                IngressStatus status,
                std::string diagnostic,
                bool require_resync);

        DataPlaneLimits limits_;
        PerceptionMapUpdate::MapUpdateApplier map_applier_;
        std::optional<ProducerIdentity> expected_producer_;
        std::vector<ProducerIdentity> retired_producers_;
        std::deque<Receipt> receipts_;
        std::optional<std::uint64_t> current_route_epoch_;
        std::uint64_t last_sequence_ = 0U;
        std::optional<std::uint64_t> last_fresh_receive_monotonic_ns_;
        bool correlation_required_ = false;
        std::optional<std::string> expected_resync_correlation_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_MAP_UPDATE_INGRESS_HPP

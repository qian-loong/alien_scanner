#ifndef SWARM_DATA_PLANE_AGGREGATE_CONTRACT_HPP
#define SWARM_DATA_PLANE_AGGREGATE_CONTRACT_HPP

#include "swarm_data_plane/DataPlaneTypes.hpp"
#include "swarm_data_plane/MapUpdateIngress.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace SwarmDataPlane {

    struct ContributorRevision {
        PerceptionMapUpdate::SourceIdentity source;
        std::uint64_t revision = 0U;
        PerceptionMapUpdate::Hash256 content_hash {};
        bool active = true;

        bool operator==(const ContributorRevision & other) const noexcept;
    };

    struct AggregateManifest {
        std::uint16_t protocol_version = kProtocolVersion;
        ProducerIdentity aggregate;
        std::uint64_t aggregate_revision = 0U;
        std::vector<ContributorRevision> contributors;
        PerceptionMapUpdate::Hash256 manifest_hash {};

        bool operator==(const AggregateManifest & other) const noexcept;
    };

    struct AggregateMapUpdate {
        RoutedMapUpdate aggregate_update;
        AggregateManifest manifest;
    };

    struct ManifestHashResult {
        bool success = false;
        PerceptionMapUpdate::Hash256 hash {};
        std::string diagnostic;
    };

    struct AggregateValidationResult {
        bool valid = false;
        std::string diagnostic;

        explicit operator bool() const noexcept { return valid; }
    };

    ManifestHashResult compute_manifest_hash(
            const AggregateManifest & manifest,
            const DataPlaneLimits & limits = {});
    AggregateValidationResult validate_aggregate_map_update(
            const AggregateMapUpdate & update,
            const DataPlaneLimits & limits = {});

    struct AggregateIngressResult {
        IngressResult map_result;
        bool manifest_changed = false;
    };

    class AggregateIngress
    {
    public:
        explicit AggregateIngress(
                DataPlaneLimits data_plane_limits = {},
                PerceptionMapUpdate::MapUpdateLimits map_update_limits = {});

        bool admit_producer(const ProducerIdentity & producer);
        bool admit_source(const PerceptionMapUpdate::SourceIdentity & source);
        bool expect_resync(std::string correlation_id);
        AggregateIngressResult receive(
                const AggregateMapUpdate & update,
                std::uint64_t local_receive_monotonic_ns);

        const std::optional<AggregateManifest> & current_manifest() const noexcept;
        const MapUpdateIngress & map_ingress() const noexcept;

    private:
        DataPlaneLimits limits_;
        MapUpdateIngress map_ingress_;
        std::optional<AggregateManifest> current_manifest_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_AGGREGATE_CONTRACT_HPP

#include "swarm_data_plane/EdgeAggregator.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"
#include "swarm_data_plane/RoutedMapValidator.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <utility>

namespace SwarmDataPlane {

    struct EdgeAggregator::ContributorState
    {
        explicit ContributorState(
                const EdgeAggregatorLimits & limits)
                : ingress(limits.data_plane, limits.map_update)
        {
        }

        MapUpdateIngress ingress;
        PerceptionMapUpdate::SourceIdentity source;
        ProducerIdentity producer;
        bool admitted = false;
        bool has_commit = false;
        bool active = false;
        bool resync_required = false;
        std::uint64_t revision = 0U;
        PerceptionMapUpdate::Hash256 content_hash {};
        std::size_t cell_count = 0U;

        ContributorState(const ContributorState &) = default;
        ContributorState & operator=(const ContributorState &) = default;
    };

    namespace {

        bool accepted(IngressStatus status) noexcept
        {
            return status == IngressStatus::AppliedKeyframe
                   || status == IngressStatus::AppliedDelta
                   || status == IngressStatus::AppliedRemove;
        }

        bool duplicate(IngressStatus status) noexcept
        {
            return status == IngressStatus::IgnoredDuplicate
                   || status == IngressStatus::AcceptedSummary;
        }

        LogicalPriority priority_for(PerceptionMapUpdate::UpdateKind kind) noexcept
        {
            switch(kind) {
                case PerceptionMapUpdate::UpdateKind::Keyframe:
                case PerceptionMapUpdate::UpdateKind::Remove:
                    return LogicalPriority::MapKeyframe;
                case PerceptionMapUpdate::UpdateKind::Delta:
                    return LogicalPriority::MapDelta;
                case PerceptionMapUpdate::UpdateKind::Summary:
                    return LogicalPriority::Summary;
            }
            return LogicalPriority::Diagnostic;
        }

        bool service_source_valid(
                const EdgeAggregatorConfig & config,
                const EdgeAggregatorLimits & limits)
        {
            return config.aggregate_source.map_epoch != 0U
                   && config.aggregate_source.mapper_session.boot_time_ns != 0U
                   && config.aggregate_producer.session.boot_time_ns != 0U
                   && config.aggregate_source.vehicle_id
                              == config.aggregate_producer.producer_id
                   && static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                           config.aggregate_source.vehicle_id,
                           limits.data_plane.max_aggregate_id_bytes,
                           "aggregate_source.vehicle_id",
                           false))
                   && config.route_epoch != 0U
                   && config.ttl_hops != 0U
                   && config.ttl_hops <= limits.data_plane.max_ttl_hops
                   && config.validity_budget_ns != 0U
                   && config.validity_budget_ns <= limits.data_plane.max_validity_budget_ns
                   && static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                           config.origin_clock_domain,
                           limits.data_plane.max_clock_domain_bytes,
                           "aggregate origin clock domain",
                           false))
                   && static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                           config.sensor_id,
                           limits.map_update.max_sensor_id_bytes,
                           "aggregate sensor id",
                           false));
        }

    }// namespace

    EdgeAggregator::EdgeAggregator(
            EdgeAggregatorConfig config,
            EdgeAggregatorLimits limits)
            : config_(std::move(config)),
              limits_(std::move(limits)),
              producer_(limits_.map_update)
    {
        contributors_.reserve(limits_.max_contributors);
    }

    EdgeAggregator::~EdgeAggregator() = default;

    const EdgeAggregatorConfig & EdgeAggregator::config() const noexcept
    {
        return config_;
    }

    const EdgeAggregatorLimits & EdgeAggregator::limits() const noexcept
    {
        return limits_;
    }

    std::uint64_t EdgeAggregator::aggregate_revision() const noexcept
    {
        return aggregate_revision_;
    }

    const std::optional<AggregateMapUpdate> & EdgeAggregator::last_update() const noexcept
    {
        return last_update_;
    }

    EdgeAggregator::ContributorState * EdgeAggregator::find_contributor(
            const std::string & vehicle_id) noexcept
    {
        const auto found = std::find_if(
                contributors_.begin(), contributors_.end(),
                [&](const std::unique_ptr<ContributorState> & value) {
                    return value->source.vehicle_id == vehicle_id;
                });
        return found == contributors_.end() ? nullptr : found->get();
    }

    const EdgeAggregator::ContributorState * EdgeAggregator::find_contributor(
            const std::string & vehicle_id) const noexcept
    {
        const auto found = std::find_if(
                contributors_.begin(), contributors_.end(),
                [&](const std::unique_ptr<ContributorState> & value) {
                    return value->source.vehicle_id == vehicle_id;
                });
        return found == contributors_.end() ? nullptr : found->get();
    }

    EdgeAggregatorResult EdgeAggregator::reject(
            EdgeAggregatorStatus status,
            std::string diagnostic) const
    {
        return {status, std::nullopt, false, std::move(diagnostic)};
    }

    EdgeAggregatorResult EdgeAggregator::receive(
            const RoutedMapUpdate & message,
            std::uint64_t local_receive_monotonic_ns)
    {
        if(!service_source_valid(config_, limits_)) {
            return reject(
                    EdgeAggregatorStatus::RejectedInternal,
                    "aggregate source configuration is invalid");
        }
        const auto envelope = validate_routed_map_update(message, limits_.data_plane);
        if(!envelope) {
            return reject(
                    EdgeAggregatorStatus::RejectedEnvelope,
                    envelope.diagnostic);
        }
        if(!message.update) {
            return reject(
                    EdgeAggregatorStatus::RejectedEnvelope,
                    "routed update payload is null");
        }
        if(geometry_.has_value() && !(*geometry_ == message.update->geometry)) {
            return reject(
                    EdgeAggregatorStatus::RejectedGeometry,
                    "contributor geometry/frame differs from aggregate geometry");
        }

        const auto found = std::find_if(
                contributors_.begin(), contributors_.end(),
                [&](const std::unique_ptr<ContributorState> & value) {
                    return value->source.vehicle_id == message.update->source.vehicle_id;
                });
        const std::optional<std::size_t> replacement_index =
                found == contributors_.end()
                        ? std::nullopt
                        : std::optional<std::size_t>(
                                  static_cast<std::size_t>(found - contributors_.begin()));
        std::unique_ptr<ContributorState> candidate;
        if(!replacement_index.has_value()) {
            if(contributors_.size() >= limits_.max_contributors) {
                return reject(
                        EdgeAggregatorStatus::RejectedResource,
                        "contributor count exceeds edge aggregator limit");
            }
            candidate = std::make_unique<ContributorState>(limits_);
            if(!candidate->ingress.admit_producer(message.producer)
               || !candidate->ingress.admit_source(message.update->source)) {
                return reject(
                        EdgeAggregatorStatus::RejectedAdmission,
                        "initial contributor identity was not admitted");
            }
            candidate->source = message.update->source;
            candidate->producer = message.producer;
            candidate->admitted = true;
        }
        else {
            candidate = std::make_unique<ContributorState>(*contributors_[*replacement_index]);
            if(candidate->producer != message.producer
               && !candidate->ingress.admit_producer(message.producer)) {
                return reject(
                        EdgeAggregatorStatus::RejectedContributor,
                        "contributor producer session is stale/conflicting");
            }
            if(candidate->source != message.update->source
               && !candidate->ingress.admit_source(message.update->source)) {
                return reject(
                        EdgeAggregatorStatus::RejectedContributor,
                        "contributor source session or map epoch is stale/conflicting");
            }
            if(candidate->source != message.update->source
               || candidate->producer != message.producer) {
                candidate->source = message.update->source;
                candidate->producer = message.producer;
                candidate->active = false;
                candidate->resync_required = true;
                candidate->has_commit = false;
                candidate->revision = 0U;
                candidate->content_hash = {};
                candidate->cell_count = 0U;
            }
        }

        // C4 ingress may accept a route-epoch keyframe as a normal recovery
        // input. C5d adds a contributor-local barrier so the aggregator owns
        // the resync correlation and never accepts an unrequested keyframe.
        if(const auto current_route_epoch = candidate->ingress.current_route_epoch();
           current_route_epoch.has_value()
           && message.route.route_epoch > *current_route_epoch
           && !candidate->resync_required) {
            candidate->ingress.require_resync();
            candidate->resync_required = true;
            if(replacement_index.has_value()) {
                contributors_[*replacement_index].swap(candidate);
            }
            else {
                contributors_.push_back(std::move(candidate));
            }
            return reject(
                    EdgeAggregatorStatus::RejectedResync,
                    "route epoch change requires an aggregator-correlated keyframe");
        }

        const auto result = candidate->ingress.receive(
                message, local_receive_monotonic_ns);
        candidate->resync_required = result.resync_required;
        if(!accepted(result.status)) {
            if(duplicate(result.status)) {
                return {EdgeAggregatorStatus::IgnoredDuplicate,
                        std::nullopt,
                        false,
                        result.diagnostic};
            }
            if(result.resync_required) {
                if(replacement_index.has_value()) {
                    contributors_[*replacement_index].swap(candidate);
                }
                else {
                    contributors_.push_back(std::move(candidate));
                }
            }
            const auto status = result.status == IngressStatus::RejectedResourceLimit
                                         ? EdgeAggregatorStatus::RejectedResource
                                         : result.resync_required
                                               ? EdgeAggregatorStatus::RejectedResync
                                               : EdgeAggregatorStatus::RejectedContributor;
            return reject(status, result.diagnostic);
        }

        const auto & reconstructed = candidate->ingress.map_applier().reconstructed_map();
        if(!reconstructed.has_value()) {
            return reject(
                    EdgeAggregatorStatus::RejectedContributor,
                    "accepted contributor update has no reconstructed map");
        }
        candidate->has_commit = true;
        candidate->revision = reconstructed->revision;
        candidate->content_hash = reconstructed->content_identity.digest;
        candidate->cell_count = reconstructed->cells.size();
        candidate->active = candidate->ingress.map_applier().state()
                            != PerceptionMapUpdate::ReceiverState::Removed;
        candidate->resync_required = false;

        auto aggregate = build_aggregate(
                local_receive_monotonic_ns, *candidate, replacement_index);
        if(!aggregate) {
            return aggregate;
        }
        if(replacement_index.has_value()) {
            contributors_[*replacement_index].swap(candidate);
        }
        else {
            contributors_.push_back(std::move(candidate));
        }
        return aggregate;
    }

    bool EdgeAggregator::expect_resync(
            const PerceptionMapUpdate::SourceIdentity & source,
            std::string correlation_id)
    {
        auto * contributor = find_contributor(source.vehicle_id);
        if(contributor == nullptr || contributor->source != source
           || !contributor->ingress.expect_resync(std::move(correlation_id))) {
            return false;
        }
        contributor->resync_required = true;
        return true;
    }

    EdgeAggregatorResult EdgeAggregator::build_aggregate(
            std::uint64_t receive_time_ns,
            const ContributorState & candidate,
            std::optional<std::size_t> replacement_index)
    {
        std::optional<PerceptionMapUpdate::MapGeometry> next_geometry = geometry_;
        std::map<PerceptionMapUpdate::VoxelIndex, PerceptionMapUpdate::CellState> merged;
        std::optional<EdgeAggregatorResult> failure;
        const auto consume = [&](const ContributorState & contributor) {
            if(failure.has_value()) {
                return;
            }
            if(!contributor.has_commit || !contributor.active) {
                return;
            }
            const auto & reconstructed = contributor.ingress.map_applier().reconstructed_map();
            if(!reconstructed.has_value()) {
                failure = reject(
                        EdgeAggregatorStatus::RejectedContributor,
                        "active contributor has no reconstructed map");
                return;
            }
            if(next_geometry.has_value() && !(*next_geometry == reconstructed->geometry)) {
                failure = reject(
                        EdgeAggregatorStatus::RejectedGeometry,
                        "contributor geometry/frame differs from aggregate geometry");
                return;
            }
            next_geometry = reconstructed->geometry;
            const auto cells = reconstructed->cells.materialize();
            for(const auto & cell : cells) {
                auto inserted = merged.emplace(cell.index, cell.state);
                if(!inserted.second && cell.state == PerceptionMapUpdate::CellState::Occupied) {
                    inserted.first->second = PerceptionMapUpdate::CellState::Occupied;
                }
                if(merged.size() > limits_.max_aggregate_cells) {
                    failure = reject(
                            EdgeAggregatorStatus::RejectedResource,
                            "aggregate cell count exceeds configured limit");
                    return;
                }
            }
        };
        for(std::size_t index = 0U; index < contributors_.size(); ++index) {
            if(replacement_index.has_value() && index == *replacement_index) {
                consume(candidate);
            }
            else {
                consume(*contributors_[index]);
            }
        }
        if(!replacement_index.has_value()) {
            consume(candidate);
        }
        if(failure.has_value()) {
            return std::move(*failure);
        }
        if(!next_geometry.has_value()) {
            return reject(
                    EdgeAggregatorStatus::RejectedContributor,
                    "aggregate has no committed contributor geometry");
        }

        PerceptionMapUpdate::CanonicalSnapshot snapshot;
        snapshot.source = config_.aggregate_source;
        snapshot.geometry = *next_geometry;
        snapshot.revision = aggregate_revision_ + 1U;
        snapshot.latest_commit = {
                config_.sensor_id,
                config_.sensor_session,
                Perception::Timestamp {static_cast<std::int64_t>(receive_time_ns)},
                config_.origin_clock_domain,
                static_cast<std::uint32_t>(std::min<std::size_t>(
                        merged.size(), std::numeric_limits<std::uint32_t>::max()))};
        snapshot.cells.reserve(merged.size());
        for(const auto & [index, state] : merged) {
            snapshot.cells.push_back({index, state});
        }
        snapshot.geometry_fingerprint =
                PerceptionMapUpdate::ContentHasher::geometry_fingerprint(snapshot.geometry);
        snapshot.content_hash = PerceptionMapUpdate::ContentHasher::content_hash(
                snapshot.source, snapshot.geometry_fingerprint, snapshot.cells);

        auto prepared = producer_.prepare(snapshot);
        if(!prepared.update.has_value()
           || prepared.status == PerceptionMapUpdate::ProduceStatus::RejectedInvalidSnapshot
           || prepared.status == PerceptionMapUpdate::ProduceStatus::RejectedResourceLimit) {
            return reject(
                    prepared.status == PerceptionMapUpdate::ProduceStatus::RejectedResourceLimit
                            ? EdgeAggregatorStatus::RejectedResource
                            : EdgeAggregatorStatus::RejectedInternal,
                    prepared.diagnostic.empty()
                            ? "aggregate map update producer rejected snapshot"
                            : prepared.diagnostic);
        }

        RoutedMapUpdate routed;
        routed.message_id = config_.aggregate_source.vehicle_id + "-aggregate-"
                            + std::to_string(snapshot.revision);
        routed.producer = config_.aggregate_producer;
        const auto next_sequence = aggregate_sequence_ + 1U;
        routed.sequence = next_sequence;
        routed.correlation_id = prepared.update->correlation_id;
        routed.priority = priority_for(prepared.update->kind);
        routed.origin = {
                config_.origin_clock_domain,
                config_.origin_clock_session,
                receive_time_ns};
        routed.validity_budget_ns = config_.validity_budget_ns;
        routed.accumulated_forwarding_ns = 0U;
        routed.route = {config_.route_epoch, 0U, config_.ttl_hops};
        routed.payload_bytes = prepared.update->canonical_payload_bytes;
        routed.payload_hash = prepared.update->update_hash;
        routed.update = std::make_shared<const PerceptionMapUpdate::MapUpdate>(
                *prepared.update);

        AggregateManifest manifest;
        // The manifest session identifies the aggregate map source (the
        // mapper session carried by MapUpdate), while the routed envelope
        // keeps the transport producer session independently.
        manifest.aggregate = {
                config_.aggregate_source.vehicle_id,
                config_.aggregate_source.mapper_session};
        manifest.aggregate_revision = snapshot.revision;
        manifest.contributors.reserve(contributors_.size() + (replacement_index.has_value() ? 0U : 1U));
        const auto append_manifest = [&](const ContributorState & contributor) {
            if(!contributor.has_commit) {
                return;
            }
            manifest.contributors.push_back({
                    contributor.source,
                    contributor.revision,
                    contributor.active ? contributor.content_hash
                                       : PerceptionMapUpdate::Hash256 {},
                    contributor.active});
        };
        for(std::size_t index = 0U; index < contributors_.size(); ++index) {
            if(replacement_index.has_value() && index == *replacement_index) {
                append_manifest(candidate);
            }
            else {
                append_manifest(*contributors_[index]);
            }
        }
        if(!replacement_index.has_value()) {
            append_manifest(candidate);
        }
        std::sort(
                manifest.contributors.begin(), manifest.contributors.end(),
                [](const ContributorRevision & left, const ContributorRevision & right) {
                    return left.source.vehicle_id < right.source.vehicle_id;
                });
        const auto manifest_hash = compute_manifest_hash(manifest, limits_.data_plane);
        if(!manifest_hash.success) {
            return reject(EdgeAggregatorStatus::RejectedInternal, manifest_hash.diagnostic);
        }
        manifest.manifest_hash = manifest_hash.hash;
        AggregateMapUpdate aggregate {std::move(routed), std::move(manifest)};
        const auto validation = validate_aggregate_map_update(aggregate, limits_.data_plane);
        if(!validation) {
            return reject(EdgeAggregatorStatus::RejectedInternal, validation.diagnostic);
        }
        auto output = aggregate;
        std::optional<AggregateMapUpdate> committed_update {std::move(aggregate)};
        auto committed_geometry = next_geometry;
        if(!producer_.commit_published(prepared)) {
            return reject(
                    EdgeAggregatorStatus::RejectedInternal,
                    "aggregate producer baseline changed before commit");
        }
        geometry_.swap(committed_geometry);
        aggregate_revision_ = snapshot.revision;
        aggregate_sequence_ = next_sequence;
        last_update_.swap(committed_update);
        return {EdgeAggregatorStatus::Aggregated, std::move(output), true, {}};
    }

    std::vector<EdgeContributorSnapshot> EdgeAggregator::contributors() const
    {
        std::vector<EdgeContributorSnapshot> result;
        result.reserve(contributors_.size());
        for(const auto & holder : contributors_) {
            const auto & contributor = *holder;
            result.push_back({
                    contributor.source,
                    contributor.revision,
                    contributor.content_hash,
                    contributor.ingress.map_applier().reconstructed_map().has_value()
                            ? contributor.ingress.map_applier()
                                      .reconstructed_map()
                                      ->content_identity
                            : PerceptionMapUpdate::VersionedContentDigest {},
                    contributor.cell_count,
                    contributor.active,
                    contributor.resync_required});
        }
        std::sort(
                result.begin(), result.end(),
                [](const EdgeContributorSnapshot & left,
                   const EdgeContributorSnapshot & right) {
                    return left.source.vehicle_id < right.source.vehicle_id;
                });
        return result;
    }

}// namespace SwarmDataPlane

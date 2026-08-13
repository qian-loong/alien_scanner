#include "swarm_data_plane/AggregateContract.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "swarm_data_plane/RoutedMapValidator.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <limits>
#include <set>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        void append_u16(std::vector<std::uint8_t> & bytes, std::uint16_t value)
        {
            bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
            bytes.push_back(static_cast<std::uint8_t>(value & 0xFFU));
        }

        void append_u32(std::vector<std::uint8_t> & bytes, std::uint32_t value)
        {
            for(int shift = 24; shift >= 0; shift -= 8) {
                bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
            }
        }

        void append_u64(std::vector<std::uint8_t> & bytes, std::uint64_t value)
        {
            for(int shift = 56; shift >= 0; shift -= 8) {
                bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
            }
        }

        bool append_string(std::vector<std::uint8_t> & bytes, const std::string & value)
        {
            if(value.size() > std::numeric_limits<std::uint16_t>::max()) {
                return false;
            }
            append_u16(bytes, static_cast<std::uint16_t>(value.size()));
            bytes.insert(bytes.end(), value.begin(), value.end());
            return true;
        }

        bool valid_text(
                const std::string & value,
                std::size_t limit,
                bool allow_empty)
        {
            return static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                    value, limit, "aggregate string", allow_empty));
        }

        IngressResult aggregate_rejection(
                IngressStatus status,
                std::string diagnostic,
                bool resync_required)
        {
            return {status, false, false, resync_required, std::move(diagnostic)};
        }

    }// namespace

    bool ContributorRevision::operator==(const ContributorRevision & other) const noexcept
    {
        return source == other.source && revision == other.revision
               && content_hash == other.content_hash && active == other.active;
    }

    bool AggregateManifest::operator==(const AggregateManifest & other) const noexcept
    {
        return protocol_version == other.protocol_version && aggregate == other.aggregate
               && aggregate_revision == other.aggregate_revision
               && contributors == other.contributors && manifest_hash == other.manifest_hash;
    }

    ManifestHashResult compute_manifest_hash(
            const AggregateManifest & manifest,
            const DataPlaneLimits & limits)
    {
        if(manifest.protocol_version != kProtocolVersion
           || manifest.aggregate.session.boot_time_ns == 0U
           || manifest.aggregate_revision == 0U
           || !valid_text(
                   manifest.aggregate.producer_id,
                   limits.max_aggregate_id_bytes,
                   false)
           || manifest.contributors.size() > limits.max_contributors
           || manifest.contributors.size() > std::numeric_limits<std::uint16_t>::max()) {
            return {false, {}, "aggregate manifest header exceeds configured limits"};
        }

        std::vector<std::uint8_t> bytes;
        bytes.reserve(128U + manifest.contributors.size() * 192U);
        const std::string domain = "alien-scanner/aggregate-manifest/v1";
        if(!append_string(bytes, domain)) {
            return {false, {}, "aggregate hash domain is too long"};
        }
        append_u16(bytes, manifest.protocol_version);
        if(!append_string(bytes, manifest.aggregate.producer_id)) {
            return {false, {}, "aggregate id is too long"};
        }
        append_u64(bytes, manifest.aggregate.session.boot_time_ns);
        append_u32(bytes, manifest.aggregate.session.random_suffix);
        append_u64(bytes, manifest.aggregate_revision);
        append_u16(bytes, static_cast<std::uint16_t>(manifest.contributors.size()));

        std::string previous_vehicle;
        for(const auto & contributor : manifest.contributors) {
            if(!valid_text(
                       contributor.source.vehicle_id,
                       limits.max_producer_id_bytes,
                       false)
               || contributor.source.mapper_session.boot_time_ns == 0U
               || contributor.source.map_epoch == 0U || contributor.revision == 0U
               || (!previous_vehicle.empty()
                   && contributor.source.vehicle_id <= previous_vehicle)) {
                return {false, {},
                        "contributors must be valid and strictly sorted by vehicle id"};
            }
            previous_vehicle = contributor.source.vehicle_id;
            if(!append_string(bytes, contributor.source.vehicle_id)) {
                return {false, {}, "contributor vehicle id is too long"};
            }
            append_u64(bytes, contributor.source.mapper_session.boot_time_ns);
            append_u32(bytes, contributor.source.mapper_session.random_suffix);
            append_u64(bytes, contributor.source.map_epoch);
            append_u64(bytes, contributor.revision);
            bytes.insert(
                    bytes.end(),
                    contributor.content_hash.begin(),
                    contributor.content_hash.end());
            bytes.push_back(contributor.active ? 1U : 0U);
        }

        PerceptionMapUpdate::Hash256 hash {};
        unsigned int digest_size = 0U;
        if(EVP_Digest(
                   bytes.data(),
                   bytes.size(),
                   hash.data(),
                   &digest_size,
                   EVP_sha256(),
                   nullptr)
                   != 1
           || digest_size != hash.size()) {
            return {false, {}, "failed to compute aggregate manifest SHA-256"};
        }
        return {true, hash, {}};
    }

    AggregateValidationResult validate_aggregate_map_update(
            const AggregateMapUpdate & update,
            const DataPlaneLimits & limits)
    {
        const auto routed_validation = validate_routed_map_update(
                update.aggregate_update, limits);
        if(!routed_validation) {
            return {false, routed_validation.diagnostic};
        }
        if(update.aggregate_update.update->kind == PerceptionMapUpdate::UpdateKind::Summary) {
            return {false, "aggregate manifest cannot be committed by a summary update"};
        }
        if(update.manifest.aggregate.producer_id
                   != update.aggregate_update.update->source.vehicle_id
           || update.manifest.aggregate.session
                      != update.aggregate_update.update->source.mapper_session
           || update.manifest.aggregate_revision
                      != update.aggregate_update.update->new_revision) {
            return {false,
                    "aggregate manifest identity or revision differs from its map update"};
        }
        const auto computed = compute_manifest_hash(update.manifest, limits);
        if(!computed.success) {
            return {false, computed.diagnostic};
        }
        if(computed.hash != update.manifest.manifest_hash) {
            return {false, "aggregate manifest hash mismatch"};
        }
        return {true, {}};
    }

    AggregateIngress::AggregateIngress(
            DataPlaneLimits data_plane_limits,
            PerceptionMapUpdate::MapUpdateLimits map_update_limits)
            : limits_(data_plane_limits),
              map_ingress_(std::move(data_plane_limits), std::move(map_update_limits))
    {
    }

    bool AggregateIngress::admit_producer(const ProducerIdentity & producer)
    {
        return map_ingress_.admit_producer(producer);
    }

    bool AggregateIngress::admit_source(
            const PerceptionMapUpdate::SourceIdentity & source)
    {
        return map_ingress_.admit_source(source);
    }

    bool AggregateIngress::expect_resync(std::string correlation_id)
    {
        return map_ingress_.expect_resync(std::move(correlation_id));
    }

    AggregateIngressResult AggregateIngress::receive(
            const AggregateMapUpdate & update,
            std::uint64_t local_receive_monotonic_ns)
    {
        const auto validation = validate_aggregate_map_update(update, limits_);
        if(!validation) {
            map_ingress_.require_resync();
            return {aggregate_rejection(
                            IngressStatus::RejectedInvalid,
                            validation.diagnostic,
                            true),
                    false};
        }
        if(current_manifest_.has_value()) {
            if(update.manifest.aggregate_revision < current_manifest_->aggregate_revision) {
                return {aggregate_rejection(
                                IngressStatus::RejectedStale,
                                "aggregate manifest revision is stale",
                                false),
                        false};
            }
            if(update.manifest.aggregate_revision == current_manifest_->aggregate_revision
               && update.manifest.manifest_hash != current_manifest_->manifest_hash) {
                map_ingress_.require_resync();
                return {aggregate_rejection(
                                IngressStatus::RejectedConflict,
                                "same aggregate revision carries a different manifest",
                                true),
                        false};
            }
        }

        auto result = map_ingress_.receive(
                update.aggregate_update, local_receive_monotonic_ns);
        const bool applied = result.status == IngressStatus::AppliedKeyframe
                             || result.status == IngressStatus::AppliedDelta
                             || result.status == IngressStatus::AppliedRemove;
        if(applied) {
            current_manifest_ = update.manifest;
            return {std::move(result), true};
        }
        if(result.status == IngressStatus::IgnoredDuplicate && current_manifest_.has_value()
           && update.manifest == *current_manifest_) {
            return {std::move(result), false};
        }
        return {std::move(result), false};
    }

    const std::optional<AggregateManifest> & AggregateIngress::current_manifest() const noexcept
    {
        return current_manifest_;
    }

    const MapUpdateIngress & AggregateIngress::map_ingress() const noexcept
    {
        return map_ingress_;
    }

}// namespace SwarmDataPlane

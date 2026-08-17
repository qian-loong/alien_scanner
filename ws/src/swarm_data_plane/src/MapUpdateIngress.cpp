#include "swarm_data_plane/MapUpdateIngress.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "swarm_data_plane/RoutedMapValidator.hpp"

#include <algorithm>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        bool valid_producer(
                const ProducerIdentity & producer,
                const DataPlaneLimits & limits)
        {
            return producer.session.boot_time_ns != 0U
                   && static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                           producer.producer_id,
                           limits.max_producer_id_bytes,
                           "producer_id",
                           false));
        }

        IngressStatus map_status(PerceptionMapUpdate::ApplyUpdateStatus status) noexcept
        {
            using ApplyStatus = PerceptionMapUpdate::ApplyUpdateStatus;
            switch(status) {
                case ApplyStatus::AppliedKeyframe:
                    return IngressStatus::AppliedKeyframe;
                case ApplyStatus::AppliedDelta:
                    return IngressStatus::AppliedDelta;
                case ApplyStatus::AppliedRemove:
                    return IngressStatus::AppliedRemove;
                case ApplyStatus::AcceptedSummary:
                    return IngressStatus::AcceptedSummary;
                case ApplyStatus::IgnoredDuplicate:
                    return IngressStatus::IgnoredDuplicate;
                case ApplyStatus::RejectedAdmission:
                    return IngressStatus::RejectedAdmission;
                case ApplyStatus::RejectedStale:
                    return IngressStatus::RejectedStale;
                case ApplyStatus::RejectedGap:
                    return IngressStatus::RejectedGap;
                case ApplyStatus::RejectedConflict:
                    return IngressStatus::RejectedConflict;
                case ApplyStatus::RejectedInvalid:
                    return IngressStatus::RejectedInvalid;
                case ApplyStatus::RejectedResourceLimit:
                    return IngressStatus::RejectedResourceLimit;
            }
            return IngressStatus::RejectedInvalid;
        }

        bool refreshes_freshness(IngressStatus status) noexcept
        {
            return status == IngressStatus::AppliedKeyframe
                   || status == IngressStatus::AppliedDelta
                   || status == IngressStatus::AppliedRemove;
        }

    }// namespace

    MapUpdateIngress::MapUpdateIngress(
            DataPlaneLimits data_plane_limits,
            PerceptionMapUpdate::MapUpdateLimits map_update_limits)
            : limits_(std::move(data_plane_limits)),
              map_applier_(std::move(map_update_limits))
    {
    }

    bool MapUpdateIngress::is_retired(const ProducerIdentity & producer) const noexcept
    {
        return std::find(retired_producers_.begin(), retired_producers_.end(), producer)
               != retired_producers_.end();
    }

    bool MapUpdateIngress::admit_producer(const ProducerIdentity & producer)
    {
        if(!valid_producer(producer, limits_) || is_retired(producer)) {
            return false;
        }
        if(expected_producer_.has_value() && *expected_producer_ == producer) {
            return true;
        }
        if(expected_producer_.has_value()) {
            if(producer.producer_id != expected_producer_->producer_id
               || !(expected_producer_->session < producer.session)
               || retired_producers_.size() >= limits_.max_retired_producers) {
                return false;
            }
            retired_producers_.push_back(*expected_producer_);
        }
        expected_producer_ = producer;
        receipts_.clear();
        current_route_epoch_.reset();
        last_sequence_ = 0U;
        correlation_required_ = false;
        expected_resync_correlation_.reset();
        return true;
    }

    bool MapUpdateIngress::admit_source(
            const PerceptionMapUpdate::SourceIdentity & source)
    {
        const bool admitted = map_applier_.admit_source(source);
        if(admitted
           && map_applier_.state() == PerceptionMapUpdate::ReceiverState::ResyncRequired) {
            correlation_required_ = true;
            expected_resync_correlation_.reset();
        }
        return admitted;
    }

    bool MapUpdateIngress::require_resync()
    {
        const bool barrier_already_active =
                map_applier_.state() == PerceptionMapUpdate::ReceiverState::ResyncRequired;
        const bool changed = map_applier_.require_resync();
        if(map_applier_.state() == PerceptionMapUpdate::ReceiverState::ResyncRequired) {
            correlation_required_ = true;
            if(!barrier_already_active) {
                expected_resync_correlation_.reset();
            }
        }
        return changed;
    }

    bool MapUpdateIngress::expect_resync(std::string correlation_id)
    {
        if(!correlation_required_
           || map_applier_.state() != PerceptionMapUpdate::ReceiverState::ResyncRequired
           || !PerceptionMapUpdate::CanonicalCodec::validate_string(
                   correlation_id,
                   limits_.max_correlation_id_bytes,
                   "resync correlation_id",
                   false)) {
            return false;
        }
        expected_resync_correlation_ = std::move(correlation_id);
        return true;
    }

    const MapUpdateIngress::Receipt * MapUpdateIngress::find_receipt(
            const std::string & message_id) const noexcept
    {
        const auto found = std::find_if(
                receipts_.begin(), receipts_.end(), [&](const Receipt & receipt) {
                    return receipt.message_id == message_id;
                });
        return found == receipts_.end() ? nullptr : &*found;
    }

    void MapUpdateIngress::remember_receipt(const RoutedMapUpdate & message)
    {
        if(limits_.max_recent_messages == 0U) {
            return;
        }
        while(receipts_.size() >= limits_.max_recent_messages) {
            receipts_.pop_front();
        }
        receipts_.push_back({message.message_id, message.payload_hash});
    }

    IngressResult MapUpdateIngress::reject(
            IngressStatus status,
            std::string diagnostic,
            bool require_resync)
    {
        if(require_resync) {
            this->require_resync();
        }
        return {status,
                false,
                false,
                require_resync
                        || map_applier_.state()
                                   == PerceptionMapUpdate::ReceiverState::ResyncRequired,
                std::move(diagnostic)};
    }

    IngressResult MapUpdateIngress::receive(
            const RoutedMapUpdate & message,
            std::uint64_t local_receive_monotonic_ns)
    {
        const auto validation = validate_routed_map_update(message, limits_);
        if(!validation) {
            switch(validation.status) {
                case EnvelopeValidationStatus::RejectedExpired:
                    return reject(
                            IngressStatus::RejectedExpired,
                            validation.diagnostic,
                            true);
                case EnvelopeValidationStatus::RejectedRoute:
                    return reject(
                            IngressStatus::RejectedRoute,
                            validation.diagnostic,
                            true);
                case EnvelopeValidationStatus::RejectedInvalid:
                case EnvelopeValidationStatus::Valid:
                    return reject(
                            IngressStatus::RejectedInvalid,
                            validation.diagnostic,
                            true);
            }
        }
        if(!expected_producer_.has_value() || message.producer != *expected_producer_
           || is_retired(message.producer)) {
            return reject(
                    IngressStatus::RejectedAdmission,
                    "data-plane producer is not admitted",
                    false);
        }
        if(const auto * receipt = find_receipt(message.message_id); receipt != nullptr) {
            if(receipt->payload_hash == message.payload_hash) {
                return {IngressStatus::IgnoredDuplicate,
                        false,
                        false,
                        map_applier_.state()
                                == PerceptionMapUpdate::ReceiverState::ResyncRequired,
                        {}};
            }
            return reject(
                    IngressStatus::RejectedConflict,
                    "message id was reused for a different payload hash",
                    true);
        }
        if(current_route_epoch_.has_value()) {
            if(message.route.route_epoch < *current_route_epoch_) {
                return reject(
                        IngressStatus::RejectedRoute,
                        "message belongs to a retired route epoch",
                        false);
            }
            if(message.route.route_epoch > *current_route_epoch_
               && message.update->kind != PerceptionMapUpdate::UpdateKind::Keyframe) {
                return reject(
                        IngressStatus::RejectedRoute,
                        "new route epoch requires a keyframe barrier",
                        true);
            }
        }
        if(message.update->kind == PerceptionMapUpdate::UpdateKind::Keyframe
           && correlation_required_) {
            if(!expected_resync_correlation_.has_value()) {
                return reject(
                        IngressStatus::RejectedGap,
                        "resync keyframe arrived before a correlation was acknowledged",
                        false);
            }
            if(message.correlation_id != *expected_resync_correlation_) {
                return reject(
                        IngressStatus::RejectedConflict,
                        "resync keyframe correlation does not match the acknowledged request",
                        false);
            }
        }
        if(last_sequence_ != 0U) {
            if(message.sequence <= last_sequence_) {
                return reject(
                        IngressStatus::RejectedStale,
                        "producer sequence is stale or conflicts with an earlier message",
                        false);
            }
            if(message.sequence - last_sequence_ > 1U
               && message.update->kind != PerceptionMapUpdate::UpdateKind::Keyframe) {
                return reject(
                        IngressStatus::RejectedGap,
                        "producer sequence gap requires a keyframe",
                        true);
            }
        }

        const auto applied = map_applier_.apply(*message.update);
        const auto status = map_status(applied.status);
        const bool accepted = status == IngressStatus::AppliedKeyframe
                              || status == IngressStatus::AppliedDelta
                              || status == IngressStatus::AppliedRemove
                              || status == IngressStatus::AcceptedSummary;
        if(accepted) {
            current_route_epoch_ = message.route.route_epoch;
            last_sequence_ = message.sequence;
            remember_receipt(message);
        }
        if(status == IngressStatus::AppliedKeyframe && correlation_required_) {
            correlation_required_ = false;
            expected_resync_correlation_.reset();
        }
        const bool fresh = refreshes_freshness(status);
        if(fresh) {
            last_fresh_receive_monotonic_ns_ = local_receive_monotonic_ns;
        }
        return {status,
                applied.state_changed,
                fresh,
                map_applier_.state() == PerceptionMapUpdate::ReceiverState::ResyncRequired,
                applied.diagnostic};
    }

    const std::optional<ProducerIdentity> & MapUpdateIngress::expected_producer() const noexcept
    {
        return expected_producer_;
    }

    std::optional<std::uint64_t> MapUpdateIngress::current_route_epoch() const noexcept
    {
        return current_route_epoch_;
    }

    std::optional<std::uint64_t>
    MapUpdateIngress::last_fresh_receive_monotonic_ns() const noexcept
    {
        return last_fresh_receive_monotonic_ns_;
    }

    const PerceptionMapUpdate::MapUpdateApplier & MapUpdateIngress::map_applier() const noexcept
    {
        return map_applier_;
    }

}// namespace SwarmDataPlane

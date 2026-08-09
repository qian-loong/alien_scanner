#include "perception_map_update/ResyncStateMachine.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        bool is_valid_resync_reason(ResyncReason reason) noexcept
        {
            switch(reason) {
                case ResyncReason::InitialBaseline:
                case ResyncReason::Gap:
                case ResyncReason::EpochChange:
                case ResyncReason::HashConflict:
                case ResyncReason::LocalStateInvalid:
                    return true;
            }
            return false;
        }

    }// namespace

    bool ResyncRequester::operator==(const ResyncRequester & other) const noexcept
    {
        return requester_id == other.requester_id
               && requester_session == other.requester_session;
    }

    bool ResyncRequest::operator==(const ResyncRequest & other) const noexcept
    {
        return requester == other.requester && client_request_id == other.client_request_id
               && expected_source == other.expected_source
               && receiver_revision == other.receiver_revision
               && receiver_content_hash == other.receiver_content_hash && reason == other.reason;
    }

    ResyncRequestLedger::ResyncRequestLedger(MapUpdateLimits limits)
            : limits_(std::move(limits))
    {
    }

    ResyncResponse ResyncRequestLedger::accept(
            const ResyncRequest & request,
            const SourceIdentity & current_source,
            std::uint64_t current_revision)
    {
        const auto current_validation = CanonicalCodec::validate_identity(current_source, limits_);
        if(!current_validation) {
            return {false, {}, current_source, current_revision,
                    "producer current source is invalid"};
        }
        const auto requester = CanonicalCodec::validate_string(
                request.requester.requester_id,
                limits_.max_identity_string_bytes,
                "requester_id",
                false);
        const auto client_request = CanonicalCodec::validate_string(
                request.client_request_id,
                limits_.max_correlation_id_bytes,
                "client_request_id",
                false);
        if(!requester || !client_request
           || request.requester.requester_session.boot_time_ns == 0U) {
            return {false, {}, current_source, current_revision,
                    "resync requester or client request id is invalid"};
        }
        if(!is_valid_resync_reason(request.reason)) {
            return {false, {}, current_source, current_revision,
                    "resync reason is invalid"};
        }
        const auto duplicate = std::find_if(
                entries_.begin(), entries_.end(), [&](const Entry & entry) {
                    return entry.request.requester == request.requester
                           && entry.request.client_request_id == request.client_request_id;
                });
        if(duplicate != entries_.end()) {
            if(duplicate->request == request) {
                if(duplicate->response.current_source != current_source) {
                    return {false, {}, current_source, current_revision,
                            "resync request belongs to a retired source chain"};
                }
                return duplicate->response;
            }
            return {false, {}, current_source, current_revision,
                    "client request id conflicts with an earlier resync request"};
        }
        const bool bootstrap = request.reason == ResyncReason::InitialBaseline
                               && !request.expected_source.has_value();
        if(!bootstrap
           && (!request.expected_source.has_value()
               || *request.expected_source != current_source)) {
            return {false, {}, current_source, current_revision,
                    "resync request targets a stale or unknown source chain"};
        }
        if(request.receiver_revision > current_revision) {
            return {false, {}, current_source, current_revision,
                    "receiver revision is ahead of producer revision"};
        }
        if(entries_.size() >= limits_.max_recent_resync_requests
           || next_correlation_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
            return {false, {}, current_source, current_revision,
                    "resync request ledger is at its configured capacity"};
        }

        std::ostringstream correlation;
        correlation << "r-" << std::hex << std::setfill('0')
                    << std::setw(16) << current_source.mapper_session.boot_time_ns << '-'
                    << std::setw(8) << current_source.mapper_session.random_suffix << '-'
                    << std::setw(16) << current_source.map_epoch << '-'
                    << std::setw(16) << next_correlation_sequence_++;
        ResyncResponse response {
                true, correlation.str(), current_source, current_revision, {}};
        if(response.correlation_id.size() > limits_.max_correlation_id_bytes) {
            return {false, {}, current_source, current_revision,
                    "generated correlation id exceeds configured limit"};
        }
        entries_.push_back({request, response});
        return response;
    }

    std::size_t ResyncRequestLedger::size() const noexcept
    {
        return entries_.size();
    }

}// namespace PerceptionMapUpdate

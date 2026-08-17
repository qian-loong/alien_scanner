#ifndef PERCEPTION_MAP_UPDATE_RESYNC_STATE_MACHINE_HPP
#define PERCEPTION_MAP_UPDATE_RESYNC_STATE_MACHINE_HPP

#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <deque>
#include <optional>

namespace PerceptionMapUpdate {

    enum class ResyncReason : std::uint8_t
    {
        InitialBaseline = 1,
        Gap             = 2,
        EpochChange     = 3,
        HashConflict    = 4,
        LocalStateInvalid = 5
    };

    struct ResyncRequester {
        std::string           requester_id;
        Perception::SessionID requester_session {0U, 0U};

        bool operator==(const ResyncRequester & other) const noexcept;
    };

    struct ResyncRequest {
        ResyncRequester              requester;
        std::string                  client_request_id;
        std::optional<SourceIdentity> expected_source;
        std::uint64_t                receiver_revision = 0U;
        VersionedContentDigest       receiver_content_identity;
        ResyncReason                reason = ResyncReason::InitialBaseline;

        bool operator==(const ResyncRequest & other) const noexcept;
    };

    struct ResyncResponse {
        bool           accepted = false;
        std::string    correlation_id;
        SourceIdentity current_source;
        std::uint64_t  current_revision = 0U;
        VersionedContentDigest current_content_identity;
        std::string    diagnostic;
    };

    class ResyncRequestLedger
    {
    public:
        explicit ResyncRequestLedger(MapUpdateLimits limits = {});

        ResyncResponse accept(
                const ResyncRequest & request,
                const SourceIdentity & current_source,
                std::uint64_t current_revision,
                const VersionedContentDigest & current_content_identity);
        std::size_t size() const noexcept;

    private:
        struct Entry {
            ResyncRequest request;
            ResyncResponse response;
        };

        MapUpdateLimits limits_;
        std::deque<Entry> entries_;
        std::uint64_t next_correlation_sequence_ = 1U;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_RESYNC_STATE_MACHINE_HPP

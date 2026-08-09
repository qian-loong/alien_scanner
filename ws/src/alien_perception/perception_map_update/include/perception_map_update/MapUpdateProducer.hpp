#ifndef PERCEPTION_MAP_UPDATE_MAP_UPDATE_PRODUCER_HPP
#define PERCEPTION_MAP_UPDATE_MAP_UPDATE_PRODUCER_HPP

#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <memory>
#include <optional>

namespace PerceptionMapUpdate {

    enum class ProduceStatus : std::uint8_t
    {
        ProducedKeyframe,
        ProducedDelta,
        NoNewRevision,
        RejectedInvalidSnapshot,
        RejectedResourceLimit
    };

    struct ProducerBaselineToken {
        SourceIdentity source;
        std::uint64_t  revision = 0U;
        Hash256        content_hash {};
    };

    struct PrepareTiming {
        std::int64_t validation_duration_ns = 0;
        std::int64_t diff_duration_ns = 0;
        std::int64_t encode_duration_ns = 0;
        std::int64_t update_hash_duration_ns = 0;
    };

    struct PreparedUpdate {
        ProduceStatus                            status = ProduceStatus::RejectedInvalidSnapshot;
        std::optional<MapUpdate>                 update;
        std::shared_ptr<const CanonicalSnapshot> target_snapshot;
        std::string                              diagnostic;
        PrepareTiming                            timing;
        std::optional<ProducerBaselineToken>     expected_baseline = std::nullopt;
    };

    class MapUpdateProducer
    {
    public:
        explicit MapUpdateProducer(MapUpdateLimits limits = {});

        PreparedUpdate prepare(
                const CanonicalSnapshot & target,
                std::uint64_t observed_coalesced_receipt_count = 0U) const;
        PreparedUpdate prepare(
                std::shared_ptr<const CanonicalSnapshot> target,
                std::uint64_t observed_coalesced_receipt_count = 0U) const;
        bool commit_published(const PreparedUpdate & prepared);
        bool request_keyframe(std::string correlation_id);
        bool cancel_keyframe_request(const std::string & correlation_id) noexcept;

        const std::shared_ptr<const CanonicalSnapshot> & baseline() const noexcept;
        std::uint64_t delta_chain_length() const noexcept;
        bool keyframe_pending() const noexcept;

    private:
        ValidationResult validate_snapshot(const CanonicalSnapshot & snapshot) const;
        PreparedUpdate make_keyframe(
                std::shared_ptr<const CanonicalSnapshot> target,
                std::uint64_t observed_coalesced_receipt_count,
                PrepareTiming timing) const;
        std::optional<ProducerBaselineToken> current_baseline_token() const;

        MapUpdateLimits limits_;
        std::shared_ptr<const CanonicalSnapshot> baseline_;
        std::uint64_t                           delta_chain_length_ = 0U;
        std::uint64_t                           last_keyframe_revision_ = 0U;
        std::optional<std::string> pending_correlation_id_;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MAP_UPDATE_PRODUCER_HPP

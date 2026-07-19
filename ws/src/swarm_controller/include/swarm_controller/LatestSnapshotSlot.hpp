#ifndef SWARM_CONTROLLER_LATESTSNAPSHOTSLOT_HPP
#define SWARM_CONTROLLER_LATESTSNAPSHOTSLOT_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    enum class SnapshotStampPolicy {
        StrictlyIncreasing,
        NonDecreasing,
    };

    enum class SnapshotSubmitStatus {
        Accepted,
        SameStampAccepted,
        RejectedRegressed,
        RejectedDuplicate,
    };

    template<typename Input, typename Result>
    class LatestSnapshotSlot
    {
    public:
        struct PendingItem {
            std::uint64_t revision {};
            std::int64_t stamp_ns {};
            double received_seconds {};
            Input payload;
        };

        struct ReadyItem {
            std::uint64_t revision {};
            std::int64_t stamp_ns {};
            double received_seconds {};
            Result payload;
        };

        explicit LatestSnapshotSlot(const SnapshotStampPolicy policy)
            : policy_(policy)
        {
        }

        SnapshotSubmitStatus submit(
                Input payload, const std::int64_t stamp_ns,
                const double received_seconds)
        {
            if(!std::isfinite(received_seconds) || received_seconds < 0.0) {
                throw std::invalid_argument("snapshot receive time must be finite and non-negative");
            }
            if(has_accepted_input_ && stamp_ns < latest_accepted_stamp_ns_) {
                ++regressed_rejected_count_;
                return SnapshotSubmitStatus::RejectedRegressed;
            }
            const bool same_stamp =
                    has_accepted_input_ && stamp_ns == latest_accepted_stamp_ns_;
            if(has_accepted_input_ && policy_ == SnapshotStampPolicy::StrictlyIncreasing
               && stamp_ns <= latest_accepted_stamp_ns_)
            {
                ++duplicate_rejected_count_;
                return SnapshotSubmitStatus::RejectedDuplicate;
            }
            if(latest_accepted_revision_ == std::numeric_limits<std::uint64_t>::max()) {
                throw std::overflow_error("snapshot revision exhausted");
            }
            ++latest_accepted_revision_;
            if(pending_.has_value()) {
                ++pending_coalesced_count_;
            }
            pending_.emplace(PendingItem {
                    latest_accepted_revision_, stamp_ns, received_seconds,
                    std::move(payload)});
            latest_accepted_stamp_ns_ = stamp_ns;
            latest_input_receive_seconds_ = received_seconds;
            has_accepted_input_ = true;
            ++accepted_count_;
            if(same_stamp) {
                ++same_stamp_accepted_count_;
                return SnapshotSubmitStatus::SameStampAccepted;
            }
            return SnapshotSubmitStatus::Accepted;
        }

        std::optional<PendingItem> takePending()
        {
            if(!pending_.has_value() || in_flight_revision_ != 0U || hasReady()) {
                return {};
            }
            std::optional<PendingItem> result = std::move(pending_);
            pending_.reset();
            in_flight_revision_ = result->revision;
            return result;
        }

        bool hasProcessablePending() const
        {
            return pending_.has_value() && in_flight_revision_ == 0U && !hasReady();
        }

        bool publishResult(
                const std::uint64_t revision, const std::int64_t stamp_ns,
                const double received_seconds, Result payload)
        {
            if(revision == 0U || revision > latest_accepted_revision_) {
                throw std::invalid_argument("snapshot result revision was not accepted");
            }
            if(revision != in_flight_revision_) {
                if(revision <= consumed_revision_) {
                    ++superseded_result_count_;
                    return false;
                }
                throw std::invalid_argument("snapshot result does not match in-flight revision");
            }
            if(revision <= consumed_revision_) {
                in_flight_revision_ = 0U;
                ++superseded_result_count_;
                return false;
            }
            if(hasReady()) {
                ++ready_blocked_count_;
                throw std::logic_error("snapshot ready result was not acknowledged");
            }
            in_flight_revision_ = 0U;
            ready_.emplace(ReadyItem {
                    revision, stamp_ns, received_seconds, std::move(payload)});
            return true;
        }

        std::optional<ReadyItem> claimReady()
        {
            if(!ready_.has_value() || ready_claimed_) {
                return {};
            }
            std::optional<ReadyItem> result = std::move(ready_);
            ready_.reset();
            ready_claimed_ = true;
            claimed_revision_ = result->revision;
            claimed_stamp_ns_ = result->stamp_ns;
            claimed_receive_seconds_ = result->received_seconds;
            return result;
        }

        bool acknowledgeReady(const std::uint64_t revision, const bool applied = true)
        {
            if(!ready_claimed_ || claimed_revision_ != revision) {
                return false;
            }
            consumed_revision_ = revision;
            if(applied) {
                last_applied_stamp_ns_ = claimed_stamp_ns_;
                last_applied_receive_seconds_ = claimed_receive_seconds_;
            } else if(latest_accepted_revision_ == revision) {
                latest_accepted_stamp_ns_ = last_applied_stamp_ns_;
                latest_input_receive_seconds_ = last_applied_receive_seconds_;
                has_accepted_input_ = last_applied_stamp_ns_ != 0;
            }
            ready_claimed_ = false;
            claimed_revision_ = 0U;
            claimed_stamp_ns_ = 0;
            claimed_receive_seconds_ = -1.0;
            return true;
        }

        std::optional<ReadyItem> takeReady()
        {
            auto result = claimReady();
            if(!result.has_value()) {
                return {};
            }
            const std::uint64_t revision = result->revision;
            if(!acknowledgeReady(revision)) {
                throw std::logic_error("snapshot ready result acknowledgement failed");
            }
            return result;
        }

        void clear()
        {
            pending_.reset();
            ready_.reset();
            in_flight_revision_ = 0U;
            ready_claimed_ = false;
            claimed_revision_ = 0U;
            claimed_stamp_ns_ = 0U;
            claimed_receive_seconds_ = -1.0;
        }

        void resetStampWatermark()
        {
            clear();
            consumed_revision_ = latest_accepted_revision_;
            has_accepted_input_ = false;
            latest_accepted_stamp_ns_ = 0;
            latest_input_receive_seconds_ = -1.0;
            last_applied_stamp_ns_ = 0;
            last_applied_receive_seconds_ = -1.0;
        }

        bool hasPending() const
        {
            return pending_.has_value();
        }

        bool hasReady() const
        {
            return ready_.has_value() || ready_claimed_;
        }

        bool readyClaimed() const
        {
            return ready_claimed_;
        }

        std::uint64_t latestAcceptedRevision() const
        {
            return latest_accepted_revision_;
        }

        std::int64_t latestAcceptedStampNs() const
        {
            return latest_accepted_stamp_ns_;
        }

        double latestInputReceiveSeconds() const
        {
            return latest_input_receive_seconds_;
        }

        std::uint64_t consumedRevision() const
        {
            return consumed_revision_;
        }

        std::int64_t lastAppliedStampNs() const
        {
            return last_applied_stamp_ns_;
        }

        double lastAppliedReceiveSeconds() const
        {
            return last_applied_receive_seconds_;
        }

        std::uint64_t pendingRevision() const
        {
            return pending_.has_value() ? pending_->revision : 0U;
        }

        std::uint64_t readyRevision() const
        {
            return ready_.has_value() ? ready_->revision : claimed_revision_;
        }

        std::uint64_t inFlightRevision() const
        {
            return in_flight_revision_;
        }

        std::uint64_t acceptedCount() const
        {
            return accepted_count_;
        }

        std::uint64_t sameStampAcceptedCount() const
        {
            return same_stamp_accepted_count_;
        }

        std::uint64_t regressedRejectedCount() const
        {
            return regressed_rejected_count_;
        }

        std::uint64_t duplicateRejectedCount() const
        {
            return duplicate_rejected_count_;
        }

        std::uint64_t pendingCoalescedCount() const
        {
            return pending_coalesced_count_;
        }

        std::uint64_t readyCoalescedCount() const
        {
            return ready_blocked_count_;
        }

        std::uint64_t readyBlockedCount() const
        {
            return ready_blocked_count_;
        }

        std::uint64_t supersededResultCount() const
        {
            return superseded_result_count_;
        }

    private:
        SnapshotStampPolicy policy_;
        bool has_accepted_input_ {false};
        std::int64_t latest_accepted_stamp_ns_ {};
        double latest_input_receive_seconds_ {-1.0};
        std::uint64_t latest_accepted_revision_ {};
        std::uint64_t consumed_revision_ {};
        std::uint64_t in_flight_revision_ {};
        std::int64_t last_applied_stamp_ns_ {};
        double last_applied_receive_seconds_ {-1.0};
        std::optional<PendingItem> pending_;
        std::optional<ReadyItem> ready_;
        bool ready_claimed_ {false};
        std::uint64_t claimed_revision_ {};
        std::int64_t claimed_stamp_ns_ {};
        double claimed_receive_seconds_ {-1.0};
        std::uint64_t accepted_count_ {};
        std::uint64_t same_stamp_accepted_count_ {};
        std::uint64_t regressed_rejected_count_ {};
        std::uint64_t duplicate_rejected_count_ {};
        std::uint64_t pending_coalesced_count_ {};
        std::uint64_t ready_blocked_count_ {};
        std::uint64_t superseded_result_count_ {};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_LATESTSNAPSHOTSLOT_HPP

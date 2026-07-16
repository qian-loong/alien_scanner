#include "swarm_controller/TaskLeaseTracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr float POINT_EPSILON = 1.0e-6F;

        bool samePoint(const Point3f & lhs, const Point3f & rhs)
        {
            return std::fabs(lhs.x - rhs.x) <= POINT_EPSILON
                   && std::fabs(lhs.y - rhs.y) <= POINT_EPSILON
                   && std::fabs(lhs.z - rhs.z) <= POINT_EPSILON;
        }

    }// namespace

    TaskLeaseTracker::TaskLeaseTracker(TaskLeaseTrackerConfig config)
        : config_(std::move(config))
    {
        if(!std::isfinite(config_.receive_watchdog_seconds)
           || config_.receive_watchdog_seconds <= 0.0)
        {
            throw std::invalid_argument(
                    "task receive watchdog must be positive and finite");
        }
    }

    TaskUpdateResult TaskLeaseTracker::update(
            const ExplorationTaskControl & control, const std::int64_t now_ros_ns,
            const double receive_steady_seconds)
    {
        const auto reject = [this](TaskUpdateStatus status, std::string reason) {
            ++rejected_count_;
            return TaskUpdateResult {status, std::move(reason)};
        };

        if(now_ros_ns < 0 || !std::isfinite(receive_steady_seconds)
           || receive_steady_seconds < 0.0)
        {
            return reject(TaskUpdateStatus::RejectedInvalid, "invalid receive time");
        }
        if(control.issued_time_ns <= 0 || control.lease_duration_ns <= 0
           || control.allocator_epoch == 0U || control.revision == 0U
           || !validMode(control.mode))
        {
            return reject(TaskUpdateStatus::RejectedInvalid, "invalid task envelope");
        }
        if(control.mode == ExplorationTaskMode::Assigned) {
            if(control.task_id == 0U || !finitePoint(control.target)) {
                return reject(TaskUpdateStatus::RejectedInvalid, "invalid assigned task");
            }
        } else if(control.task_id != 0U) {
            return reject(TaskUpdateStatus::RejectedInvalid, "non-assigned task id must be zero");
        }

        std::int64_t expiry_ns = 0;
        if(!checkedExpiry(control, expiry_ns) || expiry_ns <= now_ros_ns) {
            return reject(TaskUpdateStatus::RejectedStale, "task lease already expired");
        }

        if(last_accepted_.has_value()) {
            const ExplorationTaskControl & previous = *last_accepted_;
            if(control.allocator_epoch == previous.allocator_epoch) {
                if(control.revision < previous.revision) {
                    return reject(TaskUpdateStatus::RejectedStale, "task revision regressed");
                }
                if(control.revision == previous.revision) {
                    if(!samePayload(control, previous)) {
                        return reject(
                                TaskUpdateStatus::RejectedInvalid,
                                "task payload changed without revision");
                    }
                    if(control.issued_time_ns <= previous.issued_time_ns) {
                        return reject(
                                TaskUpdateStatus::RejectedStale,
                                "task renewal timestamp did not advance");
                    }
                    last_accepted_                = control;
                    last_expiry_ns_               = expiry_ns;
                    last_receive_steady_seconds_  = receive_steady_seconds;
                    return {TaskUpdateStatus::AcceptedRenewal, {}};
                }
            } else if(control.issued_time_ns <= previous.issued_time_ns) {
                return reject(
                        TaskUpdateStatus::RejectedStale,
                        "older allocator epoch replay");
            }
        }

        last_accepted_               = control;
        last_expiry_ns_              = expiry_ns;
        last_receive_steady_seconds_ = receive_steady_seconds;
        return {TaskUpdateStatus::AcceptedNew, {}};
    }

    ExplorationTaskSnapshot TaskLeaseTracker::snapshot(
            const std::int64_t now_ros_ns, const double now_steady_seconds) const
    {
        if(!last_accepted_.has_value() || now_ros_ns < 0
           || !std::isfinite(now_steady_seconds) || now_steady_seconds < 0.0)
        {
            return {};
        }
        const bool ros_expired = now_ros_ns >= last_expiry_ns_;
        const bool receive_expired = last_receive_steady_seconds_ < 0.0
                                     || now_steady_seconds - last_receive_steady_seconds_
                                                > config_.receive_watchdog_seconds;
        if(ros_expired || receive_expired) {
            return {};
        }
        return ExplorationTaskSnapshot {true, *last_accepted_, last_expiry_ns_};
    }

    const std::optional<ExplorationTaskControl> & TaskLeaseTracker::lastAccepted() const
    {
        return last_accepted_;
    }

    std::size_t TaskLeaseTracker::rejectedCount() const
    {
        return rejected_count_;
    }

    bool TaskLeaseTracker::validMode(const ExplorationTaskMode mode)
    {
        return mode == ExplorationTaskMode::LocalFallback
               || mode == ExplorationTaskMode::Assigned
               || mode == ExplorationTaskMode::Standby;
    }

    bool TaskLeaseTracker::finitePoint(const Point3f & point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    bool TaskLeaseTracker::samePayload(
            const ExplorationTaskControl & lhs, const ExplorationTaskControl & rhs)
    {
        return lhs.allocator_epoch == rhs.allocator_epoch
               && lhs.revision == rhs.revision && lhs.task_id == rhs.task_id
               && lhs.mode == rhs.mode && samePoint(lhs.target, rhs.target)
               && lhs.lease_duration_ns == rhs.lease_duration_ns;
    }

    bool TaskLeaseTracker::checkedExpiry(
            const ExplorationTaskControl & control, std::int64_t & expiry_ns)
    {
        if(control.issued_time_ns
           > std::numeric_limits<std::int64_t>::max() - control.lease_duration_ns)
        {
            return false;
        }
        expiry_ns = control.issued_time_ns + control.lease_duration_ns;
        return true;
    }

}// namespace SwarmController

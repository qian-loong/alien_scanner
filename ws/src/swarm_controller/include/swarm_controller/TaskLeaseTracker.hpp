#ifndef SWARM_CONTROLLER_TASKLEASETRACKER_HPP
#define SWARM_CONTROLLER_TASKLEASETRACKER_HPP

#include "swarm_controller/ExplorationTask.hpp"

#include <optional>
#include <string>

namespace SwarmController {

    struct TaskLeaseTrackerConfig {
        double receive_watchdog_seconds {3.5};
    };

    enum class TaskUpdateStatus {
        AcceptedNew,
        AcceptedRenewal,
        RejectedInvalid,
        RejectedStale,
    };

    struct TaskUpdateResult {
        TaskUpdateStatus status {TaskUpdateStatus::RejectedInvalid};
        std::string      reason;

        bool accepted() const
        {
            return status == TaskUpdateStatus::AcceptedNew
                   || status == TaskUpdateStatus::AcceptedRenewal;
        }
    };

    class TaskLeaseTracker
    {
    public:
        explicit TaskLeaseTracker(TaskLeaseTrackerConfig config = {});

        TaskUpdateResult update(
                const ExplorationTaskControl & control, std::int64_t now_ros_ns,
                double receive_steady_seconds);

        ExplorationTaskSnapshot snapshot(
                std::int64_t now_ros_ns, double now_steady_seconds) const;

        const std::optional<ExplorationTaskControl> & lastAccepted() const;
        std::size_t rejectedCount() const;

    private:
        static bool validMode(ExplorationTaskMode mode);
        static bool finitePoint(const Point3f & point);
        static bool samePayload(
                const ExplorationTaskControl & lhs,
                const ExplorationTaskControl & rhs);
        static bool checkedExpiry(
                const ExplorationTaskControl & control, std::int64_t & expiry_ns);

        TaskLeaseTrackerConfig                config_;
        std::optional<ExplorationTaskControl> last_accepted_;
        std::int64_t                          last_expiry_ns_ {};
        double                                last_receive_steady_seconds_ {-1.0};
        std::size_t                           rejected_count_ {};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_TASKLEASETRACKER_HPP

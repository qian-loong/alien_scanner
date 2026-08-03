#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "perception_interfaces/msg/pose_estimate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace {

    using SteadyClock = std::chrono::steady_clock;

    std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
    {
        return rclcpp::Time(stamp).nanoseconds();
    }

    struct PoseLineage {
        std::uint64_t session_boot_time_ns {0U};
        std::uint32_t session_random_suffix {0U};
        std::uint64_t reset_epoch {0U};

        bool operator==(const PoseLineage & other) const noexcept
        {
            return session_boot_time_ns == other.session_boot_time_ns
                   && session_random_suffix == other.session_random_suffix
                   && reset_epoch == other.reset_epoch;
        }

        bool operator<(const PoseLineage & other) const noexcept
        {
            return std::tie(session_boot_time_ns, session_random_suffix, reset_epoch)
                   < std::tie(
                           other.session_boot_time_ns,
                           other.session_random_suffix,
                           other.reset_epoch);
        }
    };

    struct PendingScan {
        sensor_msgs::msg::LaserScan message;
        std::int64_t stamp_nanoseconds {0};
        SteadyClock::time_point deadline;
        std::optional<SteadyClock::time_point> release_after;
    };

}// namespace

class PoseGatedLaserScanRelay final : public rclcpp::Node
{
public:
    PoseGatedLaserScanRelay()
        : Node("pose_gated_laser_scan_relay")
        , raw_scan_topic_(declare_parameter<std::string>("raw_scan_topic", "raw_scan"))
        , released_scan_topic_(declare_parameter<std::string>("released_scan_topic", "scan"))
        , pose_topic_(declare_parameter<std::string>("pose_topic", "perception/pose"))
        , expected_pose_frame_(declare_parameter<std::string>("expected_pose_frame", "map"))
        , expected_pose_source_(declare_parameter<std::string>("expected_pose_source", "odom"))
        , expected_clock_domain_(declare_parameter<std::string>(
                  "expected_clock_domain", "vehicle_steady_clock"))
    {
        const double lead_delay_s = declare_parameter<double>("pose_lead_delay_s", 0.1);
        const double odom_period_s = declare_parameter<double>("odom_period_s", 0.05);
        const double pending_timeout_s = declare_parameter<double>("pending_timeout_s", 1.0);
        const auto max_pending_parameter = declare_parameter<std::int64_t>("max_pending_scans", 64);
        if(raw_scan_topic_.empty() || released_scan_topic_.empty() || pose_topic_.empty()
           || expected_pose_frame_.empty() || expected_pose_source_.empty()
           || expected_clock_domain_.empty()) {
            throw std::invalid_argument("PoseGatedLaserScanRelay names must not be empty");
        }
        if(!std::isfinite(lead_delay_s) || !std::isfinite(odom_period_s)
           || !std::isfinite(pending_timeout_s) || odom_period_s <= 0.0
           || lead_delay_s < 2.0 * odom_period_s || pending_timeout_s <= lead_delay_s
           || max_pending_parameter <= 0) {
            throw std::invalid_argument(
                    "PoseGatedLaserScanRelay timing/queue parameters violate the frozen contract");
        }
        lead_delay_ = std::chrono::duration_cast<SteadyClock::duration>(
                std::chrono::duration<double>(lead_delay_s));
        pending_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
                std::chrono::duration<double>(pending_timeout_s));
        max_pending_scans_ = static_cast<std::size_t>(max_pending_parameter);

        released_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
                released_scan_topic_, rclcpp::SensorDataQoS());
        diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", rclcpp::QoS(10));
        raw_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
                raw_scan_topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
                    on_scan(*message);
                });
        pose_subscription_ = create_subscription<perception_interfaces::msg::PoseEstimate>(
                pose_topic_, rclcpp::QoS(10),
                [this](const perception_interfaces::msg::PoseEstimate::SharedPtr message) {
                    on_pose(*message);
                });
        release_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { release_ready(); });
    }

private:
    std::string raw_scan_topic_;
    std::string released_scan_topic_;
    std::string pose_topic_;
    std::string expected_pose_frame_;
    std::string expected_pose_source_;
    std::string expected_clock_domain_;
    SteadyClock::duration lead_delay_ {};
    SteadyClock::duration pending_timeout_ {};
    std::size_t max_pending_scans_ {64U};
    std::int64_t last_raw_stamp_ns_ {0};
    std::int64_t last_pose_stamp_ns_ {0};
    std::optional<PoseLineage> active_lineage_;
    std::set<PoseLineage> retired_lineages_;
    std::deque<PendingScan> pending_;
    std::unordered_map<std::int64_t, SteadyClock::time_point> pose_watermarks_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_subscription_;
    rclcpp::Subscription<perception_interfaces::msg::PoseEstimate>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr released_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr release_timer_;

    void on_scan(const sensor_msgs::msg::LaserScan & message)
    {
        const auto stamp = stamp_ns(message.header.stamp);
        if(stamp <= 0 || (last_raw_stamp_ns_ != 0 && stamp <= last_raw_stamp_ns_)) {
            reject("Dropped raw scan with non-increasing acquisition stamp");
            return;
        }
        last_raw_stamp_ns_ = stamp;
        if(pending_.size() >= max_pending_scans_) {
            pending_.pop_front();
            reject("Dropped oldest raw scan after pending queue overflow");
        }

        const auto now = SteadyClock::now();
        PendingScan pending {message, stamp, now + pending_timeout_, std::nullopt};
        const auto watermark = pose_watermarks_.find(stamp);
        if(watermark != pose_watermarks_.end()) {
            pending.release_after = watermark->second + lead_delay_;
        }
        pending_.push_back(std::move(pending));
    }

    void on_pose(const perception_interfaces::msg::PoseEstimate & message)
    {
        const auto stamp = stamp_ns(message.header.stamp);
        const PoseLineage lineage {
                message.session_boot_time_ns,
                message.session_random_suffix,
                message.reset_epoch};
        if(stamp <= 0 || message.header.frame_id != expected_pose_frame_
           || message.source_id != expected_pose_source_
           || message.clock_domain != expected_clock_domain_
           || lineage.session_boot_time_ns == 0U) {
            reject("Dropped PoseEstimate watermark outside the frozen identity contract");
            return;
        }
        if(retired_lineages_.count(lineage) != 0U) {
            reject("Dropped PoseEstimate watermark from retired lineage");
            return;
        }
        if(active_lineage_.has_value() && !(lineage == active_lineage_.value())) {
            retired_lineages_.insert(active_lineage_.value());
            pending_.clear();
            pose_watermarks_.clear();
            last_pose_stamp_ns_ = 0;
        }
        if(last_pose_stamp_ns_ != 0 && stamp <= last_pose_stamp_ns_) {
            reject("Dropped non-increasing PoseEstimate watermark");
            return;
        }
        active_lineage_ = lineage;
        last_pose_stamp_ns_ = stamp;
        const auto now = SteadyClock::now();
        pose_watermarks_[stamp] = now;
        for(auto & pending : pending_) {
            if(pending.stamp_nanoseconds == stamp && !pending.release_after.has_value()) {
                pending.release_after = now + lead_delay_;
            }
        }
        prune_watermarks();
    }

    void release_ready()
    {
        const auto now = SteadyClock::now();
        while(!pending_.empty()) {
            auto & pending = pending_.front();
            if(!pending.release_after.has_value()) {
                if(now < pending.deadline) {
                    break;
                }
                pending_.pop_front();
                reject("Dropped raw scan after PoseEstimate watermark timeout");
                continue;
            }
            if(now < pending.release_after.value()) {
                break;
            }
            released_publisher_->publish(pending.message);
            pose_watermarks_.erase(pending.stamp_nanoseconds);
            pending_.pop_front();
        }
    }

    void prune_watermarks()
    {
        if(pose_watermarks_.size() <= max_pending_scans_ * 4U) {
            return;
        }
        const auto floor_stamp = pending_.empty()
                ? last_raw_stamp_ns_
                : pending_.front().stamp_nanoseconds;
        for(auto iterator = pose_watermarks_.begin(); iterator != pose_watermarks_.end();) {
            if(iterator->first < floor_stamp) {
                iterator = pose_watermarks_.erase(iterator);
            }
            else {
                ++iterator;
            }
        }
    }

    void reject(const std::string & message)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = get_clock()->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name() + std::string(": pose_gate");
        status.hardware_id = "validation-only";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = message;
        array.status.push_back(std::move(status));
        diagnostics_publisher_->publish(array);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", message.c_str());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<PoseGatedLaserScanRelay>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("pose_gated_laser_scan_relay"), "%s", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}

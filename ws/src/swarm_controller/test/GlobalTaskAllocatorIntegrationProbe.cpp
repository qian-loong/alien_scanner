#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace {

    diagnostic_msgs::msg::KeyValue diagnosticValue(
            const std::string & key, const std::string & value)
    {
        diagnostic_msgs::msg::KeyValue result;
        result.key = key;
        result.value = value;
        return result;
    }

    void fillBox(
            octomap::OcTree & tree, float min_x, float max_x,
            float min_y, float max_y, float min_z, float max_z, bool occupied)
    {
        const auto minimum = tree.coordToKey(min_x, min_y, min_z);
        const auto maximum = tree.coordToKey(max_x, max_y, max_z);
        for(std::uint32_t x = minimum[0]; x <= maximum[0]; ++x) {
            for(std::uint32_t y = minimum[1]; y <= maximum[1]; ++y) {
                for(std::uint32_t z = minimum[2]; z <= maximum[2]; ++z) {
                    tree.updateNode(
                            octomap::OcTreeKey {
                                    static_cast<octomap::key_type>(x),
                                    static_cast<octomap::key_type>(y),
                                    static_cast<octomap::key_type>(z)},
                            occupied, true);
                }
            }
        }
        tree.updateInnerOccupancy();
    }

    octomap::OcTree globalTree()
    {
        octomap::OcTree tree(0.1);
        for(const float center_y : {-1.5F, 1.5F}) {
            fillBox(tree, 0.0F, 3.0F, center_y - 0.6F, center_y + 0.6F,
                    1.0F, 2.0F, false);
            fillBox(tree, -0.1F, -0.1F, center_y - 0.7F, center_y + 0.7F,
                    0.9F, 2.1F, true);
            fillBox(tree, 0.0F, 3.0F, center_y - 0.7F, center_y - 0.7F,
                    0.9F, 2.1F, true);
            fillBox(tree, 0.0F, 3.0F, center_y + 0.7F, center_y + 0.7F,
                    0.9F, 2.1F, true);
        }
        return tree;
    }

    octomap::OcTree localTree()
    {
        octomap::OcTree tree(0.1);
        fillBox(tree, -1.0F, 4.0F, -3.0F, 3.0F, 0.5F, 2.5F, false);
        return tree;
    }

    class Probe final : public rclcpp::Node
    {
    public:
        Probe()
            : Node("global_task_allocator_integration_probe")
            , global_tree_(globalTree())
            , local_tree_(localTree())
        {
            declare_parameter("stamp_offset_seconds", 0.0);
            declare_parameter("diagnostics_reject_after_seconds", -1.0);
            declare_parameter("controlled_global_updates", false);
            declare_parameter("publish_period_ms", 200);
            stamp_offset_seconds_ = get_parameter("stamp_offset_seconds").as_double();
            diagnostics_reject_after_seconds_ =
                    get_parameter("diagnostics_reject_after_seconds").as_double();
            controlled_global_updates_ =
                    get_parameter("controlled_global_updates").as_bool();
            publish_period_ms_ = get_parameter("publish_period_ms").as_int();
            if(!std::isfinite(stamp_offset_seconds_) || stamp_offset_seconds_ < 0.0) {
                throw std::invalid_argument(
                        "stamp_offset_seconds must be finite and non-negative");
            }
            if(!std::isfinite(diagnostics_reject_after_seconds_)) {
                throw std::invalid_argument(
                        "diagnostics_reject_after_seconds must be finite");
            }
            if(publish_period_ms_ <= 0) {
                throw std::invalid_argument("publish_period_ms must be positive");
            }
            fixed_global_stamp_ns_ = get_clock()->now().nanoseconds();

            const auto qos = rclcpp::QoS(1).reliable().transient_local();
            global_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
                    "/global_map", qos);
            diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                    "/global_map_diagnostics", qos);
            same_global_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/publish_same_stamp_global_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        same_global_update_requested_ = true;
                    });
            older_global_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/publish_older_global_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        older_global_update_requested_ = true;
                    });
            invalid_global_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/publish_invalid_global_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        invalid_global_update_requested_ = true;
                    });
            recover_global_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/recover_global_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        global_recovery_requested_ = true;
                    });
            invalid_local_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/publish_invalid_local_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        invalid_local_update_requested_ = true;
                    });
            recover_local_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/recover_local_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        local_recovery_requested_ = true;
                    });
            invalid_stamp_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/publish_invalid_stamp_inputs", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        invalid_stamp_inputs_requested_ = true;
                    });
            future_global_trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/global_task_allocator/publish_future_global_map", rclcpp::QoS(1),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        future_global_update_requested_ = true;
                    });
            for(int index = 0; index < 3; ++index) {
                local_publishers_.push_back(create_publisher<octomap_msgs::msg::Octomap>(
                        "/drone_" + std::to_string(index) + "/octomap", qos));
                odom_publishers_.push_back(create_publisher<nav_msgs::msg::Odometry>(
                        "/drone_" + std::to_string(index) + "/odom", rclcpp::QoS(10)));
            }
            timer_ = create_wall_timer(
                    std::chrono::milliseconds(publish_period_ms_),
                    [this]() { publish(); });
        }

    private:
        void publish()
        {
            const double elapsed_seconds = std::chrono::duration<double>(
                                                   std::chrono::steady_clock::now() - start_time_)
                                                   .count();
            const auto stamp = get_clock()->now()
                               - rclcpp::Duration::from_seconds(stamp_offset_seconds_);
            if(stamp.nanoseconds() <= 0) {
                return;
            }
            bool publish_global = !controlled_global_updates_;
            bool publish_older_global = false;
            bool publish_invalid_global = false;
            bool publish_global_recovery = false;
            if(controlled_global_updates_) {
                if(!global_initial_published_) {
                    publish_global = true;
                } else if(same_global_update_requested_
                          && !global_same_stamp_update_published_)
                {
                    global_tree_.updateNode(
                            octomap::point3d(4.0F, 0.0F, 1.5F), false, true);
                    global_tree_.updateInnerOccupancy();
                    global_same_stamp_update_published_ = true;
                    publish_global = true;
                } else if(older_global_update_requested_
                          && !global_older_stamp_update_published_)
                {
                    global_tree_.updateNode(
                            octomap::point3d(4.2F, 0.0F, 1.5F), false, true);
                    global_tree_.updateInnerOccupancy();
                    global_older_stamp_update_published_ = true;
                    publish_older_global = true;
                    publish_global = true;
                } else if(invalid_global_update_requested_
                          && !global_invalid_update_published_)
                {
                    global_failure_stamp_ns_ = fixed_global_stamp_ns_ + 1;
                    global_invalid_update_published_ = true;
                    publish_invalid_global = true;
                    publish_global = true;
                } else if(global_recovery_requested_
                          && global_invalid_update_published_
                          && !global_recovery_published_)
                {
                    global_recovery_published_ = true;
                    publish_global_recovery = true;
                    publish_global = true;
                }
            }
            if(publish_global) {
                octomap_msgs::msg::Octomap global;
                octomap_msgs::fullMapToMsg(global_tree_, global);
                global.header.frame_id = publish_invalid_global ? "invalid_map" : "map";
                if(controlled_global_updates_) {
                    const std::int64_t global_stamp =
                            publish_invalid_global || publish_global_recovery
                                    ? global_failure_stamp_ns_
                                    : fixed_global_stamp_ns_
                                              - (publish_older_global ? 1 : 0);
                    global.header.stamp = rclcpp::Time(
                            global_stamp, get_clock()->get_clock_type());
                } else {
                    global.header.stamp = stamp;
                }
                global_publisher_->publish(global);
                global_initial_published_ = true;
            }

            octomap_msgs::msg::Octomap local;
            octomap_msgs::fullMapToMsg(local_tree_, local);
            local.header.frame_id = "map";
            local.header.stamp = stamp;
            for(std::size_t index = 0U; index < local_publishers_.size(); ++index) {
                if(index == 0U && invalid_local_update_requested_
                   && !local_invalid_update_published_)
                {
                    local_failure_stamp_ns_ = rclcpp::Time(local.header.stamp).nanoseconds();
                    octomap_msgs::msg::Octomap invalid = local;
                    invalid.header.frame_id = "invalid_map";
                    local_publishers_[index]->publish(invalid);
                    local_invalid_update_published_ = true;
                } else if(index == 0U && local_invalid_update_published_
                          && !local_recovery_published_)
                {
                    if(local_recovery_requested_) {
                        octomap_msgs::msg::Octomap recovery = local;
                        recovery.header.stamp = rclcpp::Time(
                                local_failure_stamp_ns_, get_clock()->get_clock_type());
                        local_publishers_[index]->publish(recovery);
                        local_recovery_published_ = true;
                    }
                } else {
                    local_publishers_[index]->publish(local);
                }
                nav_msgs::msg::Odometry odom;
                odom.header.frame_id = "map";
                odom.header.stamp = stamp;
                odom.pose.pose.position.x = 0.0;
                odom.pose.pose.position.y = index == 0U ? -1.5 : (index == 1U ? 1.5 : 0.0);
                odom.pose.pose.position.z = 1.5;
                odom.pose.pose.orientation.w = 1.0;
                odom_publishers_[index]->publish(odom);
            }

            diagnostic_msgs::msg::DiagnosticArray diagnostics;
            diagnostics.header.stamp = stamp;
            diagnostic_msgs::msg::DiagnosticStatus status;
            const bool rejected = diagnostics_reject_after_seconds_ >= 0.0
                                  && elapsed_seconds >= diagnostics_reject_after_seconds_;
            status.name = "global_map_merger";
            status.hardware_id = "test";
            status.level = rejected ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                    : diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = rejected ? "one or more source updates were rejected"
                                      : "all source maps accepted";
            status.values.push_back(diagnosticValue("accepted_sources", "3"));
            status.values.push_back(diagnosticValue("missing_sources", "0"));
            status.values.push_back(diagnosticValue("stale_sources", "0"));
            status.values.push_back(diagnosticValue(
                    "last_rejections",
                    rejected ? "/drone_0/octomap: resource limit" : ""));
            diagnostics.status.push_back(std::move(status));
            diagnostics_publisher_->publish(diagnostics);

            if(invalid_stamp_inputs_requested_ && !invalid_stamp_inputs_published_) {
                builtin_interfaces::msg::Time invalid_stamp;
                invalid_stamp.sec = -1;

                octomap_msgs::msg::Octomap invalid_global;
                octomap_msgs::fullMapToMsg(global_tree_, invalid_global);
                invalid_global.header.frame_id = "map";
                invalid_global.header.stamp = invalid_stamp;
                global_publisher_->publish(invalid_global);

                octomap_msgs::msg::Octomap invalid_local;
                octomap_msgs::fullMapToMsg(local_tree_, invalid_local);
                invalid_local.header.frame_id = "map";
                invalid_local.header.stamp = invalid_stamp;
                local_publishers_.front()->publish(invalid_local);

                nav_msgs::msg::Odometry invalid_odom;
                invalid_odom.header.frame_id = "map";
                invalid_odom.header.stamp = invalid_stamp;
                invalid_odom.pose.pose.orientation.w = 1.0;
                odom_publishers_.front()->publish(invalid_odom);

                diagnostic_msgs::msg::DiagnosticArray invalid_diagnostics;
                invalid_diagnostics.header.stamp = invalid_stamp;
                diagnostic_msgs::msg::DiagnosticStatus invalid_status;
                invalid_status.name = "global_map_merger";
                invalid_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                invalid_status.values.push_back(diagnosticValue("missing_sources", "0"));
                invalid_status.values.push_back(diagnosticValue("stale_sources", "0"));
                invalid_status.values.push_back(diagnosticValue("last_rejections", ""));
                invalid_diagnostics.status.push_back(std::move(invalid_status));
                diagnostics_publisher_->publish(invalid_diagnostics);
                invalid_stamp_inputs_published_ = true;
            }

            if(future_global_update_requested_ && !future_global_update_published_) {
                octomap_msgs::msg::Octomap future_global;
                octomap_msgs::fullMapToMsg(global_tree_, future_global);
                future_global.header.frame_id = "map";
                future_global.header.stamp = get_clock()->now()
                                             + rclcpp::Duration::from_seconds(60.0);
                global_publisher_->publish(future_global);
                future_global_update_published_ = true;
            }
        }

        double stamp_offset_seconds_ {};
        double diagnostics_reject_after_seconds_ {-1.0};
        bool controlled_global_updates_ {false};
        std::int64_t publish_period_ms_ {200};
        std::int64_t fixed_global_stamp_ns_ {};
        bool global_initial_published_ {false};
        bool same_global_update_requested_ {false};
        bool global_same_stamp_update_published_ {false};
        bool older_global_update_requested_ {false};
        bool global_older_stamp_update_published_ {false};
        bool invalid_global_update_requested_ {false};
        bool global_invalid_update_published_ {false};
        bool global_recovery_requested_ {false};
        bool global_recovery_published_ {false};
        std::int64_t global_failure_stamp_ns_ {};
        bool invalid_local_update_requested_ {false};
        bool local_invalid_update_published_ {false};
        bool local_recovery_requested_ {false};
        bool local_recovery_published_ {false};
        std::int64_t local_failure_stamp_ns_ {};
        bool invalid_stamp_inputs_requested_ {false};
        bool invalid_stamp_inputs_published_ {false};
        bool future_global_update_requested_ {false};
        bool future_global_update_published_ {false};
        std::chrono::steady_clock::time_point start_time_ {
                std::chrono::steady_clock::now()};
        octomap::OcTree global_tree_;
        octomap::OcTree local_tree_;
        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr global_publisher_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                diagnostics_publisher_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                same_global_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                older_global_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                invalid_global_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                recover_global_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                invalid_local_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                recover_local_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                invalid_stamp_trigger_subscription_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
                future_global_trigger_subscription_;
        std::vector<rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr>
                local_publishers_;
        std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}// namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Probe>());
    rclcpp::shutdown();
    return 0;
}

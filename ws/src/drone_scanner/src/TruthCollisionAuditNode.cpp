#include "drone_scanner/CaveFieldFromParameters.hpp"

#include "cave_world/TruthCollisionAuditor.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace DroneScanner {

    class TruthCollisionAuditNode final : public rclcpp::Node
    {
    public:
        TruthCollisionAuditNode() : Node("truth_collision_audit")
        {
            declareCaveFieldParameters(*this);
            declare_parameter("odom_topics", std::vector<std::string> {});
            declare_parameter("body.robot_radius", 0.25);
            declare_parameter("body.robot_half_height", 0.15);
            declare_parameter("audit.sample_spacing", 0.05);
            declare_parameter("audit.clearance_direction_count", 32);
            declare_parameter("audit.clearance_max_range", 10.0);

            field_ = createCaveFieldFromParameters(*this);
            CaveWorld::TruthCollisionAuditConfig config;
            config.robot_radius = static_cast<float>(
                    get_parameter("body.robot_radius").as_double());
            config.robot_half_height = static_cast<float>(
                    get_parameter("body.robot_half_height").as_double());
            config.sample_spacing = static_cast<float>(
                    get_parameter("audit.sample_spacing").as_double());
            config.clearance_direction_count = static_cast<std::size_t>(
                    get_parameter("audit.clearance_direction_count").as_int());
            config.clearance_max_range = static_cast<float>(
                    get_parameter("audit.clearance_max_range").as_double());
            auditor_ = std::make_unique<CaveWorld::TruthCollisionAuditor>(*field_, config);

            publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                    "/truth_collision_audit", rclcpp::QoS(1).reliable().transient_local());
            const auto topics = get_parameter("odom_topics").as_string_array();
            odom_topics_ = topics;
            results_.resize(topics.size());
            received_.assign(topics.size(), false);
            minimum_seen_.assign(topics.size(), config.clearance_max_range);
            collision_counts_.assign(topics.size(), 0U);
            subscriptions_.reserve(topics.size());
            for(std::size_t index = 0U; index < topics.size(); ++index) {
                subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                        topics[index], rclcpp::QoS(10),
                        [this, index](
                                const nav_msgs::msg::Odometry::SharedPtr message) {
                            onOdometry(index, *message);
                        }));
            }
        }

    private:
        void onOdometry(
                std::size_t index, const nav_msgs::msg::Odometry & message)
        {
            const auto & position = message.pose.pose.position;
            results_[index] = auditor_->assess(CaveWorld::Point3 {
                    static_cast<float>(position.x), static_cast<float>(position.y),
                    static_cast<float>(position.z)});
            received_[index] = true;
            minimum_seen_[index] = std::min(
                    minimum_seen_[index], results_[index].minimum_clearance);
            if(results_[index].collided) {
                ++collision_counts_[index];
            }

            diagnostic_msgs::msg::DiagnosticArray array;
            array.header.stamp = get_clock()->now();
            for(std::size_t current = 0U; current < results_.size(); ++current) {
                diagnostic_msgs::msg::DiagnosticStatus status;
                status.name = "truth_collision_audit/" + std::to_string(current);
                status.hardware_id = odom_topics_[current];
                status.level = !received_[current]
                                       ? diagnostic_msgs::msg::DiagnosticStatus::STALE
                               : results_[current].collided
                                       ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                       : diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = !received_[current]
                                         ? "WaitingForOdometry"
                                 : results_[current].collided ? "TruthCollision" : "Clear";
                const auto add = [&status](const std::string & key, const auto value) {
                    diagnostic_msgs::msg::KeyValue item;
                    item.key   = key;
                    item.value = std::to_string(value);
                    status.values.push_back(std::move(item));
                };
                add("minimum_clearance", results_[current].minimum_clearance);
                add("minimum_clearance_seen", minimum_seen_[current]);
                add("solid_sample_count", results_[current].solid_sample_count);
                add("collision_frame_count", collision_counts_[current]);
                if(results_[current].first_solid_sample.has_value()) {
                    add("first_solid_x", results_[current].first_solid_sample->x);
                    add("first_solid_y", results_[current].first_solid_sample->y);
                    add("first_solid_z", results_[current].first_solid_sample->z);
                }
                array.status.push_back(std::move(status));
            }
            publisher_->publish(array);
        }

        std::shared_ptr<CaveWorld::ICaveField> field_;
        std::unique_ptr<CaveWorld::TruthCollisionAuditor> auditor_;
        std::vector<CaveWorld::TruthCollisionAuditResult> results_;
        std::vector<std::string> odom_topics_;
        std::vector<bool> received_;
        std::vector<float> minimum_seen_;
        std::vector<std::size_t> collision_counts_;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
    };

}// namespace DroneScanner

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneScanner::TruthCollisionAuditNode>());
    rclcpp::shutdown();
    return 0;
}

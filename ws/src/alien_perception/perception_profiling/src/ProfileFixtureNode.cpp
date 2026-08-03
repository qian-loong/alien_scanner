#include "ProfileRosConversions.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace PerceptionProfiling {

    class ProfileFixtureNode final : public rclcpp::Node
    {
    public:
        ProfileFixtureNode()
            : Node("perception_profile_fixture")
            , scenario_(ProfileScenario::parse_mode(
                      declare_parameter<std::string>("mode", "bounded")))
            , static_broadcaster_(*this)
        {
            observation_topic_ = declare_parameter<std::string>(
                    "observation_topic", "/profile/perception/observations");
            pose_topic_ = declare_parameter<std::string>(
                    "pose_topic", "/profile/perception/pose");
            health_topic_ = declare_parameter<std::string>(
                    "health_topic", "/profile/perception/health");
            const auto sequence_limit = declare_parameter<std::int64_t>("sequence_limit", 0);
            if(sequence_limit < 0) {
                throw std::invalid_argument("sequence_limit must be non-negative");
            }
            if(sequence_limit > 0) {
                sequence_limit_ = static_cast<std::uint64_t>(sequence_limit);
            }
            if(scenario_.canonical_sequence_limit().has_value()) {
                if(sequence_limit_.has_value()
                   && sequence_limit_.value()
                              > scenario_.canonical_sequence_limit().value()) {
                    throw std::invalid_argument("sequence_limit exceeds canonical scenario");
                }
                if(!sequence_limit_.has_value()) {
                    sequence_limit_ = scenario_.canonical_sequence_limit();
                }
            }

            const double startup_delay_s = declare_parameter<double>("startup_delay_s", 1.0);
            if(!std::isfinite(startup_delay_s) || startup_delay_s < 0.0) {
                throw std::invalid_argument("startup_delay_s must be finite and non-negative");
            }
            startup_ticks_remaining_ = static_cast<std::uint64_t>(
                    std::ceil(startup_delay_s / 0.05));

            observation_publisher_ =
                    create_publisher<perception_interfaces::msg::LidarObservation>(
                            observation_topic_, rclcpp::SensorDataQoS());
            pose_publisher_ = create_publisher<perception_interfaces::msg::PoseEstimate>(
                    pose_topic_, rclcpp::QoS(10).reliable().durability_volatile());
            health_publisher_ = create_publisher<perception_interfaces::msg::HealthState>(
                    health_topic_, rclcpp::QoS(10).reliable().durability_volatile());
            publish_static_transform();
            timer_ = create_wall_timer(
                    std::chrono::milliseconds(50), [this]() { on_tick(); });
            RCLCPP_INFO(
                    get_logger(), "profile fixture mode=%s fingerprint=%s",
                    ProfileScenario::mode_name(scenario_.mode()).c_str(),
                    scenario_.contract_fingerprint().hex_digest.c_str());
        }

    private:
        ProfileScenario scenario_;
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        std::string observation_topic_;
        std::string pose_topic_;
        std::string health_topic_;
        std::optional<std::uint64_t> sequence_limit_;
        std::uint64_t startup_ticks_remaining_ = 0U;
        std::uint64_t next_sequence_ = 1U;
        bool lead_phase_ = false;
        std::optional<ScenarioSample> current_sample_;
        rclcpp::Publisher<perception_interfaces::msg::LidarObservation>::SharedPtr
                observation_publisher_;
        rclcpp::Publisher<perception_interfaces::msg::PoseEstimate>::SharedPtr
                pose_publisher_;
        rclcpp::Publisher<perception_interfaces::msg::HealthState>::SharedPtr
                health_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void publish_static_transform()
        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = get_clock()->now();
            transform.header.frame_id = scenario_.config().body_frame;
            transform.child_frame_id = scenario_.config().sensor_frame;
            transform.transform.rotation.w = scenario_.config().body_from_sensor.w();
            transform.transform.rotation.x = scenario_.config().body_from_sensor.x();
            transform.transform.rotation.y = scenario_.config().body_from_sensor.y();
            transform.transform.rotation.z = scenario_.config().body_from_sensor.z();
            static_broadcaster_.sendTransform(transform);
        }

        void publish_health()
        {
            health_publisher_->publish(
                    Ros::make_health_message(scenario_, get_clock()->now()));
        }

        void on_tick()
        {
            publish_health();
            if(startup_ticks_remaining_ > 0U) {
                --startup_ticks_remaining_;
                return;
            }
            if(lead_phase_) {
                pose_publisher_->publish(Ros::to_message(current_sample_->lead_pose));
                lead_phase_ = false;
                return;
            }
            if(current_sample_.has_value()) {
                observation_publisher_->publish(Ros::to_message(current_sample_->observation));
                current_sample_.reset();
                ++next_sequence_;
            }
            if(!sequence_limit_.has_value() || next_sequence_ <= sequence_limit_.value()) {
                current_sample_ = scenario_.sample(next_sequence_);
                pose_publisher_->publish(Ros::to_message(current_sample_->observation_pose));
                lead_phase_ = true;
            }
        }
    };

}// namespace PerceptionProfiling

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<PerceptionProfiling::ProfileFixtureNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("perception_profile_fixture"), "%s", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}

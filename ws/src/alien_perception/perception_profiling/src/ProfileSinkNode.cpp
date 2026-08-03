#include "ProfileRosConversions.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace PerceptionProfiling {
    namespace {

        std::int64_t monotonic_now_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                    .count();
        }

    }// namespace

    class ProfileSinkNode final : public rclcpp::Node
    {
    public:
        ProfileSinkNode()
            : Node("perception_profile_sink")
            , scenario_(ProfileScenario::parse_mode(
                      declare_parameter<std::string>("mode", "bounded")))
            , sink_(required_output_directory(), scenario_)
        {
            const auto observation_topic = declare_parameter<std::string>(
                    "observation_topic", "/profile/perception/observations");
            const auto state_topic = declare_parameter<std::string>(
                    "state_topic", "/profile/local_map/state");
            const auto health_topic = declare_parameter<std::string>(
                    "health_topic", "/profile/perception/health");
            const auto octomap_topic = declare_parameter<std::string>(
                    "octomap_topic", "/profile/local_map/octomap");
            const auto diagnostics_topic = declare_parameter<std::string>(
                    "diagnostics_topic", "/diagnostics");

            observation_subscription_ =
                    create_subscription<perception_interfaces::msg::LidarObservation>(
                            observation_topic,
                            rclcpp::QoS(rclcpp::KeepLast(1000))
                                    .best_effort()
                                    .durability_volatile(),
                            [this](
                                    const perception_interfaces::msg::LidarObservation::SharedPtr
                                            message) {
                                try {
                                    sink_.record_observation(
                                            Ros::from_message(*message), monotonic_now_ns());
                                }
                                catch(const std::exception & error) {
                                    RCLCPP_ERROR(get_logger(), "observation sink: %s", error.what());
                                }
                            });
            state_subscription_ =
                    create_subscription<perception_interfaces::msg::LocalMapState>(
                            state_topic, rclcpp::QoS(100).reliable().durability_volatile(),
                            [this](
                                    const perception_interfaces::msg::LocalMapState::SharedPtr
                                            message) {
                                sink_.record_state(
                                        Ros::to_projection(*message, monotonic_now_ns()));
                            });
            health_subscription_ = create_subscription<perception_interfaces::msg::HealthState>(
                    health_topic, rclcpp::QoS(100).reliable().durability_volatile(),
                    [this](const perception_interfaces::msg::HealthState::SharedPtr message) {
                        sink_.record_health(
                                Ros::to_projection(*message, monotonic_now_ns()));
                    });
            diagnostics_subscription_ =
                    create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                            diagnostics_topic, rclcpp::QoS(100).reliable().durability_volatile(),
                            [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
                                const auto receipt = monotonic_now_ns();
                                const auto stamp = Ros::from_time(message->header.stamp).nanoseconds;
                                for(const auto & status : message->status) {
                                    sink_.record_diagnostic(
                                            {receipt, stamp, status.level, status.name,
                                             status.message});
                                }
                            });
            octomap_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
                    octomap_topic, rclcpp::QoS(10).reliable().transient_local(),
                    [this](const octomap_msgs::msg::Octomap::SharedPtr message) {
                        sink_.record_snapshot(
                                {monotonic_now_ns(),
                                 Ros::from_time(message->header.stamp).nanoseconds,
                                 ++snapshot_ordinal_,
                                 message->binary,
                                 message->resolution,
                                 message->data.size()});
                    });
        }

        void finish(bool normal_completion)
        {
            sink_.finalize(normal_completion);
        }

    private:
        ProfileScenario scenario_;
        ProfileDataSink sink_;
        std::uint64_t snapshot_ordinal_ = 0U;
        rclcpp::Subscription<perception_interfaces::msg::LidarObservation>::SharedPtr
                observation_subscription_;
        rclcpp::Subscription<perception_interfaces::msg::LocalMapState>::SharedPtr
                state_subscription_;
        rclcpp::Subscription<perception_interfaces::msg::HealthState>::SharedPtr
                health_subscription_;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                diagnostics_subscription_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;

        std::string required_output_directory()
        {
            const auto value = declare_parameter<std::string>("output_directory", "");
            if(value.empty()) {
                throw std::invalid_argument("output_directory must not be empty");
            }
            return value;
        }
    };

}// namespace PerceptionProfiling

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    std::shared_ptr<PerceptionProfiling::ProfileSinkNode> node;
    try {
        node = std::make_shared<PerceptionProfiling::ProfileSinkNode>();
        rclcpp::spin(node);
        node->finish(true);
    }
    catch(const std::exception & error) {
        if(node) {
            try {
                node->finish(false);
            }
            catch(...) {
            }
        }
        RCLCPP_FATAL(rclcpp::get_logger("perception_profile_sink"), "%s", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}

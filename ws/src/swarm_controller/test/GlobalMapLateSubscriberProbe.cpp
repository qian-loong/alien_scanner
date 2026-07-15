#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace SwarmController {

    namespace {

        diagnostic_msgs::msg::KeyValue value(
                const std::string & key, const std::string & content)
        {
            diagnostic_msgs::msg::KeyValue result;
            result.key   = key;
            result.value = content;
            return result;
        }

    }// namespace

    class GlobalMapLateSubscriberProbe : public rclcpp::Node
    {
    public:
        GlobalMapLateSubscriberProbe()
            : rclcpp::Node("global_map_late_subscriber_probe")
        {
            const auto transient_qos =
                    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            diagnostics_publisher_ =
                    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                            "/test/global_map_late_probe_diagnostics", transient_qos);
            trigger_subscription_ = create_subscription<std_msgs::msg::Empty>(
                    "/test/start_global_map_late_probe", rclcpp::QoS(1).reliable(),
                    [this](const std_msgs::msg::Empty::SharedPtr) {
                        startProbe();
                    });
        }

    private:
        void startProbe()
        {
            if(map_subscription_) {
                return;
            }
            const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            map_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
                    "/global_map", qos,
                    [this](const octomap_msgs::msg::Octomap::SharedPtr message) {
                        inspect(*message);
                    });
        }

        void inspect(const octomap_msgs::msg::Octomap & message)
        {
            if(completed_) {
                return;
            }

            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name        = "global_map_late_subscriber_probe";
            status.hardware_id = "test";
            status.values.push_back(value("message_id", message.id));
            status.values.push_back(value("binary", message.binary ? "true" : "false"));
            status.values.push_back(value("resolution", std::to_string(message.resolution)));

            try {
                std::unique_ptr<octomap::AbstractOcTree> abstract(
                        octomap_msgs::msgToMap(message));
                const auto * tree = dynamic_cast<const octomap::OcTree *>(abstract.get());
                if(tree == nullptr) {
                    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                    status.message = "late snapshot is not a deserializable OcTree";
                } else {
                    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    status.message = "late snapshot deserialized";
                    status.values.push_back(value(
                            "leaf_count", std::to_string(tree->getNumLeafNodes())));
                }
            } catch(const std::exception & exception) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = std::string("deserialization failed: ") + exception.what();
            }

            diagnostic_msgs::msg::DiagnosticArray result;
            result.header.stamp = get_clock()->now();
            result.status.push_back(std::move(status));
            diagnostics_publisher_->publish(result);
            completed_ = true;
        }

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                diagnostics_publisher_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_subscription_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
        bool completed_ {false};
    };

}// namespace SwarmController

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmController::GlobalMapLateSubscriberProbe>());
    rclcpp::shutdown();
    return 0;
}

#include "LogicalLinkAdapter.hpp"

#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace SwarmDataPlane::Test {

    namespace {

        std::uint64_t steady_now_ns()
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
        }

    }// namespace

    class MapUpdateRouteLinkFixture final : public rclcpp::Node
    {
    public:
        MapUpdateRouteLinkFixture()
                : Node("map_update_route_link_fixture"), link_(LogicalLinkConfig {})
        {
            const auto input_topic = declare_parameter<std::string>(
                    "input_topic", "/c4/routed_before_fault");
            const auto output_topic = declare_parameter<std::string>(
                    "output_topic", "/c4/routed_after_fault");
            publisher_ = create_publisher<swarm_data_interfaces::msg::RoutedMapUpdate>(
                    output_topic, Ros::map_update_qos());
            subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoutedMapUpdate>(
                    input_topic,
                    Ros::map_update_qos(),
                    [this](
                            swarm_data_interfaces::msg::RoutedMapUpdate::ConstSharedPtr message) {
                        on_message(*message);
                    });
        }

    private:
        void on_message(const swarm_data_interfaces::msg::RoutedMapUpdate & message)
        {
            auto decoded = Ros::decode_routed_map_update(message);
            if(!decoded.success || !decoded.message.has_value()) {
                RCLCPP_ERROR(get_logger(), "%s", decoded.diagnostic.c_str());
                return;
            }
            const auto now_ns = steady_now_ns();
            if(decoded.message->sequence == 2U && !dropped_first_delta_) {
                dropped_first_delta_ = true;
                link_.set_partitioned(true, now_ns);
                link_.submit(*decoded.message, now_ns);
                link_.set_partitioned(false, now_ns);
                return;
            }
            if(!link_.submit(*decoded.message, now_ns).accepted) {
                return;
            }
            for(auto & forwarded : link_.poll(now_ns)) {
                swarm_data_interfaces::msg::RoutedMapUpdate output;
                std::string diagnostic;
                if(!Ros::encode_routed_map_update(forwarded, output, diagnostic)) {
                    RCLCPP_ERROR(get_logger(), "%s", diagnostic.c_str());
                    continue;
                }
                output.map_update.header.stamp = message.map_update.header.stamp;
                publisher_->publish(output);
            }
        }

        bool dropped_first_delta_ = false;
        LogicalLinkAdapter link_;
        rclcpp::Publisher<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr publisher_;
        rclcpp::Subscription<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                subscription_;
    };

}// namespace SwarmDataPlane::Test

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<
                 SwarmDataPlane::Test::MapUpdateRouteLinkFixture>());
    rclcpp::shutdown();
    return 0;
}

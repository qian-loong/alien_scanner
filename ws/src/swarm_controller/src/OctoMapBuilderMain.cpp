#include "swarm_controller/OctoMapBuilderNode.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmController::OctoMapBuilderNode>());
    rclcpp::shutdown();
    return 0;
}

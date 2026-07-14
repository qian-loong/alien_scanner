#include "swarm_controller/SingleDroneExplorerNode.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmController::SingleDroneExplorerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions {}, 2U);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

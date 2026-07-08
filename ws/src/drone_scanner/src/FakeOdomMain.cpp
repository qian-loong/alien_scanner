#include "drone_scanner/FakeOdomNode.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneScanner::FakeOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

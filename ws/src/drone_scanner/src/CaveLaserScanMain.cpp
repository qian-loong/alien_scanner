#include "drone_scanner/CaveLaserScanNode.hpp"

#include <memory>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<DroneScanner::CaveLaserScanNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("cave_laser_scan"), "%s", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}

#ifndef DRONE_SCANNER_CAVEFIELDFROMPARAMETERS_HPP
#define DRONE_SCANNER_CAVEFIELDFROMPARAMETERS_HPP

#include "cave_world/ICaveField.hpp"

#include <memory>

namespace rclcpp {
class Node;
}

namespace DroneScanner {

    void declareCaveFieldParameters(rclcpp::Node & node);

    std::shared_ptr<CaveWorld::ICaveField> createCaveFieldFromParameters(const rclcpp::Node & node);

}// namespace DroneScanner

#endif// DRONE_SCANNER_CAVEFIELDFROMPARAMETERS_HPP

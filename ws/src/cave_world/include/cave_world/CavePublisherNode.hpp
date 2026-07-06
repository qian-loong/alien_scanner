#ifndef CAVE_WORLD_CAVEPUBLISHERNODE_HPP
#define CAVE_WORLD_CAVEPUBLISHERNODE_HPP

#include "cave_world/ICaveField.hpp"
#include "cave_world/ProceduralCaveField.hpp"
#include "cave_world/TreeCaveField.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace CaveWorld {

class CavePublisherNode : public rclcpp::Node
{
public:
    CavePublisherNode();

private:
    void declareParameters();
    ProceduralCaveFieldConfig loadYConfigFromParameters() const;
    TreeCaveFieldConfig       loadTreeConfigFromParameters() const;
    void initField();
    void initPublisher();
    void initTimer();
    void buildCachedPointCloud();
    void publishCachedPointCloud();
    void onTimer();

    std::unique_ptr<ICaveField>                                  field_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                                 timer_;
    sensor_msgs::msg::PointCloud2                                 cached_cloud_;
    std::string                                                  frame_id_;
    std::string                                                  topic_;
    double                                                       publish_rate_hz_ {};
};

}// namespace CaveWorld

#endif// CAVE_WORLD_CAVEPUBLISHERNODE_HPP

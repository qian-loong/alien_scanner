#ifndef SWARM_CONTROLLER_OCTOMAPBUILDERNODE_HPP
#define SWARM_CONTROLLER_OCTOMAPBUILDERNODE_HPP

#include "swarm_controller/OctoMapBuilder.hpp"

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace SwarmController {

    class OctoMapBuilderNode : public rclcpp::Node
    {
    public:
        OctoMapBuilderNode();

    private:
        void declareParameters();
        void initTf();
        void initBuilder();
        void initSubscriber();
        void initPublisher();
        void initTimer();

        void onScanReturns(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void onPublishTimer();
        void publishMap(const rclcpp::Time & stamp);

        static bool hasRequiredFields(const sensor_msgs::msg::PointCloud2 & cloud);
        std::vector<RayReturn> extractReturnsInMap(
                const sensor_msgs::msg::PointCloud2 & cloud,
                const geometry_msgs::msg::Transform & transform) const;
        Point3f transformPoint(
                const Point3f & point,
                const geometry_msgs::msg::Transform & transform) const;

        std::unique_ptr<OctoMapBuilder>                                      builder_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr       subscription_;
        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr             publisher_;
        std::shared_ptr<tf2_ros::Buffer>                                     tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener>                          tf_listener_;
        rclcpp::TimerBase::SharedPtr                                         publish_timer_;
        std::string                                                          map_frame_;
        std::string                                                          input_topic_;
        std::string                                                          output_topic_;
        float                                                                max_range_ {30.0F};
        double                                                               publish_rate_hz_ {2.0};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_OCTOMAPBUILDERNODE_HPP

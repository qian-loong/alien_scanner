#ifndef DRONE_SCANNER_SCANACCUMULATORNODE_HPP
#define DRONE_SCANNER_SCANACCUMULATORNODE_HPP

#include "drone_scanner/PointCloudAccumulator.hpp"

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace DroneScanner {

    class ScanAccumulatorNode : public rclcpp::Node
    {
    public:
        ScanAccumulatorNode();

    private:
        void declareParameters();
        void initSubscriber();
        void initPublisher();
        void initTf();
        void initTimer();
        void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void onPublishTimer();
        void publishCloudMap(const rclcpp::Time & stamp);

        static std::vector<Point3f> extractPoints(const sensor_msgs::msg::PointCloud2 & cloud);
        static sensor_msgs::msg::PointCloud2 makeCloud(
                const rclcpp::Time & stamp, const std::string & frame_id,
                const std::vector<Point3f> & points);
        static RigidTransform transformFromMessage(const geometry_msgs::msg::Transform & transform);

        PointCloudAccumulator                                      accumulator_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  publisher_;
        std::shared_ptr<tf2_ros::Buffer>                            tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener>                 tf_listener_;
        rclcpp::TimerBase::SharedPtr                                publish_timer_;
        std::string                                                 map_frame_;
        std::string                                                 points_topic_;
        std::string                                                 cloud_map_topic_;
        double                                                      publish_rate_hz_ {5.0};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_SCANACCUMULATORNODE_HPP

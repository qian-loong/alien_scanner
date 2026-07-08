#include "drone_scanner/ScanAccumulatorNode.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace DroneScanner {

    ScanAccumulatorNode::ScanAccumulatorNode()
        : rclcpp::Node("scan_accumulator")
    {
        declareParameters();
        accumulator_ = PointCloudAccumulator(static_cast<std::size_t>(get_parameter("max_points").as_int()));
        map_frame_        = get_parameter("map_frame").as_string();
        points_topic_     = get_parameter("points_topic").as_string();
        cloud_map_topic_  = get_parameter("cloud_map_topic").as_string();
        publish_rate_hz_  = get_parameter("publish_rate").as_double();
        if(publish_rate_hz_ <= 0.0) {
            publish_rate_hz_ = 5.0;
        }

        initTf();
        initSubscriber();
        initPublisher();
        initTimer();

        RCLCPP_INFO(
                get_logger(), "scan_accumulator: %s -> %s, map frame '%s', publish %.1f Hz",
                points_topic_.c_str(), cloud_map_topic_.c_str(), map_frame_.c_str(), publish_rate_hz_);
    }

    void ScanAccumulatorNode::declareParameters()
    {
        declare_parameter("map_frame", "map");
        declare_parameter("points_topic", "points");
        declare_parameter("cloud_map_topic", "cloud_map");
        declare_parameter("publish_rate", 5.0);
        declare_parameter("max_points", 500000);
    }

    void ScanAccumulatorNode::initTf()
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void ScanAccumulatorNode::initSubscriber()
    {
        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                points_topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    onCloud(msg);
                });
    }

    void ScanAccumulatorNode::initPublisher()
    {
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_map_topic_, rclcpp::QoS(10));
    }

    void ScanAccumulatorNode::initTimer()
    {
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_));
        publish_timer_ = create_wall_timer(period, [this]() {
            onPublishTimer();
        });
    }

    std::vector<Point3f> ScanAccumulatorNode::extractPoints(const sensor_msgs::msg::PointCloud2 & cloud)
    {
        std::vector<Point3f> points;
        const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
        points.reserve(count);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
        for(std::size_t i = 0; i < count; ++i, ++iter_x, ++iter_y, ++iter_z) {
            points.push_back(Point3f {*iter_x, *iter_y, *iter_z});
        }
        return points;
    }

    sensor_msgs::msg::PointCloud2 ScanAccumulatorNode::makeCloud(
            const rclcpp::Time & stamp, const std::string & frame_id, const std::vector<Point3f> & points)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        for(const Point3f & point : points) {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        cloud.header.stamp    = stamp;
        cloud.header.frame_id = frame_id;
        cloud.is_dense        = true;
        return cloud;
    }

    RigidTransform ScanAccumulatorNode::transformFromMessage(const geometry_msgs::msg::Transform & transform)
    {
        RigidTransform out;
        out.tx = static_cast<float>(transform.translation.x);
        out.ty = static_cast<float>(transform.translation.y);
        out.tz = static_cast<float>(transform.translation.z);
        out.qx = static_cast<float>(transform.rotation.x);
        out.qy = static_cast<float>(transform.rotation.y);
        out.qz = static_cast<float>(transform.rotation.z);
        out.qw = static_cast<float>(transform.rotation.w);
        return out;
    }

    void ScanAccumulatorNode::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if(msg->width * msg->height == 0) {
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                    map_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch(const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000, "TF %s -> %s: %s", map_frame_.c_str(),
                    msg->header.frame_id.c_str(), ex.what());
            return;
        }

        const auto points = extractPoints(*msg);
        accumulator_.appendTransformed(points, transformFromMessage(transform.transform));
    }

    void ScanAccumulatorNode::onPublishTimer()
    {
        if(accumulator_.size() == 0) {
            return;
        }
        publishCloudMap(get_clock()->now());
    }

    void ScanAccumulatorNode::publishCloudMap(const rclcpp::Time & stamp)
    {
        publisher_->publish(makeCloud(stamp, map_frame_, accumulator_.points()));
    }

}// namespace DroneScanner

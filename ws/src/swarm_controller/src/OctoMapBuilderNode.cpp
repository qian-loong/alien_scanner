#include "swarm_controller/OctoMapBuilderNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace SwarmController {

    namespace {

        tf2::TimePoint timePointFromStamp(const builtin_interfaces::msg::Time & stamp)
        {
            return tf2::TimePoint(
                    std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec));
        }

    }// namespace

    OctoMapBuilderNode::OctoMapBuilderNode()
        : rclcpp::Node("octomap_builder")
    {
        declareParameters();
        map_frame_       = get_parameter("map_frame").as_string();
        input_topic_     = get_parameter("input_topic").as_string();
        output_topic_    = get_parameter("output_topic").as_string();
        max_range_       = static_cast<float>(get_parameter("max_range").as_double());
        publish_rate_hz_ = get_parameter("publish_rate").as_double();
        if(max_range_ <= 0.0F || !std::isfinite(max_range_)) {
            max_range_ = 30.0F;
        }
        if(publish_rate_hz_ <= 0.0) {
            publish_rate_hz_ = 2.0;
        }

        initTf();
        initBuilder();
        initSubscriber();
        initPublisher();
        initTimer();

        RCLCPP_INFO(
                get_logger(), "octomap_builder: %s -> %s, frame '%s', publish %.1f Hz",
                input_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str(), publish_rate_hz_);
    }

    void OctoMapBuilderNode::declareParameters()
    {
        declare_parameter("map_frame", "map");
        declare_parameter("input_topic", "scan_returns");
        declare_parameter("output_topic", "octomap");
        declare_parameter("resolution", 0.1);
        declare_parameter("publish_rate", 2.0);
        declare_parameter("max_range", 30.0);
    }

    void OctoMapBuilderNode::initTf()
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void OctoMapBuilderNode::initBuilder()
    {
        builder_ = std::make_unique<OctoMapBuilder>(static_cast<float>(get_parameter("resolution").as_double()));
    }

    void OctoMapBuilderNode::initSubscriber()
    {
        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic_, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    onScanReturns(msg);
                });
    }

    void OctoMapBuilderNode::initPublisher()
    {
        publisher_ = create_publisher<octomap_msgs::msg::Octomap>(output_topic_, rclcpp::QoS(1).transient_local());
    }

    void OctoMapBuilderNode::initTimer()
    {
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_));
        publish_timer_ = create_wall_timer(period, [this]() {
            onPublishTimer();
        });
    }

    bool OctoMapBuilderNode::hasRequiredFields(const sensor_msgs::msg::PointCloud2 & cloud)
    {
        struct FieldSpec {
            const char * name;
            std::uint8_t datatype;
        };

        const FieldSpec required[] {
                {"x", sensor_msgs::msg::PointField::FLOAT32},
                {"y", sensor_msgs::msg::PointField::FLOAT32},
                {"z", sensor_msgs::msg::PointField::FLOAT32},
                {"range", sensor_msgs::msg::PointField::FLOAT32},
                {"hit", sensor_msgs::msg::PointField::UINT8},
                {"intensity", sensor_msgs::msg::PointField::FLOAT32},
        };

        for(const FieldSpec & spec : required) {
            const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(), [&spec](const auto & field) {
                return field.name == spec.name;
            });
            if(it == cloud.fields.end() || it->datatype != spec.datatype || it->count != 1U) {
                return false;
            }
        }
        return true;
    }

    Point3f OctoMapBuilderNode::transformPoint(
            const Point3f & point, const geometry_msgs::msg::Transform & transform) const
    {
        const double qx = transform.rotation.x;
        const double qy = transform.rotation.y;
        const double qz = transform.rotation.z;
        const double qw = transform.rotation.w;

        const double xx = qx * qx;
        const double yy = qy * qy;
        const double zz = qz * qz;
        const double xy = qx * qy;
        const double xz = qx * qz;
        const double yz = qy * qz;
        const double wx = qw * qx;
        const double wy = qw * qy;
        const double wz = qw * qz;

        Point3f out;
        out.x = static_cast<float>(
                (1.0 - 2.0 * (yy + zz)) * point.x
                + (2.0 * (xy - wz)) * point.y
                + (2.0 * (xz + wy)) * point.z
                + transform.translation.x);
        out.y = static_cast<float>(
                (2.0 * (xy + wz)) * point.x
                + (1.0 - 2.0 * (xx + zz)) * point.y
                + (2.0 * (yz - wx)) * point.z
                + transform.translation.y);
        out.z = static_cast<float>(
                (2.0 * (xz - wy)) * point.x
                + (2.0 * (yz + wx)) * point.y
                + (1.0 - 2.0 * (xx + yy)) * point.z
                + transform.translation.z);
        return out;
    }

    std::vector<RayReturn> OctoMapBuilderNode::extractReturnsInMap(
            const sensor_msgs::msg::PointCloud2 & cloud,
            const geometry_msgs::msg::Transform & transform) const
    {
        const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
        std::vector<RayReturn> returns;
        returns.reserve(count);

        sensor_msgs::PointCloud2ConstIterator<float>        iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float>        iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float>        iter_z(cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float>        iter_range(cloud, "range");
        sensor_msgs::PointCloud2ConstIterator<std::uint8_t> iter_hit(cloud, "hit");
        for(std::size_t i = 0; i < count; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_range, ++iter_hit) {
            const float range = *iter_range;
            if(!std::isfinite(range) || range <= 0.0F) {
                continue;
            }

            Point3f endpoint_lidar {*iter_x, *iter_y, *iter_z};
            if(!std::isfinite(endpoint_lidar.x) || !std::isfinite(endpoint_lidar.y)
               || !std::isfinite(endpoint_lidar.z)) {
                continue;
            }

            float clipped_range = range;
            if(range > max_range_) {
                const float scale = max_range_ / range;
                endpoint_lidar.x *= scale;
                endpoint_lidar.y *= scale;
                endpoint_lidar.z *= scale;
                clipped_range = max_range_;
            }

            returns.push_back(RayReturn {
                    transformPoint(endpoint_lidar, transform),
                    clipped_range,
                    *iter_hit != 0U && range <= max_range_,
            });
        }
        return returns;
    }

    void OctoMapBuilderNode::onScanReturns(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const rclcpp::Time scan_stamp(msg->header.stamp);
        if(scan_stamp.nanoseconds() == 0) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "ignored scan_returns with zero acquisition stamp");
            return;
        }
        if(latest_observation_stamp_.nanoseconds() != 0
           && scan_stamp <= latest_observation_stamp_)
        {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "ignored non-increasing scan_returns stamp");
            return;
        }
        if(msg->width * msg->height == 0) {
            return;
        }
        if(!hasRequiredFields(*msg)) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "octomap_builder: PointCloud2 missing one of fields x/y/z/range/hit");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            const bool time_is_zero = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0;
            const tf2::TimePoint lookup_time = time_is_zero
                                               ? tf2::TimePointZero
                                               : timePointFromStamp(msg->header.stamp);
            transform = tf_buffer_->lookupTransform(
                    map_frame_, msg->header.frame_id, lookup_time, tf2::durationFromSec(0.05));
        } catch(const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000, "TF %s -> %s: %s", map_frame_.c_str(),
                    msg->header.frame_id.c_str(), ex.what());
            return;
        }

        Point3f origin_map;
        origin_map.x = static_cast<float>(transform.transform.translation.x);
        origin_map.y = static_cast<float>(transform.transform.translation.y);
        origin_map.z = static_cast<float>(transform.transform.translation.z);
        const std::vector<RayReturn> returns = extractReturnsInMap(*msg, transform.transform);
        if(returns.empty()) {
            return;
        }
        builder_->insertScan(origin_map, returns);
        latest_observation_stamp_ = scan_stamp;
        ++observation_epoch_;
        map_dirty_ = true;
    }

    void OctoMapBuilderNode::onPublishTimer()
    {
        if(!map_dirty_ || builder_->knownCount() == 0U) {
            return;
        }
        publishMap(latest_observation_stamp_);
        map_dirty_ = false;
    }

    void OctoMapBuilderNode::publishMap(const rclcpp::Time & stamp)
    {
        octomap_msgs::msg::Octomap msg;
        if(!octomap_msgs::fullMapToMsg(builder_->tree(), msg)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "octomap_builder: failed to serialize map");
            return;
        }
        msg.header.stamp    = stamp;
        msg.header.frame_id = map_frame_;
        publisher_->publish(msg);
    }

}// namespace SwarmController

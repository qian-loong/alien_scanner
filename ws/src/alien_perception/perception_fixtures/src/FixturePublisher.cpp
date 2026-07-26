#include "perception_fixtures/FixtureScene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

    using Perception::Fixtures::CloudPoint;
    using Perception::Fixtures::FixtureScene;
    using Perception::Fixtures::FixtureSceneConfig;

    std::vector<double> default_elevation_angles_parameter()
    {
        const auto & defaults = FixtureSceneConfig::default_elevation_angles_rad();
        return std::vector<double>(defaults.begin(), defaults.end());
    }

    FixtureSceneConfig make_scene_config(rclcpp::Node & node)
    {
        FixtureSceneConfig config;
        const auto scan_point_count = node.declare_parameter<int64_t>(
                "scan_point_count", static_cast<int64_t>(config.scan_point_count));
        if(scan_point_count < 2) {
            throw std::invalid_argument("scan_point_count must be at least two");
        }
        config.scan_point_count = static_cast<std::size_t>(scan_point_count);
        config.scan_angle_min_rad = node.declare_parameter<double>(
                "scan_angle_min_rad", config.scan_angle_min_rad);
        config.scan_angle_max_rad = node.declare_parameter<double>(
                "scan_angle_max_rad", config.scan_angle_max_rad);
        config.scan_range_min_m = node.declare_parameter<double>(
                "scan_range_min_m", config.scan_range_min_m);
        config.scan_range_max_m = node.declare_parameter<double>(
                "scan_range_max_m", config.scan_range_max_m);
        config.cloud_azimuth_sample_count = static_cast<std::size_t>(std::max<int64_t>(
                1, node.declare_parameter<int64_t>("cloud_azimuth_sample_count", 360)));
        config.cloud_range_m = static_cast<float>(node.declare_parameter<double>("cloud_range_m", 5.0));

        const auto elevation_parameters = node.declare_parameter<std::vector<double>>(
                "elevation_angles_rad", default_elevation_angles_parameter());
        config.elevation_angles_rad.clear();
        config.elevation_angles_rad.reserve(elevation_parameters.size());
        for(const double elevation : elevation_parameters) {
            config.elevation_angles_rad.push_back(static_cast<float>(elevation));
        }
        config.seed = static_cast<std::uint32_t>(node.declare_parameter<int64_t>("seed", 17));
        config.inject_debug_returns = node.declare_parameter<bool>("inject_debug_returns", false);
        return config;
    }

    class FixturePublisher final : public rclcpp::Node
    {
    public:
        FixturePublisher()
            : Node("perception_fixture_publisher")
            , mode_(declare_parameter<std::string>("mode", "mixed"))
            , scan_topic_front_(declare_parameter<std::string>("scan_topic_front", "fixture/scan/front"))
            , scan_topic_rear_(declare_parameter<std::string>("scan_topic_rear", "fixture/scan/rear"))
            , cloud_topic_(declare_parameter<std::string>("cloud_topic", "fixture/points"))
            , scan_frame_(declare_parameter<std::string>("scan_frame", "fixture_scan_link"))
            , cloud_frame_(declare_parameter<std::string>("cloud_frame", "fixture_lidar_link"))
            , publish_period_(declare_parameter<double>("publish_period_s", 0.1))
            , scene_(make_scene_config(*this))
        {
            if(mode_ != "2d" && mode_ != "3d" && mode_ != "mixed") {
                throw std::invalid_argument("Fixture mode must be 2d, 3d, or mixed");
            }

            if(mode_ == "2d" || mode_ == "mixed") {
                scan_front_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
                        scan_topic_front_, rclcpp::SensorDataQoS());
                if(mode_ == "mixed") {
                    scan_rear_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
                            scan_topic_rear_, rclcpp::SensorDataQoS());
                }
            }
            if(mode_ == "3d" || mode_ == "mixed") {
                cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                        cloud_topic_, rclcpp::SensorDataQoS());
            }

            timer_ = create_wall_timer(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::duration<double>(std::max(0.01, publish_period_))),
                    [this]() { publish_fixture(); });
        }

    private:
        std::string  mode_;
        std::string  scan_topic_front_;
        std::string  scan_topic_rear_;
        std::string  cloud_topic_;
        std::string  scan_frame_;
        std::string  cloud_frame_;
        double       publish_period_;
        FixtureScene scene_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   scan_front_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   scan_rear_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
        rclcpp::TimerBase::SharedPtr                                timer_;

        sensor_msgs::msg::LaserScan make_scan(const std::string & frame_id) const
        {
            const auto &                config = scene_.config();
            const auto                  ranges = scene_.scan_ranges();
            sensor_msgs::msg::LaserScan message;
            message.header.stamp    = get_clock()->now();
            message.header.frame_id = frame_id;
            message.angle_min       = static_cast<float>(config.scan_angle_min_rad);
            message.angle_max       = static_cast<float>(config.scan_angle_max_rad);
            message.angle_increment = static_cast<float>(config.scan_angle_increment_rad());
            message.range_min       = static_cast<float>(config.scan_range_min_m);
            message.range_max       = static_cast<float>(config.scan_range_max_m);
            message.ranges          = ranges;
            message.intensities.resize(ranges.size(), 1.0F);
            return message;
        }

        sensor_msgs::msg::PointCloud2 make_cloud() const
        {
            const auto                    points = scene_.cloud_points();
            sensor_msgs::msg::PointCloud2 message;
            message.header.stamp    = get_clock()->now();
            message.header.frame_id = cloud_frame_;
            sensor_msgs::PointCloud2Modifier modifier(message);
            modifier.setPointCloud2Fields(
                    4,
                    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier.resize(points.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_intensity(message, "intensity");
            for(const CloudPoint & point : points) {
                *iter_x         = point.x;
                *iter_y         = point.y;
                *iter_z         = point.z;
                *iter_intensity = point.intensity;
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_intensity;
            }
            return message;
        }

        void publish_fixture()
        {
            if(scan_front_publisher_) {
                scan_front_publisher_->publish(make_scan(scan_frame_));
            }
            if(scan_rear_publisher_) {
                scan_rear_publisher_->publish(make_scan(scan_frame_));
            }
            if(cloud_publisher_) {
                cloud_publisher_->publish(make_cloud());
            }
        }
    };

}// namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixturePublisher>());
    rclcpp::shutdown();
    return 0;
}

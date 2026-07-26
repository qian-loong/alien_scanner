#include "geometry_msgs/msg/point.hpp"
#include "perception_core/observation/lidar_observation.hpp"
#include "perception_fixtures/RayEvidenceDebugGeometryBuilder.hpp"
#include "perception_interfaces/msg/lidar_observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

    using Perception::Cloud3D;
    using Perception::Fixtures::DebugPoint3;
    using Perception::Fixtures::RayEvidenceDebugGeometry;
    using Perception::Fixtures::RayEvidenceDebugGeometryBuilder;
    using Perception::LidarObservation;
    using Perception::RayEvidenceCapability;
    using Perception::Scan2D;

    RayEvidenceCapability parse_ray_evidence(std::uint8_t value)
    {
        switch(value) {
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_HIT_ONLY:
                return RayEvidenceCapability::HitOnly;
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_HIT_RAY:
                return RayEvidenceCapability::HitRay;
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_FULL_RAY:
                return RayEvidenceCapability::FullRay;
        }
        throw std::invalid_argument("Unknown LidarObservation ray_evidence value");
    }

    LidarObservation to_core_observation(
            const perception_interfaces::msg::LidarObservation & message)
    {
        if(message.header.frame_id.empty()) {
            throw std::invalid_argument("LidarObservation frame_id must not be empty");
        }
        if(message.sensor_id.empty()) {
            throw std::invalid_argument("LidarObservation sensor_id must not be empty");
        }
        if(message.session_boot_time_ns == 0) {
            throw std::invalid_argument(
                    "LidarObservation session_boot_time_ns must not be zero");
        }
        if(message.clock_domain.empty()) {
            throw std::invalid_argument("LidarObservation clock_domain must not be empty");
        }

        LidarObservation observation;
        observation.sensor_id.value          = message.sensor_id;
        observation.session_id.boot_time_ns  = message.session_boot_time_ns;
        observation.session_id.random_suffix = message.session_random_suffix;
        observation.frame_id                 = message.header.frame_id;
        observation.clock_domain             = message.clock_domain;
        observation.origin_stamp.nanoseconds = rclcpp::Time(message.header.stamp).nanoseconds();
        observation.ray_evidence              = parse_ray_evidence(message.ray_evidence);

        switch(message.data_type) {
            case perception_interfaces::msg::LidarObservation::DATA_TYPE_SCAN_2D: {
                if(!message.points.empty() || !message.point_intensities.empty()) {
                    throw std::invalid_argument(
                            "Scan2D observation contains Cloud3D payload");
                }
                Scan2D scan;
                scan.angle_min_rad       = message.angle_min;
                scan.angle_max_rad       = message.angle_max;
                scan.angle_increment_rad = message.angle_increment;
                scan.range_min_m         = message.range_min;
                scan.range_max_m         = message.range_max;
                scan.ranges              = message.ranges;
                scan.intensities         = message.intensities;
                observation.data         = std::move(scan);
                return observation;
            }
            case perception_interfaces::msg::LidarObservation::DATA_TYPE_CLOUD_3D: {
                if(!message.ranges.empty() || !message.intensities.empty()
                   || message.angle_min != 0.0 || message.angle_max != 0.0
                   || message.angle_increment != 0.0 || message.range_min != 0.0
                   || message.range_max != 0.0) {
                    throw std::invalid_argument(
                            "Cloud3D observation contains Scan2D payload");
                }
                if(observation.ray_evidence != RayEvidenceCapability::HitOnly) {
                    throw std::invalid_argument(
                            "Cloud3D debug observation must declare hit_only ray evidence");
                }
                if(message.points.empty()) {
                    throw std::invalid_argument("Cloud3D observation points must not be empty");
                }
                if(!message.point_intensities.empty()
                   && message.point_intensities.size() != message.points.size()) {
                    throw std::invalid_argument(
                            "Cloud3D point_intensities must be empty or match points length");
                }

                Cloud3D cloud;
                cloud.points.reserve(message.points.size());
                for(std::size_t index = 0; index < message.points.size(); ++index) {
                    const auto & point = message.points[index];
                    const float intensity = message.point_intensities.empty()
                            ? 0.0F
                            : message.point_intensities[index];
                    if(!std::isfinite(point.x) || !std::isfinite(point.y)
                       || !std::isfinite(point.z) || !std::isfinite(intensity)) {
                        throw std::invalid_argument(
                                "Cloud3D observation contains a non-finite point");
                    }
                    cloud.points.emplace_back(
                            static_cast<float>(point.x),
                            static_cast<float>(point.y),
                            static_cast<float>(point.z),
                            intensity);
                }
                observation.data = std::move(cloud);
                return observation;
            }
        }
        throw std::invalid_argument("Unknown LidarObservation data_type value");
    }

    std::int32_t marker_id_for_sensor(const std::string & sensor_id)
    {
        std::uint32_t hash = 2166136261U;
        for(const unsigned char character : sensor_id) {
            hash ^= character;
            hash *= 16777619U;
        }
        return static_cast<std::int32_t>(hash & 0x7fffffffU);
    }

    geometry_msgs::msg::Point to_ros_point(const DebugPoint3 & point)
    {
        geometry_msgs::msg::Point result;
        result.x = point.x;
        result.y = point.y;
        result.z = point.z;
        return result;
    }

    visualization_msgs::msg::Marker make_marker(
            const std_msgs::msg::Header & header,
            const std::string &           marker_namespace,
            std::int32_t                  marker_id,
            std::int32_t                  marker_type,
            float                         red,
            float                         green,
            float                         blue,
            double                        scale,
            double                        lifetime_s)
    {
        visualization_msgs::msg::Marker marker;
        marker.header             = header;
        marker.ns                 = marker_namespace;
        marker.id                 = marker_id;
        marker.type               = marker_type;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = scale;
        marker.scale.y            = scale;
        marker.scale.z            = scale;
        marker.color.r            = red;
        marker.color.g            = green;
        marker.color.b            = blue;
        marker.color.a            = 1.0F;
        marker.lifetime           = rclcpp::Duration::from_seconds(lifetime_s);
        return marker;
    }

    void append_points(
            visualization_msgs::msg::Marker & marker,
            const std::vector<DebugPoint3> &  points)
    {
        marker.points.reserve(points.size());
        for(const auto & point : points) {
            marker.points.push_back(to_ros_point(point));
        }
    }

}// namespace

class RayEvidenceDebugNode final : public rclcpp::Node
{
public:
    RayEvidenceDebugNode()
        : Node("ray_evidence_debug_node")
        , observation_topic_(declare_parameter<std::string>(
                  "observation_topic", "perception/observations"))
        , marker_topic_(declare_parameter<std::string>(
                  "marker_topic", "perception/ray_evidence_debug"))
        , beam_stride_parameter_(declare_parameter<std::int64_t>("beam_stride", 4))
        , point_size_m_(declare_parameter<double>("point_size_m", 0.08))
        , line_width_m_(declare_parameter<double>("line_width_m", 0.025))
        , marker_lifetime_s_(declare_parameter<double>("marker_lifetime_s", 0.5))
        , builder_(declare_parameter<double>("invalid_indicator_length_m", 0.25))
    {
        if(beam_stride_parameter_ < 1) {
            throw std::invalid_argument("beam_stride must be at least one");
        }
        if(!std::isfinite(point_size_m_) || point_size_m_ <= 0.0
           || !std::isfinite(line_width_m_) || line_width_m_ <= 0.0
           || !std::isfinite(marker_lifetime_s_) || marker_lifetime_s_ <= 0.0) {
            throw std::invalid_argument(
                    "Marker sizes and lifetime must be finite and greater than zero");
        }
        beam_stride_ = static_cast<std::size_t>(beam_stride_parameter_);

        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                marker_topic_, rclcpp::QoS(10));
        observation_subscription_ = create_subscription<
                perception_interfaces::msg::LidarObservation>(
                observation_topic_, rclcpp::SensorDataQoS(),
                [this](const perception_interfaces::msg::LidarObservation::SharedPtr message) {
                    on_observation(*message);
                });
    }

private:
    std::string observation_topic_;
    std::string marker_topic_;
    std::int64_t beam_stride_parameter_;
    std::size_t  beam_stride_ {1};
    double       point_size_m_;
    double       line_width_m_;
    double       marker_lifetime_s_;
    RayEvidenceDebugGeometryBuilder builder_;

    rclcpp::Subscription<perception_interfaces::msg::LidarObservation>::SharedPtr
            observation_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    void on_observation(
            const perception_interfaces::msg::LidarObservation & message)
    {
        try {
            const auto observation = to_core_observation(message);
            const auto geometry    = builder_.build(observation, beam_stride_);
            marker_publisher_->publish(make_markers(message, geometry));
        }
        catch(const std::exception & error) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Rejecting ray evidence debug batch: %s", error.what());
        }
    }

    visualization_msgs::msg::MarkerArray make_markers(
            const perception_interfaces::msg::LidarObservation & message,
            const RayEvidenceDebugGeometry &                     geometry) const
    {
        const auto marker_id = marker_id_for_sensor(message.sensor_id);
        visualization_msgs::msg::MarkerArray array;
        array.markers.reserve(4);

        auto hit_endpoints = make_marker(
                message.header, "ray_evidence/hit_endpoints", marker_id,
                visualization_msgs::msg::Marker::POINTS,
                1.0F, 0.0F, 0.0F, point_size_m_, marker_lifetime_s_);
        append_points(hit_endpoints, geometry.hit_endpoints);
        array.markers.push_back(std::move(hit_endpoints));

        auto hit_free = make_marker(
                message.header, "ray_evidence/hit_free", marker_id,
                visualization_msgs::msg::Marker::LINE_LIST,
                0.0F, 1.0F, 0.0F, line_width_m_, marker_lifetime_s_);
        append_points(hit_free, geometry.hit_free_segments);
        array.markers.push_back(std::move(hit_free));

        auto no_return_free = make_marker(
                message.header, "ray_evidence/no_return_free", marker_id,
                visualization_msgs::msg::Marker::LINE_LIST,
                0.0F, 1.0F, 1.0F, 1.5 * line_width_m_, marker_lifetime_s_);
        append_points(no_return_free, geometry.no_return_free_segments);
        array.markers.push_back(std::move(no_return_free));

        auto invalid = make_marker(
                message.header, "ray_evidence/invalid", marker_id,
                visualization_msgs::msg::Marker::LINE_LIST,
                0.6F, 0.6F, 0.6F, 5.0 * line_width_m_, marker_lifetime_s_);
        append_points(invalid, geometry.invalid_indicators);
        array.markers.push_back(std::move(invalid));

        return array;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<RayEvidenceDebugNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("ray_evidence_debug_node"), "%s", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}

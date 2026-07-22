#include "swarm_controller/FrontierComponentAuditReplay.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace SwarmController {

    namespace {

        geometry_msgs::msg::Point messagePoint(const Point3f & point)
        {
            geometry_msgs::msg::Point result;
            result.x = point.x;
            result.y = point.y;
            result.z = point.z;
            return result;
        }

        void setColor(
                visualization_msgs::msg::Marker & marker,
                const ComponentAuditColor & color)
        {
            marker.color.r = color.r;
            marker.color.g = color.g;
            marker.color.b = color.b;
            marker.color.a = color.a;
        }

        visualization_msgs::msg::Marker baseMarker(
                const std::string & frame_id, const rclcpp::Time & stamp,
                const std::string & ns, const std::int32_t id,
                const std::int32_t type)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = stamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = type;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            return marker;
        }

        std::size_t nonNegativeSize(const std::int64_t value, const char * name)
        {
            if(value < 0) {
                throw std::invalid_argument(
                        std::string(name) + " must be non-negative");
            }
            return static_cast<std::size_t>(value);
        }

    }// namespace

    class FrontierComponentAuditReplayNode final : public rclcpp::Node
    {
    public:
        FrontierComponentAuditReplayNode()
            : rclcpp::Node("frontier_component_audit_replay")
        {
            declareParameters();
            loadConfiguration();

            const FrontierComponentAuditReplay replay(config_);
            const ComponentAuditSnapshot snapshot = replay.loadSnapshot(
                    component_csv_, membership_csv_);
            const ComponentAuditAnalysis analysis = replay.analyze(snapshot);
            for(ComponentAuditStageScene & stage :
                replay.buildStageScenes(snapshot))
            {
                addPublishedScene(stage.stage, std::move(stage.scene));
            }
            publishScenes();
            if(republish_rate_hz_ > 0.0) {
                const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(1.0 / republish_rate_hz_));
                republish_timer_ = create_wall_timer(
                        period, [this]() { publishScenes(); });
            }
            RCLCPP_INFO(
                    get_logger(),
                    "component audit replay: frame=%zu components=%zu columns=%zu "
                    "gap_pairs=%zu beneficial_pairs=%zu radius2_accepted=%zu stages=%zu",
                    snapshot.frame_index, analysis.total_components,
                    analysis.total_columns, analysis.one_column_gap_pairs.size(),
                    analysis.beneficial_gap_pairs.size(),
                    analysis.radius_two_accepted_groups, published_scenes_.size());
        }

    private:
        void declareParameters()
        {
            declare_parameter("frame_id", "map");
            declare_parameter("audit.component_csv", "");
            declare_parameter("audit.membership_csv", "");
            declare_parameter("audit.expected_frame_index", 3);
            declare_parameter("audit.column_footprint_height", 0.08);
            declare_parameter("display.show_labels", true);
            declare_parameter("display.republish_rate_hz", 0.0);
        }

        void loadConfiguration()
        {
            frame_id_ = get_parameter("frame_id").as_string();
            component_csv_ =
                    get_parameter("audit.component_csv").as_string();
            membership_csv_ =
                    get_parameter("audit.membership_csv").as_string();
            config_.expected_frame_index = nonNegativeSize(
                    get_parameter("audit.expected_frame_index").as_int(),
                    "audit.expected_frame_index");
            config_.column_footprint_height = static_cast<float>(
                    get_parameter("audit.column_footprint_height").as_double());
            config_.show_labels =
                    get_parameter("display.show_labels").as_bool();
            republish_rate_hz_ =
                    get_parameter("display.republish_rate_hz").as_double();
            if(frame_id_.empty() || component_csv_.empty()
               || membership_csv_.empty())
            {
                throw std::invalid_argument(
                        "frame_id and both audit fixture paths are required");
            }
            if(!std::filesystem::is_regular_file(component_csv_)
               || !std::filesystem::is_regular_file(membership_csv_))
            {
                throw std::invalid_argument(
                        "component audit fixture paths must be regular files");
            }
            if(!std::isfinite(republish_rate_hz_) || republish_rate_hz_ < 0.0) {
                throw std::invalid_argument(
                        "display.republish_rate_hz must be finite and non-negative");
            }
        }

        struct PublishedScene {
            std::string stage;
            ComponentAuditScene scene;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                    publisher;
        };

        void addPublishedScene(std::string stage, ComponentAuditScene scene)
        {
            const std::string topic =
                    "frontier_component_audit/stages/" + stage + "/markers";
            auto publisher = create_publisher<visualization_msgs::msg::MarkerArray>(
                    topic,
                    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
            RCLCPP_INFO(
                    get_logger(), "component audit stage: %s -> /%s",
                    stage.c_str(), topic.c_str());
            published_scenes_.push_back(PublishedScene {
                    std::move(stage), std::move(scene), std::move(publisher)});
        }

        void publishScenes()
        {
            const rclcpp::Time stamp = get_clock()->now();
            for(const PublishedScene & published : published_scenes_) {
                visualization_msgs::msg::MarkerArray array;
                visualization_msgs::msg::Marker clear;
                clear.header.frame_id = frame_id_;
                clear.header.stamp = stamp;
                clear.action = visualization_msgs::msg::Marker::DELETEALL;
                array.markers.push_back(std::move(clear));

                for(const ComponentAuditBox & box : published.scene.boxes) {
                    auto marker = baseMarker(
                            frame_id_, stamp, box.ns, box.id,
                            visualization_msgs::msg::Marker::CUBE);
                    marker.pose.position = messagePoint(box.center);
                    marker.scale.x = box.scale.x;
                    marker.scale.y = box.scale.y;
                    marker.scale.z = box.scale.z;
                    setColor(marker, box.color);
                    array.markers.push_back(std::move(marker));
                }
                for(const ComponentAuditPointLayer & layer :
                    published.scene.point_layers)
                {
                    if(layer.points.empty()) {
                        continue;
                    }
                    const std::int32_t marker_type =
                            layer.shape == ComponentAuditPointShape::Cube
                                    ? visualization_msgs::msg::Marker::CUBE_LIST
                                    : visualization_msgs::msg::Marker::SPHERE_LIST;
                    auto marker = baseMarker(
                            frame_id_, stamp, layer.ns, layer.id, marker_type);
                    marker.scale.x = layer.scale.x;
                    marker.scale.y = layer.scale.y;
                    marker.scale.z = layer.scale.z;
                    setColor(marker, layer.color);
                    marker.points.reserve(layer.points.size());
                    for(const Point3f & point : layer.points) {
                        marker.points.push_back(messagePoint(point));
                    }
                    if(!layer.colors.empty()) {
                        if(layer.colors.size() != layer.points.size()) {
                            throw std::logic_error(
                                    "point layer color count differs from points");
                        }
                        marker.colors.reserve(layer.colors.size());
                        for(const ComponentAuditColor & color : layer.colors) {
                            marker.colors.emplace_back();
                            auto & target = marker.colors.back();
                            target.r = color.r;
                            target.g = color.g;
                            target.b = color.b;
                            target.a = color.a;
                        }
                    }
                    array.markers.push_back(std::move(marker));
                }
                for(const ComponentAuditLineLayer & layer :
                    published.scene.line_layers)
                {
                    if(layer.points.empty()) {
                        continue;
                    }
                    if(layer.points.size() % 2U != 0U) {
                        throw std::logic_error(
                                "line list requires an even number of points");
                    }
                    auto marker = baseMarker(
                            frame_id_, stamp, layer.ns, layer.id,
                            visualization_msgs::msg::Marker::LINE_LIST);
                    marker.scale.x = layer.width;
                    setColor(marker, layer.color);
                    marker.points.reserve(layer.points.size());
                    for(const Point3f & point : layer.points) {
                        marker.points.push_back(messagePoint(point));
                    }
                    array.markers.push_back(std::move(marker));
                }
                for(const ComponentAuditArrow & arrow : published.scene.arrows) {
                    auto marker = baseMarker(
                            frame_id_, stamp, arrow.ns, arrow.id,
                            visualization_msgs::msg::Marker::ARROW);
                    marker.points.push_back(messagePoint(arrow.start));
                    marker.points.push_back(messagePoint(arrow.end));
                    marker.scale.x = arrow.shaft_diameter;
                    marker.scale.y = arrow.head_diameter;
                    marker.scale.z = arrow.head_length;
                    setColor(marker, arrow.color);
                    array.markers.push_back(std::move(marker));
                }
                for(const ComponentAuditText & text : published.scene.texts) {
                    auto marker = baseMarker(
                            frame_id_, stamp, text.ns, text.id,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
                    marker.pose.position = messagePoint(text.position);
                    marker.scale.z = text.height;
                    marker.text = text.text;
                    setColor(marker, text.color);
                    array.markers.push_back(std::move(marker));
                }
                published.publisher->publish(array);
            }
        }

        std::string frame_id_ {"map"};
        std::filesystem::path component_csv_;
        std::filesystem::path membership_csv_;
        FrontierComponentAuditReplayConfig config_;
        double republish_rate_hz_ {0.0};
        std::vector<PublishedScene> published_scenes_;
        rclcpp::TimerBase::SharedPtr republish_timer_;
    };

}// namespace SwarmController

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(
                std::make_shared<
                        SwarmController::FrontierComponentAuditReplayNode>());
    } catch(const std::exception & exception) {
        RCLCPP_FATAL(
                rclcpp::get_logger("frontier_component_audit_replay"), "%s",
                exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

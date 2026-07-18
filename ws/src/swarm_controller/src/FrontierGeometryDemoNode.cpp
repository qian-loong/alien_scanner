#include "swarm_controller/FrontierGeometryDemo.hpp"

#include <cmath>
#include <chrono>
#include <cstdint>
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
                visualization_msgs::msg::Marker & target,
                const DemoColor & source)
        {
            target.color.r = source.r;
            target.color.g = source.g;
            target.color.b = source.b;
            target.color.a = source.a;
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

        std::size_t positiveSize(const std::int64_t value, const char * name)
        {
            if(value <= 0) {
                throw std::invalid_argument(std::string(name) + " must be positive");
            }
            return static_cast<std::size_t>(value);
        }

    }// namespace

    class FrontierGeometryDemoNode final : public rclcpp::Node
    {
    public:
        FrontierGeometryDemoNode()
            : rclcpp::Node("frontier_geometry_demo")
        {
            declareParameters();
            loadConfiguration();
            const FrontierGeometryDemo demo(config_);
            if(compose_stages_) {
                for(DemoStageScene & stage_scene : demo.buildStageScenes()) {
                    addPublishedScene(
                            stage_scene.stage,
                            "frontier_geometry_demo/stages/" + stage_scene.stage
                                    + "/markers",
                            std::move(stage_scene.scene));
                }
            } else {
                addPublishedScene(
                        config_.stage, "frontier_geometry_demo/markers",
                        demo.buildScene());
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
                    "frontier geometry demo: mode=%s stages=%zu markers=%zu frame=%s",
                    config_.mode.c_str(), published_scenes_.size(), markerCount(),
                    frame_id_.c_str());
        }

    private:
        void declareParameters()
        {
            declare_parameter("frame_id", "map");
            declare_parameter("scene.mode", "combined");
            declare_parameter("scene.stage", "accumulated_frontier");
            declare_parameter("scene.compose_stages", true);
            declare_parameter("tunnel.radius", 2.0);
            declare_parameter("tunnel.length", 8.0);
            declare_parameter("tunnel.center_z", 0.0);
            declare_parameter("cave.seed", 42);
            declare_parameter("scan.pitch_degrees", 20.0);
            declare_parameter("scan.selected_phi_degrees", 0.0);
            declare_parameter("scan.ray_count", 144);
            declare_parameter("scan.yaw_steps", 144);
            declare_parameter("scan.max_range", 3.0);
            declare_parameter("observation.initial_x", 1.0);
            declare_parameter("observation.initial_y", 0.0);
            declare_parameter("observation.hop_distance", 0.8);
            declare_parameter("scan.ring_segments", 72);
            declare_parameter("scan.show_full_ring", true);
            declare_parameter("voxel.resolution", 0.1);
            declare_parameter("voxel.radial_samples", 8);
            declare_parameter("voxel.angular_samples", 24);
            declare_parameter("voxel.slice_layers", 3);
            declare_parameter("voxel.visual_scale", 1.0);
            declare_parameter("frontier.column_stride_voxels", 2);
            declare_parameter("frontier.min_z_layers", 5);
            declare_parameter("frontier.min_z_span", 0.4);
            declare_parameter("frontier.support_depth", 0.8);
            declare_parameter("fixture.min_component_columns", 12);
            declare_parameter("frontier.max_trace_candidates", 10000);
            declare_parameter("frontier.max_trace_support_samples", 100000);
            declare_parameter("frontier.max_trace_components", 10000);
            declare_parameter("frontier.max_trace_geometry_elements", 500000);
            declare_parameter("display.show_labels", true);
            declare_parameter("display.show_unknown", true);
            declare_parameter("display.show_reference_geometry", true);
            declare_parameter("display.republish_rate_hz", 0.0);
        }

        void loadConfiguration()
        {
            frame_id_ = get_parameter("frame_id").as_string();
            config_.mode = get_parameter("scene.mode").as_string();
            config_.stage = get_parameter("scene.stage").as_string();
            compose_stages_ = get_parameter("scene.compose_stages").as_bool();
            config_.tunnel_radius = static_cast<float>(
                    get_parameter("tunnel.radius").as_double());
            config_.tunnel_length = static_cast<float>(
                    get_parameter("tunnel.length").as_double());
            config_.tunnel_center_z = static_cast<float>(
                    get_parameter("tunnel.center_z").as_double());
            config_.cave_seed = static_cast<std::uint32_t>(
                    get_parameter("cave.seed").as_int());
            config_.pitch_degrees = static_cast<float>(
                    get_parameter("scan.pitch_degrees").as_double());
            config_.selected_phi_degrees = static_cast<float>(
                    get_parameter("scan.selected_phi_degrees").as_double());
            config_.ray_count = positiveSize(
                    get_parameter("scan.ray_count").as_int(), "scan.ray_count");
            config_.yaw_steps = positiveSize(
                    get_parameter("scan.yaw_steps").as_int(), "scan.yaw_steps");
            config_.lidar_max_range = static_cast<float>(
                    get_parameter("scan.max_range").as_double());
            config_.initial_x = static_cast<float>(
                    get_parameter("observation.initial_x").as_double());
            config_.initial_y = static_cast<float>(
                    get_parameter("observation.initial_y").as_double());
            config_.hop_distance = static_cast<float>(
                    get_parameter("observation.hop_distance").as_double());
            config_.tunnel_ring_segments = positiveSize(
                    get_parameter("scan.ring_segments").as_int(),
                    "scan.ring_segments");
            config_.show_full_ring = get_parameter("scan.show_full_ring").as_bool();
            config_.voxel_resolution = static_cast<float>(
                    get_parameter("voxel.resolution").as_double());
            config_.voxel_radial_samples = positiveSize(
                    get_parameter("voxel.radial_samples").as_int(),
                    "voxel.radial_samples");
            config_.voxel_angular_samples = positiveSize(
                    get_parameter("voxel.angular_samples").as_int(),
                    "voxel.angular_samples");
            config_.voxel_thickness_layers = positiveSize(
                    get_parameter("voxel.slice_layers").as_int(),
                    "voxel.slice_layers");
            config_.voxel_visual_scale = static_cast<float>(
                    get_parameter("voxel.visual_scale").as_double());
            config_.column_stride_voxels = positiveSize(
                    get_parameter("frontier.column_stride_voxels").as_int(),
                    "frontier.column_stride_voxels");
            config_.min_z_layers = positiveSize(
                    get_parameter("frontier.min_z_layers").as_int(),
                    "frontier.min_z_layers");
            config_.min_z_span = static_cast<float>(
                    get_parameter("frontier.min_z_span").as_double());
            config_.support_depth = get_parameter("frontier.support_depth").as_double();
            config_.min_component_columns = positiveSize(
                    get_parameter("fixture.min_component_columns").as_int(),
                    "fixture.min_component_columns");
            config_.max_trace_candidates = positiveSize(
                    get_parameter("frontier.max_trace_candidates").as_int(),
                    "frontier.max_trace_candidates");
            config_.max_trace_support_samples = positiveSize(
                    get_parameter("frontier.max_trace_support_samples").as_int(),
                    "frontier.max_trace_support_samples");
            config_.max_trace_components = positiveSize(
                    get_parameter("frontier.max_trace_components").as_int(),
                    "frontier.max_trace_components");
            config_.max_trace_geometry_elements = positiveSize(
                    get_parameter("frontier.max_trace_geometry_elements").as_int(),
                    "frontier.max_trace_geometry_elements");
            config_.show_labels = get_parameter("display.show_labels").as_bool();
            config_.show_unknown = get_parameter("display.show_unknown").as_bool();
            config_.show_reference_geometry =
                    get_parameter("display.show_reference_geometry").as_bool();
            republish_rate_hz_ =
                    get_parameter("display.republish_rate_hz").as_double();
            if(frame_id_.empty()) {
                throw std::invalid_argument("invalid frontier geometry demo node configuration");
            }
            if(!std::isfinite(republish_rate_hz_) || republish_rate_hz_ < 0.0) {
                throw std::invalid_argument(
                        "display.republish_rate_hz must be finite and non-negative");
            }
        }

        struct PublishedScene {
            std::string stage;
            std::string topic;
            DemoScene scene;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
        };

        void addPublishedScene(
                std::string stage, std::string topic, DemoScene scene)
        {
            auto publisher = create_publisher<visualization_msgs::msg::MarkerArray>(
                    topic,
                    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
            RCLCPP_INFO(
                    get_logger(), "frontier demo stage: %s -> /%s",
                    stage.c_str(), topic.c_str());
            published_scenes_.push_back(PublishedScene {
                    std::move(stage), std::move(topic), std::move(scene),
                    std::move(publisher)});
        }

        static std::size_t markerCount(const DemoScene & scene)
        {
            return scene.cubes.size() + scene.spheres.size()
                   + scene.lines.size() + scene.arrows.size()
                   + scene.triangles.size() + scene.texts.size();
        }

        std::size_t markerCount() const
        {
            std::size_t count = 0U;
            for(const PublishedScene & published : published_scenes_) {
                count += markerCount(published.scene);
            }
            return count;
        }

        void publishScenes()
        {
            const auto stamp = get_clock()->now();
            for(const PublishedScene & published : published_scenes_) {
                publishScene(published, stamp);
            }
        }

        void publishScene(
                const PublishedScene & published, const rclcpp::Time & stamp)
        {
            visualization_msgs::msg::MarkerArray array;
            visualization_msgs::msg::Marker clear;
            clear.header.frame_id = frame_id_;
            clear.header.stamp = stamp;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            array.markers.push_back(std::move(clear));

            for(const DemoCube & cube : published.scene.cubes) {
                auto marker = baseMarker(
                        frame_id_, stamp, cube.ns, cube.id,
                        visualization_msgs::msg::Marker::CUBE);
                marker.pose.position = messagePoint(cube.center);
                marker.scale.x = cube.scale.x;
                marker.scale.y = cube.scale.y;
                marker.scale.z = cube.scale.z;
                setColor(marker, cube.color);
                array.markers.push_back(std::move(marker));
            }
            for(const DemoSphere & sphere : published.scene.spheres) {
                auto marker = baseMarker(
                        frame_id_, stamp, sphere.ns, sphere.id,
                        visualization_msgs::msg::Marker::SPHERE);
                marker.pose.position = messagePoint(sphere.center);
                marker.scale.x = sphere.diameter;
                marker.scale.y = sphere.diameter;
                marker.scale.z = sphere.diameter;
                setColor(marker, sphere.color);
                array.markers.push_back(std::move(marker));
            }
            for(const DemoLine & line : published.scene.lines) {
                auto marker = baseMarker(
                        frame_id_, stamp, line.ns, line.id,
                        visualization_msgs::msg::Marker::LINE_LIST);
                marker.points.push_back(messagePoint(line.start));
                marker.points.push_back(messagePoint(line.end));
                marker.scale.x = line.width;
                setColor(marker, line.color);
                array.markers.push_back(std::move(marker));
            }
            for(const DemoArrow & arrow : published.scene.arrows) {
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
            for(const DemoTriangle & triangle : published.scene.triangles) {
                auto marker = baseMarker(
                        frame_id_, stamp, triangle.ns, triangle.id,
                        visualization_msgs::msg::Marker::TRIANGLE_LIST);
                marker.points.push_back(messagePoint(triangle.first));
                marker.points.push_back(messagePoint(triangle.second));
                marker.points.push_back(messagePoint(triangle.third));
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;
                setColor(marker, triangle.color);
                array.markers.push_back(std::move(marker));
            }
            for(const DemoText & text : published.scene.texts) {
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

        std::string frame_id_ {"map"};
        FrontierGeometryDemoConfig config_;
        bool compose_stages_ {true};
        double republish_rate_hz_ {0.0};
        std::vector<PublishedScene> published_scenes_;
        rclcpp::TimerBase::SharedPtr republish_timer_;
    };

}// namespace SwarmController

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmController::FrontierGeometryDemoNode>());
    } catch(const std::exception & exception) {
        RCLCPP_FATAL(
                rclcpp::get_logger("frontier_geometry_demo"), "%s",
                exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

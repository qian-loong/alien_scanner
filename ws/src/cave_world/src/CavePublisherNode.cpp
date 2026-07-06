#include "cave_world/CavePublisherNode.hpp"

#include <chrono>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace CaveWorld {

CavePublisherNode::CavePublisherNode()
    : rclcpp::Node("cave_publisher")
{
    declareParameters();
    frame_id_ = get_parameter("frame_id").as_string();
    topic_ = get_parameter("topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate").as_double();
    if(publish_rate_hz_ <= 0.0) {
        publish_rate_hz_ = 1.0;
    }

    initField();
    buildCachedPointCloud();
    initPublisher();
    initTimer();
    publishCachedPointCloud();
}

void CavePublisherNode::declareParameters()
{
    declare_parameter("cave_mode", "tree");

    declare_parameter("length", 40.0);
    declare_parameter("branch_length", 20.0);
    declare_parameter("base_radius", 2.5);
    declare_parameter("n_segments", 200);
    declare_parameter("branch", true);
    declare_parameter("branch_angle", 0.55);
    declare_parameter("chamber_at", 0.5);
    declare_parameter("chamber_scale", 3.0);
    declare_parameter("noise_scale", 0.4);
    declare_parameter("density", 400);
    declare_parameter("seed", 42);

    declare_parameter("tree.approach_length", 12.0);
    declare_parameter("tree.loop_yaw", 0.50);
    declare_parameter("tree.loop_direct_length", 16.0);
    declare_parameter("tree.loop_bulge", 12.0);
    declare_parameter("tree.exit1_length", 14.0);
    declare_parameter("tree.right_yaw", -0.12);
    declare_parameter("tree.right_corridor_length", 10.0);
    declare_parameter("tree.exit_yaw_spread", 0.35);
    declare_parameter("tree.exit_arm_length", 14.0);
    declare_parameter("tree.vertical_step", -0.10);
    declare_parameter("tree.asymmetry", 0.22);
    declare_parameter("tree.chamber_on_approach", false);
    declare_parameter("tree.chamber_at", 0.55);
    declare_parameter("tree.chamber_scale", 2.2);

    declare_parameter("frame_id", "map");
    declare_parameter("topic", "/cave/points");
    declare_parameter("publish_rate", 1.0);
}

ProceduralCaveFieldConfig CavePublisherNode::loadYConfigFromParameters() const
{
    ProceduralCaveFieldConfig config;
    config.length = static_cast<float>(get_parameter("length").as_double());
    config.branch_length = static_cast<float>(get_parameter("branch_length").as_double());
    config.base_radius = static_cast<float>(get_parameter("base_radius").as_double());
    config.n_segments = get_parameter("n_segments").as_int();
    config.branch = get_parameter("branch").as_bool();
    config.branch_angle = static_cast<float>(get_parameter("branch_angle").as_double());
    config.chamber_at = static_cast<float>(get_parameter("chamber_at").as_double());
    config.chamber_scale = static_cast<float>(get_parameter("chamber_scale").as_double());
    config.noise_scale = static_cast<float>(get_parameter("noise_scale").as_double());
    config.density = get_parameter("density").as_int();
    config.seed = static_cast<std::uint32_t>(get_parameter("seed").as_int());
    return config;
}

TreeCaveFieldConfig CavePublisherNode::loadTreeConfigFromParameters() const
{
    TreeCaveFieldConfig config;
    config.approach_length = static_cast<float>(get_parameter("tree.approach_length").as_double());
    config.base_radius = static_cast<float>(get_parameter("base_radius").as_double());
    config.n_segments = get_parameter("n_segments").as_int();
    config.density = get_parameter("density").as_int();
    config.noise_scale = static_cast<float>(get_parameter("noise_scale").as_double());
    config.seed = static_cast<std::uint32_t>(get_parameter("seed").as_int());
    config.loop_yaw = static_cast<float>(get_parameter("tree.loop_yaw").as_double());
    config.loop_direct_length = static_cast<float>(get_parameter("tree.loop_direct_length").as_double());
    config.loop_bulge = static_cast<float>(get_parameter("tree.loop_bulge").as_double());
    config.exit1_length = static_cast<float>(get_parameter("tree.exit1_length").as_double());
    config.right_yaw = static_cast<float>(get_parameter("tree.right_yaw").as_double());
    config.right_corridor_length = static_cast<float>(get_parameter("tree.right_corridor_length").as_double());
    config.exit_yaw_spread = static_cast<float>(get_parameter("tree.exit_yaw_spread").as_double());
    config.exit_arm_length = static_cast<float>(get_parameter("tree.exit_arm_length").as_double());
    config.vertical_step = static_cast<float>(get_parameter("tree.vertical_step").as_double());
    config.asymmetry = static_cast<float>(get_parameter("tree.asymmetry").as_double());
    config.chamber_on_approach = get_parameter("tree.chamber_on_approach").as_bool();
    config.chamber_at = static_cast<float>(get_parameter("tree.chamber_at").as_double());
    config.chamber_scale = static_cast<float>(get_parameter("tree.chamber_scale").as_double());
    return config;
}

void CavePublisherNode::initField()
{
    const std::string cave_mode = get_parameter("cave_mode").as_string();
    if(cave_mode == "y") {
        field_ = std::make_unique<ProceduralCaveField>(loadYConfigFromParameters());
        RCLCPP_INFO(get_logger(), "Cave geometry mode: classic Y (ProceduralCaveField)");
        return;
    }

    field_ = std::make_unique<TreeCaveField>(loadTreeConfigFromParameters());
    RCLCPP_INFO(get_logger(), "Cave geometry mode: network (TreeCaveField: 1 in / 3 out / 1 loop)");
}

void CavePublisherNode::initPublisher()
{
    const rclcpp::QoS qos = rclcpp::QoS(1).transient_local();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);
}

void CavePublisherNode::initTimer()
{
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period_ns, [this]() {
        onTimer();
    });
}

void CavePublisherNode::buildCachedPointCloud()
{
    const std::vector<Point3> points = field_->sampleSurface();

    sensor_msgs::PointCloud2Modifier modifier(cached_cloud_);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cached_cloud_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cached_cloud_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cached_cloud_, "z");
    for(const Point3 & point : points) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    cached_cloud_.header.frame_id = frame_id_;
    cached_cloud_.header.stamp = get_clock()->now();
    cached_cloud_.is_dense = true;
}

void CavePublisherNode::publishCachedPointCloud()
{
    cached_cloud_.header.stamp = get_clock()->now();
    publisher_->publish(cached_cloud_);
}

void CavePublisherNode::onTimer()
{
    publishCachedPointCloud();
    RCLCPP_INFO(get_logger(), "Published %u cave surface points on '%s'",
        cached_cloud_.width * cached_cloud_.height, topic_.c_str());
}

}// namespace CaveWorld

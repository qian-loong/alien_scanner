#include "swarm_controller/GlobalFrontierDetector.hpp"
#include "swarm_controller/GlobalTaskAllocator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_controller_interfaces/msg/exploration_task.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace SwarmController {

    namespace {

        using SteadyClock = std::chrono::steady_clock;

        tf2::TimePoint timePointFromStamp(const builtin_interfaces::msg::Time & stamp)
        {
            return tf2::TimePoint(
                    std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec));
        }

        geometry_msgs::msg::Point point(const Point3f & value)
        {
            geometry_msgs::msg::Point result;
            result.x = value.x;
            result.y = value.y;
            result.z = value.z;
            return result;
        }

        diagnostic_msgs::msg::KeyValue value(
                const std::string & key, const std::string & content)
        {
            diagnostic_msgs::msg::KeyValue result;
            result.key = key;
            result.value = content;
            return result;
        }

        template<typename T>
        diagnostic_msgs::msg::KeyValue numericValue(const std::string & key, const T content)
        {
            return value(key, std::to_string(content));
        }

        std::uint64_t makeAllocatorEpoch()
        {
            const auto raw = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count();
            return raw > 0 ? static_cast<std::uint64_t>(raw) : 1U;
        }

        bool freshStamp(
                const std::int64_t stamp_ns, const std::int64_t now_ns,
                const double timeout_seconds)
        {
            if(stamp_ns <= 0 || now_ns < stamp_ns) {
                return false;
            }
            return now_ns - stamp_ns
                   <= static_cast<std::int64_t>(std::llround(timeout_seconds * 1.0e9));

        }
        std::shared_ptr<octomap::OcTree> decodeMap(
                const octomap_msgs::msg::Octomap & message,
                const std::string & map_frame, const double resolution,
                std::string & reason)
        {
            if(message.header.frame_id != map_frame) {
                reason = "map frame mismatch";
                return {};
            }
            if(message.binary) {
                reason = "binary Octomap is not supported";
                return {};
            }
            std::unique_ptr<octomap::AbstractOcTree> abstract(
                    octomap_msgs::fullMsgToMap(message));
            auto * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
            if(tree == nullptr) {
                reason = "message does not contain an OcTree";
                return {};
            }
            if(std::fabs(tree->getResolution() - resolution) > 1.0e-5) {
                reason = "Octomap resolution mismatch";
                return {};
            }
            abstract.release();
            reason.clear();
            return std::shared_ptr<octomap::OcTree>(tree);
        }

    }// namespace

    class GlobalTaskAllocatorNode final : public rclcpp::Node
    {
    public:
        GlobalTaskAllocatorNode()
            : Node("global_task_allocator")
            , steady_start_(SteadyClock::now())
            , allocator_epoch_(makeAllocatorEpoch())
        {
            declareParameters();
            loadConfiguration();
            detector_ = std::make_unique<GlobalFrontierDetector>(detector_config_);
            allocator_ = std::make_unique<GlobalTaskAllocator>(allocator_config_);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            setupDrones();
            setupGlobalInputs();
            diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                    "global_task_diagnostics",
                    rclcpp::QoS(1).reliable().transient_local());
            marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    "global_task_markers", rclcpp::QoS(1));
            const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / allocation_rate_hz_));
            timer_ = create_wall_timer(period, [this]() { onTimer(); });
            RCLCPP_INFO(
                    get_logger(), "global_task_allocator: %zu drones at %.2f Hz epoch=%llu",
                    drones_.size(), allocation_rate_hz_,
                    static_cast<unsigned long long>(allocator_epoch_));
        }

    private:
        struct DroneTrack {
            std::string id;
            std::string prefix;
            octomap_msgs::msg::Octomap::SharedPtr pending_map;
            std::int64_t pending_map_stamp_ns {};
            std::shared_ptr<octomap::OcTree> local_map;
            std::int64_t local_map_stamp_ns {};
            double local_map_receive_seconds {-1.0};
            nav_msgs::msg::Odometry odom;
            bool has_odom {false};
            double odom_receive_seconds {-1.0};
            rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
            rclcpp::Publisher<swarm_controller_interfaces::msg::ExplorationTask>::SharedPtr
                    task_publisher;
        };

        void declareParameters()
        {
            declare_parameter("map_frame", "map");
            declare_parameter<std::vector<std::string>>(
                    "drone_namespaces", {"drone_0", "drone_1", "drone_2"});
            declare_parameter("task_allocation.rate", 1.0);
            declare_parameter("task.lease", 3.0);
            declare_parameter("global_map.stale_timeout", 5.0);
            declare_parameter("global_map.diagnostics_timeout", 5.0);
            declare_parameter("drone.odom_timeout", 2.0);
            declare_parameter("drone.local_map_timeout", 5.0);

            declare_parameter("resolution", 0.1);
            declare_parameter("frontier.column_stride_voxels", 2);
            declare_parameter("frontier.min_z_layers", 5);
            declare_parameter("frontier.min_z_span", 0.4);
            declare_parameter("frontier.support_depth", 0.8);
            declare_parameter("frontier.support_width", 1.0);
            declare_parameter("frontier.min_columns", 12);
            declare_parameter("frontier.min_area", 0.48);
            declare_parameter("frontier.min_span", 0.6);
            declare_parameter("frontier.min_direction_consistency", 0.65);
            declare_parameter("frontier.max_scanned_free_voxels", 2000000);
            declare_parameter("frontier.max_support_samples_per_column", 10000);
            declare_parameter("frontier.max_frontier_columns", 250000);
            declare_parameter("frontier.max_columns_per_region", 50000);
            declare_parameter("frontier.max_regions", 64);
            declare_parameter("frontier.collect_stage_timings", false);
            declare_parameter("frontier.min_persistence_updates", 3);
            declare_parameter("frontier.min_persistence_time", 2.0);
            declare_parameter("frontier.missed_update_grace", 2);

            declare_parameter("allocation.max_assignment_distance", 8.0);
            declare_parameter("allocation.first_hop_distance", 1.0);
            declare_parameter("allocation.activation_min_regions", 2);
            declare_parameter("allocation.activation_min_drones", 2);
            declare_parameter("allocation.activation_updates", 2);
            declare_parameter("allocation.activation_time", 1.0);
            declare_parameter("allocation.deactivation_grace", 2.0);
            declare_parameter("allocation.no_progress_timeout", 35.0);
            declare_parameter("allocation.min_owner_progress", 0.30);
            declare_parameter("allocation.max_tracks", 128);
            declare_parameter("allocation.max_eligible_edges", 4096);
            declare_parameter("allocation.max_assignment_dimension", 64);
            declare_parameter("motion.timeout", 20.0);
            declare_parameter("hold.timeout", 2.0);
            declare_parameter("task.rescan_max_steps", 4);
            declare_parameter("task.rescan_step_timeout", 2.5);
            declare_parameter("body.robot_radius", 0.25);
            declare_parameter("body.robot_half_height", 0.15);
            declare_parameter("body.safety_margin", 0.25);
            declare_parameter("body.vertical_margin", 0.20);
            declare_parameter("body.sample_spacing_fraction", 0.50);
        }

        static std::size_t positiveSize(std::int64_t value, const char * name)
        {
            if(value <= 0) {
                throw std::invalid_argument(std::string(name) + " must be positive");
            }
            return static_cast<std::size_t>(value);
        }

        static std::size_t nonNegativeSize(std::int64_t value, const char * name)
        {
            if(value < 0) {
                throw std::invalid_argument(std::string(name) + " must be non-negative");
            }
            return static_cast<std::size_t>(value);
        }

        void loadConfiguration()
        {
            map_frame_ = get_parameter("map_frame").as_string();
            drone_namespaces_ = get_parameter("drone_namespaces").as_string_array();
            allocation_rate_hz_ = get_parameter("task_allocation.rate").as_double();
            task_lease_seconds_ = get_parameter("task.lease").as_double();
            global_map_timeout_seconds_ = get_parameter("global_map.stale_timeout").as_double();
            diagnostics_timeout_seconds_ =
                    get_parameter("global_map.diagnostics_timeout").as_double();
            odom_timeout_seconds_ = get_parameter("drone.odom_timeout").as_double();
            local_map_timeout_seconds_ = get_parameter("drone.local_map_timeout").as_double();
            const double resolution = get_parameter("resolution").as_double();
            const std::array<double, 7U> positive_values {
                    allocation_rate_hz_, task_lease_seconds_, global_map_timeout_seconds_,
                    diagnostics_timeout_seconds_, odom_timeout_seconds_,
                    local_map_timeout_seconds_, resolution};
            if(map_frame_.empty() || drone_namespaces_.empty()
               || !std::all_of(positive_values.begin(), positive_values.end(), [](double value) {
                      return std::isfinite(value) && value > 0.0;
                  }))
            {
                throw std::invalid_argument("invalid global task allocator node configuration");
            }
            if(task_lease_seconds_ <= 2.0 / allocation_rate_hz_) {
                throw std::invalid_argument("task.lease must exceed two allocation periods");
            }

            detector_config_.resolution = resolution;
            detector_config_.column_stride_voxels = positiveSize(
                    get_parameter("frontier.column_stride_voxels").as_int(),
                    "frontier.column_stride_voxels");
            detector_config_.min_z_layers = positiveSize(
                    get_parameter("frontier.min_z_layers").as_int(),
                    "frontier.min_z_layers");
            detector_config_.min_z_span = static_cast<float>(
                    get_parameter("frontier.min_z_span").as_double());
            detector_config_.support_depth = static_cast<float>(
                    get_parameter("frontier.support_depth").as_double());
            detector_config_.support_width = static_cast<float>(
                    get_parameter("frontier.support_width").as_double());
            detector_config_.min_columns = positiveSize(
                    get_parameter("frontier.min_columns").as_int(),
                    "frontier.min_columns");
            detector_config_.min_area = static_cast<float>(
                    get_parameter("frontier.min_area").as_double());
            detector_config_.min_span = static_cast<float>(
                    get_parameter("frontier.min_span").as_double());
            detector_config_.min_direction_consistency = static_cast<float>(
                    get_parameter("frontier.min_direction_consistency").as_double());
            detector_config_.max_scanned_free_voxels = positiveSize(
                    get_parameter("frontier.max_scanned_free_voxels").as_int(),
                    "frontier.max_scanned_free_voxels");
            detector_config_.max_support_samples_per_column = positiveSize(
                    get_parameter("frontier.max_support_samples_per_column").as_int(),
                    "frontier.max_support_samples_per_column");
            detector_config_.max_frontier_columns = positiveSize(
                    get_parameter("frontier.max_frontier_columns").as_int(),
                    "frontier.max_frontier_columns");
            detector_config_.max_columns_per_region = positiveSize(
                    get_parameter("frontier.max_columns_per_region").as_int(),
                    "frontier.max_columns_per_region");
            detector_config_.max_regions = positiveSize(
                    get_parameter("frontier.max_regions").as_int(),
                    "frontier.max_regions");
            detector_config_.collect_stage_timings =
                    get_parameter("frontier.collect_stage_timings").as_bool();

            allocator_config_.min_persistence_updates = positiveSize(
                    get_parameter("frontier.min_persistence_updates").as_int(),
                    "frontier.min_persistence_updates");
            allocator_config_.min_persistence_seconds =
                    get_parameter("frontier.min_persistence_time").as_double();
            allocator_config_.missed_update_grace = nonNegativeSize(
                    get_parameter("frontier.missed_update_grace").as_int(),
                    "frontier.missed_update_grace");
            allocator_config_.max_assignment_distance = static_cast<float>(
                    get_parameter("allocation.max_assignment_distance").as_double());
            allocator_config_.first_hop_distance = static_cast<float>(
                    get_parameter("allocation.first_hop_distance").as_double());
            allocator_config_.activation_min_regions = positiveSize(
                    get_parameter("allocation.activation_min_regions").as_int(),
                    "allocation.activation_min_regions");
            allocator_config_.activation_min_drones = positiveSize(
                    get_parameter("allocation.activation_min_drones").as_int(),
                    "allocation.activation_min_drones");
            allocator_config_.activation_updates = positiveSize(
                    get_parameter("allocation.activation_updates").as_int(),
                    "allocation.activation_updates");
            allocator_config_.activation_seconds =
                    get_parameter("allocation.activation_time").as_double();
            allocator_config_.deactivation_grace_seconds =
                    get_parameter("allocation.deactivation_grace").as_double();
            allocator_config_.no_progress_timeout_seconds =
                    get_parameter("allocation.no_progress_timeout").as_double();
            allocator_config_.min_owner_progress = static_cast<float>(
                    get_parameter("allocation.min_owner_progress").as_double());
            allocator_config_.max_tracks = positiveSize(
                    get_parameter("allocation.max_tracks").as_int(), "allocation.max_tracks");
            allocator_config_.max_eligible_edges = positiveSize(
                    get_parameter("allocation.max_eligible_edges").as_int(),
                    "allocation.max_eligible_edges");
            allocator_config_.max_assignment_dimension = positiveSize(
                    get_parameter("allocation.max_assignment_dimension").as_int(),
                    "allocation.max_assignment_dimension");
            allocator_config_.body_envelope.robot_radius = static_cast<float>(
                    get_parameter("body.robot_radius").as_double());
            allocator_config_.body_envelope.robot_half_height = static_cast<float>(
                    get_parameter("body.robot_half_height").as_double());
            allocator_config_.body_envelope.safety_margin = static_cast<float>(
                    get_parameter("body.safety_margin").as_double());
            allocator_config_.body_envelope.vertical_margin = static_cast<float>(
                    get_parameter("body.vertical_margin").as_double());
            allocator_config_.body_envelope.sample_spacing_fraction = static_cast<float>(
                    get_parameter("body.sample_spacing_fraction").as_double());

            const double required_no_progress =
                    get_parameter("motion.timeout").as_double()
                    + static_cast<double>(get_parameter("task.rescan_max_steps").as_int())
                              * get_parameter("task.rescan_step_timeout").as_double()
                    + get_parameter("hold.timeout").as_double();
            if(allocator_config_.no_progress_timeout_seconds <= required_no_progress) {
                throw std::invalid_argument(
                        "allocation.no_progress_timeout must exceed motion + task rescan + hold");
            }
        }

        void setupDrones()
        {
            std::sort(drone_namespaces_.begin(), drone_namespaces_.end());
            if(std::adjacent_find(drone_namespaces_.begin(), drone_namespaces_.end())
               != drone_namespaces_.end())
            {
                throw std::invalid_argument("drone_namespaces contains duplicates");
            }
            const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
            const auto task_qos = rclcpp::QoS(1).reliable().transient_local();
            for(const std::string & raw : drone_namespaces_) {
                if(raw.empty()) {
                    throw std::invalid_argument("drone namespace must not be empty");
                }
                DroneTrack track;
                track.id = raw.front() == '/' ? raw.substr(1) : raw;
                track.prefix = raw.front() == '/' ? raw : "/" + raw;
                drones_.push_back(std::move(track));
            }
            for(std::size_t index = 0U; index < drones_.size(); ++index) {
                drones_[index].map_subscription = create_subscription<octomap_msgs::msg::Octomap>(
                        drones_[index].prefix + "/octomap", map_qos,
                        [this, index](const octomap_msgs::msg::Octomap::SharedPtr message) {
                            std::lock_guard<std::mutex> lock(mutex_);
                            const std::int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
                            if(stamp > drones_[index].pending_map_stamp_ns) {
                                drones_[index].pending_map = message;
                                drones_[index].pending_map_stamp_ns = stamp;
                            }
                        });
                drones_[index].odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
                        drones_[index].prefix + "/odom", rclcpp::QoS(10),
                        [this, index](const nav_msgs::msg::Odometry::SharedPtr message) {
                            std::lock_guard<std::mutex> lock(mutex_);
                            drones_[index].odom = *message;
                            drones_[index].has_odom = true;
                            drones_[index].odom_receive_seconds = monotonicNow();
                        });
                drones_[index].task_publisher = create_publisher<
                        swarm_controller_interfaces::msg::ExplorationTask>(
                        drones_[index].prefix + "/exploration_task", task_qos);
            }
        }

        void setupGlobalInputs()
        {
            const auto qos = rclcpp::QoS(1).reliable().transient_local();
            global_map_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
                    "/global_map", qos,
                    [this](const octomap_msgs::msg::Octomap::SharedPtr message) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        const std::int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
                        ++global_map_received_count_;
                        // The merger stamp is the maximum source observation stamp. A slower
                        // source can change the merged tree without advancing that maximum.
                        if(stamp >= pending_global_stamp_ns_) {
                            if(stamp == pending_global_stamp_ns_
                               && pending_global_stamp_ns_ > 0)
                            {
                                ++global_map_same_stamp_count_;
                            }
                            pending_global_map_ = message;
                            pending_global_stamp_ns_ = stamp;
                        }
                        else {
                            ++global_map_regressed_stamp_count_;
                        }
                    });
            global_diagnostics_subscription_ = create_subscription<
                    diagnostic_msgs::msg::DiagnosticArray>(
                    "/global_map_diagnostics", qos,
                    [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
                        bool healthy = false;
                        for(const auto & status : message->status) {
                            if(status.name == "global_map_merger") {
                                healthy = status.level
                                          != diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                                for(const auto & item : status.values) {
                                    if((item.key == "missing_sources"
                                        || item.key == "stale_sources")
                                       && item.value != "0")
                                    {
                                        healthy = false;
                                    }
                                    if(item.key == "last_rejections"
                                       && !item.value.empty())
                                    {
                                        healthy = false;
                                    }
                                }
                            }
                        }
                        std::lock_guard<std::mutex> lock(mutex_);
                        global_diagnostics_healthy_ = healthy;
                        global_diagnostics_stamp_ns_ =
                                rclcpp::Time(message->header.stamp).nanoseconds();
                        global_diagnostics_receive_seconds_ = monotonicNow();
                    });
        }

        double monotonicNow() const
        {
            return std::chrono::duration<double>(SteadyClock::now() - steady_start_).count();
        }

        bool transformOdometry(const nav_msgs::msg::Odometry & odom, Pose3D & pose)
        {
            geometry_msgs::msg::PoseStamped source;
            source.header = odom.header;
            source.pose = odom.pose.pose;
            geometry_msgs::msg::PoseStamped mapped = source;
            if(source.header.frame_id.empty()) {
                return false;
            }
            if(source.header.frame_id != map_frame_) {
                try {
                    const auto transform = tf_buffer_->lookupTransform(
                            map_frame_, source.header.frame_id,
                            timePointFromStamp(source.header.stamp), tf2::durationFromSec(0.0));
                    tf2::doTransform(source, mapped, transform);
                } catch(const tf2::TransformException &) {
                    return false;
                }
            }
            pose.position = Point3f {
                    static_cast<float>(mapped.pose.position.x),
                    static_cast<float>(mapped.pose.position.y),
                    static_cast<float>(mapped.pose.position.z)};
            const auto & q = mapped.pose.orientation;
            pose.yaw = static_cast<float>(std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
            return std::isfinite(pose.position.x) && std::isfinite(pose.position.y)
                   && std::isfinite(pose.position.z) && std::isfinite(pose.yaw);
        }

        void processPendingMaps(double now_seconds)
        {
            octomap_msgs::msg::Octomap::SharedPtr global;
            std::vector<octomap_msgs::msg::Octomap::SharedPtr> local(drones_.size());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                global = std::move(pending_global_map_);
                for(std::size_t index = 0U; index < drones_.size(); ++index) {
                    local[index] = std::move(drones_[index].pending_map);
                }
            }
            if(global) {
                std::string reason;
                auto map = decodeMap(*global, map_frame_, detector_config_.resolution, reason);
                if(map) {
                    const FrontierDetectionResult detection = detector_->detect(*map);
                    detector_status_ = detection.status;
                    detector_diagnostics_ = detection.diagnostics;
                    vertical_rejected_columns_ = detection.vertical_rejected_columns;
                    scanned_free_voxels_ = detection.scanned_free_voxels;
                    support_rejected_columns_ = detection.support_rejected_columns;
                    raw_frontier_columns_ = detection.raw_columns;
                    supported_frontier_columns_ = detection.supported_columns;
                    detected_region_count_ = detection.regions.size();
                    if(detection.accepted()) {
                        global_map_ = std::move(map);
                        global_map_stamp_ns_ = rclcpp::Time(global->header.stamp).nanoseconds();
                        regions_ = detection.regions;
                        ++global_update_sequence_;
                        global_map_receive_seconds_ = now_seconds;
                        detector_reason_.clear();
                    } else {
                        detector_reason_ = detection.reason;
                    }
                } else {
                    detector_status_ = FrontierDetectionStatus::Invalid;
                    detector_diagnostics_ = {};
                    raw_frontier_columns_ = 0U;
                    scanned_free_voxels_ = 0U;
                    supported_frontier_columns_ = 0U;
                    vertical_rejected_columns_ = 0U;
                    support_rejected_columns_ = 0U;
                    detected_region_count_ = 0U;
                    detector_reason_ = reason;
                }
            }
            for(std::size_t index = 0U; index < local.size(); ++index) {
                if(!local[index]) {
                    continue;
                }
                std::string reason;
                auto map = decodeMap(
                        *local[index], map_frame_, detector_config_.resolution, reason);
                if(map) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    drones_[index].local_map = std::move(map);
                    drones_[index].local_map_stamp_ns =
                            rclcpp::Time(local[index]->header.stamp).nanoseconds();
                    drones_[index].local_map_receive_seconds = now_seconds;
                }
            }
        }

        void onTimer()
        {
            const std::int64_t now_ros_ns = get_clock()->now().nanoseconds();
            const double now = monotonicNow();
            processPendingMaps(now);
            GlobalAllocationInput input;
            input.regions = regions_;
            input.global_update_sequence = global_update_sequence_;
            input.monotonic_time_seconds = now;
            bool diagnostics_healthy = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostics_healthy = global_diagnostics_healthy_
                                      && global_diagnostics_receive_seconds_ >= 0.0
                                      && now - global_diagnostics_receive_seconds_
                                                 <= diagnostics_timeout_seconds_
                                      && freshStamp(
                                              global_diagnostics_stamp_ns_, now_ros_ns,
                                              diagnostics_timeout_seconds_);
                input.drones.reserve(drones_.size());
                for(const DroneTrack & track : drones_) {
                    DroneAllocationState state;
                    state.id = track.id;
                    state.local_map = track.local_map.get();
                    state.local_map_fresh = track.local_map != nullptr
                                            && now - track.local_map_receive_seconds
                                                       <= local_map_timeout_seconds_
                                            && freshStamp(
                                                    track.local_map_stamp_ns, now_ros_ns,
                                                    local_map_timeout_seconds_);
                    state.odom_fresh = track.has_odom
                                       && now - track.odom_receive_seconds
                                                  <= odom_timeout_seconds_
                                       && freshStamp(
                                               rclcpp::Time(track.odom.header.stamp).nanoseconds(),
                                               now_ros_ns, odom_timeout_seconds_);
                    if(state.odom_fresh && !transformOdometry(track.odom, state.pose)) {
                        state.odom_fresh = false;
                    }
                    input.drones.push_back(state);
                }
            }
            const bool global_fresh = global_map_ != nullptr
                                      && global_map_receive_seconds_ >= 0.0
                                      && now - global_map_receive_seconds_
                                                 <= global_map_timeout_seconds_
                                      && freshStamp(
                                              global_map_stamp_ns_, now_ros_ns,
                                              global_map_timeout_seconds_);
            input.healthy = global_fresh && diagnostics_healthy
                            && detector_reason_.empty();
            last_result_ = allocator_->update(input);
            publishTasks(last_result_);
            publishDiagnostics(input.healthy, global_fresh, diagnostics_healthy);
            publishMarkers(last_result_);
        }

        void publishTasks(const GlobalAllocationResult & result)
        {
            const rclcpp::Time now = get_clock()->now();
            const std::int64_t lease_ns = static_cast<std::int64_t>(
                    std::llround(task_lease_seconds_ * 1.0e9));
            for(const auto & assignment : result.assignments) {
                const auto found = std::find_if(
                        drones_.begin(), drones_.end(), [&](const DroneTrack & track) {
                            return track.id == assignment.drone_id;
                        });
                if(found == drones_.end()) {
                    continue;
                }
                swarm_controller_interfaces::msg::ExplorationTask message;
                message.header.frame_id = map_frame_;
                message.header.stamp = now;
                message.allocator_epoch = allocator_epoch_;
                message.revision = assignment.revision;
                message.task_id = assignment.task_id;
                message.mode = static_cast<std::uint8_t>(assignment.mode);
                message.target.position.x = assignment.target.x;
                message.target.position.y = assignment.target.y;
                message.target.position.z = assignment.target.z;
                message.target.orientation.w = 1.0;
                message.lease.sec = static_cast<std::int32_t>(lease_ns / 1'000'000'000LL);
                message.lease.nanosec = static_cast<std::uint32_t>(
                        lease_ns % 1'000'000'000LL);
                found->task_publisher->publish(message);
            }
        }

        void publishDiagnostics(
                bool healthy, bool global_fresh, bool diagnostics_healthy)
        {
            diagnostic_msgs::msg::DiagnosticArray message;
            message.header.stamp = get_clock()->now();
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = "global_task_allocator";
            status.hardware_id = "swarm";
            status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                   : diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = last_result_.reason.empty()
                                     ? (last_result_.coordination_mode
                                                        == CoordinationMode::Coordinated
                                                ? "Coordinated"
                                                : "LocalFallback")
                                     : last_result_.reason;
            status.values.push_back(numericValue("allocator_epoch", allocator_epoch_));
            status.values.push_back(numericValue("global_map_fresh", global_fresh ? 1 : 0));
            status.values.push_back(numericValue(
                    "global_map_diagnostics_healthy", diagnostics_healthy ? 1 : 0));
            status.values.push_back(numericValue("global_update_sequence", global_update_sequence_));
            status.values.push_back(numericValue(
                    "global_map_received", global_map_received_count_));
            status.values.push_back(numericValue(
                    "global_map_same_stamp", global_map_same_stamp_count_));
            status.values.push_back(numericValue(
                    "global_map_regressed_stamp", global_map_regressed_stamp_count_));
            status.values.push_back(numericValue("raw_frontier_columns", raw_frontier_columns_));
            status.values.push_back(numericValue("scanned_free_voxels", scanned_free_voxels_));
            status.values.push_back(numericValue(
                    "detector_status", static_cast<int>(detector_status_)));
            status.values.push_back(numericValue(
                    "detector_diagnostics_complete",
                    detector_diagnostics_.complete ? 1 : 0));
            status.values.push_back(numericValue(
                    "sampled_free_columns",
                    detector_diagnostics_.sampled_free_columns));
            status.values.push_back(numericValue(
                    "unknown_neighbor_candidate_columns",
                    detector_diagnostics_.unknown_neighbor_candidate_columns));
            status.values.push_back(numericValue(
                    "vertical_passed_columns",
                    detector_diagnostics_.vertical_passed_columns));
            status.values.push_back(numericValue(
                    "max_support_samples_per_column",
                    detector_config_.max_support_samples_per_column));
            status.values.push_back(numericValue(
                    "supported_frontier_columns", supported_frontier_columns_));
            status.values.push_back(numericValue(
                    "vertical_rejected_columns", vertical_rejected_columns_));
            status.values.push_back(numericValue(
                    "support_rejected_columns", support_rejected_columns_));
            status.values.push_back(numericValue(
                    "support_passed_columns",
                    detector_diagnostics_.support_passed_columns));
            status.values.push_back(numericValue(
                    "support_rejected_unknown",
                    detector_diagnostics_.support_rejected_unknown));
            status.values.push_back(numericValue(
                    "support_rejected_occupied",
                    detector_diagnostics_.support_rejected_occupied));
            status.values.push_back(numericValue(
                    "support_rejected_out_of_bounds",
                    detector_diagnostics_.support_rejected_out_of_bounds));
            status.values.push_back(numericValue(
                    "support_samples_attempted",
                    detector_diagnostics_.support_samples_attempted));
            status.values.push_back(numericValue(
                    "support_failure_position_unavailable",
                    detector_diagnostics_.support_failure_position_unavailable));
            for(std::size_t index = 0U;
                index < detector_diagnostics_.support_failure_depth_octiles.size();
                ++index)
            {
                status.values.push_back(numericValue(
                        "support_failure_depth_octile_" + std::to_string(index),
                        detector_diagnostics_.support_failure_depth_octiles[index]));
            }
            for(std::size_t index = 0U;
                index < detector_diagnostics_.support_failure_lateral_bins.size();
                ++index)
            {
                status.values.push_back(numericValue(
                        "support_failure_lateral_bin_" + std::to_string(index),
                        detector_diagnostics_.support_failure_lateral_bins[index]));
            }
            for(std::size_t index = 0U;
                index < detector_diagnostics_.support_failure_vertical_bins.size();
                ++index)
            {
                status.values.push_back(numericValue(
                        "support_failure_vertical_bin_" + std::to_string(index),
                        detector_diagnostics_.support_failure_vertical_bins[index]));
            }
            status.values.push_back(numericValue(
                    "components_built", detector_diagnostics_.components_built));
            for(std::size_t index = 0U;
                index < detector_diagnostics_.component_size_buckets.size();
                ++index)
            {
                status.values.push_back(numericValue(
                        "component_size_bucket_" + std::to_string(index),
                        detector_diagnostics_.component_size_buckets[index]));
            }
            status.values.push_back(numericValue(
                    "component_primary_rejected_columns",
                    detector_diagnostics_.component_primary_rejected_columns));
            status.values.push_back(numericValue(
                    "component_primary_rejected_area",
                    detector_diagnostics_.component_primary_rejected_area));
            status.values.push_back(numericValue(
                    "component_primary_rejected_span",
                    detector_diagnostics_.component_primary_rejected_span));
            status.values.push_back(numericValue(
                    "component_primary_rejected_direction",
                    detector_diagnostics_.component_primary_rejected_direction));
            status.values.push_back(numericValue(
                    "components_accepted", detector_diagnostics_.components_accepted));
            status.values.push_back(numericValue(
                    "detector_leaf_scan_seconds",
                    detector_diagnostics_.timings.leaf_scan_seconds));
            status.values.push_back(numericValue(
                    "detector_vertical_seconds",
                    detector_diagnostics_.timings.vertical_seconds));
            status.values.push_back(numericValue(
                    "detector_support_seconds",
                    detector_diagnostics_.timings.support_seconds));
            status.values.push_back(numericValue(
                    "detector_component_seconds",
                    detector_diagnostics_.timings.component_seconds));
            status.values.push_back(numericValue(
                    "detector_total_seconds",
                    detector_diagnostics_.timings.total_seconds));
            status.values.push_back(numericValue(
                    "detected_regions", detected_region_count_));
            status.values.push_back(numericValue("tracked_regions", last_result_.tracks.size()));
            status.values.push_back(numericValue("eligible_edges", last_result_.eligible_edges));
            status.values.push_back(numericValue(
                    "matching_cardinality", last_result_.matching_cardinality));
            status.values.push_back(value("detector_reason", detector_reason_));
            for(const auto & assignment : last_result_.assignments) {
                status.values.push_back(value(
                        assignment.drone_id + ".mode",
                        std::to_string(static_cast<int>(assignment.mode))));
                status.values.push_back(numericValue(
                        assignment.drone_id + ".task_id", assignment.task_id));
                status.values.push_back(numericValue(
                        assignment.drone_id + ".revision", assignment.revision));
            }
            message.status.push_back(std::move(status));
            diagnostics_publisher_->publish(message);
        }

        void publishMarkers(const GlobalAllocationResult & result)
        {
            visualization_msgs::msg::MarkerArray array;
            visualization_msgs::msg::Marker regions;
            regions.header.frame_id = map_frame_;
            regions.header.stamp = get_clock()->now();
            regions.ns = "global_frontier_regions";
            regions.id = 0;
            regions.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            regions.action = visualization_msgs::msg::Marker::ADD;
            regions.pose.orientation.w = 1.0;
            regions.scale.x = regions.scale.y = regions.scale.z = 0.35;
            regions.color.r = 0.1F;
            regions.color.g = 0.9F;
            regions.color.b = 0.4F;
            regions.color.a = 0.9F;
            for(const auto & track : result.tracks) {
                if(track.stable) {
                    regions.points.push_back(point(track.region.representative));
                }
            }
            array.markers.push_back(std::move(regions));

            visualization_msgs::msg::Marker tasks;
            tasks.header.frame_id = map_frame_;
            tasks.header.stamp = get_clock()->now();
            tasks.ns = "global_task_assignments";
            tasks.id = 0;
            tasks.type = visualization_msgs::msg::Marker::LINE_LIST;
            tasks.action = visualization_msgs::msg::Marker::ADD;
            tasks.pose.orientation.w = 1.0;
            tasks.scale.x = 0.06;
            tasks.color.r = 0.2F;
            tasks.color.g = 0.65F;
            tasks.color.b = 1.0F;
            tasks.color.a = 0.9F;
            std::lock_guard<std::mutex> lock(mutex_);
            for(const auto & assignment : result.assignments) {
                if(assignment.mode != ExplorationTaskMode::Assigned) {
                    continue;
                }
                const auto found = std::find_if(
                        drones_.begin(), drones_.end(), [&](const DroneTrack & track) {
                            return track.id == assignment.drone_id;
                        });
                if(found == drones_.end() || !found->has_odom) {
                    continue;
                }
                Pose3D pose;
                if(transformOdometry(found->odom, pose)) {
                    tasks.points.push_back(point(pose.position));
                    tasks.points.push_back(point(assignment.target));
                }
            }
            array.markers.push_back(std::move(tasks));
            marker_publisher_->publish(array);
        }

        std::string map_frame_ {"map"};
        std::vector<std::string> drone_namespaces_;
        double allocation_rate_hz_ {1.0};
        double task_lease_seconds_ {3.0};
        double global_map_timeout_seconds_ {5.0};
        double diagnostics_timeout_seconds_ {5.0};
        double odom_timeout_seconds_ {2.0};
        double local_map_timeout_seconds_ {5.0};
        GlobalFrontierDetectorConfig detector_config_;
        GlobalTaskAllocatorConfig allocator_config_;
        std::unique_ptr<GlobalFrontierDetector> detector_;
        std::unique_ptr<GlobalTaskAllocator> allocator_;
        SteadyClock::time_point steady_start_;
        std::uint64_t allocator_epoch_ {};
        std::mutex mutex_;
        std::vector<DroneTrack> drones_;
        octomap_msgs::msg::Octomap::SharedPtr pending_global_map_;
        std::int64_t pending_global_stamp_ns_ {};
        std::shared_ptr<octomap::OcTree> global_map_;
        std::int64_t global_map_stamp_ns_ {};
        double global_map_receive_seconds_ {-1.0};
        bool global_diagnostics_healthy_ {false};
        std::int64_t global_diagnostics_stamp_ns_ {};
        double global_diagnostics_receive_seconds_ {-1.0};
        std::vector<FrontierRegion> regions_;
        std::uint64_t global_update_sequence_ {};
        std::uint64_t global_map_received_count_ {};
        std::uint64_t global_map_same_stamp_count_ {};
        std::uint64_t global_map_regressed_stamp_count_ {};
        std::size_t raw_frontier_columns_ {};
        std::size_t scanned_free_voxels_ {};
        std::size_t supported_frontier_columns_ {};
        std::size_t vertical_rejected_columns_ {};
        std::size_t support_rejected_columns_ {};
        std::size_t detected_region_count_ {};
        FrontierDetectionStatus detector_status_ {FrontierDetectionStatus::Invalid};
        FrontierDetectionDiagnostics detector_diagnostics_;
        std::string detector_reason_;
        GlobalAllocationResult last_result_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr global_map_subscription_;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                global_diagnostics_subscription_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                diagnostics_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

}// namespace SwarmController

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmController::GlobalTaskAllocatorNode>());
    rclcpp::shutdown();
    return 0;
}

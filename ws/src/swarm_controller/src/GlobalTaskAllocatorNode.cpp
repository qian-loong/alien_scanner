#include "swarm_controller/GlobalFrontierDetector.hpp"
#include "swarm_controller/GlobalTaskAllocator.hpp"
#include "swarm_controller/LatestSnapshotSlot.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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

        std::optional<std::int64_t> positiveStampNanoseconds(
                const builtin_interfaces::msg::Time & stamp)
        {
            if(stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
                return {};
            }
            const std::int64_t nanoseconds =
                    static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL
                    + static_cast<std::int64_t>(stamp.nanosec);
            if(nanoseconds <= 0) {
                return {};
            }
            return nanoseconds;
        }

        tf2::TimePoint timePointFromNanoseconds(const std::int64_t stamp_ns)
        {
            return tf2::TimePoint(std::chrono::nanoseconds(stamp_ns));
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
        enum class MapProcessingStatus {
            Accepted,
            Invalid,
            ResourceLimit,
        };

        struct DecodedMapResult {
            MapProcessingStatus status {MapProcessingStatus::Invalid};
            std::shared_ptr<octomap::OcTree> map;
            std::size_t leaf_count {};
            std::uint64_t input_sequence {};
            std::string reason;
        };

        struct MapProcessingResult {
            MapProcessingStatus status {MapProcessingStatus::Invalid};
            std::shared_ptr<octomap::OcTree> map;
            FrontierDetectionResult detection;
            std::size_t leaf_count {};
            std::uint64_t input_sequence {};
            std::string reason;

            bool accepted() const
            {
                return status == MapProcessingStatus::Accepted;
            }
        };

        DecodedMapResult decodeMap(
                const octomap_msgs::msg::Octomap & message,
                const std::string & map_frame, const double resolution,
                const std::size_t max_serialized_bytes,
                const std::size_t max_voxels)
        {
            DecodedMapResult result;
            if(message.header.frame_id != map_frame) {
                result.reason = "map frame mismatch";
                return result;
            }
            if(message.binary) {
                result.reason = "binary Octomap is not supported";
                return result;
            }
            if(message.data.size() > max_serialized_bytes) {
                result.status = MapProcessingStatus::ResourceLimit;
                result.reason = "serialized map exceeds byte limit";
                return result;
            }
            try {
                std::unique_ptr<octomap::AbstractOcTree> abstract(
                        octomap_msgs::fullMsgToMap(message));
                auto * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
                if(tree == nullptr) {
                    result.reason = "message does not contain an OcTree";
                    return result;
                }
                if(std::fabs(tree->getResolution() - resolution) > 1.0e-5) {
                    result.reason = "Octomap resolution mismatch";
                    return result;
                }
                result.leaf_count = tree->getNumLeafNodes();
                if(result.leaf_count > max_voxels) {
                    result.status = MapProcessingStatus::ResourceLimit;
                    result.reason = "decoded map exceeds leaf voxel limit";
                    return result;
                }
                abstract.release();
                result.map = std::shared_ptr<octomap::OcTree>(tree);
                result.status = MapProcessingStatus::Accepted;
                return result;
            } catch(const std::bad_alloc &) {
                result.status = MapProcessingStatus::ResourceLimit;
                result.reason = "Octomap deserialization allocation exceeded resource limit";
                return result;
            } catch(const std::exception & exception) {
                result.reason = std::string("Octomap deserialization failed: ")
                                + exception.what();
                return result;
            } catch(...) {
                result.reason = "Octomap deserialization failed with an unknown exception";
                return result;
            }
        }

    }// namespace

    class GlobalTaskAllocatorNode final : public rclcpp::Node
    {
    public:
        GlobalTaskAllocatorNode()
            : Node("global_task_allocator")
            , steady_start_(SteadyClock::now())
            , allocator_epoch_(makeAllocatorEpoch())
            , global_snapshot_slot_(SnapshotStampPolicy::NonDecreasing)
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
            worker_ = std::thread([this]() { workerLoop(); });
            RCLCPP_INFO(
                    get_logger(), "global_task_allocator: %zu drones at %.2f Hz epoch=%llu",
                    drones_.size(), allocation_rate_hz_,
                    static_cast<unsigned long long>(allocator_epoch_));
        }

        ~GlobalTaskAllocatorNode() override
        {
            stopWorker();
        }

    private:
        using MapMessage = octomap_msgs::msg::Octomap;

        struct MapSnapshot {
            MapMessage::SharedPtr message;
            std::uint64_t input_sequence {};
        };

        using GlobalSnapshotSlot = LatestSnapshotSlot<
                MapSnapshot, MapProcessingResult>;
        using LocalSnapshotSlot = LatestSnapshotSlot<
                MapSnapshot, DecodedMapResult>;

        struct DroneTrack {
            std::string id;
            std::string prefix;
            LocalSnapshotSlot snapshot_slot {SnapshotStampPolicy::StrictlyIncreasing};
            std::shared_ptr<octomap::OcTree> local_map;
            std::int64_t local_map_stamp_ns {};
            double local_map_receive_seconds {-1.0};
            std::uint64_t local_map_applied_revision {};
            std::uint64_t local_map_received_count {};
            std::uint64_t local_map_invalid_envelope_count {};
            std::uint64_t local_map_regressed_count {};
            std::uint64_t local_map_duplicate_count {};
            std::uint64_t local_map_resource_rejection_count {};
            std::uint64_t local_map_failure_count {};
            std::size_t local_map_leaf_count {};
            std::uint64_t local_map_input_reason_sequence {};
            std::string local_map_reason;
            nav_msgs::msg::Odometry odom;
            std::int64_t odom_stamp_ns {};
            bool has_odom {false};
            double odom_receive_seconds {-1.0};
            std::uint64_t odom_invalid_stamp_count {};
            std::string odom_reason;
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
            declare_parameter(
                    "global_map.max_serialized_bytes_per_source",
                    std::int64_t {67'108'864});
            declare_parameter(
                    "global_map.max_voxels_per_source",
                    std::int64_t {5'000'000});
            declare_parameter(
                    "global_map.max_global_voxels",
                    std::int64_t {10'000'000});
            declare_parameter("drone.odom_timeout", 2.0);
            declare_parameter("drone.local_map_timeout", 5.0);

            declare_parameter("resolution", 0.1);
            declare_parameter("frontier.column_stride_voxels", 2);
            declare_parameter("frontier.min_z_layers", 5);
            declare_parameter("frontier.min_z_span", 0.4);
            declare_parameter("frontier.support_depth", 0.8);
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
            if(static_cast<std::uint64_t>(value)
               > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
            {
                throw std::invalid_argument(std::string(name) + " exceeds size_t range");
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

        static std::size_t checkedMultiply(
                const std::size_t lhs, const std::size_t rhs,
                const char * name)
        {
            if(lhs != 0U && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
                throw std::invalid_argument(std::string(name) + " overflows size_t");
            }
            return lhs * rhs;
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
            max_serialized_bytes_per_source_ = positiveSize(
                    get_parameter("global_map.max_serialized_bytes_per_source").as_int(),
                    "global_map.max_serialized_bytes_per_source");
            max_voxels_per_source_ = positiveSize(
                    get_parameter("global_map.max_voxels_per_source").as_int(),
                    "global_map.max_voxels_per_source");
            max_global_voxels_ = positiveSize(
                    get_parameter("global_map.max_global_voxels").as_int(),
                    "global_map.max_global_voxels");
            max_global_serialized_bytes_ = checkedMultiply(
                    drone_namespaces_.size(), max_serialized_bytes_per_source_,
                    "drone count * global_map.max_serialized_bytes_per_source");
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
            detector_config_.support_depth =
                    get_parameter("frontier.support_depth").as_double();
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
                            bool notify = false;
                            const std::int64_t now_ros_ns =
                                    get_clock()->now().nanoseconds();
                            {
                                std::lock_guard<std::mutex> lock(mutex_);
                                if(stopping_ || message == nullptr) {
                                    return;
                                }
                                observeRosClockLocked(now_ros_ns);
                                DroneTrack & track = drones_.at(index);
                                ++track.local_map_received_count;
                                const std::uint64_t input_sequence =
                                        track.local_map_received_count;
                                const auto stamp = positiveStampNanoseconds(
                                        message->header.stamp);
                                if(!stamp.has_value() || *stamp > now_ros_ns
                                   || message->data.size() > max_serialized_bytes_per_source_)
                                {
                                    ++track.local_map_invalid_envelope_count;
                                    if(message->data.size() > max_serialized_bytes_per_source_) {
                                        ++track.local_map_resource_rejection_count;
                                        track.local_map_reason =
                                                "serialized map exceeds byte limit";
                                    } else if(!stamp.has_value()) {
                                        track.local_map_reason =
                                                "observation stamp must be positive";
                                    } else if(*stamp > now_ros_ns) {
                                        track.local_map_reason =
                                                "observation stamp is in the future";
                                    } else {
                                        track.local_map_reason = "invalid map envelope";
                                    }
                                    ++track.local_map_failure_count;
                                    track.local_map_input_reason_sequence = input_sequence;
                                    return;
                                }
                                try {
                                    const SnapshotSubmitStatus status =
                                            track.snapshot_slot.submit(
                                                    MapSnapshot {message, input_sequence},
                                                    *stamp, monotonicNow());
                                    if(status == SnapshotSubmitStatus::RejectedRegressed) {
                                        ++track.local_map_regressed_count;
                                    } else if(status
                                              == SnapshotSubmitStatus::RejectedDuplicate)
                                    {
                                        ++track.local_map_duplicate_count;
                                    } else {
                                        notify = true;
                                    }
                                } catch(const std::exception & exception) {
                                    ++track.local_map_failure_count;
                                    track.local_map_input_reason_sequence = input_sequence;
                                    track.local_map_reason =
                                            std::string("local snapshot admission failed: ")
                                            + exception.what();
                                } catch(...) {
                                    ++track.local_map_failure_count;
                                    track.local_map_input_reason_sequence = input_sequence;
                                    track.local_map_reason =
                                            "local snapshot admission failed with an unknown exception";
                                }
                            }
                            if(notify) {
                                worker_cv_.notify_one();
                            }
                        });
                drones_[index].odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
                        drones_[index].prefix + "/odom", rclcpp::QoS(10),
                        [this, index](const nav_msgs::msg::Odometry::SharedPtr message) {
                            const std::int64_t now_ros_ns =
                                    get_clock()->now().nanoseconds();
                            std::lock_guard<std::mutex> lock(mutex_);
                            if(stopping_ || message == nullptr) {
                                return;
                            }
                            observeRosClockLocked(now_ros_ns);
                            DroneTrack & track = drones_[index];
                            const auto stamp = positiveStampNanoseconds(
                                    message->header.stamp);
                            if(!stamp.has_value() || *stamp > now_ros_ns) {
                                ++track.odom_invalid_stamp_count;
                                track.odom_reason = !stamp.has_value()
                                                            ? "observation stamp must be positive"
                                                            : "observation stamp is in the future";
                                return;
                            }
                            track.odom = *message;
                            track.odom_stamp_ns = *stamp;
                            track.has_odom = true;
                            track.odom_receive_seconds = monotonicNow();
                            track.odom_reason.clear();
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
                        bool notify = false;
                        const std::int64_t now_ros_ns =
                                get_clock()->now().nanoseconds();
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if(stopping_ || message == nullptr) {
                                return;
                            }
                            observeRosClockLocked(now_ros_ns);
                            const auto stamp = positiveStampNanoseconds(
                                    message->header.stamp);
                            ++global_map_received_count_;
                            const std::uint64_t input_sequence =
                                    global_map_received_count_;
                            if(!stamp.has_value() || *stamp > now_ros_ns
                               || message->data.size() > max_global_serialized_bytes_)
                            {
                                ++global_map_invalid_envelope_count_;
                                if(message->data.size() > max_global_serialized_bytes_) {
                                    ++global_map_resource_rejection_count_;
                                    global_map_input_reason_ =
                                            "serialized map exceeds byte limit";
                                } else if(!stamp.has_value()) {
                                    global_map_input_reason_ =
                                            "observation stamp must be positive";
                                } else if(*stamp > now_ros_ns) {
                                    global_map_input_reason_ =
                                            "observation stamp is in the future";
                                } else {
                                    global_map_input_reason_ = "invalid map envelope";
                                }
                                global_map_input_reason_sequence_ = input_sequence;
                                return;
                            }
                            try {
                                const SnapshotSubmitStatus status =
                                        global_snapshot_slot_.submit(
                                                MapSnapshot {message, input_sequence},
                                                *stamp, monotonicNow());
                                if(status == SnapshotSubmitStatus::RejectedRegressed) {
                                    ++global_map_regressed_stamp_count_;
                                } else {
                                    if(status == SnapshotSubmitStatus::SameStampAccepted) {
                                        ++global_map_same_stamp_count_;
                                    }
                                    notify = true;
                                }
                            } catch(const std::exception & exception) {
                                ++global_map_worker_failure_count_;
                                global_map_input_reason_sequence_ = input_sequence;
                                global_map_input_reason_ =
                                        std::string("global snapshot admission failed: ")
                                        + exception.what();
                            } catch(...) {
                                ++global_map_worker_failure_count_;
                                global_map_input_reason_sequence_ = input_sequence;
                                global_map_input_reason_ =
                                        "global snapshot admission failed with an unknown exception";
                            }
                        }
                        if(notify) {
                            worker_cv_.notify_one();
                        }
                    });
            global_diagnostics_subscription_ = create_subscription<
                    diagnostic_msgs::msg::DiagnosticArray>(
                    "/global_map_diagnostics", qos,
                    [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
                        if(message == nullptr) {
                            return;
                        }
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
                        const std::int64_t now_ros_ns =
                                get_clock()->now().nanoseconds();
                        std::lock_guard<std::mutex> lock(mutex_);
                        if(stopping_) {
                            return;
                        }
                        observeRosClockLocked(now_ros_ns);
                        const auto stamp = positiveStampNanoseconds(
                                message->header.stamp);
                        if(!stamp.has_value() || *stamp > now_ros_ns) {
                            ++global_diagnostics_invalid_stamp_count_;
                            global_diagnostics_healthy_ = false;
                            global_diagnostics_reason_ = !stamp.has_value()
                                                                 ? "observation stamp must be positive"
                                                                 : "observation stamp is in the future";
                            return;
                        }
                        global_diagnostics_healthy_ = healthy;
                        global_diagnostics_stamp_ns_ = *stamp;
                        global_diagnostics_receive_seconds_ = monotonicNow();
                        global_diagnostics_reason_.clear();
                    });
        }

        void observeRosClockLocked(const std::int64_t now_ros_ns)
        {
            if(now_ros_ns <= 0) {
                return;
            }
            if(last_ros_now_ns_ > 0 && now_ros_ns < last_ros_now_ns_) {
                ++ros_clock_reset_count_;
                global_snapshot_slot_.resetStampWatermark();
                global_map_.reset();
                global_map_stamp_ns_ = 0;
                global_map_receive_seconds_ = -1.0;
                global_map_applied_revision_ = 0U;
                global_map_leaf_count_ = 0U;
                regions_.clear();
                global_map_input_reason_sequence_ = global_map_received_count_;
                global_map_input_reason_ =
                        "ROS clock moved backwards; waiting for a fresh global map";
                global_diagnostics_healthy_ = false;
                global_diagnostics_stamp_ns_ = 0;
                global_diagnostics_receive_seconds_ = -1.0;
                global_diagnostics_reason_ =
                        "ROS clock moved backwards; waiting for fresh diagnostics";
                for(DroneTrack & track : drones_) {
                    track.snapshot_slot.resetStampWatermark();
                    track.local_map.reset();
                    track.local_map_stamp_ns = 0;
                    track.local_map_receive_seconds = -1.0;
                    track.local_map_applied_revision = 0U;
                    track.local_map_leaf_count = 0U;
                    track.local_map_input_reason_sequence =
                            track.local_map_received_count;
                    track.local_map_reason =
                            "ROS clock moved backwards; waiting for a fresh local map";
                    track.has_odom = false;
                    track.odom_stamp_ns = 0;
                    track.odom_receive_seconds = -1.0;
                    track.odom_reason =
                            "ROS clock moved backwards; waiting for fresh odometry";
                }
            }
            last_ros_now_ns_ = now_ros_ns;
        }

        double monotonicNow() const
        {
            return std::chrono::duration<double>(SteadyClock::now() - steady_start_).count();
        }

        bool transformOdometry(
                const nav_msgs::msg::Odometry & odom,
                const std::int64_t stamp_ns, Pose3D & pose)
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
                            timePointFromNanoseconds(stamp_ns), tf2::durationFromSec(0.0));
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

        MapProcessingResult processGlobalMap(const MapMessage & message)
        {
            MapProcessingResult result;
            const DecodedMapResult decoded = decodeMap(
                    message, map_frame_, detector_config_.resolution,
                    max_global_serialized_bytes_, max_global_voxels_);
            result.status = decoded.status;
            result.map = decoded.map;
            result.leaf_count = decoded.leaf_count;
            result.reason = decoded.reason;
            if(!decoded.map) {
                result.detection.status =
                        decoded.status == MapProcessingStatus::ResourceLimit
                                ? FrontierDetectionStatus::ResourceLimit
                                : FrontierDetectionStatus::Invalid;
                result.detection.reason = decoded.reason;
                return result;
            }
            try {
                result.detection = detector_->detect(*decoded.map);
                result.status = result.detection.accepted()
                                        ? MapProcessingStatus::Accepted
                                        : (result.detection.status
                                                           == FrontierDetectionStatus::ResourceLimit
                                                   ? MapProcessingStatus::ResourceLimit
                                                   : MapProcessingStatus::Invalid);
                result.reason = result.detection.reason;
                if(!result.detection.accepted() && result.reason.empty()) {
                    result.reason = "frontier detector rejected the decoded map";
                    result.detection.reason = result.reason;
                }
            } catch(const std::bad_alloc &) {
                result.status = MapProcessingStatus::ResourceLimit;
                result.reason = "frontier detection allocation exceeded resource limit";
                result.detection.status = FrontierDetectionStatus::ResourceLimit;
                result.detection.reason = result.reason;
                result.map.reset();
            } catch(const std::exception & exception) {
                result.status = MapProcessingStatus::Invalid;
                result.reason = std::string("frontier detection failed: ")
                                + exception.what();
                result.detection.status = FrontierDetectionStatus::Invalid;
                result.detection.reason = result.reason;
                result.map.reset();
            } catch(...) {
                result.status = MapProcessingStatus::Invalid;
                result.reason = "frontier detection failed with an unknown exception";
                result.detection.status = FrontierDetectionStatus::Invalid;
                result.detection.reason = result.reason;
                result.map.reset();
            }
            return result;
        }

        DecodedMapResult processLocalMap(const MapMessage & message)
        {
            return decodeMap(
                    message, map_frame_, detector_config_.resolution,
                    max_serialized_bytes_per_source_, max_voxels_per_source_);
        }

        bool workerHasWorkLocked() const
        {
            if(global_snapshot_slot_.hasProcessablePending()) {
                return true;
            }
            return std::any_of(
                    drones_.begin(), drones_.end(), [](const DroneTrack & track) {
                        return track.snapshot_slot.hasProcessablePending();
                    });
        }

        void workerLoop()
        {
            while(true) {
                std::optional<GlobalSnapshotSlot::PendingItem> global_pending;
                std::optional<LocalSnapshotSlot::PendingItem> local_pending;
                std::size_t local_index = 0U;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    worker_cv_.wait(lock, [this]() {
                        return stopping_ || workerHasWorkLocked();
                    });
                    if(stopping_) {
                        return;
                    }
                    const std::size_t source_count = drones_.size() + 1U;
                    for(std::size_t offset = 0U; offset < source_count; ++offset) {
                        const std::size_t source =
                                (worker_next_source_ + offset) % source_count;
                        if(source == 0U
                           && global_snapshot_slot_.hasProcessablePending())
                        {
                            global_pending = global_snapshot_slot_.takePending();
                            worker_next_source_ = 1U % source_count;
                            break;
                        }
                        if(source > 0U
                           && drones_[source - 1U]
                                      .snapshot_slot.hasProcessablePending())
                        {
                            local_index = source - 1U;
                            local_pending = drones_[local_index].snapshot_slot.takePending();
                            worker_next_source_ = (source + 1U) % source_count;
                            break;
                        }
                    }
                }

                if(global_pending.has_value()) {
                    MapProcessingResult result;
                    try {
                        result = processGlobalMap(*global_pending->payload.message);
                    } catch(const std::exception & exception) {
                        result.status = MapProcessingStatus::Invalid;
                        result.reason = std::string("global map worker failed: ")
                                        + exception.what();
                    } catch(...) {
                        result.status = MapProcessingStatus::Invalid;
                        result.reason = "global map worker failed with an unknown exception";
                    }
                    result.input_sequence = global_pending->payload.input_sequence;
                    const std::uint64_t input_sequence = result.input_sequence;
                    std::lock_guard<std::mutex> lock(mutex_);
                    if(stopping_) {
                        global_snapshot_slot_.clear();
                        return;
                    }
                    try {
                        global_snapshot_slot_.publishResult(
                                global_pending->revision, global_pending->stamp_ns,
                                global_pending->received_seconds, std::move(result));
                    } catch(const std::exception & exception) {
                        ++global_map_worker_failure_count_;
                        if(input_sequence >= global_map_input_reason_sequence_) {
                            global_map_input_reason_sequence_ = input_sequence;
                            global_map_input_reason_ =
                                    std::string("global result handoff failed: ")
                                    + exception.what();
                        }
                        global_snapshot_slot_.clear();
                    }
                    worker_cv_.notify_one();
                    continue;
                }

                if(local_pending.has_value()) {
                    DecodedMapResult result;
                    try {
                        result = processLocalMap(*local_pending->payload.message);
                    } catch(const std::exception & exception) {
                        result.reason = std::string("local map worker failed: ")
                                        + exception.what();
                    } catch(...) {
                        result.reason = "local map worker failed with an unknown exception";
                    }
                    result.input_sequence = local_pending->payload.input_sequence;
                    const std::uint64_t input_sequence = result.input_sequence;
                    std::lock_guard<std::mutex> lock(mutex_);
                    if(stopping_) {
                        drones_[local_index].snapshot_slot.clear();
                        return;
                    }
                    try {
                        drones_[local_index].snapshot_slot.publishResult(
                                local_pending->revision, local_pending->stamp_ns,
                                local_pending->received_seconds, std::move(result));
                    } catch(const std::exception & exception) {
                        DroneTrack & track = drones_[local_index];
                        ++track.local_map_failure_count;
                        if(input_sequence >= track.local_map_input_reason_sequence) {
                            track.local_map_input_reason_sequence = input_sequence;
                            track.local_map_reason =
                                    std::string("local result handoff failed: ")
                                    + exception.what();
                        }
                        track.snapshot_slot.clear();
                    }
                    worker_cv_.notify_one();
                }
            }
        }

        void applyReadyMaps()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) {
                return;
            }

            if(auto ready = global_snapshot_slot_.claimReady(); ready.has_value()) {
                const std::uint64_t revision = ready->revision;
                bool applied = false;
                try {
                    MapProcessingResult & processed = ready->payload;
                    detector_status_ = processed.detection.status;
                    detector_diagnostics_ = processed.detection.diagnostics;
                    vertical_rejected_columns_ = processed.detection.vertical_rejected_columns;
                    scanned_free_voxels_ = processed.detection.scanned_free_voxels;
                    support_rejected_columns_ = processed.detection.support_rejected_columns;
                    raw_frontier_columns_ = processed.detection.raw_columns;
                    supported_frontier_columns_ = processed.detection.supported_columns;
                    detected_region_count_ = processed.detection.regions.size();
                    detector_reason_ = processed.reason;
                    if(processed.accepted() && processed.detection.accepted()) {
                        if(global_update_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
                            throw std::overflow_error("global update sequence exhausted");
                        }
                        global_map_ = processed.map;
                        global_map_stamp_ns_ = ready->stamp_ns;
                        global_map_receive_seconds_ = ready->received_seconds;
                        regions_ = std::move(processed.detection.regions);
                        ++global_update_sequence_;
                        detector_reason_.clear();
                        if(processed.input_sequence
                           >= global_map_input_reason_sequence_)
                        {
                            global_map_input_reason_.clear();
                            global_map_input_reason_sequence_ = 0U;
                        }
                        global_map_applied_revision_ = revision;
                        global_map_leaf_count_ = processed.leaf_count;
                        applied = true;
                    } else {
                        ++global_map_processing_failure_count_;
                        if(processed.status == MapProcessingStatus::ResourceLimit) {
                            ++global_map_resource_rejection_count_;
                        }
                        if(processed.input_sequence
                           >= global_map_input_reason_sequence_)
                        {
                            global_map_input_reason_sequence_ = processed.input_sequence;
                            global_map_input_reason_ = processed.reason;
                        }
                    }
                } catch(const std::exception & exception) {
                    ++global_map_worker_failure_count_;
                    if(ready->payload.input_sequence
                       >= global_map_input_reason_sequence_)
                    {
                        global_map_input_reason_sequence_ =
                                ready->payload.input_sequence;
                        global_map_input_reason_ =
                                std::string("global result apply failed: ")
                                + exception.what();
                    }
                } catch(...) {
                    ++global_map_worker_failure_count_;
                    if(ready->payload.input_sequence
                       >= global_map_input_reason_sequence_)
                    {
                        global_map_input_reason_sequence_ =
                                ready->payload.input_sequence;
                        global_map_input_reason_ =
                                "global result apply failed with an unknown exception";
                    }
                }
                if(!global_snapshot_slot_.acknowledgeReady(revision, applied)) {
                    ++global_map_worker_failure_count_;
                    global_map_input_reason_ =
                            "global result acknowledgement failed";
                    global_map_input_reason_sequence_ =
                            std::max(
                                    global_map_input_reason_sequence_,
                                    ready->payload.input_sequence);
                    global_snapshot_slot_.clear();
                }
            }

            for(std::size_t index = 0U; index < drones_.size(); ++index) {
                auto ready = drones_[index].snapshot_slot.claimReady();
                if(!ready.has_value()) {
                    continue;
                }
                DroneTrack & track = drones_[index];
                const std::uint64_t revision = ready->revision;
                bool applied = false;
                try {
                    const DecodedMapResult & processed = ready->payload;
                    if(processed.status == MapProcessingStatus::Accepted && processed.map) {
                        track.local_map = processed.map;
                        track.local_map_stamp_ns = ready->stamp_ns;
                        track.local_map_receive_seconds = ready->received_seconds;
                        track.local_map_applied_revision = revision;
                        track.local_map_leaf_count = processed.leaf_count;
                        if(processed.input_sequence
                           >= track.local_map_input_reason_sequence)
                        {
                            track.local_map_reason.clear();
                            track.local_map_input_reason_sequence = 0U;
                        }
                        applied = true;
                    } else {
                        ++track.local_map_failure_count;
                        if(processed.status == MapProcessingStatus::ResourceLimit) {
                            ++track.local_map_resource_rejection_count;
                        }
                        if(processed.input_sequence
                           >= track.local_map_input_reason_sequence)
                        {
                            track.local_map_input_reason_sequence =
                                    processed.input_sequence;
                            track.local_map_reason = processed.reason;
                        }
                    }
                } catch(const std::exception & exception) {
                    ++track.local_map_failure_count;
                    if(ready->payload.input_sequence
                       >= track.local_map_input_reason_sequence)
                    {
                        track.local_map_input_reason_sequence =
                                ready->payload.input_sequence;
                        track.local_map_reason =
                                std::string("local result apply failed: ")
                                + exception.what();
                    }
                } catch(...) {
                    ++track.local_map_failure_count;
                    if(ready->payload.input_sequence
                       >= track.local_map_input_reason_sequence)
                    {
                        track.local_map_input_reason_sequence =
                                ready->payload.input_sequence;
                        track.local_map_reason =
                                "local result apply failed with an unknown exception";
                    }
                }
                if(!track.snapshot_slot.acknowledgeReady(revision, applied)) {
                    ++track.local_map_failure_count;
                    track.local_map_reason = "local result acknowledgement failed";
                    track.local_map_input_reason_sequence =
                            std::max(
                                    track.local_map_input_reason_sequence,
                                    ready->payload.input_sequence);
                    track.snapshot_slot.clear();
                }
            }
            worker_cv_.notify_one();
        }

        void onTimer()
        {
            const std::int64_t now_ros_ns = get_clock()->now().nanoseconds();
            const double now = monotonicNow();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(stopping_) {
                    return;
                }
                observeRosClockLocked(now_ros_ns);
            }
            applyReadyMaps();
            GlobalAllocationInput input;
            input.monotonic_time_seconds = now;
            bool diagnostics_healthy = false;
            bool global_fresh = false;
            std::vector<std::shared_ptr<octomap::OcTree>> local_map_holders;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(stopping_) {
                    return;
                }
                input.regions = regions_;
                input.global_update_sequence = global_update_sequence_;
                diagnostics_healthy = global_diagnostics_healthy_
                                      && global_diagnostics_receive_seconds_ >= 0.0
                                      && now - global_diagnostics_receive_seconds_
                                                 <= diagnostics_timeout_seconds_
                                      && freshStamp(
                                              global_diagnostics_stamp_ns_, now_ros_ns,
                                              diagnostics_timeout_seconds_);
                global_fresh = global_map_ != nullptr
                               && global_map_receive_seconds_ >= 0.0
                               && now - global_map_receive_seconds_
                                          <= global_map_timeout_seconds_
                               && freshStamp(
                                       global_map_stamp_ns_, now_ros_ns,
                                       global_map_timeout_seconds_);
                input.drones.reserve(drones_.size());
                local_map_holders.reserve(drones_.size());
                for(const DroneTrack & track : drones_) {
                    local_map_holders.push_back(track.local_map);
                    DroneAllocationState state;
                    state.id = track.id;
                    state.local_map = local_map_holders.back().get();
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
                                               track.odom_stamp_ns,
                                               now_ros_ns, odom_timeout_seconds_);
                    if(state.odom_fresh
                       && !transformOdometry(
                               track.odom, track.odom_stamp_ns, state.pose))
                    {
                        state.odom_fresh = false;
                    }
                    input.drones.push_back(state);
                }
                input.healthy = global_fresh && diagnostics_healthy
                                && detector_reason_.empty()
                                && global_map_input_reason_.empty();
            }
            last_result_ = allocator_->update(input);
            publishTasks(last_result_);
            publishDiagnostics(input, global_fresh, diagnostics_healthy);
            publishMarkers(last_result_);
        }

        void stopWorker()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(stopping_) {
                    return;
                }
                stopping_ = true;
            }
            if(timer_) {
                timer_->cancel();
            }
            global_map_subscription_.reset();
            global_diagnostics_subscription_.reset();
            for(DroneTrack & track : drones_) {
                track.map_subscription.reset();
                track.odom_subscription.reset();
            }
            worker_cv_.notify_all();
            if(worker_.joinable()) {
                worker_.join();
            }
        }

        void publishTasks(const GlobalAllocationResult & result)
        {
            const rclcpp::Time now = get_clock()->now();
            const std::int64_t lease_ns = static_cast<std::int64_t>(
                    std::llround(task_lease_seconds_ * 1.0e9));
            std::lock_guard<std::mutex> lock(mutex_);
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
                const GlobalAllocationInput & input,
                bool global_fresh, bool diagnostics_healthy)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(stopping_) {
                return;
            }
            diagnostic_msgs::msg::DiagnosticArray message;
            message.header.stamp = get_clock()->now();
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = "global_task_allocator";
            status.hardware_id = "swarm";
            status.level = input.healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK
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
            status.values.push_back(numericValue(
                    "global_map_diagnostics_invalid_stamp",
                    global_diagnostics_invalid_stamp_count_));
            status.values.push_back(value(
                    "global_map_diagnostics_reason", global_diagnostics_reason_));
            status.values.push_back(numericValue(
                    "ros_clock_resets", ros_clock_reset_count_));
            status.values.push_back(numericValue("global_update_sequence", global_update_sequence_));
            status.values.push_back(numericValue(
                    "global_map_received", global_map_received_count_));
            status.values.push_back(numericValue(
                    "global_map_same_stamp", global_map_same_stamp_count_));
            status.values.push_back(numericValue(
                    "global_map_regressed_stamp", global_map_regressed_stamp_count_));
            status.values.push_back(numericValue(
                    "global_map_invalid_envelope", global_map_invalid_envelope_count_));
            status.values.push_back(numericValue(
                    "global_map_resource_rejections", global_map_resource_rejection_count_));
            status.values.push_back(numericValue(
                    "global_map_processing_failures", global_map_processing_failure_count_));
            status.values.push_back(numericValue(
                    "global_map_worker_failures", global_map_worker_failure_count_));
            status.values.push_back(numericValue(
                    "global_map_latest_revision", global_snapshot_slot_.latestAcceptedRevision()));
            status.values.push_back(numericValue(
                    "global_map_consumed_revision", global_snapshot_slot_.consumedRevision()));
            status.values.push_back(numericValue(
                    "global_map_applied_revision", global_map_applied_revision_));
            status.values.push_back(numericValue(
                    "global_map_pending_revision", global_snapshot_slot_.pendingRevision()));
            status.values.push_back(numericValue(
                    "global_map_in_flight_revision", global_snapshot_slot_.inFlightRevision()));
            status.values.push_back(numericValue(
                    "global_map_ready_revision", global_snapshot_slot_.readyRevision()));
            status.values.push_back(numericValue(
                    "global_map_pending_coalesced", global_snapshot_slot_.pendingCoalescedCount()));
            status.values.push_back(numericValue(
                    "global_map_ready_blocked", global_snapshot_slot_.readyBlockedCount()));
            status.values.push_back(numericValue(
                    "global_map_leaf_count", global_map_leaf_count_));
            status.values.push_back(numericValue(
                    "global_map_valid_age_seconds",
                    global_map_receive_seconds_ < 0.0
                            ? -1.0
                            : input.monotonic_time_seconds
                                      - global_map_receive_seconds_));
            status.values.push_back(numericValue(
                    "max_serialized_bytes_per_source",
                    max_serialized_bytes_per_source_));
            status.values.push_back(numericValue(
                    "max_voxels_per_source", max_voxels_per_source_));
            status.values.push_back(numericValue(
                    "max_global_serialized_bytes", max_global_serialized_bytes_));
            status.values.push_back(numericValue(
                    "max_global_voxels", max_global_voxels_));
            status.values.push_back(value("global_map_input_reason", global_map_input_reason_));
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
            for(std::size_t index = 0U; index < drones_.size(); ++index) {
                const auto & track = drones_[index];
                const auto & drone_input = input.drones[index];
                status.values.push_back(numericValue(
                        track.id + ".local_map_fresh",
                        drone_input.local_map_fresh ? 1 : 0));
                status.values.push_back(numericValue(
                        track.id + ".local_map_valid_age_seconds",
                        track.local_map_receive_seconds < 0.0
                                ? -1.0
                                : input.monotonic_time_seconds
                                          - track.local_map_receive_seconds));
                status.values.push_back(numericValue(
                        track.id + ".odom_fresh",
                        drone_input.odom_fresh ? 1 : 0));
                status.values.push_back(numericValue(
                        track.id + ".odom_invalid_stamp",
                        track.odom_invalid_stamp_count));
                status.values.push_back(value(
                        track.id + ".odom_reason", track.odom_reason));
                status.values.push_back(numericValue(
                        track.id + ".local_map_received", track.local_map_received_count));
                status.values.push_back(numericValue(
                        track.id + ".local_map_invalid_envelope",
                        track.local_map_invalid_envelope_count));
                status.values.push_back(numericValue(
                        track.id + ".local_map_regressed", track.local_map_regressed_count));
                status.values.push_back(numericValue(
                        track.id + ".local_map_duplicate", track.local_map_duplicate_count));
                status.values.push_back(numericValue(
                        track.id + ".local_map_resource_rejections",
                        track.local_map_resource_rejection_count));
                status.values.push_back(numericValue(
                        track.id + ".local_map_failures", track.local_map_failure_count));
                status.values.push_back(numericValue(
                        track.id + ".local_map_latest_revision",
                        track.snapshot_slot.latestAcceptedRevision()));
                status.values.push_back(numericValue(
                        track.id + ".local_map_consumed_revision",
                        track.snapshot_slot.consumedRevision()));
                status.values.push_back(numericValue(
                        track.id + ".local_map_applied_revision",
                        track.local_map_applied_revision));
                status.values.push_back(numericValue(
                        track.id + ".local_map_pending_revision",
                        track.snapshot_slot.pendingRevision()));
                status.values.push_back(numericValue(
                        track.id + ".local_map_in_flight_revision",
                        track.snapshot_slot.inFlightRevision()));
                status.values.push_back(numericValue(
                        track.id + ".local_map_ready_revision",
                        track.snapshot_slot.readyRevision()));
                status.values.push_back(numericValue(
                        track.id + ".local_map_pending_coalesced",
                        track.snapshot_slot.pendingCoalescedCount()));
                status.values.push_back(value(
                        track.id + ".local_map_reason", track.local_map_reason));
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
            if(stopping_) {
                return;
            }
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
                if(transformOdometry(found->odom, found->odom_stamp_ns, pose)) {
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
        std::size_t max_serialized_bytes_per_source_ {67'108'864U};
        std::size_t max_voxels_per_source_ {5'000'000U};
        std::size_t max_global_serialized_bytes_ {201'326'592U};
        std::size_t max_global_voxels_ {10'000'000U};
        double odom_timeout_seconds_ {2.0};
        double local_map_timeout_seconds_ {5.0};
        GlobalFrontierDetectorConfig detector_config_;
        GlobalTaskAllocatorConfig allocator_config_;
        std::unique_ptr<GlobalFrontierDetector> detector_;
        std::unique_ptr<GlobalTaskAllocator> allocator_;
        SteadyClock::time_point steady_start_;
        std::uint64_t allocator_epoch_ {};
        GlobalSnapshotSlot global_snapshot_slot_;
        std::mutex mutex_;
        std::condition_variable worker_cv_;
        std::thread worker_;
        bool stopping_ {false};
        std::size_t worker_next_source_ {};
        std::vector<DroneTrack> drones_;
        std::shared_ptr<octomap::OcTree> global_map_;
        std::int64_t global_map_stamp_ns_ {};
        double global_map_receive_seconds_ {-1.0};
        std::uint64_t global_map_applied_revision_ {};
        std::size_t global_map_leaf_count_ {};
        bool global_diagnostics_healthy_ {false};
        std::int64_t global_diagnostics_stamp_ns_ {};
        double global_diagnostics_receive_seconds_ {-1.0};
        std::uint64_t global_diagnostics_invalid_stamp_count_ {};
        std::string global_diagnostics_reason_;
        std::int64_t last_ros_now_ns_ {};
        std::uint64_t ros_clock_reset_count_ {};
        std::vector<FrontierRegion> regions_;
        std::uint64_t global_update_sequence_ {};
        std::uint64_t global_map_received_count_ {};
        std::uint64_t global_map_same_stamp_count_ {};
        std::uint64_t global_map_regressed_stamp_count_ {};
        std::uint64_t global_map_invalid_envelope_count_ {};
        std::uint64_t global_map_resource_rejection_count_ {};
        std::uint64_t global_map_processing_failure_count_ {};
        std::uint64_t global_map_worker_failure_count_ {};
        std::uint64_t global_map_input_reason_sequence_ {};
        std::string global_map_input_reason_;
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

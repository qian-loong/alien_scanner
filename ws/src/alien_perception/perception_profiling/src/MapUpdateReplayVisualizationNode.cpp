#include "perception_profiling/MapUpdateAcceptanceScenarios.hpp"
#include "perception_profiling/MapUpdateReplayOracle.hpp"

#include "perception_map_update/OctoMapViewAdapter.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

    using PerceptionMapUpdate::CanonicalCell;
    using PerceptionMapUpdate::ApplyUpdateStatus;
    using PerceptionMapUpdate::MapGeometry;
    using PerceptionMapUpdate::ReceiverState;
    using PerceptionMapUpdate::ReconstructedMap;
    using PerceptionMapUpdate::VoxelIndex;
    using PerceptionProfiling::MapUpdateAcceptanceRun;
    using PerceptionProfiling::MapUpdateAcceptanceScenarios;
    using PerceptionProfiling::MapUpdateAcceptanceStage;
    using PerceptionProfiling::MapUpdateAcceptanceStageKind;
    using PerceptionProfiling::MapUpdateReplayOracle;
    using PerceptionProfiling::MapUpdateReplayOptions;
    using PerceptionProfiling::MapUpdateReplayRun;
    using PerceptionProfiling::ProfileScenario;
    using PerceptionProfiling::ReplayComparison;

    diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
    {
        diagnostic_msgs::msg::KeyValue result;
        result.key = std::move(key);
        result.value = std::move(value);
        return result;
    }

    std::string update_kind_name(PerceptionMapUpdate::UpdateKind kind)
    {
        switch(kind) {
            case PerceptionMapUpdate::UpdateKind::Keyframe:
                return "keyframe";
            case PerceptionMapUpdate::UpdateKind::Delta:
                return "delta";
            case PerceptionMapUpdate::UpdateKind::Summary:
                return "summary";
            case PerceptionMapUpdate::UpdateKind::Remove:
                return "remove";
        }
        return "unknown";
    }

    std::string receiver_state_name(ReceiverState state)
    {
        switch(state) {
            case ReceiverState::Empty:
                return "empty";
            case ReceiverState::Ready:
                return "ready";
            case ReceiverState::ResyncRequired:
                return "resync_required";
            case ReceiverState::Removed:
                return "removed";
        }
        return "unknown";
    }

    std::string apply_status_name(ApplyUpdateStatus status)
    {
        switch(status) {
            case ApplyUpdateStatus::AppliedKeyframe:
                return "applied_keyframe";
            case ApplyUpdateStatus::AppliedDelta:
                return "applied_delta";
            case ApplyUpdateStatus::AppliedRemove:
                return "applied_remove";
            case ApplyUpdateStatus::AcceptedSummary:
                return "accepted_summary";
            case ApplyUpdateStatus::IgnoredDuplicate:
                return "ignored_duplicate";
            case ApplyUpdateStatus::RejectedAdmission:
                return "rejected_admission";
            case ApplyUpdateStatus::RejectedStale:
                return "rejected_stale";
            case ApplyUpdateStatus::RejectedGap:
                return "rejected_gap";
            case ApplyUpdateStatus::RejectedConflict:
                return "rejected_conflict";
            case ApplyUpdateStatus::RejectedInvalid:
                return "rejected_invalid";
            case ApplyUpdateStatus::RejectedResourceLimit:
                return "rejected_resource_limit";
        }
        return "unknown";
    }

    std::string stage_name(MapUpdateAcceptanceStageKind kind)
    {
        switch(kind) {
            case MapUpdateAcceptanceStageKind::ReadyBaseline:
                return "READY BASELINE";
            case MapUpdateAcceptanceStageKind::DeltaDropped:
                return "DELTA DROPPED";
            case MapUpdateAcceptanceStageKind::GapRejected:
                return "GAP REJECTED";
            case MapUpdateAcceptanceStageKind::ResyncRecovered:
                return "RESYNC RECOVERED";
            case MapUpdateAcceptanceStageKind::EpochBaseline:
                return "EPOCH 1 READY";
            case MapUpdateAcceptanceStageKind::NewEpochAdmitted:
                return "EPOCH 2 ADMITTED";
            case MapUpdateAcceptanceStageKind::OldEpochRejected:
                return "OLD EPOCH REJECTED";
            case MapUpdateAcceptanceStageKind::EpochRecovered:
                return "EPOCH 2 READY";
        }
        return "UNKNOWN STAGE";
    }

    struct MarkerColor {
        float red = 1.0F;
        float green = 1.0F;
        float blue = 1.0F;
    };

    MarkerColor stage_color(MapUpdateAcceptanceStageKind kind)
    {
        switch(kind) {
            case MapUpdateAcceptanceStageKind::ReadyBaseline:
            case MapUpdateAcceptanceStageKind::EpochBaseline:
                return {0.1F, 0.75F, 0.85F};
            case MapUpdateAcceptanceStageKind::DeltaDropped:
            case MapUpdateAcceptanceStageKind::NewEpochAdmitted:
                return {1.0F, 0.7F, 0.1F};
            case MapUpdateAcceptanceStageKind::GapRejected:
            case MapUpdateAcceptanceStageKind::OldEpochRejected:
                return {0.9F, 0.15F, 0.15F};
            case MapUpdateAcceptanceStageKind::ResyncRecovered:
            case MapUpdateAcceptanceStageKind::EpochRecovered:
                return {0.2F, 0.85F, 0.35F};
        }
        return {};
    }

    std::string scenario_frame(const std::string & prefix)
    {
        return prefix + "_view";
    }

    geometry_msgs::msg::Point voxel_center(
            const VoxelIndex & index,
            const MapGeometry & geometry)
    {
        geometry_msgs::msg::Point point;
        point.x = geometry.lattice_origin.x
                  + (static_cast<double>(index.x) + 0.5) * geometry.resolution_m;
        point.y = geometry.lattice_origin.y
                  + (static_cast<double>(index.y) + 0.5) * geometry.resolution_m;
        point.z = geometry.lattice_origin.z
                  + (static_cast<double>(index.z) + 0.5) * geometry.resolution_m;
        return point;
    }

    visualization_msgs::msg::Marker difference_marker(
            const std_msgs::msg::Header & header,
            std::string marker_namespace,
            std::int32_t marker_id,
            const MapGeometry & geometry,
            float red,
            float green,
            float blue)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = std::move(marker_namespace);
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = geometry.resolution_m * 0.8;
        marker.scale.y = geometry.resolution_m * 0.8;
        marker.scale.z = geometry.resolution_m * 0.8;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 0.95F;
        marker.frame_locked = true;
        return marker;
    }

    void append_cell_samples(
            visualization_msgs::msg::Marker & marker,
            const std::vector<CanonicalCell> & cells,
            const MapGeometry & geometry)
    {
        marker.points.reserve(cells.size());
        for(const auto & cell : cells) {
            marker.points.push_back(voxel_center(cell.index, geometry));
        }
    }

    void append_index_samples(
            visualization_msgs::msg::Marker & marker,
            const std::vector<VoxelIndex> & indices,
            const MapGeometry & geometry)
    {
        marker.points.reserve(indices.size());
        for(const auto & index : indices) {
            marker.points.push_back(voxel_center(index, geometry));
        }
    }

    template<typename Visitor>
    void visit_cells(
            const std::vector<CanonicalCell> & cells,
            const Visitor & visitor)
    {
        for(const auto & cell : cells) {
            visitor(cell);
        }
    }

    template<typename Visitor>
    void visit_cells(
            const PerceptionMapUpdate::CanonicalCellView & cells,
            const Visitor & visitor)
    {
        cells.for_each(visitor);
    }

    template<typename Cells>
    visualization_msgs::msg::Marker occupied_map_marker(
            const std_msgs::msg::Header & header,
            const std::string & marker_namespace,
            std::int32_t marker_id,
            const MapGeometry & geometry,
            const Cells & cells,
            std::size_t max_voxels,
            float red,
            float green,
            float blue)
    {
        auto marker = difference_marker(
                header, marker_namespace, marker_id, geometry, red, green, blue);
        marker.color.a = 0.72F;
        std::size_t occupied_count = 0U;
        visit_cells(cells, [&occupied_count](const CanonicalCell & cell) {
            if(cell.state == PerceptionMapUpdate::CellState::Occupied) {
                ++occupied_count;
            }
        });
        const std::size_t stride = occupied_count > max_voxels
                ? (occupied_count + max_voxels - 1U) / max_voxels
                : 1U;
        marker.points.reserve(std::min(occupied_count, max_voxels));
        std::size_t occupied_index = 0U;
        visit_cells(cells, [&](const CanonicalCell & cell) {
            if(cell.state != PerceptionMapUpdate::CellState::Occupied) {
                return;
            }
            if(occupied_index % stride == 0U && marker.points.size() < max_voxels) {
                marker.points.push_back(voxel_center(cell.index, geometry));
            }
            ++occupied_index;
        });
        return marker;
    }

    geometry_msgs::msg::Point label_position(const ReconstructedMap & map)
    {
        geometry_msgs::msg::Point position;
        if(map.cells.empty()) {
            position.z = 1.0;
            return position;
        }
        auto first = voxel_center(map.cells.front().index, map.geometry);
        double minimum_x = first.x;
        double maximum_x = first.x;
        double minimum_y = first.y;
        double maximum_y = first.y;
        double maximum_z = first.z;
        map.cells.for_each([&](const CanonicalCell & cell) {
            const auto point = voxel_center(cell.index, map.geometry);
            minimum_x = std::min(minimum_x, point.x);
            maximum_x = std::max(maximum_x, point.x);
            minimum_y = std::min(minimum_y, point.y);
            maximum_y = std::max(maximum_y, point.y);
            maximum_z = std::max(maximum_z, point.z);
        });
        position.x = (minimum_x + maximum_x) * 0.5;
        position.y = (minimum_y + maximum_y) * 0.5;
        position.z = maximum_z + 1.5;
        return position;
    }

    visualization_msgs::msg::Marker stage_label_marker(
            const std_msgs::msg::Header & header,
            const std::string & marker_namespace,
            std::int32_t marker_id,
            const MapUpdateAcceptanceStage & stage,
            const MarkerColor & color)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = marker_namespace;
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.8;
        marker.color.r = color.red;
        marker.color.g = color.green;
        marker.color.b = color.blue;
        marker.color.a = 1.0F;
        marker.frame_locked = true;
        if(stage.receiver_map.has_value()) {
            marker.pose.position = label_position(*stage.receiver_map);
        }

        std::ostringstream label;
        label << stage_name(stage.kind) << '\n'
              << "receiver: " << receiver_state_name(stage.receiver_state);
        if(stage.receiver_map.has_value()) {
            label << "  epoch " << stage.receiver_map->source.map_epoch
                  << "  revision " << stage.receiver_map->revision;
        }
        if(stage.attempted_new_revision != 0U) {
            label << '\n' << "update " << stage.attempted_base_revision << " -> "
                  << stage.attempted_new_revision;
        }
        if(!stage.correlation_id.empty()) {
            label << '\n' << "correlation: " << stage.correlation_id;
        }
        marker.text = label.str();
        return marker;
    }

}// namespace

class MapUpdateReplayVisualizationNode final : public rclcpp::Node
{
public:
    MapUpdateReplayVisualizationNode()
        : Node("map_update_replay_visualization_node")
        , scenario_mode_(declare_parameter<std::string>("scenario_mode", "canonical"))
        , sequence_count_(declare_parameter<std::int64_t>("sequence_count", 60))
        , max_difference_markers_(
                  declare_parameter<std::int64_t>("max_difference_markers", 4096))
        , oracle_topic_(declare_parameter<std::string>(
                  "oracle_octomap_topic", "map_update_replay/oracle_octomap"))
        , reconstructed_topic_(declare_parameter<std::string>(
                  "reconstructed_octomap_topic",
                  "map_update_replay/reconstructed_octomap"))
        , difference_topic_(declare_parameter<std::string>(
                  "difference_topic", "map_update_replay/differences"))
        , map_markers_topic_(declare_parameter<std::string>(
                  "map_markers_topic", "map_update_replay/map_markers"))
        , diagnostics_topic_(declare_parameter<std::string>(
                  "diagnostics_topic", "map_update_replay/diagnostics"))
        , enable_resync_scenario_(
                  declare_parameter<bool>("enable_resync_scenario", true))
        , enable_epoch_reset_scenario_(
                  declare_parameter<bool>("enable_epoch_reset_scenario", true))
        , resync_markers_topic_(declare_parameter<std::string>(
                  "resync_markers_topic", "map_update_replay/resync/markers"))
        , epoch_reset_markers_topic_(declare_parameter<std::string>(
                  "epoch_reset_markers_topic",
                  "map_update_replay/epoch_reset/markers"))
        , world_frame_(declare_parameter<std::string>("world_frame", "replay_world"))
        , oracle_frame_(declare_parameter<std::string>(
                  "oracle_frame", "replay_oracle_map"))
        , reconstructed_frame_(declare_parameter<std::string>(
                  "reconstructed_frame", "replay_reconstructed_map"))
        , difference_frame_(declare_parameter<std::string>(
                  "difference_frame", "replay_difference_map"))
        , resync_frame_prefix_(declare_parameter<std::string>(
                  "resync_frame_prefix", "replay_resync"))
        , epoch_reset_frame_prefix_(declare_parameter<std::string>(
                  "epoch_reset_frame_prefix", "replay_epoch_reset"))
        , view_separation_m_(declare_parameter<double>("view_separation_m", 10.0))
        , max_visualization_voxels_(
                  declare_parameter<std::int64_t>("max_visualization_voxels", 250000))
        , visualization_publish_rate_hz_(
                  declare_parameter<double>("visualization_publish_rate_hz", 1.0))
        , scenario_step_period_s_(
                  declare_parameter<double>("scenario_step_period_s", 2.0))
    {
        validate_parameters();
        const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        oracle_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
                oracle_topic_, latched);
        reconstructed_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
                reconstructed_topic_, latched);
        difference_publisher_ =
                create_publisher<visualization_msgs::msg::MarkerArray>(
                        difference_topic_, latched);
        map_markers_publisher_ =
                create_publisher<visualization_msgs::msg::MarkerArray>(
                        map_markers_topic_, latched);
        diagnostics_publisher_ =
                create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                        diagnostics_topic_, latched);
        if(enable_resync_scenario_) {
            resync_markers_publisher_ =
                    create_publisher<visualization_msgs::msg::MarkerArray>(
                            resync_markers_topic_, latched);
        }
        if(enable_epoch_reset_scenario_) {
            epoch_reset_markers_publisher_ =
                    create_publisher<visualization_msgs::msg::MarkerArray>(
                            epoch_reset_markers_topic_, latched);
        }
        static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publish_static_frames();
        replay_timer_ = create_wall_timer(
                std::chrono::milliseconds(100), [this]() { run_once(); });
    }

private:
    std::string scenario_mode_;
    std::int64_t sequence_count_ = 0;
    std::int64_t max_difference_markers_ = 0;
    std::string oracle_topic_;
    std::string reconstructed_topic_;
    std::string difference_topic_;
    std::string map_markers_topic_;
    std::string diagnostics_topic_;
    bool enable_resync_scenario_ = true;
    bool enable_epoch_reset_scenario_ = true;
    std::string resync_markers_topic_;
    std::string epoch_reset_markers_topic_;
    std::string world_frame_;
    std::string oracle_frame_;
    std::string reconstructed_frame_;
    std::string difference_frame_;
    std::string resync_frame_prefix_;
    std::string epoch_reset_frame_prefix_;
    double view_separation_m_ = 0.0;
    std::int64_t max_visualization_voxels_ = 0;
    double visualization_publish_rate_hz_ = 0.0;
    double scenario_step_period_s_ = 0.0;
    bool completed_ = false;
    std::size_t acceptance_stage_index_ = 0U;

    std::optional<visualization_msgs::msg::MarkerArray> cached_map_markers_;
    std::optional<visualization_msgs::msg::MarkerArray> cached_difference_markers_;
    std::vector<visualization_msgs::msg::MarkerArray> cached_resync_stages_;
    std::vector<visualization_msgs::msg::MarkerArray> cached_epoch_reset_stages_;

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr oracle_publisher_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr reconstructed_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            difference_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            map_markers_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
            diagnostics_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            resync_markers_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            epoch_reset_markers_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr replay_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::TimerBase::SharedPtr scenario_step_timer_;

    void validate_parameters() const
    {
        if(sequence_count_ <= 0) {
            throw std::invalid_argument("sequence_count must be positive");
        }
        if(max_difference_markers_ < 0) {
            throw std::invalid_argument("max_difference_markers must not be negative");
        }
        if(max_visualization_voxels_ <= 0) {
            throw std::invalid_argument("max_visualization_voxels must be positive");
        }
        if(!std::isfinite(view_separation_m_) || view_separation_m_ <= 0.0) {
            throw std::invalid_argument("view_separation_m must be finite and positive");
        }
        (void) visualization_publish_period();
        (void) scenario_step_period();
        if(oracle_topic_.empty() || reconstructed_topic_.empty()
           || difference_topic_.empty() || map_markers_topic_.empty()
           || diagnostics_topic_.empty()
           || world_frame_.empty() || oracle_frame_.empty()
           || reconstructed_frame_.empty() || difference_frame_.empty()) {
            throw std::invalid_argument("replay topics and frames must not be empty");
        }
        if((enable_resync_scenario_
            && (resync_markers_topic_.empty() || resync_frame_prefix_.empty()))
           || (enable_epoch_reset_scenario_
               && (epoch_reset_markers_topic_.empty()
                   || epoch_reset_frame_prefix_.empty()))) {
            throw std::invalid_argument(
                    "enabled acceptance scenario topics and frame prefixes must not be empty");
        }
        if(enable_resync_scenario_
           && (resync_markers_topic_ == map_markers_topic_
               || resync_markers_topic_ == difference_topic_)) {
            throw std::invalid_argument(
                    "resync markers topic must be isolated from baseline topics");
        }
        if(enable_epoch_reset_scenario_
           && (epoch_reset_markers_topic_ == map_markers_topic_
               || epoch_reset_markers_topic_ == difference_topic_
               || (enable_resync_scenario_
                   && epoch_reset_markers_topic_ == resync_markers_topic_))) {
            throw std::invalid_argument(
                    "epoch-reset markers topic must be isolated from other scenes");
        }
        if(enable_resync_scenario_ && enable_epoch_reset_scenario_
           && resync_frame_prefix_ == epoch_reset_frame_prefix_) {
            throw std::invalid_argument(
                    "acceptance scenario frame prefixes must be distinct");
        }
        if(world_frame_ == oracle_frame_ || world_frame_ == reconstructed_frame_
           || world_frame_ == difference_frame_ || oracle_frame_ == reconstructed_frame_
           || oracle_frame_ == difference_frame_
           || reconstructed_frame_ == difference_frame_) {
            throw std::invalid_argument("replay visualization frames must be distinct");
        }
    }

    std::chrono::nanoseconds visualization_publish_period() const
    {
        if(!std::isfinite(visualization_publish_rate_hz_)
           || visualization_publish_rate_hz_ <= 0.0) {
            throw std::invalid_argument(
                    "visualization_publish_rate_hz must be finite and positive");
        }
        const double period_s = 1.0 / visualization_publish_rate_hz_;
        const double max_period_s =
                static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9;
        if(!std::isfinite(period_s) || period_s > max_period_s) {
            throw std::invalid_argument(
                    "visualization_publish_rate_hz produces an invalid period");
        }
        const auto period_ns = static_cast<std::int64_t>(period_s * 1e9);
        if(period_ns <= 0) {
            throw std::invalid_argument(
                    "visualization_publish_rate_hz produces a zero period");
        }
        return std::chrono::nanoseconds(period_ns);
    }

    std::chrono::nanoseconds scenario_step_period() const
    {
        if(!std::isfinite(scenario_step_period_s_) || scenario_step_period_s_ <= 0.0) {
            throw std::invalid_argument(
                    "scenario_step_period_s must be finite and positive");
        }
        const double max_period_s =
                static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9;
        if(scenario_step_period_s_ > max_period_s) {
            throw std::invalid_argument(
                    "scenario_step_period_s produces an invalid period");
        }
        const auto period_ns = static_cast<std::int64_t>(scenario_step_period_s_ * 1e9);
        if(period_ns <= 0) {
            throw std::invalid_argument(
                    "scenario_step_period_s produces a zero period");
        }
        return std::chrono::nanoseconds(period_ns);
    }

    geometry_msgs::msg::TransformStamped static_transform(
            const std::string & child,
            double x,
            double y) const
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = child;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.rotation.w = 1.0;
        return transform;
    }

    void publish_static_frames()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.reserve(5U);
        transforms.push_back(static_transform(
                oracle_frame_, 0.0, -view_separation_m_));
        transforms.push_back(static_transform(
                reconstructed_frame_, 0.0, view_separation_m_));
        transforms.push_back(static_transform(difference_frame_, 0.0, 0.0));
        if(enable_resync_scenario_) {
            transforms.push_back(static_transform(
                    scenario_frame(resync_frame_prefix_),
                    view_separation_m_ * 2.0,
                    -view_separation_m_ * 2.0));
        }
        if(enable_epoch_reset_scenario_) {
            transforms.push_back(static_transform(
                    scenario_frame(epoch_reset_frame_prefix_),
                    view_separation_m_ * 2.0,
                    view_separation_m_ * 2.0));
        }
        static_broadcaster_->sendTransform(transforms);
    }

    void run_once()
    {
        if(completed_) {
            return;
        }
        completed_ = true;
        replay_timer_->cancel();
        try {
            MapUpdateReplayOptions options;
            options.sequence_count = static_cast<std::uint64_t>(sequence_count_);
            options.max_difference_samples =
                    static_cast<std::size_t>(max_difference_markers_);
            const MapUpdateReplayOracle oracle(
                    ProfileScenario(ProfileScenario::parse_mode(scenario_mode_)));
            const auto run = oracle.run(options);
            publish_run(run);
        }
        catch(const std::exception & error) {
            publish_failure(error.what());
            RCLCPP_ERROR(get_logger(), "Map-update replay failed: %s", error.what());
        }
    }

    void publish_run(const MapUpdateReplayRun & run)
    {
        if(!run.final_oracle.has_value() || !run.final_reconstructed.has_value()) {
            throw std::runtime_error("replay run has no final maps");
        }
        std::optional<MapUpdateAcceptanceRun> resync_run;
        std::optional<MapUpdateAcceptanceRun> epoch_reset_run;
        if(enable_resync_scenario_) {
            resync_run = MapUpdateAcceptanceScenarios::resync_recovery(
                    run.acceptance_snapshots);
        }
        if(enable_epoch_reset_scenario_) {
            epoch_reset_run = MapUpdateAcceptanceScenarios::epoch_reset(
                    run.acceptance_snapshots);
        }
        PerceptionMapUpdate::CanonicalCellView oracle_cells;
        oracle_cells = run.final_oracle->cells;
        const ReconstructedMap oracle_view {
                run.final_oracle->source,
                run.final_oracle->geometry,
                run.final_oracle->revision,
                PerceptionMapUpdate::VersionedContentDigest {
                        PerceptionMapUpdate::ContentIdentityDescriptor {},
                        run.final_oracle->content_hash},
                std::move(oracle_cells)};

        octomap_msgs::msg::Octomap oracle_message;
        octomap_msgs::msg::Octomap reconstructed_message;
        std::string diagnostic;
        if(!PerceptionMapUpdate::OctoMapViewAdapter::materialize(
                   oracle_view, oracle_message, diagnostic)) {
            throw std::runtime_error("oracle OctoMap materialization failed: " + diagnostic);
        }
        if(!PerceptionMapUpdate::OctoMapViewAdapter::materialize(
                   *run.final_reconstructed, reconstructed_message, diagnostic)) {
            throw std::runtime_error(
                    "reconstructed OctoMap materialization failed: " + diagnostic);
        }
        const auto stamp = now();
        oracle_message.header.stamp = stamp;
        oracle_message.header.frame_id = oracle_frame_;
        reconstructed_message.header.stamp = stamp;
        reconstructed_message.header.frame_id = reconstructed_frame_;
        oracle_publisher_->publish(oracle_message);
        reconstructed_publisher_->publish(reconstructed_message);
        cached_map_markers_ = make_map_markers(
                *run.final_oracle, *run.final_reconstructed, stamp);
        cached_difference_markers_ = make_difference_markers(
                run.final_comparison, run.final_oracle->geometry, stamp);
        if(resync_run.has_value()) {
            cached_resync_stages_ = make_acceptance_stages(
                    *resync_run, resync_frame_prefix_, "replay_resync", stamp);
        }
        if(epoch_reset_run.has_value()) {
            cached_epoch_reset_stages_ = make_acceptance_stages(
                    *epoch_reset_run,
                    epoch_reset_frame_prefix_,
                    "replay_epoch_reset",
                    stamp);
        }
        acceptance_stage_index_ = 0U;
        publish_cached_visualization();
        visualization_timer_ = create_wall_timer(
                visualization_publish_period(),
                [this]() { publish_cached_visualization(); });
        if(!cached_resync_stages_.empty() || !cached_epoch_reset_stages_.empty()) {
            scenario_step_timer_ = create_wall_timer(
                    scenario_step_period(), [this]() { advance_acceptance_stage(); });
        }
        diagnostics_publisher_->publish(
                make_diagnostics(run, resync_run, epoch_reset_run, stamp));

        if(run.all_checkpoints_match()) {
            RCLCPP_INFO(
                    get_logger(),
                    "Replay matched at %zu checkpoints through revision %llu",
                    run.checkpoints.size(),
                    static_cast<unsigned long long>(run.final_oracle->revision));
        } else {
            RCLCPP_ERROR(
                    get_logger(), "Replay mismatch: %s",
                    run.final_comparison.diagnostic.c_str());
        }
    }

    void publish_cached_visualization()
    {
        if(!cached_map_markers_.has_value()
           || !cached_difference_markers_.has_value()) {
            return;
        }
        const auto stamp = now();
        for(auto & marker : cached_map_markers_->markers) {
            marker.header.stamp = stamp;
        }
        for(auto & marker : cached_difference_markers_->markers) {
            marker.header.stamp = stamp;
        }
        map_markers_publisher_->publish(*cached_map_markers_);
        difference_publisher_->publish(*cached_difference_markers_);
        if(!cached_resync_stages_.empty() && resync_markers_publisher_) {
            publish_acceptance_stage(
                    cached_resync_stages_, resync_markers_publisher_, stamp);
        }
        if(!cached_epoch_reset_stages_.empty() && epoch_reset_markers_publisher_) {
            publish_acceptance_stage(
                    cached_epoch_reset_stages_, epoch_reset_markers_publisher_, stamp);
        }
    }

    void publish_acceptance_stage(
            std::vector<visualization_msgs::msg::MarkerArray> & stages,
            const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
            const rclcpp::Time & stamp)
    {
        if(stages.empty()) {
            return;
        }
        auto & stage_markers = stages[acceptance_stage_index_ % stages.size()];
        for(auto & marker : stage_markers.markers) {
            marker.header.stamp = stamp;
        }
        publisher->publish(stage_markers);
    }

    void advance_acceptance_stage()
    {
        const std::size_t stage_count = !cached_resync_stages_.empty()
                ? cached_resync_stages_.size()
                : cached_epoch_reset_stages_.size();
        if(stage_count == 0U) {
            return;
        }
        acceptance_stage_index_ = (acceptance_stage_index_ + 1U) % stage_count;
    }

    visualization_msgs::msg::MarkerArray make_map_markers(
            const PerceptionMapUpdate::CanonicalSnapshot & oracle,
            const ReconstructedMap & reconstructed,
            const rclcpp::Time & stamp) const
    {
        std_msgs::msg::Header oracle_header;
        oracle_header.stamp = stamp;
        oracle_header.frame_id = oracle_frame_;
        std_msgs::msg::Header reconstructed_header;
        reconstructed_header.stamp = stamp;
        reconstructed_header.frame_id = reconstructed_frame_;

        visualization_msgs::msg::MarkerArray array;
        array.markers.reserve(3U);
        visualization_msgs::msg::Marker clear;
        clear.header = oracle_header;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        array.markers.push_back(std::move(clear));
        array.markers.push_back(occupied_map_marker(
                oracle_header,
                "replay_map/oracle_occupied",
                1,
                oracle.geometry,
                oracle.cells,
                static_cast<std::size_t>(max_visualization_voxels_),
                0.1F,
                0.75F,
                0.85F));
        array.markers.push_back(occupied_map_marker(
                reconstructed_header,
                "replay_map/reconstructed_occupied",
                2,
                reconstructed.geometry,
                reconstructed.cells,
                static_cast<std::size_t>(max_visualization_voxels_),
                0.95F,
                0.8F,
                0.2F));
        return array;
    }

    visualization_msgs::msg::MarkerArray make_difference_markers(
            const ReplayComparison & comparison,
            const MapGeometry & geometry,
            const rclcpp::Time & stamp) const
    {
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = difference_frame_;

        visualization_msgs::msg::MarkerArray array;
        array.markers.reserve(4U);
        visualization_msgs::msg::Marker clear;
        clear.header = header;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        array.markers.push_back(std::move(clear));

        auto missing = difference_marker(
                header, "replay_diff/missing", 1, geometry, 0.9F, 0.15F, 0.15F);
        append_cell_samples(missing, comparison.missing_cell_samples, geometry);
        array.markers.push_back(std::move(missing));

        auto unexpected = difference_marker(
                header, "replay_diff/unexpected", 2, geometry, 1.0F, 0.7F, 0.1F);
        append_cell_samples(unexpected, comparison.unexpected_cell_samples, geometry);
        array.markers.push_back(std::move(unexpected));

        auto state = difference_marker(
                header, "replay_diff/state", 3, geometry, 0.85F, 0.2F, 0.9F);
        append_index_samples(state, comparison.state_mismatch_samples, geometry);
        array.markers.push_back(std::move(state));
        return array;
    }

    std::vector<visualization_msgs::msg::MarkerArray> make_acceptance_stages(
            const MapUpdateAcceptanceRun & run,
            const std::string & frame_prefix,
            const std::string & marker_prefix,
            const rclcpp::Time & stamp) const
    {
        std::vector<visualization_msgs::msg::MarkerArray> stages;
        stages.reserve(run.stages.size());
        for(std::size_t index = 0U; index < run.stages.size(); ++index) {
            const auto & acceptance_stage = run.stages[index];
            if(!acceptance_stage.receiver_map.has_value()) {
                throw std::runtime_error(
                        "acceptance stage has no retained receiver map");
            }
            std_msgs::msg::Header header;
            header.stamp = stamp;
            header.frame_id = scenario_frame(frame_prefix);
            const auto color = stage_color(acceptance_stage.kind);
            const std::string stage_namespace = marker_prefix + "/stage_"
                                                + std::to_string(index);
            visualization_msgs::msg::MarkerArray array;
            array.markers.reserve(3U);
            visualization_msgs::msg::Marker clear;
            clear.header = header;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            array.markers.push_back(std::move(clear));
            array.markers.push_back(occupied_map_marker(
                    header,
                    stage_namespace + "/map",
                    static_cast<std::int32_t>(index * 2U + 1U),
                    acceptance_stage.receiver_map->geometry,
                    acceptance_stage.receiver_map->cells,
                    static_cast<std::size_t>(max_visualization_voxels_),
                    color.red,
                    color.green,
                    color.blue));
            array.markers.push_back(stage_label_marker(
                    header,
                    stage_namespace + "/status",
                    static_cast<std::int32_t>(index * 2U + 2U),
                    acceptance_stage,
                    color));
            stages.push_back(std::move(array));
        }
        return stages;
    }

    diagnostic_msgs::msg::DiagnosticArray make_diagnostics(
            const MapUpdateReplayRun & run,
            const std::optional<MapUpdateAcceptanceRun> & resync_run,
            const std::optional<MapUpdateAcceptanceRun> & epoch_reset_run,
            const rclcpp::Time & stamp) const
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = stamp;
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name()
                      + std::string(": exact_revision_replay_oracle");
        status.hardware_id = "deterministic-c2-map-fixture";
        status.level = run.all_checkpoints_match()
                ? diagnostic_msgs::msg::DiagnosticStatus::OK
                : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = run.all_checkpoints_match()
                ? "all exact-revision checkpoints match"
                : run.final_comparison.diagnostic;
        const auto & final_checkpoint = run.checkpoints.back();
        status.values = {
                key_value("match", run.all_checkpoints_match() ? "true" : "false"),
                key_value("scenario_mode", ProfileScenario::mode_name(run.mode)),
                key_value("checkpoint_count", std::to_string(run.checkpoints.size())),
                key_value(
                        "input_sample_count",
                        std::to_string(
                                run.accepted_input_count + run.applied_input_count
                                + run.no_evidence_input_count + run.rejected_input_count
                                + run.unavailable_input_count
                                + run.backend_fault_input_count)),
                key_value("unavailable_input_count",
                          std::to_string(run.unavailable_input_count)),
                key_value("revision", std::to_string(final_checkpoint.revision)),
                key_value("source_known_cells",
                          std::to_string(final_checkpoint.source_cell_count)),
                key_value("reconstructed_known_cells",
                          std::to_string(final_checkpoint.reconstructed_cell_count)),
                key_value("keyframe_count", std::to_string(run.keyframe_count)),
                key_value("delta_count", std::to_string(run.delta_count)),
                key_value("last_update_kind",
                          update_kind_name(final_checkpoint.update_kind)),
                key_value("missing_cells",
                          std::to_string(run.final_comparison.missing_cell_count)),
                key_value("unexpected_cells",
                          std::to_string(run.final_comparison.unexpected_cell_count)),
                key_value("state_mismatches",
                          std::to_string(run.final_comparison.state_mismatch_count)),
                key_value("content_hash",
                          PerceptionMapUpdate::hash_to_hex(final_checkpoint.content_hash))};
        array.status.push_back(std::move(status));
        if(resync_run.has_value()) {
            array.status.push_back(make_acceptance_diagnostic(
                    *resync_run, "resync_recovery"));
        }
        if(epoch_reset_run.has_value()) {
            array.status.push_back(make_acceptance_diagnostic(
                    *epoch_reset_run, "epoch_reset"));
        }
        return array;
    }

    diagnostic_msgs::msg::DiagnosticStatus make_acceptance_diagnostic(
            const MapUpdateAcceptanceRun & run,
            const std::string & name) const
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = std::string(get_fully_qualified_name()) + ": " + name;
        status.hardware_id = "deterministic-c3-acceptance-fixture";
        status.level = run.passed
                ? diagnostic_msgs::msg::DiagnosticStatus::OK
                : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = run.passed ? "acceptance scenario passed" : run.diagnostic;
        status.values.push_back(
                key_value("passed", run.passed ? "true" : "false"));
        status.values.push_back(
                key_value("stage_count", std::to_string(run.stages.size())));
        if(!run.stages.empty()) {
            const auto & final_stage = run.stages.back();
            status.values.push_back(key_value(
                    "final_receiver_state",
                    receiver_state_name(final_stage.receiver_state)));
            if(final_stage.receiver_map.has_value()) {
                status.values.push_back(key_value(
                        "final_epoch",
                        std::to_string(final_stage.receiver_map->source.map_epoch)));
                status.values.push_back(key_value(
                        "final_revision",
                        std::to_string(final_stage.receiver_map->revision)));
            }
        }
        for(std::size_t index = 0U; index < run.stages.size(); ++index) {
            if(run.stages[index].apply_status.has_value()) {
                status.values.push_back(key_value(
                        "stage_" + std::to_string(index) + "_apply_status",
                        apply_status_name(*run.stages[index].apply_status)));
            }
        }
        return status;
    }

    void publish_failure(const std::string & diagnostic)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name()
                      + std::string(": exact_revision_replay_oracle");
        status.hardware_id = "deterministic-c2-map-fixture";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = diagnostic;
        status.values.push_back(key_value("match", "false"));
        array.status.push_back(std::move(status));
        diagnostics_publisher_->publish(array);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<MapUpdateReplayVisualizationNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("map_update_replay_visualization_node"),
                "%s", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}

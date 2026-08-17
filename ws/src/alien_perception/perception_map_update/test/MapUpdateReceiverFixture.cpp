#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "perception_map_update/ResyncStateMachine.hpp"
#include "perception_map_update/ros/MapUpdateConversions.hpp"

#include "perception_interfaces/msg/local_map_state.hpp"
#include "perception_interfaces/msg/map_update.hpp"
#include "perception_interfaces/srv/request_map_resync.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

    using namespace std::chrono_literals;
    using PerceptionMapUpdate::CanonicalCell;
    using PerceptionMapUpdate::CanonicalSnapshot;
    using PerceptionMapUpdate::CellState;
    using PerceptionMapUpdate::ContentHasher;
    using PerceptionMapUpdate::MapUpdateProducer;
    using PerceptionMapUpdate::ResyncRequest;
    using PerceptionMapUpdate::ResyncRequestLedger;
    using PerceptionMapUpdate::SourceIdentity;

    CanonicalCell cell(std::int64_t x, CellState state)
    {
        return {{x, 0, 0}, state};
    }

    CanonicalSnapshot make_snapshot(
            const SourceIdentity & source,
            std::uint64_t revision,
            std::vector<CanonicalCell> cells)
    {
        CanonicalSnapshot snapshot;
        snapshot.source = source;
        snapshot.geometry = {0.5, {0.0, 0.0, 0.0}, "fixture/map"};
        snapshot.revision = revision;
        snapshot.latest_commit = {
                "fixture-lidar",
                {2'000U, 9U},
                Perception::Timestamp {static_cast<std::int64_t>(revision * 1'000'000U)},
                "fixture-clock",
                static_cast<std::uint32_t>(cells.size())};
        snapshot.cells = std::move(cells);
        snapshot.geometry_fingerprint = ContentHasher::geometry_fingerprint(snapshot.geometry);
        snapshot.content_hash = ContentHasher::content_hash(
                snapshot.source, snapshot.geometry_fingerprint, snapshot.cells);
        return snapshot;
    }

}// namespace

class MapUpdateReceiverFixture final : public rclcpp::Node
{
public:
    MapUpdateReceiverFixture()
        : Node("map_update_receiver_fixture")
        , source_({"fixture-vehicle", {1'000U, 3U}, 1U})
        , current_(snapshot_for(source_, 1U))
    {
        state_publisher_ = create_publisher<perception_interfaces::msg::LocalMapState>(
                "/fixture/local_map/state",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
        update_publisher_ = create_publisher<perception_interfaces::msg::MapUpdate>(
                "/fixture/local_map/updates",
                rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
        control_subscription_ = create_subscription<std_msgs::msg::String>(
                "/fixture/control",
                rclcpp::QoS(10).reliable().durability_volatile(),
                [this](const std_msgs::msg::String::SharedPtr message) {
                    on_control(message->data);
                });
        resync_service_ = create_service<perception_interfaces::srv::RequestMapResync>(
                "/fixture/local_map/request_resync",
                [this](
                        const std::shared_ptr<
                                perception_interfaces::srv::RequestMapResync::Request> request,
                        std::shared_ptr<
                                perception_interfaces::srv::RequestMapResync::Response> response) {
                    on_resync(*request, *response);
                });
        timer_ = create_wall_timer(50ms, [this]() {
            publish_state();
            dispatch_resync();
        });
    }

private:
    struct PendingResync {
        std::string correlation_id;
        std::chrono::steady_clock::time_point ready_at;
    };

    SourceIdentity source_;
    CanonicalSnapshot current_;
    MapUpdateProducer producer_;
    ResyncRequestLedger ledger_;
    std::deque<PendingResync> pending_resync_;
    std::optional<perception_interfaces::msg::MapUpdate> first_epoch_update_;
    std::optional<perception_interfaces::msg::MapUpdate> delta_two_;
    std::uint64_t state_sequence_ = 0U;

    rclcpp::Publisher<perception_interfaces::msg::LocalMapState>::SharedPtr state_publisher_;
    rclcpp::Publisher<perception_interfaces::msg::MapUpdate>::SharedPtr update_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscription_;
    rclcpp::Service<perception_interfaces::srv::RequestMapResync>::SharedPtr resync_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    static CanonicalSnapshot snapshot_for(
            const SourceIdentity & source,
            std::uint64_t revision)
    {
        switch(revision) {
            case 1U:
                return make_snapshot(source, revision, {cell(0, CellState::Free)});
            case 2U:
                return make_snapshot(
                        source,
                        revision,
                        {cell(0, CellState::Occupied), cell(1, CellState::Free)});
            case 3U:
                return make_snapshot(
                        source,
                        revision,
                        {cell(0, CellState::Occupied), cell(1, CellState::Occupied)});
            case 4U:
                return make_snapshot(
                        source,
                        revision,
                        {cell(0, CellState::Occupied),
                         cell(1, CellState::Occupied),
                         cell(2, CellState::Free)});
            default:
                return make_snapshot(
                        source,
                        revision,
                        {cell(0, CellState::Free),
                         cell(1, CellState::Occupied),
                         cell(2, CellState::Free)});
        }
    }

    std::optional<perception_interfaces::msg::MapUpdate> prepare(
            CanonicalSnapshot target,
            bool publish)
    {
        const auto prepared = producer_.prepare(target);
        if(!prepared.update.has_value()) {
            RCLCPP_ERROR(get_logger(), "fixture update preparation failed: %s",
                         prepared.diagnostic.c_str());
            return std::nullopt;
        }
        perception_interfaces::msg::MapUpdate message;
        message.header.stamp = get_clock()->now();
        std::string diagnostic;
        if(!PerceptionMapUpdate::Ros::encode_map_update(
                   *prepared.update, message, diagnostic)
           || !producer_.commit_published(prepared)) {
            RCLCPP_ERROR(get_logger(), "fixture update commit failed: %s", diagnostic.c_str());
            return std::nullopt;
        }
        current_ = std::move(target);
        if(publish) {
            update_publisher_->publish(message);
        }
        return message;
    }

    void publish_state()
    {
        perception_interfaces::msg::LocalMapState message;
        message.header.stamp = get_clock()->now();
        message.header.frame_id = current_.geometry.frame_id;
        message.vehicle_id = source_.vehicle_id;
        message.mapper_session_boot_time_ns = source_.mapper_session.boot_time_ns;
        message.mapper_session_random_suffix = source_.mapper_session.random_suffix;
        message.state_sequence = ++state_sequence_;
        message.map_epoch = source_.map_epoch;
        message.revision = current_.revision;
        message.resolution_m = current_.geometry.resolution_m;
        state_publisher_->publish(message);
    }

    void on_resync(
            const perception_interfaces::srv::RequestMapResync::Request & message,
            perception_interfaces::srv::RequestMapResync::Response & response_message)
    {
        ResyncRequest request;
        std::string diagnostic;
        const auto committed_state = producer_.committed_state();
        const auto committed_baseline = producer_.baseline();
        const auto current_identity = committed_state
                                              ? committed_state->identity()
                                              : PerceptionMapUpdate::VersionedContentDigest {};
        const auto current_revision = committed_baseline
                                              ? committed_baseline->revision
                                              : current_.revision;
        PerceptionMapUpdate::ResyncResponse response {
                false,
                {},
                source_,
                current_revision,
                current_identity,
                {}};
        if(!PerceptionMapUpdate::Ros::decode_resync_request(
                   message, request, {}, diagnostic)) {
            response.diagnostic = std::move(diagnostic);
        } else {
            response = ledger_.accept(
                    request,
                    source_,
                    current_revision,
                    current_identity);
            if(response.accepted) {
                const auto queued = std::find_if(
                        pending_resync_.begin(),
                        pending_resync_.end(),
                        [&](const PendingResync & pending) {
                            return pending.correlation_id == response.correlation_id;
                        });
                if(queued == pending_resync_.end()) {
                    pending_resync_.push_back(
                            {response.correlation_id,
                             std::chrono::steady_clock::now() + 300ms});
                }
            }
        }
        PerceptionMapUpdate::Ros::encode_resync_response(response, response_message);
    }

    void dispatch_resync()
    {
        if(pending_resync_.empty()
           || pending_resync_.front().ready_at > std::chrono::steady_clock::now()) {
            return;
        }
        auto correlation = std::move(pending_resync_.front().correlation_id);
        pending_resync_.pop_front();
        if(!producer_.request_keyframe(correlation)) {
            RCLCPP_ERROR(get_logger(), "fixture producer rejected resync correlation");
            return;
        }
        const auto message = prepare(current_, true);
        if(message.has_value() && source_.map_epoch == 1U
           && !first_epoch_update_.has_value()) {
            first_epoch_update_ = message;
        }
    }

    void on_control(const std::string & command)
    {
        if(command == "delta2") {
            delta_two_ = prepare(snapshot_for(source_, 2U), true);
        } else if(command == "duplicate2" && delta_two_.has_value()) {
            update_publisher_->publish(*delta_two_);
        } else if(command == "gap4") {
            prepare(snapshot_for(source_, 3U), false);
            prepare(snapshot_for(source_, 4U), true);
        } else if(command == "old2" && delta_two_.has_value()) {
            update_publisher_->publish(*delta_two_);
        } else if(command == "corrupt5") {
            auto corrupt = prepare(snapshot_for(source_, 5U), false);
            if(corrupt.has_value()) {
                ++corrupt->canonical_payload_bytes;
                update_publisher_->publish(*corrupt);
            }
        } else if(command == "epoch2") {
            source_.map_epoch = 2U;
            current_ = snapshot_for(source_, 1U);
            publish_state();
            prepare(current_, true);
        } else if(command == "old_epoch" && first_epoch_update_.has_value()) {
            update_publisher_->publish(*first_epoch_update_);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapUpdateReceiverFixture>());
    rclcpp::shutdown();
    return 0;
}

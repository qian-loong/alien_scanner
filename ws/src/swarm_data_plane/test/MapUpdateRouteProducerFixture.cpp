#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "perception_map_update/ros/MapUpdateConversions.hpp"
#include "swarm_data_plane/RoutedResync.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedResyncConversions.hpp"
#include "swarm_data_interfaces/srv/request_routed_map_resync.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        PerceptionMapUpdate::CanonicalSnapshot snapshot(
                std::uint64_t revision,
                std::vector<PerceptionMapUpdate::CanonicalCell> cells)
        {
            using namespace PerceptionMapUpdate;
            CanonicalSnapshot result;
            result.source = {"drone_0", {100U, 7U}, 1U};
            result.geometry = {0.1, {0.0, 0.0, 0.0}, "drone_0/map"};
            result.revision = revision;
            result.latest_commit = {
                    "lidar",
                    {200U, 9U},
                    Perception::Timestamp {3'000'000U
                                           + static_cast<std::int64_t>(revision)},
                    "sim-clock",
                    1U};
            result.cells = std::move(cells);
            result.geometry_fingerprint = ContentHasher::geometry_fingerprint(
                    result.geometry);
            return result;
        }

    }// namespace

    class MapUpdateRouteProducerFixture final : public rclcpp::Node
    {
    public:
        MapUpdateRouteProducerFixture()
                : Node("map_update_route_producer_fixture"),
                  current_snapshot_(snapshot(
                          1U,
                          {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free},
                           {{1, 0, 0}, PerceptionMapUpdate::CellState::Free},
                           {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}})),
                  resync_ledger_({"mapper_endpoint", {300U, 11U}}, 1U)
        {
            const auto output_topic = declare_parameter<std::string>(
                    "output_topic", "/c4/source_map");
            const auto service_name = declare_parameter<std::string>(
                    "resync_service", "/c4/resync");
            publisher_ = create_publisher<perception_interfaces::msg::MapUpdate>(
                    output_topic, Ros::map_update_qos());
            resync_service_ = create_service<
                    swarm_data_interfaces::srv::RequestRoutedMapResync>(
                    service_name,
                    [this](
                            const std::shared_ptr<swarm_data_interfaces::srv::
                                                          RequestRoutedMapResync::Request>
                                    request,
                            std::shared_ptr<swarm_data_interfaces::srv::
                                                    RequestRoutedMapResync::Response>
                                    response) {
                        on_resync(*request, *response);
                    });
            start_time_ = std::chrono::steady_clock::now();
            publish_timer_ = create_wall_timer(
                    std::chrono::milliseconds(50), [this]() { publish_next(); });
        }

    private:
        bool publish_prepared(PerceptionMapUpdate::PreparedUpdate prepared)
        {
            if(!prepared.update.has_value()) {
                RCLCPP_ERROR(get_logger(), "%s", prepared.diagnostic.c_str());
                return false;
            }
            perception_interfaces::msg::MapUpdate message;
            std::string diagnostic;
            if(!PerceptionMapUpdate::Ros::encode_map_update(
                       *prepared.update, message, diagnostic)) {
                RCLCPP_ERROR(get_logger(), "%s", diagnostic.c_str());
                return false;
            }
            message.header.stamp = now();
            if(!producer_.commit_published(prepared)) {
                RCLCPP_ERROR(get_logger(), "cannot commit prepared fixture update");
                return false;
            }
            publisher_->publish(message);
            return true;
        }

        void publish_next()
        {
            if(publisher_->get_subscription_count() == 0U
               || std::chrono::steady_clock::now() - start_time_
                          < std::chrono::seconds(1)) {
                return;
            }
            if(stage_ == 0U) {
                if(publish_prepared(producer_.prepare(current_snapshot_))) {
                    ++stage_;
                    next_stage_time_ = std::chrono::steady_clock::now()
                                       + std::chrono::milliseconds(300);
                }
                return;
            }
            if(std::chrono::steady_clock::now() < next_stage_time_) {
                return;
            }
            if(stage_ == 1U) {
                current_snapshot_ = snapshot(
                        2U,
                        {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                         {{1, 0, 0}, PerceptionMapUpdate::CellState::Free},
                         {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
                if(publish_prepared(producer_.prepare(current_snapshot_))) {
                    ++stage_;
                    next_stage_time_ = std::chrono::steady_clock::now()
                                       + std::chrono::milliseconds(300);
                }
                return;
            }
            if(stage_ == 2U) {
                current_snapshot_ = snapshot(
                        3U,
                        {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                         {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                         {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
                if(publish_prepared(producer_.prepare(current_snapshot_))) {
                    ++stage_;
                }
            }
        }

        void on_resync(
                const swarm_data_interfaces::srv::RequestRoutedMapResync::Request & request,
                swarm_data_interfaces::srv::RequestRoutedMapResync::Response & response)
        {
            const auto current_content_identity = producer_.committed_state()
                                                          ? producer_.committed_state()->identity()
                                                          : PerceptionMapUpdate::
                                                                    VersionedContentDigest {};
            const auto decoded = Ros::decode_resync_intent(request.intent);
            if(!decoded.success || !decoded.intent.has_value()) {
                RoutedResyncAck rejected;
                rejected.target_producer = {"mapper_endpoint", {300U, 11U}};
                rejected.current_source = current_snapshot_.source;
                rejected.current_revision = current_snapshot_.revision;
                rejected.current_content_identity = current_content_identity;
                rejected.diagnostic = decoded.diagnostic;
                std::string diagnostic;
                Ros::encode_resync_ack(rejected, response.ack, diagnostic);
                return;
            }
            const auto ack = resync_ledger_.accept(
                    *decoded.intent,
                    current_snapshot_.source,
                    current_snapshot_.revision,
                    current_content_identity);
            std::string diagnostic;
            if(!Ros::encode_resync_ack(ack, response.ack, diagnostic)) {
                RCLCPP_ERROR(get_logger(), "%s", diagnostic.c_str());
                return;
            }
            if(!ack.accepted || !producer_.request_keyframe(ack.correlation_id)) {
                return;
            }
            auto prepared = producer_.prepare(current_snapshot_);
            if(!prepared.update.has_value()) {
                RCLCPP_ERROR(get_logger(), "%s", prepared.diagnostic.c_str());
                return;
            }
            perception_interfaces::msg::MapUpdate message;
            if(!PerceptionMapUpdate::Ros::encode_map_update(
                       *prepared.update, message, diagnostic)
               || !producer_.commit_published(prepared)) {
                RCLCPP_ERROR(get_logger(), "cannot prepare correlated recovery keyframe");
                return;
            }
            message.header.stamp = now();
            pending_recovery_ = std::move(message);
            recovery_timer_ = create_wall_timer(
                    std::chrono::milliseconds(100), [this]() {
                        if(pending_recovery_.has_value()) {
                            publisher_->publish(*pending_recovery_);
                            pending_recovery_.reset();
                        }
                        recovery_timer_->cancel();
                    });
        }

        PerceptionMapUpdate::MapUpdateProducer producer_;
        PerceptionMapUpdate::CanonicalSnapshot current_snapshot_;
        RoutedResyncLedger resync_ledger_;
        std::uint8_t stage_ = 0U;
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::steady_clock::time_point next_stage_time_;
        std::optional<perception_interfaces::msg::MapUpdate> pending_recovery_;
        rclcpp::Publisher<perception_interfaces::msg::MapUpdate>::SharedPtr publisher_;
        rclcpp::Service<swarm_data_interfaces::srv::RequestRoutedMapResync>::SharedPtr
                resync_service_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr recovery_timer_;
    };

}// namespace SwarmDataPlane::Test

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<
                 SwarmDataPlane::Test::MapUpdateRouteProducerFixture>());
    rclcpp::shutdown();
    return 0;
}

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
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        PerceptionMapUpdate::CanonicalSnapshot snapshot(
                std::uint64_t revision,
                std::vector<PerceptionMapUpdate::CanonicalCell> cells,
                const std::string & source_vehicle_id = "drone_0",
                Perception::SessionID source_session = {100U, 7U},
                const std::string & map_frame = "drone_0/map")
        {
            using namespace PerceptionMapUpdate;
            CanonicalSnapshot result;
            result.source = {source_vehicle_id, source_session, 1U};
            result.geometry = {0.1, {0.0, 0.0, 0.0}, map_frame};
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
                : Node("map_update_route_producer_fixture")
        {
            const auto output_topic = declare_parameter<std::string>(
                    "output_topic", "/c4/source_map");
            const auto service_name = declare_parameter<std::string>(
                    "resync_service", "/c4/resync");
            source_vehicle_id_ = declare_parameter<std::string>(
                    "source_vehicle_id", "drone_0");
            const auto source_boot_time = declare_parameter<std::int64_t>(
                    "source_session_boot_time_ns", 100);
            const auto source_random_suffix = declare_parameter<std::int64_t>(
                    "source_session_random_suffix", 7);
            map_frame_ = declare_parameter<std::string>(
                    "map_frame", "drone_0/map");
            producer_identity_.producer_id = declare_parameter<std::string>(
                    "producer_id", "mapper_endpoint");
            const auto producer_boot_time = declare_parameter<std::int64_t>(
                    "producer_session_boot_time_ns", 300);
            const auto producer_random_suffix = declare_parameter<std::int64_t>(
                    "producer_session_random_suffix", 11);
            const auto route_epoch = declare_parameter<std::int64_t>(
                    "resync_route_epoch", 1);
            const auto continuous_update_period_ms = declare_parameter<std::int64_t>(
                    "continuous_update_period_ms", 0);
            const auto initial_publish_delay_ms = declare_parameter<std::int64_t>(
                    "initial_publish_delay_ms", 1'000);
            if(source_vehicle_id_.empty() || map_frame_.empty()
                || source_boot_time <= 0 || source_random_suffix <= 0
               || producer_identity_.producer_id.empty()
               || producer_boot_time <= 0 || producer_random_suffix <= 0
               || route_epoch <= 0 || continuous_update_period_ms < 0
               || initial_publish_delay_ms < 0
               || static_cast<std::uint64_t>(source_random_suffix)
                          > std::numeric_limits<std::uint32_t>::max()
               || static_cast<std::uint64_t>(producer_random_suffix)
                          > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument("fixture source identity is invalid");
            }
            source_session_ = {
                    static_cast<std::uint64_t>(source_boot_time),
                    static_cast<std::uint32_t>(source_random_suffix)};
            producer_identity_.session = {
                    static_cast<std::uint64_t>(producer_boot_time),
                    static_cast<std::uint32_t>(producer_random_suffix)};
            resync_ledger_ = std::make_unique<RoutedResyncLedger>(
                    producer_identity_, static_cast<std::uint64_t>(route_epoch));
            continuous_update_period_ =
                    std::chrono::milliseconds(continuous_update_period_ms);
            initial_publish_delay_ =
                    std::chrono::milliseconds(initial_publish_delay_ms);
            current_snapshot_ = snapshot(
                    1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Free},
                     {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}},
                    source_vehicle_id_, source_session_, map_frame_);
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
                          < initial_publish_delay_) {
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
                         {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}},
                        source_vehicle_id_, source_session_, map_frame_);
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
                         {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}},
                        source_vehicle_id_, source_session_, map_frame_);
                if(publish_prepared(producer_.prepare(current_snapshot_))) {
                    ++stage_;
                    if(continuous_update_period_.count() > 0) {
                        next_stage_time_ = std::chrono::steady_clock::now()
                                           + continuous_update_period_;
                    }
                }
                return;
            }
            if(stage_ == 3U && continuous_update_period_.count() > 0) {
                const auto next_revision = current_snapshot_.revision + 1U;
                const auto tail_state = next_revision % 2U == 0U
                        ? PerceptionMapUpdate::CellState::Free
                        : PerceptionMapUpdate::CellState::Occupied;
                current_snapshot_ = snapshot(
                        next_revision,
                        {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                         {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                         {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                         {{3, 0, 0}, tail_state}},
                        source_vehicle_id_, source_session_, map_frame_);
                if(publish_prepared(producer_.prepare(current_snapshot_))) {
                    next_stage_time_ = std::chrono::steady_clock::now()
                                       + continuous_update_period_;
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
                rejected.target_producer = producer_identity_;
                rejected.current_source = current_snapshot_.source;
                rejected.current_revision = current_snapshot_.revision;
                rejected.current_content_identity = current_content_identity;
                rejected.diagnostic = decoded.diagnostic;
                std::string diagnostic;
                Ros::encode_resync_ack(rejected, response.ack, diagnostic);
                return;
            }
            const auto ack = resync_ledger_->accept(
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
            message.correlation_id = ack.correlation_id;
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
        ProducerIdentity producer_identity_;
        std::unique_ptr<RoutedResyncLedger> resync_ledger_;
        std::string source_vehicle_id_ {"drone_0"};
        Perception::SessionID source_session_ {100U, 7U};
        std::string map_frame_ {"drone_0/map"};
        std::uint8_t stage_ = 0U;
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::milliseconds initial_publish_delay_ {1'000};
        std::chrono::steady_clock::time_point next_stage_time_;
        std::chrono::milliseconds continuous_update_period_ {0};
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

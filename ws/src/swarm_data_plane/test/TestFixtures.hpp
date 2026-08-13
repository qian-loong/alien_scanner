#ifndef SWARM_DATA_PLANE_TEST_TEST_FIXTURES_HPP
#define SWARM_DATA_PLANE_TEST_TEST_FIXTURES_HPP

#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    inline PerceptionMapUpdate::CanonicalSnapshot snapshot(
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
                Perception::Timestamp {3'000'000U + static_cast<std::int64_t>(revision)},
                "sim-clock",
                static_cast<std::uint32_t>(cells.size())};
        result.cells = std::move(cells);
        result.geometry_fingerprint = ContentHasher::geometry_fingerprint(result.geometry);
        result.content_hash = ContentHasher::content_hash(
                result.source, result.geometry_fingerprint, result.cells);
        return result;
    }

    inline std::shared_ptr<const PerceptionMapUpdate::MapUpdate> shared_update(
            const PerceptionMapUpdate::PreparedUpdate & prepared)
    {
        if(!prepared.update.has_value()) {
            return nullptr;
        }
        return std::make_shared<const PerceptionMapUpdate::MapUpdate>(*prepared.update);
    }

    inline LogicalPriority priority_for(PerceptionMapUpdate::UpdateKind kind)
    {
        switch(kind) {
            case PerceptionMapUpdate::UpdateKind::Keyframe:
            case PerceptionMapUpdate::UpdateKind::Remove:
                return LogicalPriority::MapKeyframe;
            case PerceptionMapUpdate::UpdateKind::Delta:
                return LogicalPriority::MapDelta;
            case PerceptionMapUpdate::UpdateKind::Summary:
                return LogicalPriority::Summary;
        }
        return LogicalPriority::Diagnostic;
    }

    inline RoutedMapUpdate routed(
            std::shared_ptr<const PerceptionMapUpdate::MapUpdate> update,
            std::uint64_t sequence,
            std::string message_id,
            std::uint64_t route_epoch = 1U)
    {
        RoutedMapUpdate message;
        message.message_id = std::move(message_id);
        message.producer = {"mapper_endpoint", {300U, 11U}};
        message.sequence = sequence;
        message.correlation_id = update->correlation_id;
        message.priority = priority_for(update->kind);
        message.origin = {"steady-sim", {400U, 12U}, 5'000'000U};
        message.validity_budget_ns = 1'000'000'000U;
        message.accumulated_forwarding_ns = 10U;
        message.route = {route_epoch, 0U, 8U};
        message.payload_bytes = update->canonical_payload_bytes;
        message.payload_hash = update->update_hash;
        message.update = std::move(update);
        return message;
    }

}// namespace SwarmDataPlane::Test

#endif// SWARM_DATA_PLANE_TEST_TEST_FIXTURES_HPP

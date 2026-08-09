#ifndef PERCEPTION_MAP_UPDATE_MAP_UPDATE_LIMITS_HPP
#define PERCEPTION_MAP_UPDATE_MAP_UPDATE_LIMITS_HPP

#include <cstddef>
#include <cstdint>

namespace PerceptionMapUpdate {

    struct MapUpdateLimits {
        std::size_t max_identity_string_bytes = 256U;
        std::size_t max_frame_id_bytes         = 256U;
        std::size_t max_sensor_id_bytes        = 256U;
        std::size_t max_clock_domain_bytes     = 256U;
        std::size_t max_correlation_id_bytes   = 256U;
        std::size_t max_known_cells            = 3'000'000U;
        std::size_t max_delta_operations       = 3'000'000U;
        std::size_t max_keyframe_payload_bytes = 128U * 1024U * 1024U;
        std::size_t max_delta_payload_bytes    = 128U * 1024U * 1024U;
        std::size_t max_receiver_cells         = 3'000'000U;
        std::size_t max_peak_apply_bytes       = 384U * 1024U * 1024U;
        std::size_t max_retained_snapshot_bytes = 192U * 1024U * 1024U;
        std::size_t max_recent_resync_requests = 64U;
        std::uint64_t max_delta_chain_length   = 128U;
        std::uint64_t max_revision_span        = 1'000'000U;
        std::uint64_t periodic_keyframe_revision_interval = 0U;
        bool delta_enabled = true;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MAP_UPDATE_LIMITS_HPP

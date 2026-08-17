#ifndef PERCEPTION_PROFILING_PROFILE_DATA_SINK_HPP
#define PERCEPTION_PROFILING_PROFILE_DATA_SINK_HPP

#include "perception_profiling/ProfileOracle.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace PerceptionProfiling {

    enum class C3ProfileMode {
        Disabled,
        Enabled,
        KeyframeOnly,
    };

    C3ProfileMode parse_c3_profile_mode(const std::string & value);
    const char * c3_profile_mode_name(C3ProfileMode mode) noexcept;

    struct HealthProjection {
        std::int64_t receipt_monotonic_ns = 0;
        std::int64_t stamp_ns = 0;
        std::uint8_t state = 0U;
        std::string producer_source_id;
        Perception::SessionID producer_session {0U, 0U};
        std::string contract_fingerprint;
        bool has_full_no_return_rays = false;
        std::uint32_t active_sensor_count = 0U;
    };

    struct DiagnosticProjection {
        std::int64_t receipt_monotonic_ns = 0;
        std::int64_t stamp_ns = 0;
        std::uint8_t level = 0U;
        std::string name;
        std::string message;
        std::vector<std::pair<std::string, std::string>> values;
    };

    struct MapUpdateProjection {
        std::int64_t receipt_monotonic_ns = 0;
        std::int64_t stamp_ns = 0;
        std::uint16_t protocol_version = 2U;
        std::uint16_t canonical_encoding_version = 1U;
        std::uint8_t hash_algorithm = 1U;
        std::uint16_t content_identity_scheme = 2U;
        std::uint32_t content_identity_chunk_edge = 16U;
        std::uint16_t content_identity_coordinate_key_version = 1U;
        std::uint16_t content_identity_node_encoding_version = 1U;
        std::uint8_t update_kind = 0U;
        std::string vehicle_id;
        Perception::SessionID mapper_session {0U, 0U};
        std::uint64_t map_epoch = 0U;
        std::uint64_t base_revision = 0U;
        std::uint64_t new_revision = 0U;
        std::uint64_t revision_span = 0U;
        std::uint64_t observed_coalesced_receipt_count = 0U;
        std::uint64_t known_cell_count = 0U;
        std::uint64_t operation_count = 0U;
        std::uint64_t canonical_payload_bytes = 0U;
        std::array<std::uint8_t, 32> base_content_hash {};
        std::array<std::uint8_t, 32> content_hash {};
        std::array<std::uint8_t, 32> update_hash {};
    };

    struct SnapshotProjection {
        std::int64_t receipt_monotonic_ns = 0;
        std::int64_t stamp_ns = 0;
        std::uint64_t ordinal = 0U;
        bool binary = false;
        double resolution_m = 0.0;
        std::uint64_t data_bytes = 0U;
    };

    struct SinkSummary {
        std::uint64_t observation_count = 0U;
        std::uint64_t unique_observation_count = 0U;
        std::uint64_t duplicate_observation_count = 0U;
        std::uint64_t out_of_order_observation_count = 0U;
        std::uint64_t observation_sequence_gaps = 0U;
        std::uint64_t observation_schema_errors = 0U;
        std::uint64_t observation_digest_mismatches = 0U;
        std::uint64_t state_count = 0U;
        std::uint64_t distinct_revision_count = 0U;
        std::uint64_t state_sequence_duplicates = 0U;
        std::uint64_t state_sequence_out_of_order = 0U;
        std::uint64_t state_sequence_gaps = 0U;
        std::uint64_t revision_heartbeat_count = 0U;
        std::uint64_t revision_regressions = 0U;
        std::uint64_t revision_gaps = 0U;
        std::uint64_t epoch_changes = 0U;
        std::uint64_t fingerprint_mismatches = 0U;
        std::uint64_t health_count = 0U;
        std::uint64_t diagnostic_warn_or_error_count = 0U;
        std::uint64_t snapshot_count = 0U;
        std::uint64_t map_update_count = 0U;
        std::uint64_t map_update_keyframe_count = 0U;
        std::uint64_t map_update_delta_count = 0U;
        std::uint64_t map_update_revision_only_delta_count = 0U;
        std::uint64_t producer_diagnostic_count = 0U;
        bool normal_completion = false;
    };

    class ProfileDataSink
    {
    public:
        ProfileDataSink(
                std::filesystem::path output_directory,
                ProfileScenario scenario,
                C3ProfileMode c3_mode = C3ProfileMode::Disabled);
        ~ProfileDataSink();

        ProfileDataSink(const ProfileDataSink &) = delete;
        ProfileDataSink & operator=(const ProfileDataSink &) = delete;

        void record_observation(
                const Perception::LidarObservation & observation,
                std::int64_t receipt_monotonic_ns);
        void record_state(const ProductionStateProjection & state);
        void record_health(const HealthProjection & health);
        void record_diagnostic(const DiagnosticProjection & diagnostic);
        void record_snapshot(const SnapshotProjection & snapshot);
        void record_map_update(const MapUpdateProjection & update);
        void finalize(bool normal_completion);

        const SinkSummary & summary() const noexcept;

    private:
        std::filesystem::path output_directory_;
        ProfileScenario scenario_;
        C3ProfileMode c3_mode_ = C3ProfileMode::Disabled;
        SinkSummary summary_;
        std::ofstream observations_;
        std::ofstream states_;
        std::ofstream health_;
        std::ofstream diagnostics_;
        std::ofstream snapshots_;
        std::ofstream map_updates_;
        std::ofstream producer_diagnostics_;
        std::optional<std::uint64_t> last_observation_sequence_;
        std::set<std::uint64_t> observation_sequences_;
        std::optional<std::uint64_t> last_revision_;
        std::optional<std::uint64_t> last_state_sequence_;
        std::optional<std::uint64_t> map_epoch_;
        bool finalized_ = false;

        void write_manifest() const;
    };

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_PROFILE_DATA_SINK_HPP

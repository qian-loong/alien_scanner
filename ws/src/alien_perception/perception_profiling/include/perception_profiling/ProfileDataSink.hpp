#ifndef PERCEPTION_PROFILING_PROFILE_DATA_SINK_HPP
#define PERCEPTION_PROFILING_PROFILE_DATA_SINK_HPP

#include "perception_profiling/ProfileOracle.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <string>

namespace PerceptionProfiling {

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
        bool normal_completion = false;
    };

    class ProfileDataSink
    {
    public:
        ProfileDataSink(std::filesystem::path output_directory, ProfileScenario scenario);
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
        void finalize(bool normal_completion);

        const SinkSummary & summary() const noexcept;

    private:
        std::filesystem::path output_directory_;
        ProfileScenario scenario_;
        SinkSummary summary_;
        std::ofstream observations_;
        std::ofstream states_;
        std::ofstream health_;
        std::ofstream diagnostics_;
        std::ofstream snapshots_;
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

#include "perception_profiling/ProfileDataSink.hpp"

#include "perception_map_update/MapUpdateTypes.hpp"

#include <yaml-cpp/yaml.h>

#include <charconv>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

namespace PerceptionProfiling {
    namespace {

        std::string csv_escape(const std::string & value)
        {
            if(value.find_first_of(",\"\r\n") == std::string::npos) {
                return value;
            }
            std::string escaped = "\"";
            for(const char character : value) {
                if(character == '\"') {
                    escaped += "\"\"";
                }
                else {
                    escaped += character;
                }
            }
            escaped += '\"';
            return escaped;
        }

        std::ofstream open_new_file(
                const std::filesystem::path & directory,
                const std::string & name)
        {
            const auto path = directory / name;
            if(std::filesystem::exists(path)) {
                throw std::invalid_argument("profile sink refuses to overwrite " + path.string());
            }
            std::ofstream stream(path, std::ios::out | std::ios::trunc);
            if(!stream) {
                throw std::runtime_error("could not create " + path.string());
            }
            return stream;
        }

        std::string hash_hex(const std::array<std::uint8_t, 32> & value)
        {
            std::ostringstream stream;
            stream << std::hex << std::setfill('0');
            for(const auto byte : value) {
                stream << std::setw(2) << static_cast<unsigned int>(byte);
            }
            return stream.str();
        }

        bool is_map_update_producer_diagnostic(const std::string & name)
        {
            constexpr std::string_view suffix = ": map_update_producer";
            return name.size() >= suffix.size()
                   && name.compare(
                              name.size() - suffix.size(), suffix.size(),
                              suffix.data(), suffix.size()) == 0;
        }

        std::map<std::string, std::string> diagnostic_values(
                const DiagnosticProjection & diagnostic)
        {
            std::map<std::string, std::string> result;
            for(const auto & field : diagnostic.values) {
                if(field.first.empty() || !result.emplace(field).second) {
                    throw std::invalid_argument(
                            "producer diagnostic contains an empty or duplicate key");
                }
            }
            return result;
        }

        template<typename Integer>
        Integer required_integer(
                const std::map<std::string, std::string> & values,
                const char * key)
        {
            const auto found = values.find(key);
            if(found == values.end() || found->second.empty()) {
                throw std::invalid_argument(
                        std::string("producer diagnostic is missing ") + key);
            }
            Integer result {};
            const char * begin = found->second.data();
            const char * end = begin + found->second.size();
            const auto parsed = std::from_chars(begin, end, result);
            if(parsed.ec != std::errc {} || parsed.ptr != end) {
                throw std::invalid_argument(
                        std::string("producer diagnostic has invalid ") + key);
            }
            return result;
        }

        bool required_bool(
                const std::map<std::string, std::string> & values,
                const char * key)
        {
            const auto value = required_integer<unsigned int>(values, key);
            if(value > 1U) {
                throw std::invalid_argument(
                        std::string("producer diagnostic has invalid ") + key);
            }
            return value == 1U;
        }

    }// namespace

    C3ProfileMode parse_c3_profile_mode(const std::string & value)
    {
        if(value == "disabled") {
            return C3ProfileMode::Disabled;
        }
        if(value == "enabled") {
            return C3ProfileMode::Enabled;
        }
        if(value == "keyframe-only") {
            return C3ProfileMode::KeyframeOnly;
        }
        throw std::invalid_argument("c3_mode must be disabled, enabled, or keyframe-only");
    }

    const char * c3_profile_mode_name(C3ProfileMode mode) noexcept
    {
        switch(mode) {
            case C3ProfileMode::Disabled:
                return "disabled";
            case C3ProfileMode::Enabled:
                return "enabled";
            case C3ProfileMode::KeyframeOnly:
                return "keyframe-only";
        }
        return "invalid";
    }

    ProfileDataSink::ProfileDataSink(
            std::filesystem::path output_directory,
            ProfileScenario scenario,
            C3ProfileMode c3_mode)
        : output_directory_(std::move(output_directory))
        , scenario_(std::move(scenario))
        , c3_mode_(c3_mode)
    {
        if(output_directory_.empty()) {
            throw std::invalid_argument("profile sink output directory must not be empty");
        }
        std::filesystem::create_directories(output_directory_);
        observations_ = open_new_file(output_directory_, "observations.csv");
        states_ = open_new_file(output_directory_, "states.csv");
        health_ = open_new_file(output_directory_, "health.csv");
        diagnostics_ = open_new_file(output_directory_, "diagnostics.csv");
        snapshots_ = open_new_file(output_directory_, "snapshots.csv");
        map_updates_ = open_new_file(output_directory_, "map_updates.csv");
        producer_diagnostics_ = open_new_file(
                output_directory_, "map_update_producer_diagnostics.csv");
        observations_ << "receipt_monotonic_ns,sequence,stamp_ns,payload_digest,"
                         "expected_digest,schema_valid,digest_matches\n";
        states_ << "receipt_monotonic_ns,state_sequence,map_epoch,revision,stamp_ns,fingerprint,"
                   "changed_cell_count,has_bounds,min_x,min_y,min_z,max_x,max_y,max_z\n";
        health_ << "receipt_monotonic_ns,stamp_ns,state,producer_source_id,"
                   "session_boot_ns,session_suffix,fingerprint,full_no_return,active_sensor_count\n";
        diagnostics_ << "receipt_monotonic_ns,stamp_ns,level,name,message\n";
        snapshots_ << "receipt_monotonic_ns,stamp_ns,ordinal,binary,resolution_m,data_bytes\n";
        map_updates_ << "receipt_monotonic_ns,stamp_ns,update_kind,vehicle_id,"
                        "mapper_session_boot_ns,mapper_session_suffix,map_epoch,base_revision,"
                        "new_revision,revision_span,observed_coalesced_receipt_count,"
                        "known_cell_count,operation_count,canonical_payload_bytes,"
                        "base_content_hash,content_hash,update_hash\n";
        producer_diagnostics_ << "receipt_monotonic_ns,stamp_ns,level,name,message,pending,"
                                 "in_flight,pending_revision,in_flight_revision,"
                                 "published_revision,coalesced_receipts,superseded_receipts,"
                                 "published_keyframes,published_deltas,revision_only_deltas,"
                                 "publish_failures,resource_rejections,snapshot_cells,"
                                 "delta_operations,payload_bytes,acquire_duration_ns,"
                                 "materialize_duration_ns,traversal_duration_ns,"
                                 "canonicalize_duration_ns,content_hash_duration_ns,"
                                 "prepare_duration_ns,validation_duration_ns,diff_duration_ns,"
                                 "encode_duration_ns,update_hash_duration_ns,publish_duration_ns\n";
        write_manifest();
    }

    ProfileDataSink::~ProfileDataSink()
    {
        try {
            finalize(false);
        }
        catch(...) {
        }
    }

    void ProfileDataSink::record_observation(
            const Perception::LidarObservation & observation,
            std::int64_t receipt_monotonic_ns)
    {
        ++summary_.observation_count;
        const auto sequence = scenario_.sequence_for_stamp(observation.origin_stamp);
        bool schema_valid = sequence.has_value();
        std::string expected_digest;
        const std::string actual_digest = ProfileScenario::payload_digest(observation);
        if(sequence.has_value()) {
            const auto expected = scenario_.sample(sequence.value());
            expected_digest = expected.payload_digest;
            schema_valid = observation.is_2d()
                           && observation.sensor_id == expected.observation.sensor_id
                           && observation.session_id == expected.observation.session_id
                           && observation.frame_id == expected.observation.frame_id
                           && observation.clock_domain == expected.observation.clock_domain
                           && observation.ray_evidence == expected.observation.ray_evidence;
            if(observation_sequences_.insert(sequence.value()).second) {
                ++summary_.unique_observation_count;
            }
            else {
                ++summary_.duplicate_observation_count;
            }
            if(last_observation_sequence_.has_value()) {
                if(sequence.value() < last_observation_sequence_.value()) {
                    ++summary_.out_of_order_observation_count;
                }
                else if(sequence.value() > last_observation_sequence_.value() + 1U) {
                    summary_.observation_sequence_gaps +=
                            sequence.value() - last_observation_sequence_.value() - 1U;
                }
            }
            if(!last_observation_sequence_.has_value()
               || sequence.value() > last_observation_sequence_.value()) {
                last_observation_sequence_ = sequence;
            }
        }
        if(!schema_valid) {
            ++summary_.observation_schema_errors;
        }
        const bool digest_matches = schema_valid && actual_digest == expected_digest;
        if(!digest_matches) {
            ++summary_.observation_digest_mismatches;
        }
        observations_ << receipt_monotonic_ns << ','
                      << (sequence.has_value() ? std::to_string(sequence.value()) : "") << ','
                      << observation.origin_stamp.nanoseconds << ',' << actual_digest << ','
                      << expected_digest << ',' << (schema_valid ? 1 : 0) << ','
                      << (digest_matches ? 1 : 0) << '\n';
        observations_.flush();
    }

    void ProfileDataSink::record_state(const ProductionStateProjection & state)
    {
        ++summary_.state_count;
        if(last_state_sequence_.has_value()) {
            if(state.state_sequence == last_state_sequence_.value()) {
                ++summary_.state_sequence_duplicates;
            }
            else if(state.state_sequence < last_state_sequence_.value()) {
                ++summary_.state_sequence_out_of_order;
            }
            else if(state.state_sequence > last_state_sequence_.value() + 1U) {
                summary_.state_sequence_gaps +=
                        state.state_sequence - last_state_sequence_.value() - 1U;
            }
        }
        if(!last_state_sequence_.has_value()
           || state.state_sequence > last_state_sequence_.value()) {
            last_state_sequence_ = state.state_sequence;
        }
        if(state.contract_fingerprint != scenario_.contract_fingerprint().hex_digest) {
            ++summary_.fingerprint_mismatches;
        }
        if(!map_epoch_.has_value()) {
            map_epoch_ = state.map_epoch;
        }
        else if(map_epoch_.value() != state.map_epoch) {
            ++summary_.epoch_changes;
            map_epoch_ = state.map_epoch;
            last_revision_.reset();
        }
        if(state.revision > 0U) {
            if(!last_revision_.has_value()) {
                ++summary_.distinct_revision_count;
                last_revision_ = state.revision;
            }
            else if(state.revision == last_revision_.value()) {
                ++summary_.revision_heartbeat_count;
            }
            else if(state.revision < last_revision_.value()) {
                ++summary_.revision_regressions;
            }
            else {
                if(state.revision != last_revision_.value() + 1U) {
                    summary_.revision_gaps += state.revision - last_revision_.value() - 1U;
                }
                ++summary_.distinct_revision_count;
                last_revision_ = state.revision;
            }
        }

        states_ << state.receipt_monotonic_ns << ',' << state.state_sequence << ','
                << state.map_epoch << ','
                << state.revision << ',' << state.observation_stamp_ns << ','
                << state.contract_fingerprint << ',' << state.changed_cell_count << ','
                << (state.bounds.has_value() ? 1 : 0);
        if(state.bounds.has_value()) {
            states_ << ',' << state.bounds->minimum.x << ',' << state.bounds->minimum.y
                    << ',' << state.bounds->minimum.z << ',' << state.bounds->maximum.x
                    << ',' << state.bounds->maximum.y << ',' << state.bounds->maximum.z;
        }
        else {
            states_ << ",,,,,,";
        }
        states_ << '\n';
        states_.flush();
    }

    void ProfileDataSink::record_health(const HealthProjection & health)
    {
        ++summary_.health_count;
        if(health.contract_fingerprint != scenario_.contract_fingerprint().hex_digest) {
            ++summary_.fingerprint_mismatches;
        }
        health_ << health.receipt_monotonic_ns << ',' << health.stamp_ns << ','
                << static_cast<unsigned int>(health.state) << ','
                << csv_escape(health.producer_source_id) << ','
                << health.producer_session.boot_time_ns << ','
                << health.producer_session.random_suffix << ','
                << health.contract_fingerprint << ','
                << (health.has_full_no_return_rays ? 1 : 0) << ','
                << health.active_sensor_count << '\n';
        health_.flush();
    }

    void ProfileDataSink::record_diagnostic(const DiagnosticProjection & diagnostic)
    {
        if(diagnostic.level >= 1U) {
            ++summary_.diagnostic_warn_or_error_count;
        }
        diagnostics_ << diagnostic.receipt_monotonic_ns << ',' << diagnostic.stamp_ns << ','
                     << static_cast<unsigned int>(diagnostic.level) << ','
                     << csv_escape(diagnostic.name) << ','
                     << csv_escape(diagnostic.message) << '\n';
        diagnostics_.flush();

        if(!is_map_update_producer_diagnostic(diagnostic.name)) {
            return;
        }
        const auto values = diagnostic_values(diagnostic);
        ++summary_.producer_diagnostic_count;
        producer_diagnostics_
                << diagnostic.receipt_monotonic_ns << ',' << diagnostic.stamp_ns << ','
                << static_cast<unsigned int>(diagnostic.level) << ','
                << csv_escape(diagnostic.name) << ',' << csv_escape(diagnostic.message) << ','
                << (required_bool(values, "pending") ? 1 : 0) << ','
                << (required_bool(values, "in_flight") ? 1 : 0) << ','
                << required_integer<std::uint64_t>(values, "pending_revision") << ','
                << required_integer<std::uint64_t>(values, "in_flight_revision") << ','
                << required_integer<std::uint64_t>(values, "published_revision") << ','
                << required_integer<std::uint64_t>(values, "coalesced_receipts") << ','
                << required_integer<std::uint64_t>(values, "superseded_receipts") << ','
                << required_integer<std::uint64_t>(values, "published_keyframes") << ','
                << required_integer<std::uint64_t>(values, "published_deltas") << ','
                << required_integer<std::uint64_t>(values, "revision_only_deltas") << ','
                << required_integer<std::uint64_t>(values, "publish_failures") << ','
                << required_integer<std::uint64_t>(values, "resource_rejections") << ','
                << required_integer<std::uint64_t>(values, "snapshot_cells") << ','
                << required_integer<std::uint64_t>(values, "delta_operations") << ','
                << required_integer<std::uint64_t>(values, "payload_bytes") << ','
                << required_integer<std::int64_t>(values, "acquire_duration_ns") << ','
                << required_integer<std::int64_t>(values, "materialize_duration_ns") << ','
                << required_integer<std::int64_t>(values, "traversal_duration_ns") << ','
                << required_integer<std::int64_t>(values, "canonicalize_duration_ns") << ','
                << required_integer<std::int64_t>(values, "content_hash_duration_ns") << ','
                << required_integer<std::int64_t>(values, "prepare_duration_ns") << ','
                << required_integer<std::int64_t>(values, "validation_duration_ns") << ','
                << required_integer<std::int64_t>(values, "diff_duration_ns") << ','
                << required_integer<std::int64_t>(values, "encode_duration_ns") << ','
                << required_integer<std::int64_t>(values, "update_hash_duration_ns") << ','
                << required_integer<std::int64_t>(values, "publish_duration_ns") << '\n';
        producer_diagnostics_.flush();
    }

    void ProfileDataSink::record_snapshot(const SnapshotProjection & snapshot)
    {
        ++summary_.snapshot_count;
        snapshots_ << snapshot.receipt_monotonic_ns << ',' << snapshot.stamp_ns << ','
                   << snapshot.ordinal << ',' << (snapshot.binary ? 1 : 0) << ','
                   << snapshot.resolution_m << ',' << snapshot.data_bytes << '\n';
        snapshots_.flush();
    }

    void ProfileDataSink::record_map_update(const MapUpdateProjection & update)
    {
        ++summary_.map_update_count;
        if(update.update_kind
           == static_cast<std::uint8_t>(PerceptionMapUpdate::UpdateKind::Keyframe)) {
            ++summary_.map_update_keyframe_count;
        }
        else if(update.update_kind
                == static_cast<std::uint8_t>(PerceptionMapUpdate::UpdateKind::Delta)) {
            ++summary_.map_update_delta_count;
            if(update.operation_count == 0U && update.new_revision > update.base_revision) {
                ++summary_.map_update_revision_only_delta_count;
            }
        }
        map_updates_ << update.receipt_monotonic_ns << ',' << update.stamp_ns << ','
                     << static_cast<unsigned int>(update.update_kind) << ','
                     << csv_escape(update.vehicle_id) << ','
                     << update.mapper_session.boot_time_ns << ','
                     << update.mapper_session.random_suffix << ',' << update.map_epoch << ','
                     << update.base_revision << ',' << update.new_revision << ','
                     << update.revision_span << ','
                     << update.observed_coalesced_receipt_count << ','
                     << update.known_cell_count << ',' << update.operation_count << ','
                     << update.canonical_payload_bytes << ','
                     << hash_hex(update.base_content_hash) << ','
                     << hash_hex(update.content_hash) << ',' << hash_hex(update.update_hash)
                     << '\n';
        map_updates_.flush();
    }

    void ProfileDataSink::finalize(bool normal_completion)
    {
        if(finalized_) {
            return;
        }
        summary_.normal_completion = normal_completion;
        observations_.flush();
        states_.flush();
        health_.flush();
        diagnostics_.flush();
        snapshots_.flush();
        map_updates_.flush();
        producer_diagnostics_.flush();
        write_manifest();
        finalized_ = true;
    }

    const SinkSummary & ProfileDataSink::summary() const noexcept
    {
        return summary_;
    }

    void ProfileDataSink::write_manifest() const
    {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "schema" << YAML::Value
                << "alien-scanner/perception-profile-sink/v2";
        emitter << YAML::Key << "mode" << YAML::Value
                << ProfileScenario::mode_name(scenario_.mode());
        emitter << YAML::Key << "c3_mode" << YAML::Value
                << c3_profile_mode_name(c3_mode_);
        emitter << YAML::Key << "payload_digest_schema" << YAML::Value
                << "fnv1a64-v1";
        emitter << YAML::Key << "expected_contract_fingerprint" << YAML::Value
                << scenario_.contract_fingerprint().hex_digest;
        emitter << YAML::Key << "frozen_workload" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "beam_count" << YAML::Value
                << FrozenProfileConfig::kBeamCount;
        emitter << YAML::Key << "observation_rate_hz" << YAML::Value << 10.0;
        emitter << YAML::Key << "pose_rate_hz" << YAML::Value << 20.0;
        emitter << YAML::Key << "range_min_m" << YAML::Value
                << scenario_.config().range_min_m;
        emitter << YAML::Key << "range_max_m" << YAML::Value
                << scenario_.config().range_max_m;
        emitter << YAML::Key << "resolution_m" << YAML::Value
                << scenario_.config().resolution_m;
        emitter << YAML::Key << "body_from_sensor_xyzw" << YAML::Value
                << YAML::Flow << YAML::BeginSeq
                << scenario_.config().body_from_sensor.x()
                << scenario_.config().body_from_sensor.y()
                << scenario_.config().body_from_sensor.z()
                << scenario_.config().body_from_sensor.w() << YAML::EndSeq;
        emitter << YAML::EndMap;
        emitter << YAML::Key << "summary" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "observation_count" << YAML::Value
                << summary_.observation_count;
        emitter << YAML::Key << "unique_observation_count" << YAML::Value
                << summary_.unique_observation_count;
        emitter << YAML::Key << "duplicate_observation_count" << YAML::Value
                << summary_.duplicate_observation_count;
        emitter << YAML::Key << "out_of_order_observation_count" << YAML::Value
                << summary_.out_of_order_observation_count;
        emitter << YAML::Key << "observation_sequence_gaps" << YAML::Value
                << summary_.observation_sequence_gaps;
        emitter << YAML::Key << "observation_schema_errors" << YAML::Value
                << summary_.observation_schema_errors;
        emitter << YAML::Key << "observation_digest_mismatches" << YAML::Value
                << summary_.observation_digest_mismatches;
        emitter << YAML::Key << "state_count" << YAML::Value << summary_.state_count;
        emitter << YAML::Key << "distinct_revision_count" << YAML::Value
                << summary_.distinct_revision_count;
        emitter << YAML::Key << "state_sequence_duplicates" << YAML::Value
                << summary_.state_sequence_duplicates;
        emitter << YAML::Key << "state_sequence_out_of_order" << YAML::Value
                << summary_.state_sequence_out_of_order;
        emitter << YAML::Key << "state_sequence_gaps" << YAML::Value
                << summary_.state_sequence_gaps;
        emitter << YAML::Key << "revision_heartbeat_count" << YAML::Value
                << summary_.revision_heartbeat_count;
        emitter << YAML::Key << "revision_regressions" << YAML::Value
                << summary_.revision_regressions;
        emitter << YAML::Key << "revision_gaps" << YAML::Value << summary_.revision_gaps;
        emitter << YAML::Key << "epoch_changes" << YAML::Value << summary_.epoch_changes;
        emitter << YAML::Key << "fingerprint_mismatches" << YAML::Value
                << summary_.fingerprint_mismatches;
        emitter << YAML::Key << "health_count" << YAML::Value << summary_.health_count;
        emitter << YAML::Key << "diagnostic_warn_or_error_count" << YAML::Value
                << summary_.diagnostic_warn_or_error_count;
        emitter << YAML::Key << "snapshot_count" << YAML::Value
                << summary_.snapshot_count;
        emitter << YAML::Key << "map_update_count" << YAML::Value
                << summary_.map_update_count;
        emitter << YAML::Key << "map_update_keyframe_count" << YAML::Value
                << summary_.map_update_keyframe_count;
        emitter << YAML::Key << "map_update_delta_count" << YAML::Value
                << summary_.map_update_delta_count;
        emitter << YAML::Key << "map_update_revision_only_delta_count" << YAML::Value
                << summary_.map_update_revision_only_delta_count;
        emitter << YAML::Key << "producer_diagnostic_count" << YAML::Value
                << summary_.producer_diagnostic_count;
        emitter << YAML::Key << "normal_completion" << YAML::Value
                << summary_.normal_completion;
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;

        const auto temporary = output_directory_ / "sink_manifest.yaml.tmp";
        const auto destination = output_directory_ / "sink_manifest.yaml";
        {
            std::ofstream stream(temporary, std::ios::out | std::ios::trunc);
            if(!stream) {
                throw std::runtime_error("could not write sink manifest");
            }
            stream << emitter.c_str() << '\n';
        }
        std::error_code error;
        std::filesystem::rename(temporary, destination, error);
        if(error) {
            std::filesystem::remove(destination, error);
            error.clear();
            std::filesystem::rename(temporary, destination, error);
            if(error) {
                throw std::runtime_error("could not finalize sink manifest: " + error.message());
            }
        }
    }

}// namespace PerceptionProfiling

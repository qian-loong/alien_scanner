#include "perception_profiling/ProfileDataSink.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
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

    }// namespace

    ProfileDataSink::ProfileDataSink(
            std::filesystem::path output_directory,
            ProfileScenario scenario)
        : output_directory_(std::move(output_directory))
        , scenario_(std::move(scenario))
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
        observations_ << "receipt_monotonic_ns,sequence,stamp_ns,payload_digest,"
                         "expected_digest,schema_valid,digest_matches\n";
        states_ << "receipt_monotonic_ns,state_sequence,map_epoch,revision,stamp_ns,fingerprint,"
                   "changed_cell_count,has_bounds,min_x,min_y,min_z,max_x,max_y,max_z\n";
        health_ << "receipt_monotonic_ns,stamp_ns,state,producer_source_id,"
                   "session_boot_ns,session_suffix,fingerprint,full_no_return,active_sensor_count\n";
        diagnostics_ << "receipt_monotonic_ns,stamp_ns,level,name,message\n";
        snapshots_ << "receipt_monotonic_ns,stamp_ns,ordinal,binary,resolution_m,data_bytes\n";
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
    }

    void ProfileDataSink::record_snapshot(const SnapshotProjection & snapshot)
    {
        ++summary_.snapshot_count;
        snapshots_ << snapshot.receipt_monotonic_ns << ',' << snapshot.stamp_ns << ','
                   << snapshot.ordinal << ',' << (snapshot.binary ? 1 : 0) << ','
                   << snapshot.resolution_m << ',' << snapshot.data_bytes << '\n';
        snapshots_.flush();
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
                << "alien-scanner/perception-profile-sink/v1";
        emitter << YAML::Key << "mode" << YAML::Value
                << ProfileScenario::mode_name(scenario_.mode());
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

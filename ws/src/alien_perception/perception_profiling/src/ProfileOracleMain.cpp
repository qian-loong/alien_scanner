#include "perception_profiling/ProfileOracle.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

    std::uint64_t parse_count(const std::string & value)
    {
        std::size_t consumed = 0U;
        const auto result = std::stoull(value, &consumed, 10);
        if(consumed != value.size() || result == 0U) {
            throw std::invalid_argument("sequence count must be a positive integer");
        }
        return result;
    }

    void write_bounds(
            std::ofstream & stream,
            const std::optional<PerceptionLocalMap::AxisAlignedBounds> & bounds)
    {
        if(!bounds.has_value()) {
            stream << ",,,,,,";
            return;
        }
        stream << ',' << bounds->minimum.x << ',' << bounds->minimum.y << ','
               << bounds->minimum.z << ',' << bounds->maximum.x << ','
               << bounds->maximum.y << ',' << bounds->maximum.z;
    }

    void write_results(
            const std::filesystem::path & output_directory,
            const PerceptionProfiling::OracleRun & run)
    {
        std::filesystem::create_directories(output_directory);
        const auto checkpoints_path = output_directory / "oracle_checkpoints.csv";
        const auto milestones_path = output_directory / "oracle_milestones.csv";
        const auto manifest_path = output_directory / "oracle_manifest.yaml";
        if(std::filesystem::exists(checkpoints_path)
           || std::filesystem::exists(milestones_path)
           || std::filesystem::exists(manifest_path)) {
            throw std::invalid_argument("oracle refuses to overwrite existing output");
        }

        std::ofstream checkpoints(checkpoints_path);
        checkpoints << "sequence,stamp_ns,payload_digest,map_epoch,revision,known,free,"
                       "occupied,fingerprint,min_x,min_y,min_z,max_x,max_y,max_z\n";
        for(const auto & checkpoint : run.checkpoints) {
            checkpoints << checkpoint.sequence << ',' << checkpoint.observation_stamp_ns
                        << ',' << checkpoint.payload_digest << ',' << checkpoint.map_epoch
                        << ',' << checkpoint.revision << ',' << checkpoint.counts.known
                        << ',' << checkpoint.counts.free << ','
                        << checkpoint.counts.occupied << ','
                        << checkpoint.contract_fingerprint;
            write_bounds(checkpoints, checkpoint.bounds);
            checkpoints << '\n';
        }

        std::ofstream milestones(milestones_path);
        milestones << "threshold,sequence,stamp_ns,revision,known\n";
        for(const auto & milestone : run.milestones) {
            milestones << milestone.threshold << ',' << milestone.sequence << ','
                       << milestone.observation_stamp_ns << ',' << milestone.revision
                       << ',' << milestone.known << '\n';
        }

        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "schema" << YAML::Value
                << "alien-scanner/perception-profile-oracle/v1";
        emitter << YAML::Key << "mode" << YAML::Value
                << PerceptionProfiling::ProfileScenario::mode_name(run.mode);
        emitter << YAML::Key << "accepted" << YAML::Value << run.accepted;
        emitter << YAML::Key << "applied" << YAML::Value << run.applied;
        emitter << YAML::Key << "no_evidence" << YAML::Value << run.no_evidence;
        emitter << YAML::Key << "rejected" << YAML::Value << run.rejected;
        emitter << YAML::Key << "unavailable" << YAML::Value << run.unavailable;
        emitter << YAML::Key << "backend_fault" << YAML::Value << run.backend_fault;
        emitter << YAML::Key << "final_map_epoch" << YAML::Value << run.final_map_epoch;
        emitter << YAML::Key << "final_revision" << YAML::Value << run.final_revision;
        emitter << YAML::Key << "plateau_start_revision" << YAML::Value;
        if(run.plateau_start_revision.has_value()) {
            emitter << run.plateau_start_revision.value();
        }
        else {
            emitter << YAML::Null;
        }
        emitter << YAML::Key << "capacity" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "status" << YAML::Value
                << PerceptionProfiling::capacity_status_name(run.capacity.status);
        emitter << YAML::Key << "crossing_revision" << YAML::Value;
        if(run.capacity.crossing_revision.has_value()) {
            emitter << run.capacity.crossing_revision.value();
        }
        else {
            emitter << YAML::Null;
        }
        emitter << YAML::Key << "required_end_revision" << YAML::Value;
        if(run.capacity.required_end_revision.has_value()) {
            emitter << run.capacity.required_end_revision.value();
        }
        else {
            emitter << YAML::Null;
        }
        emitter << YAML::EndMap;
        emitter << YAML::EndMap;
        std::ofstream manifest(manifest_path);
        manifest << emitter.c_str() << '\n';
    }

}// namespace

int main(int argc, char ** argv)
{
    if(argc != 4) {
        std::cerr << "usage: perception_profile_oracle <mode> <sequence-count> <output-dir>\n";
        return 2;
    }
    try {
        const auto mode = PerceptionProfiling::ProfileScenario::parse_mode(argv[1]);
        PerceptionProfiling::OracleOptions options;
        options.sequence_count = parse_count(argv[2]);
        const PerceptionProfiling::ProfileOracle oracle {
                PerceptionProfiling::ProfileScenario(mode)};
        const auto run = oracle.run(options);
        write_results(argv[3], run);
        return run.rejected == 0U && run.backend_fault == 0U ? 0 : 1;
    }
    catch(const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}

#include "perception_profiling/ChunkLayoutEstimator.hpp"
#include "perception_profiling/MapUpdateReplayOracle.hpp"
#include "perception_profiling/ProfileScenario.hpp"

#include "perception_map_update/MapUpdateTypes.hpp"
#include "perception_map_update/CellSnapshotStore.hpp"
#include "perception_map_update/SpatialChunkLayout.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

    using PerceptionMapUpdate::CanonicalCell;
    using PerceptionMapUpdate::CellState;
    using PerceptionMapUpdate::DeltaOperation;
    using PerceptionMapUpdate::DeltaOperationKind;

    constexpr std::array<std::uint32_t, 3> kChunkEdges {8U, 16U, 32U};
    constexpr std::size_t kBucketCount = 256U;
    constexpr std::size_t kDeltaOperations = 256U;
    constexpr std::array<std::size_t, 3> kC4CellScales {10'000U, 100'000U, 500'000U};

    struct ReportRow {
        std::string workload;
        std::string input_identity;
        std::uint64_t transition = 0U;
        std::string base_content_hash;
        std::string target_content_hash;
        std::string update_hash;
        PerceptionProfiling::ChunkLayoutEstimate estimate;
    };

    struct BuildIdentity {
        std::string git_commit;
        std::string git_tree_state;
        std::string executable_sha256;
    };

    std::uint64_t parse_count(const std::string & value)
    {
        std::size_t consumed = 0U;
        const auto result = std::stoull(value, &consumed, 10);
        if(consumed != value.size() || result == 0U) {
            throw std::invalid_argument("sequence count must be a positive integer");
        }
        return result;
    }

    void ensure_output_directory(const std::filesystem::path & output)
    {
        if(output.empty()) {
            throw std::invalid_argument("output directory must not be empty");
        }
        std::filesystem::create_directories(output);
        if(std::filesystem::exists(output / "chunk_layout_estimates.csv")
           || std::filesystem::exists(output / "chunk_layout_manifest.yaml")) {
            throw std::invalid_argument("chunk layout estimator refuses to overwrite output");
        }
    }

    std::string csv_escape(const std::string & value)
    {
        if(value.find_first_of(",\"\r\n") == std::string::npos) {
            return value;
        }
        std::string escaped = "\"";
        for(const char character : value) {
            escaped += character == '\"' ? "\"\"" : std::string(1U, character);
        }
        escaped += '\"';
        return escaped;
    }

    std::string environment_or_unknown(const char * name)
    {
        const char * value = std::getenv(name);
        return value == nullptr || *value == '\0' ? "unknown" : value;
    }

    std::vector<CanonicalCell> make_linear_cells(std::size_t count)
    {
        std::vector<CanonicalCell> cells;
        cells.reserve(count);
        for(std::size_t index = 0U; index < count; ++index) {
            cells.push_back({
                    {static_cast<std::int64_t>(index), 0, 0},
                    (index % 2U) == 0U ? CellState::Free : CellState::Occupied});
        }
        return cells;
    }

    std::vector<DeltaOperation> make_linear_flip_operations(
            std::size_t count,
            std::size_t offset = 0U)
    {
        std::vector<DeltaOperation> operations;
        operations.reserve(std::min(count, kDeltaOperations));
        for(std::size_t index = 0U; index < std::min(count, kDeltaOperations); ++index) {
            operations.push_back({
                    {static_cast<std::int64_t>(offset + index), 0, 0},
                    (index % 2U) == 0U ? DeltaOperationKind::UpsertOccupied
                                       : DeltaOperationKind::UpsertFree});
        }
        return operations;
    }

    void add_estimates(
            std::vector<ReportRow> & rows,
            const std::string & workload,
            const std::string & input_identity,
            std::uint64_t transition,
            const std::vector<CanonicalCell> & base_cells,
            const std::vector<DeltaOperation> & operations,
            std::string base_content_hash = {},
            std::string target_content_hash = {},
            std::string update_hash = {})
    {
        for(const auto edge : kChunkEdges) {
            auto estimate = PerceptionProfiling::ChunkLayoutEstimator::estimate(
                    workload,
                    input_identity,
                    base_cells,
                    operations,
                    edge,
                    kBucketCount);
            PerceptionMapUpdate::CellSnapshotStore actual(
                    {PerceptionMapUpdate::CellStorageMode::Chunked,
                     edge,
                     kBucketCount});
            const auto replaced = actual.replace(base_cells);
            if(!replaced.success) {
                throw std::runtime_error(
                        "actual chunk store rejected estimator base: "
                        + replaced.diagnostic);
            }
            const auto base_metrics = actual.metrics();
            const auto applied = actual.apply(operations);
            if(!applied.success) {
                throw std::runtime_error(
                        "actual chunk store rejected estimator operations: "
                        + applied.diagnostic);
            }
            const auto actual_touched_existing =
                    base_metrics.total_chunks - applied.metrics.shared_chunks;
            if(estimate.known_cells != base_cells.size()
               || estimate.operation_count != operations.size()
               || estimate.total_chunks != base_metrics.total_chunks
               || estimate.touched_chunks != applied.metrics.touched_chunks
               || estimate.touched_existing_chunks != actual_touched_existing
               || estimate.copied_cells != applied.metrics.copied_cells) {
                throw std::runtime_error(
                        "chunk layout estimate does not match actual COW metrics");
            }
            rows.push_back({
                    workload,
                    input_identity,
                    transition,
                    base_content_hash,
                    target_content_hash,
                    update_hash,
                    std::move(estimate)});
        }
    }

    void append_c4_rows(std::vector<ReportRow> & rows)
    {
        for(const auto cell_count : kC4CellScales) {
            const auto bounded = make_linear_cells(cell_count);
            add_estimates(
                    rows,
                    "c4-1d-bounded",
                    "cells=" + std::to_string(cell_count),
                    1U,
                    bounded,
                    make_linear_flip_operations(cell_count));

            const auto initial_count = cell_count / 2U;
            const auto expanding = make_linear_cells(initial_count);
            add_estimates(
                    rows,
                    "c4-1d-expanding",
                    "initial_cells=" + std::to_string(initial_count)
                            + ",target_cells=" + std::to_string(cell_count),
                    1U,
                    expanding,
                    make_linear_flip_operations(kDeltaOperations, initial_count));
        }
    }

    void append_spread_rows(std::vector<ReportRow> & rows)
    {
        std::vector<CanonicalCell> base_cells;
        std::vector<DeltaOperation> operations;
        base_cells.reserve(kDeltaOperations * 2U);
        operations.reserve(kDeltaOperations);
        for(std::size_t index = 0U; index < kDeltaOperations * 2U; ++index) {
            const auto x = static_cast<std::int64_t>(index) * 32;
            const auto y = static_cast<std::int64_t>(index / 16U) * 32;
            const auto z = static_cast<std::int64_t>(index % 16U) * 32;
            base_cells.push_back({{x, y, z}, CellState::Free});
            if(index < kDeltaOperations) {
                operations.push_back({
                        {x, y, z}, DeltaOperationKind::UpsertOccupied});
            }
        }
        add_estimates(
                rows,
                "artificial-spread-worst-case",
                "operations=" + std::to_string(kDeltaOperations),
                1U,
                base_cells,
                operations);
    }

    void append_replay_rows(
            std::vector<ReportRow> & rows,
            std::uint64_t sequence_count)
    {
        PerceptionProfiling::MapUpdateReplayOptions options;
        options.sequence_count = sequence_count;
        options.snapshot_transition_observer =
                [&rows](
                        const PerceptionMapUpdate::CanonicalSnapshot * base,
                        const PerceptionMapUpdate::CanonicalSnapshot & target,
                        const std::vector<DeltaOperation> & operations,
                        const PerceptionMapUpdate::MapUpdate & update) {
                    if(base == nullptr) {
                        return;
                    }
                    add_estimates(
                            rows,
                            "production-3d-replay",
                            "sequence=" + std::to_string(target.revision),
                            target.revision,
                            base->cells,
                            operations,
                            PerceptionMapUpdate::hash_to_hex(base->content_hash),
                            PerceptionMapUpdate::hash_to_hex(target.content_hash),
                            PerceptionMapUpdate::hash_to_hex(update.update_hash));
                };
        const PerceptionProfiling::MapUpdateReplayOracle oracle(
                PerceptionProfiling::ProfileScenario(
                        PerceptionProfiling::ScenarioMode::Canonical));
        const auto run = oracle.run(options);
        if(!run.all_checkpoints_match()) {
            throw std::runtime_error(
                    "production 3D replay did not pass its canonical equivalence gate");
        }
    }

    std::uint64_t p95_metric(
            const std::vector<ReportRow> & rows,
            const std::string & workload,
            std::uint32_t edge,
            std::uint64_t (*metric)(const PerceptionProfiling::ChunkLayoutEstimate &))
    {
        std::vector<std::uint64_t> values;
        for(const auto & row : rows) {
            if(row.workload == workload && row.estimate.chunk_edge == edge) {
                values.push_back(metric(row.estimate));
            }
        }
        return PerceptionProfiling::ChunkLayoutEstimator::summarize(std::move(values)).p95;
    }

    std::uint64_t copied_cells_metric(
            const PerceptionProfiling::ChunkLayoutEstimate & estimate)
    {
        return estimate.copied_cells;
    }

    std::uint64_t metadata_metric(
            const PerceptionProfiling::ChunkLayoutEstimate & estimate)
    {
        return estimate.estimated_chunk_metadata_bytes
               + estimate.estimated_bucket_metadata_bytes
               + estimate.estimated_directory_bytes;
    }

    std::uint32_t choose_edge(const std::vector<ReportRow> & rows)
    {
        const auto baseline_copied = p95_metric(
                rows, "production-3d-replay", 16U, copied_cells_metric);
        const auto baseline_metadata = p95_metric(
                rows, "production-3d-replay", 16U, metadata_metric);
        const auto baseline_worst_metadata = p95_metric(
                rows, "artificial-spread-worst-case", 16U, metadata_metric);
        for(const auto edge : kChunkEdges) {
            const auto copied = p95_metric(rows, "production-3d-replay", edge, copied_cells_metric);
            const auto metadata = p95_metric(rows, "production-3d-replay", edge, metadata_metric);
            const auto worst_metadata = p95_metric(
                    rows, "artificial-spread-worst-case", edge, metadata_metric);
            if(edge != 16U && copied * 5U <= baseline_copied * 4U
               && metadata * 5U <= baseline_metadata * 6U
               && worst_metadata * 5U <= baseline_worst_metadata * 6U) {
                return edge;
            }
        }
        return 16U;
    }

    void write_csv(
            const std::filesystem::path & output,
            const std::vector<ReportRow> & rows)
    {
        std::ofstream stream(output / "chunk_layout_estimates.csv");
        if(!stream) {
            throw std::runtime_error("could not write chunk layout csv");
        }
        stream << "workload,input_identity,transition,base_content_hash,target_content_hash,"
                  "update_hash,chunk_edge,bucket_count,known_cells,"
                  "operation_count,total_chunks,touched_chunks,touched_existing_chunks,"
                  "copied_cells,write_amplification,shared_chunk_ratio,sparse_fill_ratio,"
                  "chunk_cells_min,chunk_cells_p50,chunk_cells_p95,chunk_cells_max,"
                  "bucket_occupancy_min,bucket_occupancy_p50,bucket_occupancy_p95,"
                  "bucket_occupancy_max,chunk_payload_bytes,chunk_metadata_bytes,"
                  "bucket_metadata_bytes,directory_bytes\n";
        stream << std::setprecision(17);
        for(const auto & row : rows) {
            const auto & e = row.estimate;
            stream << csv_escape(row.workload) << ',' << csv_escape(row.input_identity) << ','
                   << row.transition << ',' << row.base_content_hash << ','
                   << row.target_content_hash << ',' << row.update_hash << ','
                   << e.chunk_edge << ',' << e.bucket_count << ',' << e.known_cells << ','
                   << e.operation_count << ',' << e.total_chunks << ',' << e.touched_chunks << ','
                   << e.touched_existing_chunks << ',' << e.copied_cells << ','
                   << e.write_amplification << ',' << e.shared_chunk_ratio << ','
                   << e.sparse_fill_ratio << ',' << e.cells_per_chunk.minimum << ','
                   << e.cells_per_chunk.p50 << ',' << e.cells_per_chunk.p95 << ','
                   << e.cells_per_chunk.maximum << ',' << e.bucket_occupancy.minimum << ','
                   << e.bucket_occupancy.p50 << ',' << e.bucket_occupancy.p95 << ','
                   << e.bucket_occupancy.maximum << ',' << e.estimated_chunk_payload_bytes << ','
                   << e.estimated_chunk_metadata_bytes << ',' << e.estimated_bucket_metadata_bytes
                   << ',' << e.estimated_directory_bytes << '\n';
        }
    }

    void write_manifest(
            const std::filesystem::path & output,
            const std::vector<ReportRow> & rows,
            std::uint64_t sequence_count,
            std::uint32_t selected_edge,
            const BuildIdentity & identity)
    {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "schema" << YAML::Value
                << "alien-scanner/chunk-layout-estimator/v1";
        emitter << YAML::Key << "bucket_count" << YAML::Value << kBucketCount;
        emitter << YAML::Key << "sequence_count" << YAML::Value << sequence_count;
        emitter << YAML::Key << "git_commit" << YAML::Value << identity.git_commit;
        emitter << YAML::Key << "git_tree_state" << YAML::Value
                << identity.git_tree_state;
        emitter << YAML::Key << "executable_sha256" << YAML::Value
                << identity.executable_sha256;
        emitter << YAML::Key << "abi_sizes_bytes" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "canonical_cell" << YAML::Value << sizeof(CanonicalCell);
        emitter << YAML::Key << "chunk_coordinate" << YAML::Value
                << sizeof(PerceptionMapUpdate::ChunkCoordinate);
        emitter << YAML::Key << "shared_pointer" << YAML::Value
                << sizeof(std::shared_ptr<const void>);
        emitter << YAML::Key << "cell_vector" << YAML::Value
                << sizeof(std::vector<CanonicalCell>);
        emitter << YAML::EndMap;
        emitter << YAML::Key << "candidate_edges" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << 8U << 16U << 32U << YAML::EndSeq;
        emitter << YAML::Key << "selected_edge" << YAML::Value << selected_edge;
        emitter << YAML::Key << "actual_metric_conformance" << YAML::Value << "passed";
        emitter << YAML::Key << "selection_rule" << YAML::Value
                << "representative 3D copied_cells P95 improves >=20%; replay and artificial "
                   "metadata P95 do not worsen >20%; otherwise 16";
        emitter << YAML::Key << "candidate_summary" << YAML::Value << YAML::BeginSeq;
        for(const auto edge : kChunkEdges) {
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "edge" << YAML::Value << edge;
            emitter << YAML::Key << "replay_copied_cells_p95" << YAML::Value
                    << p95_metric(rows, "production-3d-replay", edge, copied_cells_metric);
            emitter << YAML::Key << "replay_metadata_bytes_p95" << YAML::Value
                    << p95_metric(rows, "production-3d-replay", edge, metadata_metric);
            emitter << YAML::Key << "worst_case_metadata_bytes_p95" << YAML::Value
                    << p95_metric(
                               rows,
                               "artificial-spread-worst-case",
                               edge,
                               metadata_metric);
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
        emitter << YAML::Key << "row_count" << YAML::Value << rows.size();
        emitter << YAML::Key << "input_workloads" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << "c4-1d-bounded" << "c4-1d-expanding"
                << "production-3d-replay" << "artificial-spread-worst-case" << YAML::EndSeq;
        emitter << YAML::EndMap;
        std::ofstream stream(output / "chunk_layout_manifest.yaml");
        if(!stream) {
            throw std::runtime_error("could not write chunk layout manifest");
        }
        stream << emitter.c_str() << '\n';
    }

}// namespace

int main(int argc, char ** argv)
{
    if(argc != 3) {
        std::cerr << "usage: perception_chunk_layout_estimator <sequence-count> <output-dir>\n";
        return 2;
    }
    try {
        const auto sequence_count = parse_count(argv[1]);
        const std::filesystem::path output = argv[2];
        ensure_output_directory(output);
        std::vector<ReportRow> rows;
        append_c4_rows(rows);
        append_spread_rows(rows);
        append_replay_rows(rows, sequence_count);
        const auto selected_edge = choose_edge(rows);
        const BuildIdentity identity {
                environment_or_unknown("ALIEN_SCANNER_GIT_COMMIT"),
                environment_or_unknown("ALIEN_SCANNER_GIT_TREE_STATE"),
                environment_or_unknown("ALIEN_SCANNER_EXECUTABLE_SHA256")};
        write_csv(output, rows);
        write_manifest(output, rows, sequence_count, selected_edge, identity);
        std::cout << "selected_chunk_edge=" << selected_edge << "\n";
        std::cout << "rows=" << rows.size() << "\n";
        return 0;
    }
    catch(const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}

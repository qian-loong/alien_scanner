#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MerklePatricia.hpp"
#include "perception_map_update/MerklePrototypeApplier.hpp"
#include "perception_map_update/SpatialChunkLayout.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

    using namespace PerceptionMapUpdate;
    using Clock = std::chrono::steady_clock;

    enum class RunMode
    {
        Correctness,
        Flat,
        Merkle
    };

    enum class MutationPattern
    {
        UpdateExisting,
        InsertNew
    };

    struct Options {
        std::filesystem::path output_dir;
        std::size_t cells = 10000U;
        std::size_t iterations = 100U;
        std::size_t warmup = 10U;
        std::size_t touched_chunks = 4U;
        std::size_t hold_seconds = 0U;
        RunMode mode = RunMode::Correctness;
        MutationPattern pattern = MutationPattern::UpdateExisting;
        CellStorageMode flat_storage = CellStorageMode::Chunked;
    };

    const char * mode_name(RunMode mode) noexcept
    {
        switch(mode) {
            case RunMode::Correctness:
                return "correctness";
            case RunMode::Flat:
                return "flat";
            case RunMode::Merkle:
                return "merkle";
        }
        return "unknown";
    }

    RunMode parse_mode(const std::string & value)
    {
        if(value == "correctness") {
            return RunMode::Correctness;
        }
        if(value == "flat") {
            return RunMode::Flat;
        }
        if(value == "merkle") {
            return RunMode::Merkle;
        }
        throw std::invalid_argument("--mode must be correctness, flat, or merkle");
    }

    const char * pattern_name(MutationPattern pattern) noexcept
    {
        return pattern == MutationPattern::UpdateExisting ? "update" : "insert";
    }

    MutationPattern parse_pattern(const std::string & value)
    {
        if(value == "update") {
            return MutationPattern::UpdateExisting;
        }
        if(value == "insert") {
            return MutationPattern::InsertNew;
        }
        throw std::invalid_argument("--pattern must be update or insert");
    }

    const char * storage_mode_name(CellStorageMode mode) noexcept
    {
        return mode == CellStorageMode::Vector ? "vector" : "chunked";
    }

    CellStorageMode parse_storage_mode(const std::string & value)
    {
        if(value == "vector") {
            return CellStorageMode::Vector;
        }
        if(value == "chunked") {
            return CellStorageMode::Chunked;
        }
        throw std::invalid_argument("--flat-storage must be vector or chunked");
    }

    std::size_t parse_size(const std::string & value, const char * name)
    {
        std::size_t offset = 0U;
        const auto parsed = std::stoull(value, &offset);
        if(offset != value.size() || parsed == 0U) {
            throw std::invalid_argument(std::string(name) + " must be a positive integer");
        }
        return static_cast<std::size_t>(parsed);
    }

    std::size_t parse_nonnegative_size(const std::string & value, const char * name)
    {
        std::size_t offset = 0U;
        const auto parsed = std::stoull(value, &offset);
        if(offset != value.size()) {
            throw std::invalid_argument(std::string(name) + " must be a non-negative integer");
        }
        return static_cast<std::size_t>(parsed);
    }

    Options parse_options(int argc, char ** argv)
    {
        Options options;
        for(int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if(index + 1 >= argc) {
                throw std::invalid_argument("missing value for " + argument);
            }
            const std::string value = argv[++index];
            if(argument == "--output-dir") {
                options.output_dir = value;
            }
            else if(argument == "--cells") {
                options.cells = parse_size(value, "--cells");
            }
            else if(argument == "--iterations") {
                options.iterations = parse_size(value, "--iterations");
            }
            else if(argument == "--warmup") {
                options.warmup = parse_size(value, "--warmup");
            }
            else if(argument == "--touched-chunks") {
                options.touched_chunks = parse_size(value, "--touched-chunks");
            }
            else if(argument == "--mode") {
                options.mode = parse_mode(value);
            }
            else if(argument == "--hold-seconds") {
                options.hold_seconds = parse_nonnegative_size(value, "--hold-seconds");
            }
            else if(argument == "--pattern") {
                options.pattern = parse_pattern(value);
            }
            else if(argument == "--flat-storage") {
                options.flat_storage = parse_storage_mode(value);
            }
            else {
                throw std::invalid_argument("unknown argument: " + argument);
            }
        }
        if(options.pattern == MutationPattern::UpdateExisting
           && options.touched_chunks > options.cells) {
            throw std::invalid_argument("--touched-chunks cannot exceed --cells");
        }
        if(options.output_dir.empty()) {
            throw std::invalid_argument("--output-dir is required");
        }
        return options;
    }

    bool coordinate_less(const CanonicalCell & left, const CanonicalCell & right)
    {
        return left.index < right.index;
    }

    std::vector<CanonicalCell> make_cells(std::size_t count)
    {
        std::vector<CanonicalCell> cells;
        cells.reserve(count);
        constexpr std::int64_t kCellsPerChunk = 64;
        constexpr std::int64_t kChunksPerAxis = 64;
        for(std::size_t index = 0U; index < count; ++index) {
            const auto chunk_index = static_cast<std::int64_t>(index / kCellsPerChunk);
            const auto local_index = static_cast<std::int64_t>(index % kCellsPerChunk);
            const auto chunk_x = chunk_index % kChunksPerAxis;
            const auto chunk_y = (chunk_index / kChunksPerAxis) % kChunksPerAxis;
            const auto chunk_z = chunk_index / (kChunksPerAxis * kChunksPerAxis);
            const auto local_x = local_index % 4;
            const auto local_y = (local_index / 4) % 4;
            const auto local_z = local_index / 16;
            cells.push_back({
                    {chunk_x * 16 + local_x, chunk_y * 16 + local_y, chunk_z * 16 + local_z},
                    index % 2U == 0U ? CellState::Free : CellState::Occupied});
        }
        std::sort(cells.begin(), cells.end(), coordinate_less);
        return cells;
    }

    std::map<ChunkCoordinate, std::vector<CanonicalCell>> group_cells(
            const std::vector<CanonicalCell> & cells)
    {
        std::map<ChunkCoordinate, std::vector<CanonicalCell>> grouped;
        for(const auto & cell : cells) {
            const auto address = locate_chunk(cell.index, kMerkleChunkEdge);
            if(!address) {
                throw std::runtime_error(address.diagnostic);
            }
            grouped[address.address.chunk].push_back(cell);
        }
        return grouped;
    }

    std::vector<CanonicalCell> flatten(
            const std::map<VoxelIndex, CellState> & cells)
    {
        std::vector<CanonicalCell> result;
        result.reserve(cells.size());
        for(const auto & entry : cells) {
            result.push_back({entry.first, entry.second});
        }
        return result;
    }

    struct Sample {
        std::size_t iteration = 0U;
        std::size_t known_cells = 0U;
        std::size_t touched_chunks = 0U;
        std::uint64_t flat_cow_apply_ns = 0U;
        std::uint64_t flat_hash_ns = 0U;
        std::uint64_t flat_total_apply_ns = 0U;
        std::uint64_t merkle_apply_ns = 0U;
        std::uint64_t merkle_storage_ns = 0U;
        std::uint64_t merkle_mutation_build_ns = 0U;
        std::uint64_t merkle_tree_ns = 0U;
        std::uint64_t merkle_commit_ns = 0U;
        std::uint64_t merkle_leaf_hash_ns = 0U;
        std::uint64_t merkle_branch_hash_ns = 0U;
        std::uint64_t merkle_content_hash_ns = 0U;
        std::size_t merkle_allocated_nodes = 0U;
        std::size_t merkle_path_nodes = 0U;
        std::size_t merkle_owned_bytes = 0U;
        std::size_t merkle_candidate_owned_bytes = 0U;
        std::size_t storage_candidate_owned_bytes = 0U;
        bool validation_performed = false;
        bool flat_hash_match = false;
        bool content_match = false;
    };

    std::uint64_t elapsed_ns(Clock::time_point begin, Clock::time_point end)
    {
        return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count());
    }

    std::uint64_t percentile(
            std::vector<std::uint64_t> values,
            std::size_t numerator,
            std::size_t denominator)
    {
        if(values.empty()) {
            return 0U;
        }
        std::sort(values.begin(), values.end());
        const auto rank = std::max<std::size_t>(
                1U,
                (values.size() * numerator + denominator - 1U) / denominator);
        return values[rank - 1U];
    }

    void write_csv(
            const std::filesystem::path & path,
            const std::vector<Sample> & samples)
    {
        std::ofstream output(path);
        if(!output) {
            throw std::runtime_error("failed to open benchmark CSV: " + path.string());
        }
        output << "iteration,known_cells,touched_chunks,flat_cow_apply_ns,flat_hash_ns,"
                  "flat_total_apply_ns,merkle_apply_ns,merkle_allocated_nodes,"
                  "merkle_storage_ns,merkle_mutation_build_ns,merkle_tree_ns,"
                  "merkle_commit_ns,merkle_leaf_hash_ns,merkle_branch_hash_ns,"
                  "merkle_content_hash_ns,"
                  "merkle_path_nodes,merkle_owned_bytes,merkle_candidate_owned_bytes,"
                  "storage_candidate_owned_bytes,validation_performed,flat_hash_match,"
                  "content_match\n";
        for(const auto & sample : samples) {
            output << sample.iteration << ',' << sample.known_cells << ','
                   << sample.touched_chunks << ',' << sample.flat_cow_apply_ns << ','
                   << sample.flat_hash_ns << ',' << sample.flat_total_apply_ns << ','
                   << sample.merkle_apply_ns << ',' << sample.merkle_allocated_nodes << ','
                   << sample.merkle_storage_ns << ',' << sample.merkle_mutation_build_ns << ','
                   << sample.merkle_tree_ns << ',' << sample.merkle_commit_ns << ','
                   << sample.merkle_leaf_hash_ns << ',' << sample.merkle_branch_hash_ns << ','
                   << sample.merkle_content_hash_ns << ','
                   << sample.merkle_path_nodes << ',' << sample.merkle_owned_bytes << ','
                   << sample.merkle_candidate_owned_bytes << ','
                   << sample.storage_candidate_owned_bytes << ','
                   << (sample.validation_performed ? 1 : 0) << ','
                   << (sample.flat_hash_match ? 1 : 0) << ','
                   << (sample.content_match ? 1 : 0) << '\n';
        }
    }

    void write_summary(
            const std::filesystem::path & path,
            const Options & options,
            const std::vector<Sample> & samples,
            std::uint64_t flat_keyframe_storage_ns,
            std::uint64_t flat_keyframe_hash_ns,
            std::uint64_t flat_keyframe_total_ns,
            const MerklePrototypeTimings & merkle_keyframe_timings,
            const MerkleTreeMetrics & keyframe_metrics)
    {
        std::vector<std::uint64_t> flat;
        std::vector<std::uint64_t> flat_cow;
        std::vector<std::uint64_t> flat_total;
        std::vector<std::uint64_t> merkle;
        flat.reserve(samples.size());
        flat_cow.reserve(samples.size());
        flat_total.reserve(samples.size());
        merkle.reserve(samples.size());
        for(const auto & sample : samples) {
            flat.push_back(sample.flat_hash_ns);
            flat_cow.push_back(sample.flat_cow_apply_ns);
            flat_total.push_back(sample.flat_total_apply_ns);
            merkle.push_back(sample.merkle_apply_ns);
        }
        std::ofstream output(path);
        if(!output) {
            throw std::runtime_error("failed to open benchmark summary: " + path.string());
        }
        const auto flat_mean = flat.empty()
                                       ? 0U
                                       : std::accumulate(flat.begin(), flat.end(), std::uint64_t {0U})
                                                 / flat.size();
        const auto merkle_mean = merkle.empty()
                                         ? 0U
                                         : std::accumulate(
                                                   merkle.begin(), merkle.end(), std::uint64_t {0U})
                                                   / merkle.size();
        output << "{\n"
               << "  \"mode\": \"" << mode_name(options.mode) << "\",\n"
               << "  \"pattern\": \"" << pattern_name(options.pattern) << "\",\n"
               << "  \"flat_storage\": \""
               << storage_mode_name(options.flat_storage) << "\",\n"
               << "  \"cells\": " << options.cells << ",\n"
               << "  \"iterations\": " << options.iterations << ",\n"
               << "  \"warmup\": " << options.warmup << ",\n"
               << "  \"touched_chunks\": " << options.touched_chunks << ",\n"
               << "  \"hold_seconds\": " << options.hold_seconds << ",\n"
               << "  \"flat_keyframe_storage_ns\": "
               << flat_keyframe_storage_ns << ",\n"
               << "  \"flat_keyframe_hash_ns\": " << flat_keyframe_hash_ns << ",\n"
               << "  \"flat_keyframe_total_ns\": " << flat_keyframe_total_ns << ",\n"
               << "  \"merkle_keyframe_storage_ns\": "
               << merkle_keyframe_timings.storage_ns << ",\n"
               << "  \"merkle_keyframe_tree_ns\": "
               << merkle_keyframe_timings.merkle_ns << ",\n"
               << "  \"merkle_keyframe_commit_ns\": "
               << merkle_keyframe_timings.commit_ns << ",\n"
               << "  \"keyframe_leaf_count\": " << keyframe_metrics.leaf_count << ",\n"
               << "  \"keyframe_node_count\": " << keyframe_metrics.node_count << ",\n"
               << "  \"keyframe_owned_bytes\": " << keyframe_metrics.owned_bytes << ",\n"
               << "  \"flat_cow_apply_mean_ns\": "
               << (flat_cow.empty() ? 0U : std::accumulate(
                                                  flat_cow.begin(), flat_cow.end(),
                                                  std::uint64_t {0U}) / flat_cow.size())
               << ",\n"
               << "  \"flat_hash_mean_ns\": " << flat_mean << ",\n"
               << "  \"flat_hash_p95_ns\": " << percentile(flat, 95U, 100U) << ",\n"
               << "  \"flat_total_apply_mean_ns\": "
               << (flat_total.empty() ? 0U : std::accumulate(
                                                    flat_total.begin(), flat_total.end(),
                                                    std::uint64_t {0U}) / flat_total.size())
               << ",\n"
               << "  \"merkle_apply_mean_ns\": " << merkle_mean << ",\n"
               << "  \"merkle_apply_p95_ns\": " << percentile(merkle, 95U, 100U) << ",\n"
               << "  \"validation_performed\": "
               << (options.mode == RunMode::Correctness ? "true" : "false") << ",\n"
               << "  \"all_flat_hash_match\": ";
        if(options.mode == RunMode::Correctness) {
            output << (std::all_of(
                               samples.begin(),
                               samples.end(),
                               [](const Sample & sample) { return sample.flat_hash_match; })
                               ? "true"
                               : "false");
        }
        else {
            output << "null";
        }
        output << ",\n"
               << "  \"all_content_match\": ";
        if(options.mode == RunMode::Correctness) {
            output << (std::all_of(
                               samples.begin(),
                               samples.end(),
                               [](const Sample & sample) { return sample.content_match; })
                               ? "true"
                               : "false");
        }
        else {
            output << "null";
        }
        output
               << "\n}\n";
    }

}// namespace

int main(int argc, char ** argv)
{
    try {
        const auto options = parse_options(argc, argv);
        if(std::filesystem::exists(options.output_dir)) {
            throw std::invalid_argument("--output-dir must not already exist");
        }
        if(!std::filesystem::create_directories(options.output_dir)) {
            throw std::runtime_error("failed to create benchmark output directory");
        }

        const SourceIdentity source {"merkle-benchmark", {1U, 1U}, 1U};
        const auto geometry_fingerprint = ContentHasher::geometry_fingerprint(
                {0.1, {0.0, 0.0, 0.0}, "map"});
        auto cells = make_cells(options.cells);
        struct ToggleTarget {
            ChunkCoordinate coordinate;
            VoxelIndex index;
            CellState state = CellState::Free;
        };
        std::vector<ToggleTarget> targets;
        {
            const auto initial_chunks = group_cells(cells);
            targets.reserve(initial_chunks.size());
            for(const auto & entry : initial_chunks) {
                if(entry.second.empty()) {
                    throw std::logic_error("initial benchmark chunk is empty");
                }
                targets.push_back({
                        entry.first,
                        entry.second.front().index,
                        entry.second.front().state});
            }
        }
        if(options.pattern == MutationPattern::UpdateExisting
           && options.touched_chunks > targets.size()) {
            throw std::invalid_argument("workload touches more chunks than the keyframe contains");
        }

        CellSnapshotStore flat_store({options.flat_storage, kMerkleChunkEdge, 256U});
        std::uint64_t flat_keyframe_storage_ns = 0U;
        std::uint64_t flat_keyframe_hash_ns = 0U;
        std::uint64_t flat_keyframe_total_ns = 0U;
        if(options.mode != RunMode::Merkle) {
            const auto flat_keyframe_begin = Clock::now();
            const auto flat_keyframe = flat_store.replace(cells);
            const auto flat_keyframe_storage_end = Clock::now();
            if(!flat_keyframe.success) {
                throw std::runtime_error(flat_keyframe.diagnostic);
            }
            const auto flat_keyframe_hash = ContentHasher::content_hash(
                    source, geometry_fingerprint, flat_store.view());
            const auto flat_keyframe_end = Clock::now();
            if(is_zero_hash(flat_keyframe_hash)) {
                throw std::runtime_error("flat keyframe hash unexpectedly produced a zero digest");
            }
            flat_keyframe_storage_ns = elapsed_ns(
                    flat_keyframe_begin, flat_keyframe_storage_end);
            flat_keyframe_hash_ns = elapsed_ns(
                    flat_keyframe_storage_end, flat_keyframe_end);
            flat_keyframe_total_ns = elapsed_ns(flat_keyframe_begin, flat_keyframe_end);
        }
        MerkleTreeMetrics keyframe_metrics;
        MerklePrototypeTimings merkle_keyframe_timings;
        std::shared_ptr<MerklePrototypeApplier> merkle_state;
        if(options.mode != RunMode::Flat) {
            auto base = MerklePrototypeApplier::build(
                    source,
                    geometry_fingerprint,
                    cells,
                    {CellStorageMode::Chunked, kMerkleChunkEdge, 256U});
            if(!base || !base.candidate) {
                throw std::runtime_error(base.diagnostic);
            }
            if(options.mode == RunMode::Correctness
               && base.candidate->store().view() != cells) {
                throw std::runtime_error("Merkle keyframe does not materialize canonical cells");
            }
            keyframe_metrics = base.merkle_metrics;
            merkle_keyframe_timings = base.timings;
            merkle_state = std::move(base.candidate);
        }

        std::map<VoxelIndex, CellState> oracle;
        if(options.mode == RunMode::Correctness) {
            for(const auto & cell : cells) {
                oracle[cell.index] = cell.state;
            }
        }
        cells.clear();
        cells.shrink_to_fit();

        std::vector<Sample> samples;
        samples.reserve(options.iterations);
        for(std::size_t iteration = 0U;
            iteration < options.warmup + options.iterations;
            ++iteration) {
            std::vector<DeltaOperation> operations;
            operations.reserve(options.touched_chunks);
            if(options.pattern == MutationPattern::UpdateExisting) {
                for(std::size_t offset = 0U; offset < options.touched_chunks; ++offset) {
                    auto & target = targets[
                            (iteration * options.touched_chunks + offset) % targets.size()];
                    target.state = target.state == CellState::Free
                                           ? CellState::Occupied
                                           : CellState::Free;
                    operations.push_back({
                            target.index,
                            target.state == CellState::Free
                                    ? DeltaOperationKind::UpsertFree
                                    : DeltaOperationKind::UpsertOccupied});
                }
            }
            else {
                constexpr std::uint64_t kFirstInsertedChunk = 1000000U;
                for(std::size_t offset = 0U; offset < options.touched_chunks; ++offset) {
                    if(iteration > (std::numeric_limits<std::uint64_t>::max() - offset)
                                           / options.touched_chunks) {
                        throw std::overflow_error("insert workload sequence overflows");
                    }
                    const auto sequence = static_cast<std::uint64_t>(iteration)
                                                  * options.touched_chunks
                                          + offset;
                    if(sequence > static_cast<std::uint64_t>(
                                          std::numeric_limits<std::int64_t>::max() / 16)
                                          - kFirstInsertedChunk) {
                        throw std::overflow_error("insert workload coordinate overflows");
                    }
                    const auto chunk_x = static_cast<std::int64_t>(
                            kFirstInsertedChunk + sequence);
                    operations.push_back({
                            {chunk_x * 16, -16, 16},
                            sequence % 2U == 0U
                                    ? DeltaOperationKind::UpsertFree
                                    : DeltaOperationKind::UpsertOccupied});
                }
            }
            std::sort(
                    operations.begin(),
                    operations.end(),
                    [](const DeltaOperation & left, const DeltaOperation & right) {
                        return left.index < right.index;
                    });
            Sample sample;
            sample.iteration = iteration >= options.warmup
                                       ? iteration - options.warmup
                                       : 0U;
            if(options.pattern == MutationPattern::InsertNew) {
                if(options.touched_chunks > (std::numeric_limits<std::size_t>::max()
                                                     - options.cells)
                                                    / (iteration + 1U)) {
                    throw std::overflow_error("insert workload cell count overflows");
                }
                sample.known_cells = options.cells
                                     + (iteration + 1U) * options.touched_chunks;
            }
            else {
                sample.known_cells = options.cells;
            }
            sample.touched_chunks = options.touched_chunks;

            std::optional<Hash256> flat_hash;
            if(options.mode != RunMode::Merkle) {
                auto flat_candidate = flat_store;
                const auto flat_cow_begin = Clock::now();
                const auto flat_apply = flat_candidate.apply(operations);
                const auto flat_cow_end = Clock::now();
                if(!flat_apply.success) {
                    throw std::runtime_error(flat_apply.diagnostic);
                }
                const auto flat_begin = Clock::now();
                flat_hash = ContentHasher::content_hash(
                        source, geometry_fingerprint, flat_candidate.view());
                const auto flat_end = Clock::now();
                if(is_zero_hash(*flat_hash)) {
                    throw std::runtime_error("flat hash unexpectedly produced a zero digest");
                }
                sample.flat_cow_apply_ns = elapsed_ns(flat_cow_begin, flat_cow_end);
                sample.flat_hash_ns = elapsed_ns(flat_begin, flat_end);
                sample.flat_total_apply_ns = elapsed_ns(flat_cow_begin, flat_end);
                flat_store = std::move(flat_candidate);
            }

            MerklePrototypeResult candidate;
            if(options.mode != RunMode::Flat) {
                const auto merkle_begin = Clock::now();
                candidate = merkle_state->apply(operations);
                const auto merkle_end = Clock::now();
                if(!candidate) {
                    throw std::runtime_error(candidate.diagnostic);
                }
                sample.merkle_apply_ns = elapsed_ns(merkle_begin, merkle_end);
                sample.merkle_allocated_nodes = candidate.merkle_metrics.allocated_nodes;
                sample.merkle_storage_ns = candidate.timings.storage_ns;
                sample.merkle_mutation_build_ns = candidate.timings.mutation_build_ns;
                sample.merkle_tree_ns = candidate.timings.merkle_ns;
                sample.merkle_commit_ns = candidate.timings.commit_ns;
                sample.merkle_leaf_hash_ns = candidate.merkle_metrics.leaf_hash_ns;
                sample.merkle_branch_hash_ns = candidate.merkle_metrics.branch_hash_ns;
                sample.merkle_content_hash_ns = candidate.merkle_metrics.content_hash_ns;
                sample.merkle_path_nodes = candidate.merkle_metrics.path_nodes_rebuilt;
                sample.merkle_owned_bytes = candidate.merkle_metrics.owned_bytes;
                sample.merkle_candidate_owned_bytes =
                        candidate.merkle_metrics.candidate_owned_bytes;
                sample.storage_candidate_owned_bytes =
                        candidate.storage_metrics.candidate_owned_bytes;
                merkle_state = candidate.candidate;
            }

            if(options.mode == RunMode::Correctness) {
                for(const auto & operation : operations) {
                    oracle[operation.index] = operation.kind == DeltaOperationKind::UpsertFree
                                                      ? CellState::Free
                                                      : CellState::Occupied;
                }
                const auto next_cells = flatten(oracle);
                const auto expected_flat_hash = ContentHasher::content_hash(
                        source, geometry_fingerprint, next_cells);
                const auto rebuilt = MerklePatriciaTree::full_rebuild(
                        source, geometry_fingerprint, next_cells);
                if(!rebuilt || !rebuilt.tree) {
                    throw std::runtime_error(rebuilt.diagnostic);
                }
                sample.validation_performed = true;
                sample.flat_hash_match = flat_hash.has_value()
                                         && *flat_hash == expected_flat_hash;
                sample.content_match =
                        candidate.candidate->store().view() == next_cells
                        && flat_store.view() == next_cells
                        && candidate.candidate->tree().content_root()
                                   == rebuilt.tree->content_root();
                if(!sample.flat_hash_match || !sample.content_match) {
                    throw std::runtime_error("correctness replay diverged");
                }
            }

            if(iteration >= options.warmup) {
                samples.push_back(sample);
            }
        }

        write_csv(options.output_dir / "merkle-hash-benchmark.csv", samples);
        write_summary(
                options.output_dir / "merkle-hash-benchmark-summary.json",
                options,
                samples,
                flat_keyframe_storage_ns,
                flat_keyframe_hash_ns,
                flat_keyframe_total_ns,
                merkle_keyframe_timings,
                keyframe_metrics);
        std::cout << "wrote " << samples.size() << " benchmark samples to "
                  << options.output_dir.string() << '\n';
        if(options.hold_seconds > 0U) {
            std::cout << "holding committed benchmark state for " << options.hold_seconds
                      << " seconds\n";
            std::cout.flush();
            std::this_thread::sleep_for(std::chrono::seconds(options.hold_seconds));
        }
        return 0;
    }
    catch(const std::exception & error) {
        std::cerr << "perception_merkle_profile: " << error.what() << '\n';
        return 2;
    }
}

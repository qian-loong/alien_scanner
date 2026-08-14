#include "perception_profiling/ChunkLayoutEstimator.hpp"

#include "perception_map_update/SpatialChunkLayout.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>

namespace PerceptionProfiling {

    namespace {

        using PerceptionMapUpdate::ChunkCoordinate;

        std::uint64_t checked_add(std::uint64_t left, std::uint64_t right)
        {
            if(right > std::numeric_limits<std::uint64_t>::max() - left) {
                throw std::overflow_error("chunk layout metric overflow");
            }
            return left + right;
        }

        std::uint64_t checked_multiply(std::uint64_t left, std::uint64_t right)
        {
            if(left != 0U && right > std::numeric_limits<std::uint64_t>::max() / left) {
                throw std::overflow_error("chunk layout metric overflow");
            }
            return left * right;
        }

        ChunkCoordinate coordinate_for(
                const PerceptionMapUpdate::VoxelIndex & index,
                std::uint32_t chunk_edge)
        {
            const auto address = PerceptionMapUpdate::locate_chunk(index, chunk_edge);
            if(!address) {
                throw std::invalid_argument(address.diagnostic);
            }
            return address.address.chunk;
        }

        std::uint64_t percentile(
                const std::vector<std::uint64_t> & sorted,
                std::uint64_t numerator,
                std::uint64_t denominator)
        {
            if(sorted.empty()) {
                return 0U;
            }
            const auto rank = (checked_multiply(sorted.size(), numerator) + denominator - 1U)
                              / denominator;
            return sorted[static_cast<std::size_t>(std::max<std::uint64_t>(1U, rank) - 1U)];
        }

    }// namespace

    DistributionSummary ChunkLayoutEstimator::summarize(
            std::vector<std::uint64_t> samples)
    {
        if(samples.empty()) {
            return {};
        }
        std::sort(samples.begin(), samples.end());
        return {
                samples.front(),
                percentile(samples, 50U, 100U),
                percentile(samples, 95U, 100U),
                samples.back()};
    }

    ChunkLayoutEstimate ChunkLayoutEstimator::estimate(
            std::string workload,
            std::string input_identity,
            const std::vector<PerceptionMapUpdate::CanonicalCell> & base_cells,
            const std::vector<PerceptionMapUpdate::DeltaOperation> & operations,
            std::uint32_t chunk_edge,
            std::size_t bucket_count)
    {
        if(chunk_edge == 0U) {
            throw std::invalid_argument("chunk edge must be positive");
        }
        if(bucket_count == 0U) {
            throw std::invalid_argument("bucket count must be positive");
        }

        std::map<ChunkCoordinate, std::uint64_t> chunk_cells;
        for(const auto & cell : base_cells) {
            ++chunk_cells[coordinate_for(cell.index, chunk_edge)];
        }

        std::set<ChunkCoordinate> touched_chunks;
        for(const auto & operation : operations) {
            touched_chunks.insert(coordinate_for(operation.index, chunk_edge));
        }

        std::vector<std::uint64_t> chunk_occupancy;
        chunk_occupancy.reserve(chunk_cells.size());
        std::vector<std::uint64_t> bucket_occupancy(bucket_count, 0U);
        for(const auto & entry : chunk_cells) {
            chunk_occupancy.push_back(entry.second);
            ++bucket_occupancy[PerceptionMapUpdate::chunk_bucket_index(
                    entry.first, bucket_count)];
        }

        std::uint64_t copied_cells = 0U;
        std::uint64_t touched_existing_chunks = 0U;
        for(const auto & coordinate : touched_chunks) {
            const auto found = chunk_cells.find(coordinate);
            if(found != chunk_cells.end()) {
                ++touched_existing_chunks;
                copied_cells = checked_add(copied_cells, found->second);
            }
        }

        const auto known_cells = static_cast<std::uint64_t>(base_cells.size());
        const auto total_chunks = static_cast<std::uint64_t>(chunk_cells.size());
        const auto operation_count = static_cast<std::uint64_t>(operations.size());
        const auto edge = static_cast<std::uint64_t>(chunk_edge);
        const auto chunk_capacity = checked_multiply(checked_multiply(edge, edge), edge);
        const auto total_capacity = checked_multiply(total_chunks, chunk_capacity);

        ChunkLayoutEstimate estimate;
        estimate.workload = std::move(workload);
        estimate.input_identity = std::move(input_identity);
        estimate.chunk_edge = chunk_edge;
        estimate.bucket_count = bucket_count;
        estimate.known_cells = known_cells;
        estimate.operation_count = operation_count;
        estimate.total_chunks = total_chunks;
        estimate.touched_chunks = static_cast<std::uint64_t>(touched_chunks.size());
        estimate.touched_existing_chunks = touched_existing_chunks;
        estimate.copied_cells = copied_cells;
        estimate.estimated_chunk_payload_bytes = checked_multiply(
                known_cells, sizeof(PerceptionMapUpdate::CanonicalCell));
        estimate.estimated_chunk_metadata_bytes = checked_multiply(
                total_chunks,
                sizeof(ChunkCoordinate) + sizeof(std::shared_ptr<const void>)
                        + sizeof(std::vector<PerceptionMapUpdate::CanonicalCell>));
        estimate.estimated_bucket_metadata_bytes = checked_multiply(
                total_chunks,
                sizeof(ChunkCoordinate) + sizeof(std::shared_ptr<const void>));
        estimate.estimated_directory_bytes = checked_multiply(
                bucket_count, sizeof(std::shared_ptr<const void>));
        estimate.write_amplification = operation_count == 0U
                                               ? 0.0
                                               : static_cast<double>(copied_cells)
                                                         / static_cast<double>(operation_count);
        estimate.shared_chunk_ratio = total_chunks == 0U
                                              ? 1.0
                                              : 1.0
                                                        - static_cast<double>(
                                                                  touched_existing_chunks)
                                                                  / static_cast<double>(
                                                                          total_chunks);
        estimate.sparse_fill_ratio = total_capacity == 0U
                                             ? 0.0
                                             : static_cast<double>(known_cells)
                                                       / static_cast<double>(total_capacity);
        estimate.cells_per_chunk = summarize(std::move(chunk_occupancy));
        estimate.bucket_occupancy = summarize(std::move(bucket_occupancy));
        return estimate;
    }

}// namespace PerceptionProfiling

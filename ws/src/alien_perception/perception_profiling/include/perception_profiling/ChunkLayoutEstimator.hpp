#ifndef PERCEPTION_PROFILING_CHUNK_LAYOUT_ESTIMATOR_HPP
#define PERCEPTION_PROFILING_CHUNK_LAYOUT_ESTIMATOR_HPP

#include "perception_map_update/MapUpdateTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace PerceptionProfiling {

    struct DistributionSummary {
        std::uint64_t minimum = 0U;
        std::uint64_t p50 = 0U;
        std::uint64_t p95 = 0U;
        std::uint64_t maximum = 0U;
    };

    struct ChunkLayoutEstimate {
        std::string workload;
        std::string input_identity;
        std::uint32_t chunk_edge = 0U;
        std::size_t bucket_count = 0U;
        std::uint64_t known_cells = 0U;
        std::uint64_t operation_count = 0U;
        std::uint64_t total_chunks = 0U;
        std::uint64_t touched_chunks = 0U;
        std::uint64_t touched_existing_chunks = 0U;
        std::uint64_t copied_cells = 0U;
        std::uint64_t estimated_chunk_payload_bytes = 0U;
        std::uint64_t estimated_chunk_metadata_bytes = 0U;
        std::uint64_t estimated_bucket_metadata_bytes = 0U;
        std::uint64_t estimated_directory_bytes = 0U;
        double write_amplification = 0.0;
        double shared_chunk_ratio = 0.0;
        double sparse_fill_ratio = 0.0;
        DistributionSummary cells_per_chunk;
        DistributionSummary bucket_occupancy;
    };

    class ChunkLayoutEstimator
    {
    public:
        static ChunkLayoutEstimate estimate(
                std::string workload,
                std::string input_identity,
                const std::vector<PerceptionMapUpdate::CanonicalCell> & base_cells,
                const std::vector<PerceptionMapUpdate::DeltaOperation> & operations,
                std::uint32_t chunk_edge,
                std::size_t bucket_count);

        static DistributionSummary summarize(std::vector<std::uint64_t> samples);
    };

}// namespace PerceptionProfiling

#endif// PERCEPTION_PROFILING_CHUNK_LAYOUT_ESTIMATOR_HPP

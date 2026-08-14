#ifndef PERCEPTION_MAP_UPDATE_SPATIAL_CHUNK_LAYOUT_HPP
#define PERCEPTION_MAP_UPDATE_SPATIAL_CHUNK_LAYOUT_HPP

#include "perception_map_update/MapUpdateTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace PerceptionMapUpdate {

    struct ChunkCoordinate {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        bool operator==(const ChunkCoordinate & other) const noexcept;
        bool operator!=(const ChunkCoordinate & other) const noexcept;
        bool operator<(const ChunkCoordinate & other) const noexcept;
    };

    struct ChunkLocalCoordinate {
        std::uint32_t x = 0U;
        std::uint32_t y = 0U;
        std::uint32_t z = 0U;

        bool operator==(const ChunkLocalCoordinate & other) const noexcept;
    };

    struct ChunkAddress {
        ChunkCoordinate      chunk;
        ChunkLocalCoordinate local;

        bool operator==(const ChunkAddress & other) const noexcept;
    };

    struct ChunkAddressResult {
        bool         success = false;
        ChunkAddress address;
        std::string  diagnostic;

        explicit operator bool() const noexcept { return success; }
    };

    ChunkAddressResult locate_chunk(
            const VoxelIndex & index,
            std::uint32_t chunk_edge) noexcept;

    std::size_t chunk_bucket_index(
            const ChunkCoordinate & coordinate,
            std::size_t bucket_count) noexcept;

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_SPATIAL_CHUNK_LAYOUT_HPP

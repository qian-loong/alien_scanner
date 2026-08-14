#include "perception_map_update/SpatialChunkLayout.hpp"

#include <tuple>

namespace PerceptionMapUpdate {

    namespace {

        struct AxisAddress {
            std::int64_t  chunk = 0;
            std::uint32_t local = 0U;
        };

        AxisAddress locate_axis(std::int64_t value, std::uint32_t edge) noexcept
        {
            const auto divisor = static_cast<std::int64_t>(edge);
            auto quotient = value / divisor;
            auto remainder = value % divisor;
            if(remainder < 0) {
                --quotient;
                remainder += divisor;
            }
            return {quotient, static_cast<std::uint32_t>(remainder)};
        }

        std::uint64_t mix64(std::uint64_t value) noexcept
        {
            value += 0x9e3779b97f4a7c15ULL;
            value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
            value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
            return value ^ (value >> 31U);
        }

    }// namespace

    bool ChunkCoordinate::operator==(const ChunkCoordinate & other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool ChunkCoordinate::operator!=(const ChunkCoordinate & other) const noexcept
    {
        return !(*this == other);
    }

    bool ChunkCoordinate::operator<(const ChunkCoordinate & other) const noexcept
    {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }

    bool ChunkLocalCoordinate::operator==(
            const ChunkLocalCoordinate & other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool ChunkAddress::operator==(const ChunkAddress & other) const noexcept
    {
        return chunk == other.chunk && local == other.local;
    }

    ChunkAddressResult locate_chunk(
            const VoxelIndex & index,
            std::uint32_t chunk_edge) noexcept
    {
        if(chunk_edge == 0U) {
            return {false, {}, "chunk edge must be positive"};
        }
        const auto x = locate_axis(index.x, chunk_edge);
        const auto y = locate_axis(index.y, chunk_edge);
        const auto z = locate_axis(index.z, chunk_edge);
        return {
                true,
                {{x.chunk, y.chunk, z.chunk}, {x.local, y.local, z.local}},
                {}};
    }

    std::size_t chunk_bucket_index(
            const ChunkCoordinate & coordinate,
            std::size_t bucket_count) noexcept
    {
        if(bucket_count == 0U) {
            return 0U;
        }
        const auto x = mix64(static_cast<std::uint64_t>(coordinate.x));
        const auto y = mix64(static_cast<std::uint64_t>(coordinate.y) ^ x);
        const auto z = mix64(static_cast<std::uint64_t>(coordinate.z) ^ y);
        return static_cast<std::size_t>(z % static_cast<std::uint64_t>(bucket_count));
    }

}// namespace PerceptionMapUpdate

#include "perception_map_update/MapUpdateTypes.hpp"

#include <algorithm>

namespace PerceptionMapUpdate {

    bool Point3d::operator==(const Point3d & other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool VoxelIndex::operator==(const VoxelIndex & other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool VoxelIndex::operator!=(const VoxelIndex & other) const noexcept
    {
        return !(*this == other);
    }

    bool VoxelIndex::operator<(const VoxelIndex & other) const noexcept
    {
        if(x != other.x) {
            return x < other.x;
        }
        if(y != other.y) {
            return y < other.y;
        }
        return z < other.z;
    }

    bool SourceIdentity::operator==(const SourceIdentity & other) const noexcept
    {
        return vehicle_id == other.vehicle_id && mapper_session == other.mapper_session
               && map_epoch == other.map_epoch;
    }

    bool SourceIdentity::operator!=(const SourceIdentity & other) const noexcept
    {
        return !(*this == other);
    }

    bool SourceIdentity::operator<(const SourceIdentity & other) const noexcept
    {
        if(vehicle_id != other.vehicle_id) {
            return vehicle_id < other.vehicle_id;
        }
        if(mapper_session != other.mapper_session) {
            return mapper_session < other.mapper_session;
        }
        return map_epoch < other.map_epoch;
    }

    bool MapGeometry::operator==(const MapGeometry & other) const noexcept
    {
        return resolution_m == other.resolution_m && lattice_origin == other.lattice_origin
               && frame_id == other.frame_id;
    }

    bool RevisionProvenance::operator==(const RevisionProvenance & other) const noexcept
    {
        return sensor_id == other.sensor_id && sensor_session == other.sensor_session
               && observation_stamp == other.observation_stamp
               && clock_domain == other.clock_domain
               && changed_cell_count == other.changed_cell_count;
    }

    bool CanonicalCell::operator==(const CanonicalCell & other) const noexcept
    {
        return index == other.index && state == other.state;
    }

    bool DeltaOperation::operator==(const DeltaOperation & other) const noexcept
    {
        return index == other.index && kind == other.kind;
    }

    bool is_zero_hash(const Hash256 & hash) noexcept
    {
        return std::all_of(hash.begin(), hash.end(), [](std::uint8_t byte) { return byte == 0U; });
    }

    std::string hash_to_hex(const Hash256 & hash)
    {
        constexpr char digits[] = "0123456789abcdef";
        std::string result;
        result.resize(hash.size() * 2U);
        for(std::size_t index = 0U; index < hash.size(); ++index) {
            result[index * 2U]      = digits[(hash[index] >> 4U) & 0x0fU];
            result[index * 2U + 1U] = digits[hash[index] & 0x0fU];
        }
        return result;
    }

}// namespace PerceptionMapUpdate

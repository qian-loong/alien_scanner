#ifndef PERCEPTION_MAP_UPDATE_CANONICAL_ENCODING_HPP
#define PERCEPTION_MAP_UPDATE_CANONICAL_ENCODING_HPP

#include "perception_map_update/MapUpdateTypes.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace PerceptionMapUpdate::Encoding {

    template<typename Sink>
    void write_u8(Sink & sink, std::uint8_t value)
    {
        sink.append(&value, 1U);
    }

    template<typename Sink>
    void write_u16(Sink & sink, std::uint16_t value)
    {
        const std::uint8_t bytes[] = {
                static_cast<std::uint8_t>((value >> 8U) & 0xffU),
                static_cast<std::uint8_t>(value & 0xffU)};
        sink.append(bytes, sizeof(bytes));
    }

    template<typename Sink>
    void write_u32(Sink & sink, std::uint32_t value)
    {
        const std::uint8_t bytes[] = {
                static_cast<std::uint8_t>((value >> 24U) & 0xffU),
                static_cast<std::uint8_t>((value >> 16U) & 0xffU),
                static_cast<std::uint8_t>((value >> 8U) & 0xffU),
                static_cast<std::uint8_t>(value & 0xffU)};
        sink.append(bytes, sizeof(bytes));
    }

    template<typename Sink>
    void write_u64(Sink & sink, std::uint64_t value)
    {
        std::uint8_t bytes[8];
        for(std::size_t index = 0U; index < sizeof(bytes); ++index) {
            bytes[index] = static_cast<std::uint8_t>(
                    (value >> ((sizeof(bytes) - index - 1U) * 8U)) & 0xffU);
        }
        sink.append(bytes, sizeof(bytes));
    }

    template<typename Sink>
    void write_i64(Sink & sink, std::int64_t value)
    {
        std::uint64_t bits = 0U;
        static_assert(sizeof(bits) == sizeof(value), "int64 size mismatch");
        std::memcpy(&bits, &value, sizeof(bits));
        write_u64(sink, bits);
    }

    template<typename Sink>
    void write_double(Sink & sink, double value)
    {
        if(!std::isfinite(value)) {
            throw std::invalid_argument("canonical double must be finite");
        }
        // Canonical form uses the positive IEEE-754 zero representation.
        if(value == 0.0) {
            value = 0.0;
        }
        std::uint64_t bits = 0U;
        static_assert(sizeof(bits) == sizeof(value), "double size mismatch");
        std::memcpy(&bits, &value, sizeof(bits));
        write_u64(sink, bits);
    }

    template<typename Sink>
    void write_string(Sink & sink, const std::string & value)
    {
        if(value.size() > std::numeric_limits<std::uint32_t>::max()) {
            throw std::length_error("canonical string length exceeds uint32");
        }
        write_u32(sink, static_cast<std::uint32_t>(value.size()));
        if(!value.empty()) {
            sink.append(reinterpret_cast<const std::uint8_t *>(value.data()), value.size());
        }
    }

    template<typename Sink>
    void write_hash(Sink & sink, const Hash256 & hash)
    {
        sink.append(hash.data(), hash.size());
    }

    template<typename Sink>
    void write_identity(Sink & sink, const SourceIdentity & identity)
    {
        write_string(sink, identity.vehicle_id);
        write_u64(sink, identity.mapper_session.boot_time_ns);
        write_u32(sink, identity.mapper_session.random_suffix);
        write_u64(sink, identity.map_epoch);
    }

    template<typename Sink>
    void write_geometry(Sink & sink, const MapGeometry & geometry)
    {
        write_double(sink, geometry.resolution_m);
        write_double(sink, geometry.lattice_origin.x);
        write_double(sink, geometry.lattice_origin.y);
        write_double(sink, geometry.lattice_origin.z);
        write_string(sink, geometry.frame_id);
    }

    template<typename Sink>
    void write_index(Sink & sink, const VoxelIndex & index)
    {
        write_i64(sink, index.x);
        write_i64(sink, index.y);
        write_i64(sink, index.z);
    }

    template<typename Sink>
    void write_cell(Sink & sink, const CanonicalCell & cell)
    {
        write_index(sink, cell.index);
        write_u8(sink, static_cast<std::uint8_t>(cell.state));
    }

    template<typename Sink>
    void write_operation(Sink & sink, const DeltaOperation & operation)
    {
        write_index(sink, operation.index);
        write_u8(sink, static_cast<std::uint8_t>(operation.kind));
    }

    template<typename Sink>
    void write_provenance(Sink & sink, const RevisionProvenance & provenance)
    {
        write_string(sink, provenance.sensor_id);
        write_u64(sink, provenance.sensor_session.boot_time_ns);
        write_u32(sink, provenance.sensor_session.random_suffix);
        write_i64(sink, provenance.observation_stamp.nanoseconds);
        write_string(sink, provenance.clock_domain);
        write_u32(sink, provenance.changed_cell_count);
    }

}// namespace PerceptionMapUpdate::Encoding

#endif// PERCEPTION_MAP_UPDATE_CANONICAL_ENCODING_HPP

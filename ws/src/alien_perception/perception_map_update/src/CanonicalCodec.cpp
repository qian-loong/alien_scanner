#include "perception_map_update/CanonicalCodec.hpp"

#include "CanonicalEncoding.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        constexpr std::size_t kPayloadHeaderBytes = 2U + 1U + 8U;
        constexpr std::size_t kEntryBytes         = 8U * 3U + 1U;

        class VectorSink
        {
        public:
            explicit VectorSink(std::size_t reserve)
            {
                bytes.reserve(reserve);
            }

            void append(const std::uint8_t * data, std::size_t size)
            {
                bytes.insert(bytes.end(), data, data + size);
            }

            std::vector<std::uint8_t> bytes;
        };

        bool checked_payload_size(std::size_t count, std::size_t limit, std::size_t & result)
        {
            if(count > (std::numeric_limits<std::size_t>::max() - kPayloadHeaderBytes)
                               / kEntryBytes) {
                return false;
            }
            result = kPayloadHeaderBytes + count * kEntryBytes;
            return result <= limit;
        }

        ValidationResult validate_payload_size(
                std::uint64_t declared_count,
                std::size_t payload_bytes,
                std::size_t count_limit,
                std::size_t byte_limit,
                const char * kind)
        {
            if(declared_count > count_limit
               || declared_count > std::numeric_limits<std::size_t>::max()) {
                return {false, std::string(kind) + " payload count exceeds configured limit"};
            }
            std::size_t expected_size = 0U;
            if(!checked_payload_size(
                       static_cast<std::size_t>(declared_count),
                       byte_limit,
                       expected_size)
               || payload_bytes != expected_size) {
                return {false,
                        std::string(kind)
                                + " payload size does not match declared count"};
            }
            return {true, {}};
        }

        bool valid_utf8(const std::string & value) noexcept
        {
            const auto * bytes = reinterpret_cast<const unsigned char *>(value.data());
            std::size_t index = 0U;
            while(index < value.size()) {
                const unsigned char first = bytes[index];
                if(first <= 0x7fU) {
                    ++index;
                    continue;
                }
                std::size_t continuation = 0U;
                std::uint32_t code_point = 0U;
                if((first & 0xe0U) == 0xc0U) {
                    continuation = 1U;
                    code_point = first & 0x1fU;
                } else if((first & 0xf0U) == 0xe0U) {
                    continuation = 2U;
                    code_point = first & 0x0fU;
                } else if((first & 0xf8U) == 0xf0U) {
                    continuation = 3U;
                    code_point = first & 0x07U;
                } else {
                    return false;
                }
                if(index + continuation >= value.size()) {
                    return false;
                }
                for(std::size_t offset = 1U; offset <= continuation; ++offset) {
                    const unsigned char next = bytes[index + offset];
                    if((next & 0xc0U) != 0x80U) {
                        return false;
                    }
                    code_point = (code_point << 6U) | (next & 0x3fU);
                }
                const bool overlong = (continuation == 1U && code_point < 0x80U)
                                      || (continuation == 2U && code_point < 0x800U)
                                      || (continuation == 3U && code_point < 0x10000U);
                if(overlong || code_point > 0x10ffffU
                   || (code_point >= 0xd800U && code_point <= 0xdfffU)) {
                    return false;
                }
                index += continuation + 1U;
            }
            return true;
        }

        ValidationResult validate_canonical_string(
                const std::string & value,
                std::size_t limit,
                const char * field,
                bool allow_empty)
        {
            if((!allow_empty && value.empty()) || value.size() > limit
               || value.find('\0') != std::string::npos || !valid_utf8(value)) {
                return {false, std::string(field) + " is empty, oversized, or not canonical UTF-8"};
            }
            return {true, {}};
        }

        class Reader
        {
        public:
            explicit Reader(const std::vector<std::uint8_t> & bytes) : bytes_(bytes) {}

            bool u8(std::uint8_t & value)
            {
                if(!available(1U)) {
                    return false;
                }
                value = bytes_[offset_++];
                return true;
            }

            bool u16(std::uint16_t & value)
            {
                if(!available(2U)) {
                    return false;
                }
                value = static_cast<std::uint16_t>(bytes_[offset_]) << 8U
                        | static_cast<std::uint16_t>(bytes_[offset_ + 1U]);
                offset_ += 2U;
                return true;
            }

            bool u64(std::uint64_t & value)
            {
                if(!available(8U)) {
                    return false;
                }
                value = 0U;
                for(std::size_t index = 0U; index < 8U; ++index) {
                    value = (value << 8U) | bytes_[offset_ + index];
                }
                offset_ += 8U;
                return true;
            }

            bool i64(std::int64_t & value)
            {
                std::uint64_t bits = 0U;
                if(!u64(bits)) {
                    return false;
                }
                static_assert(sizeof(bits) == sizeof(value), "int64 size mismatch");
                std::memcpy(&value, &bits, sizeof(value));
                return true;
            }

            bool finished() const noexcept { return offset_ == bytes_.size(); }

        private:
            bool available(std::size_t count) const noexcept
            {
                return count <= bytes_.size() - offset_;
            }

            const std::vector<std::uint8_t> & bytes_;
            std::size_t offset_ = 0U;
        };

        template<typename Entry, typename Validate, typename Write>
        EncodeResult encode_payload(
                UpdateKind kind,
                const std::vector<Entry> & entries,
                std::size_t byte_limit,
                Validate validate,
                Write write)
        {
            const auto validation = validate(entries);
            if(!validation) {
                return {false, {}, validation.diagnostic};
            }
            std::size_t size = 0U;
            if(!checked_payload_size(entries.size(), byte_limit, size)) {
                return {false, {}, "canonical payload exceeds configured byte limit"};
            }
            try {
                VectorSink sink(size);
                Encoding::write_u16(sink, kCanonicalEncodingVersion);
                Encoding::write_u8(sink, static_cast<std::uint8_t>(kind));
                Encoding::write_u64(sink, static_cast<std::uint64_t>(entries.size()));
                for(const auto & entry : entries) {
                    write(sink, entry);
                }
                return {true, std::move(sink.bytes), {}};
            }
            catch(const std::exception & error) {
                return {false, {}, error.what()};
            }
        }

        bool read_header(
                Reader & reader,
                UpdateKind expected_kind,
                std::size_t max_count,
                std::uint64_t & count,
                std::string & diagnostic)
        {
            std::uint16_t version = 0U;
            std::uint8_t kind = 0U;
            if(!reader.u16(version) || !reader.u8(kind) || !reader.u64(count)) {
                diagnostic = "canonical payload header is truncated";
                return false;
            }
            if(version != kCanonicalEncodingVersion
               || kind != static_cast<std::uint8_t>(expected_kind)) {
                diagnostic = "canonical payload version or kind mismatch";
                return false;
            }
            if(count > max_count || count > std::numeric_limits<std::size_t>::max()) {
                diagnostic = "canonical payload count exceeds configured limit";
                return false;
            }
            return true;
        }

    }// namespace

    ValidationResult CanonicalCodec::validate_string(
            const std::string & value,
            std::size_t limit,
            const char * field,
            bool allow_empty)
    {
        return validate_canonical_string(value, limit, field, allow_empty);
    }

    ValidationResult CanonicalCodec::validate_identity(
            const SourceIdentity & identity,
            const MapUpdateLimits & limits)
    {
        const auto vehicle = validate_canonical_string(
                identity.vehicle_id, limits.max_identity_string_bytes, "vehicle_id", false);
        if(!vehicle) {
            return vehicle;
        }
        if(identity.mapper_session.boot_time_ns == 0U || identity.map_epoch == 0U) {
            return {false, "source session and map epoch must be non-zero"};
        }
        return {true, {}};
    }

    ValidationResult CanonicalCodec::validate_geometry(
            const MapGeometry & geometry,
            const MapUpdateLimits & limits)
    {
        if(!std::isfinite(geometry.resolution_m) || geometry.resolution_m <= 0.0
           || !std::isfinite(geometry.lattice_origin.x)
           || !std::isfinite(geometry.lattice_origin.y)
           || !std::isfinite(geometry.lattice_origin.z)) {
            return {false, "map geometry must be finite with positive resolution"};
        }
        return validate_canonical_string(
                geometry.frame_id, limits.max_frame_id_bytes, "frame_id", false);
    }

    ValidationResult CanonicalCodec::validate_provenance(
            const RevisionProvenance & provenance,
            const MapUpdateLimits & limits)
    {
        const auto sensor = validate_canonical_string(
                provenance.sensor_id, limits.max_sensor_id_bytes, "sensor_id", false);
        if(!sensor) {
            return sensor;
        }
        const auto clock = validate_canonical_string(
                provenance.clock_domain,
                limits.max_clock_domain_bytes,
                "clock_domain",
                false);
        if(!clock) {
            return clock;
        }
        if(provenance.sensor_session.boot_time_ns == 0U
           || provenance.observation_stamp.nanoseconds < 0) {
            return {false, "revision provenance session/stamp is invalid"};
        }
        return {true, {}};
    }

    ValidationResult CanonicalCodec::validate_cells(
            const std::vector<CanonicalCell> & cells,
            const MapUpdateLimits & limits)
    {
        if(cells.size() > limits.max_known_cells) {
            return {false, "known cell count exceeds configured limit"};
        }
        for(std::size_t index = 0U; index < cells.size(); ++index) {
            if(cells[index].state != CellState::Free
               && cells[index].state != CellState::Occupied) {
                return {false, "canonical cell has invalid state"};
            }
            if(index > 0U && !(cells[index - 1U].index < cells[index].index)) {
                return {false, "canonical cells are not strictly ordered"};
            }
        }
        return {true, {}};
    }

    ValidationResult CanonicalCodec::validate_operations(
            const std::vector<DeltaOperation> & operations,
            const MapUpdateLimits & limits)
    {
        if(operations.size() > limits.max_delta_operations) {
            return {false, "delta operation count exceeds configured limit"};
        }
        for(std::size_t index = 0U; index < operations.size(); ++index) {
            const auto kind = operations[index].kind;
            if(kind != DeltaOperationKind::UpsertFree
               && kind != DeltaOperationKind::UpsertOccupied
               && kind != DeltaOperationKind::RemoveToUnknown) {
                return {false, "delta operation has invalid kind"};
            }
            if(index > 0U && !(operations[index - 1U].index < operations[index].index)) {
                return {false, "delta operations are not strictly ordered"};
            }
        }
        return {true, {}};
    }

    ValidationResult CanonicalCodec::validate_keyframe_payload_size(
            std::uint64_t declared_count,
            std::size_t payload_bytes,
            const MapUpdateLimits & limits)
    {
        return validate_payload_size(
                declared_count,
                payload_bytes,
                limits.max_known_cells,
                limits.max_keyframe_payload_bytes,
                "keyframe");
    }

    ValidationResult CanonicalCodec::validate_delta_payload_size(
            std::uint64_t declared_count,
            std::size_t payload_bytes,
            const MapUpdateLimits & limits)
    {
        return validate_payload_size(
                declared_count,
                payload_bytes,
                limits.max_delta_operations,
                limits.max_delta_payload_bytes,
                "delta");
    }

    EncodeResult CanonicalCodec::encode_keyframe_payload(
            const std::vector<CanonicalCell> & cells,
            const MapUpdateLimits & limits)
    {
        return encode_payload(
                UpdateKind::Keyframe, cells, limits.max_keyframe_payload_bytes,
                [&](const auto & value) { return validate_cells(value, limits); },
                [](auto & sink, const auto & value) { Encoding::write_cell(sink, value); });
    }

    EncodeResult CanonicalCodec::encode_delta_payload(
            const std::vector<DeltaOperation> & operations,
            const MapUpdateLimits & limits)
    {
        return encode_payload(
                UpdateKind::Delta, operations, limits.max_delta_payload_bytes,
                [&](const auto & value) { return validate_operations(value, limits); },
                [](auto & sink, const auto & value) { Encoding::write_operation(sink, value); });
    }

    DecodeCellsResult CanonicalCodec::decode_keyframe_payload(
            const std::vector<std::uint8_t> & payload,
            const MapUpdateLimits & limits)
    {
        if(payload.size() > limits.max_keyframe_payload_bytes) {
            return {false, {}, "keyframe payload exceeds configured byte limit"};
        }
        Reader reader(payload);
        std::uint64_t count = 0U;
        std::string diagnostic;
        if(!read_header(reader, UpdateKind::Keyframe, limits.max_known_cells, count, diagnostic)) {
            return {false, {}, diagnostic};
        }
        const auto payload_size = validate_keyframe_payload_size(
                count, payload.size(), limits);
        if(!payload_size) {
            return {false, {}, payload_size.diagnostic};
        }
        try {
            std::vector<CanonicalCell> cells;
            cells.reserve(static_cast<std::size_t>(count));
            for(std::uint64_t index = 0U; index < count; ++index) {
                CanonicalCell cell;
                std::uint8_t state = 0U;
                if(!reader.i64(cell.index.x) || !reader.i64(cell.index.y)
                   || !reader.i64(cell.index.z) || !reader.u8(state)) {
                    return {false, {}, "keyframe payload is truncated"};
                }
                cell.state = static_cast<CellState>(state);
                cells.push_back(cell);
            }
            if(!reader.finished()) {
                return {false, {}, "keyframe payload has trailing bytes"};
            }
            const auto validation = validate_cells(cells, limits);
            if(!validation) {
                return {false, {}, validation.diagnostic};
            }
            return {true, std::move(cells), {}};
        }
        catch(const std::bad_alloc &) {
            return {false, {}, "keyframe payload allocation failed"};
        }
    }

    DecodeOperationsResult CanonicalCodec::decode_delta_payload(
            const std::vector<std::uint8_t> & payload,
            const MapUpdateLimits & limits)
    {
        if(payload.size() > limits.max_delta_payload_bytes) {
            return {false, {}, "delta payload exceeds configured byte limit"};
        }
        Reader reader(payload);
        std::uint64_t count = 0U;
        std::string diagnostic;
        if(!read_header(
                   reader, UpdateKind::Delta, limits.max_delta_operations, count, diagnostic)) {
            return {false, {}, diagnostic};
        }
        const auto payload_size = validate_delta_payload_size(
                count, payload.size(), limits);
        if(!payload_size) {
            return {false, {}, payload_size.diagnostic};
        }
        try {
            std::vector<DeltaOperation> operations;
            operations.reserve(static_cast<std::size_t>(count));
            for(std::uint64_t index = 0U; index < count; ++index) {
                DeltaOperation operation;
                std::uint8_t kind = 0U;
                if(!reader.i64(operation.index.x) || !reader.i64(operation.index.y)
                   || !reader.i64(operation.index.z) || !reader.u8(kind)) {
                    return {false, {}, "delta payload is truncated"};
                }
                operation.kind = static_cast<DeltaOperationKind>(kind);
                operations.push_back(operation);
            }
            if(!reader.finished()) {
                return {false, {}, "delta payload has trailing bytes"};
            }
            const auto validation = validate_operations(operations, limits);
            if(!validation) {
                return {false, {}, validation.diagnostic};
            }
            return {true, std::move(operations), {}};
        }
        catch(const std::bad_alloc &) {
            return {false, {}, "delta payload allocation failed"};
        }
    }

}// namespace PerceptionMapUpdate

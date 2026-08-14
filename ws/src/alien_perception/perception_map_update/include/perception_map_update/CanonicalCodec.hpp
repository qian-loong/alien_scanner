#ifndef PERCEPTION_MAP_UPDATE_CANONICAL_CODEC_HPP
#define PERCEPTION_MAP_UPDATE_CANONICAL_CODEC_HPP

#include "perception_map_update/CellSnapshotStore.hpp"
#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <string>
#include <vector>

namespace PerceptionMapUpdate {

    struct EncodeResult {
        bool                      success = false;
        std::vector<std::uint8_t> bytes;
        std::string               diagnostic;
    };

    struct DecodeCellsResult {
        bool                       success = false;
        std::vector<CanonicalCell> cells;
        std::string                diagnostic;
    };

    struct DecodeOperationsResult {
        bool                        success = false;
        std::vector<DeltaOperation> operations;
        std::string                 diagnostic;
    };

    class CanonicalCodec
    {
    public:
        static ValidationResult validate_string(
                const std::string & value,
                std::size_t limit,
                const char * field,
                bool allow_empty);
        static ValidationResult validate_identity(
                const SourceIdentity & identity,
                const MapUpdateLimits & limits);
        static ValidationResult validate_geometry(
                const MapGeometry & geometry,
                const MapUpdateLimits & limits);
        static ValidationResult validate_provenance(
                const RevisionProvenance & provenance,
                const MapUpdateLimits & limits);
        static ValidationResult validate_cells(
                const std::vector<CanonicalCell> & cells,
                const MapUpdateLimits & limits);
        static ValidationResult validate_cells(
                const CanonicalCellView & cells,
                const MapUpdateLimits & limits);
        static ValidationResult validate_operations(
                const std::vector<DeltaOperation> & operations,
                const MapUpdateLimits & limits);
        static ValidationResult validate_keyframe_payload_size(
                std::uint64_t declared_count,
                std::size_t payload_bytes,
                const MapUpdateLimits & limits);
        static ValidationResult validate_delta_payload_size(
                std::uint64_t declared_count,
                std::size_t payload_bytes,
                const MapUpdateLimits & limits);

        static EncodeResult encode_keyframe_payload(
                const std::vector<CanonicalCell> & cells,
                const MapUpdateLimits & limits);
        static EncodeResult encode_delta_payload(
                const std::vector<DeltaOperation> & operations,
                const MapUpdateLimits & limits);
        static DecodeCellsResult decode_keyframe_payload(
                const std::vector<std::uint8_t> & payload,
                const MapUpdateLimits & limits);
        static DecodeOperationsResult decode_delta_payload(
                const std::vector<std::uint8_t> & payload,
                const MapUpdateLimits & limits);
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_CANONICAL_CODEC_HPP

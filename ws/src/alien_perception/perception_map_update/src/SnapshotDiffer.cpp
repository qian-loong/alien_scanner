#include "perception_map_update/SnapshotDiffer.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>

namespace PerceptionMapUpdate {

    namespace {

        DeltaOperationKind upsert_kind(CellState state)
        {
            return state == CellState::Free ? DeltaOperationKind::UpsertFree
                                            : DeltaOperationKind::UpsertOccupied;
        }

    }// namespace

    DiffResult SnapshotDiffer::compare(
            const CanonicalSnapshot & base,
            const CanonicalSnapshot & target,
            const MapUpdateLimits & limits)
    {
        if(base.source != target.source || !(base.geometry == target.geometry)
           || base.geometry_fingerprint != target.geometry_fingerprint) {
            return {false, {}, "snapshot identity or geometry mismatch"};
        }
        if(target.revision <= base.revision
           || target.revision - base.revision > limits.max_revision_span) {
            return {false, {}, "target revision does not form an allowed forward span"};
        }
        const auto base_validation = CanonicalCodec::validate_cells(base.cells, limits);
        const auto target_validation = CanonicalCodec::validate_cells(target.cells, limits);
        if(!base_validation) {
            return {false, {}, base_validation.diagnostic};
        }
        if(!target_validation) {
            return {false, {}, target_validation.diagnostic};
        }

        std::vector<DeltaOperation> operations;
        operations.reserve(std::min(
                limits.max_delta_operations, std::max(base.cells.size(), target.cells.size())));
        std::size_t base_index = 0U;
        std::size_t target_index = 0U;
        while(base_index < base.cells.size() || target_index < target.cells.size()) {
            if(operations.size() == limits.max_delta_operations) {
                return {false, {}, "delta operation count exceeds configured limit"};
            }
            if(base_index == base.cells.size()) {
                const auto & cell = target.cells[target_index++];
                operations.push_back({cell.index, upsert_kind(cell.state)});
                continue;
            }
            if(target_index == target.cells.size()) {
                operations.push_back(
                        {base.cells[base_index++].index, DeltaOperationKind::RemoveToUnknown});
                continue;
            }
            const auto & base_cell = base.cells[base_index];
            const auto & target_cell = target.cells[target_index];
            if(base_cell.index < target_cell.index) {
                operations.push_back(
                        {base_cell.index, DeltaOperationKind::RemoveToUnknown});
                ++base_index;
            } else if(target_cell.index < base_cell.index) {
                operations.push_back({target_cell.index, upsert_kind(target_cell.state)});
                ++target_index;
            } else {
                if(base_cell.state != target_cell.state) {
                    operations.push_back({target_cell.index, upsert_kind(target_cell.state)});
                }
                ++base_index;
                ++target_index;
            }
        }
        return {true, std::move(operations), {}};
    }

}// namespace PerceptionMapUpdate

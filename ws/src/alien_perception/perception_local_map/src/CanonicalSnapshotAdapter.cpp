#include "perception_local_map/CanonicalSnapshotAdapter.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <new>
#include <utility>

namespace PerceptionLocalMap {

    namespace {

        PerceptionMapUpdate::SourceIdentity convert_identity(const MapIdentity & identity)
        {
            return {identity.vehicle_id, identity.mapper_session, identity.map_epoch};
        }

        PerceptionMapUpdate::MapGeometry convert_geometry(const MapGeometry & geometry)
        {
            return {
                    geometry.resolution_m,
                    {geometry.lattice_origin.x,
                     geometry.lattice_origin.y,
                     geometry.lattice_origin.z},
                    geometry.frame_id};
        }

        PerceptionMapUpdate::RevisionProvenance convert_provenance(
                const CommitProvenance & provenance)
        {
            return {
                    provenance.sensor_id.value,
                    provenance.sensor_session,
                    provenance.observation_stamp,
                    provenance.clock_domain,
                    provenance.changed_cell_count};
        }

        bool valid_cell_location(
                const OccupancyCell & cell,
                const MapGeometry & geometry)
        {
            if(!is_representable_voxel(cell.index, geometry)
               || !std::isfinite(cell.center.x) || !std::isfinite(cell.center.y)
               || !std::isfinite(cell.center.z)) {
                return false;
            }
            const auto quantized = quantize_point(cell.center, geometry);
            return quantized.status == QueryStatus::Ok && quantized.index == cell.index;
        }

        std::int64_t elapsed_ns(std::chrono::steady_clock::time_point start) noexcept
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - start)
                    .count();
        }

    }// namespace

    CanonicalSnapshotAdapter::CanonicalSnapshotAdapter(
            PerceptionMapUpdate::MapUpdateLimits limits)
            : limits_(std::move(limits))
    {
    }

    CanonicalSnapshotResult CanonicalSnapshotAdapter::materialize(
            const MapReadTransaction & transaction) const
    {
        CanonicalSnapshotTiming timing;
        const auto metadata = transaction.metadata();
        if(!transaction.is_open() || !metadata.has_value()) {
            return {CanonicalSnapshotStatus::TransactionClosed, std::nullopt,
                    "map read transaction is closed", timing};
        }
        if(metadata->identity.revision == 0U || !metadata->last_commit.has_value()
           || !metadata->contract_fingerprint.is_well_formed()) {
            return {CanonicalSnapshotStatus::InvalidMetadata, std::nullopt,
                    "exact map revision lacks commit provenance or contract fingerprint",
                    timing};
        }

        PerceptionMapUpdate::CanonicalSnapshot snapshot;
        snapshot.source = convert_identity(metadata->identity);
        snapshot.geometry = convert_geometry(metadata->geometry);
        snapshot.revision = metadata->identity.revision;
        snapshot.latest_commit = convert_provenance(*metadata->last_commit);

        const auto identity_validation = PerceptionMapUpdate::CanonicalCodec::validate_identity(
                snapshot.source, limits_);
        const auto geometry_validation = PerceptionMapUpdate::CanonicalCodec::validate_geometry(
                snapshot.geometry, limits_);
        const auto provenance_validation =
                PerceptionMapUpdate::CanonicalCodec::validate_provenance(
                        snapshot.latest_commit, limits_);
        if(!identity_validation || !geometry_validation || !provenance_validation) {
            const auto & diagnostic = !identity_validation
                                              ? identity_validation.diagnostic
                                              : (!geometry_validation
                                                         ? geometry_validation.diagnostic
                                                         : provenance_validation.diagnostic);
            return {CanonicalSnapshotStatus::InvalidMetadata,
                    std::nullopt,
                    diagnostic,
                    timing};
        }

        CanonicalSnapshotStatus cell_status = CanonicalSnapshotStatus::Ready;
        std::string cell_diagnostic;
        const auto traversal_start = std::chrono::steady_clock::now();
        try {
            const bool traversed = transaction.for_each_known_cell(
                    [&](OccupancyCell cell) {
                        if(cell_status != CanonicalSnapshotStatus::Ready) {
                            return;
                        }
                        const std::size_t max_retained_cells =
                                limits_.max_retained_snapshot_bytes
                                / sizeof(PerceptionMapUpdate::CanonicalCell);
                        if(snapshot.cells.size() >= limits_.max_known_cells
                           || snapshot.cells.size() >= max_retained_cells) {
                            cell_status = CanonicalSnapshotStatus::ResourceLimit;
                            cell_diagnostic =
                                    "canonical snapshot cell or retained-byte limit exceeded";
                            return;
                        }
                        if(cell.state == OccupancyState::Unknown
                           || !valid_cell_location(cell, metadata->geometry)) {
                            cell_status = CanonicalSnapshotStatus::InvalidCell;
                            cell_diagnostic = "backend yielded unknown or geometrically invalid cell";
                            return;
                        }
                        const auto state = cell.state == OccupancyState::Free
                                                   ? PerceptionMapUpdate::CellState::Free
                                                   : PerceptionMapUpdate::CellState::Occupied;
                        snapshot.cells.push_back(
                                {{cell.index.x, cell.index.y, cell.index.z}, state});
                    });
            if(!traversed && cell_status == CanonicalSnapshotStatus::Ready) {
                timing.traversal_duration_ns = elapsed_ns(traversal_start);
                return {CanonicalSnapshotStatus::BackendFault, std::nullopt,
                        "backend failed canonical known-cell traversal", timing};
            }
        }
        catch(const std::bad_alloc &) {
            timing.traversal_duration_ns = elapsed_ns(traversal_start);
            return {CanonicalSnapshotStatus::ResourceLimit, std::nullopt,
                    "canonical snapshot allocation failed", timing};
        }
        catch(const std::exception & error) {
            timing.traversal_duration_ns = elapsed_ns(traversal_start);
            return {CanonicalSnapshotStatus::BackendFault,
                    std::nullopt,
                    error.what(),
                    timing};
        }
        catch(...) {
            timing.traversal_duration_ns = elapsed_ns(traversal_start);
            return {CanonicalSnapshotStatus::BackendFault, std::nullopt,
                    "unknown canonical snapshot traversal failure", timing};
        }
        timing.traversal_duration_ns = elapsed_ns(traversal_start);
        if(cell_status != CanonicalSnapshotStatus::Ready) {
            return {cell_status, std::nullopt, std::move(cell_diagnostic), timing};
        }

        const auto canonicalize_start = std::chrono::steady_clock::now();
        std::sort(
                snapshot.cells.begin(), snapshot.cells.end(),
                [](const auto & left, const auto & right) {
                    return left.index < right.index;
                });
        const auto cells_validation = PerceptionMapUpdate::CanonicalCodec::validate_cells(
                snapshot.cells, limits_);
        if(!cells_validation) {
            timing.canonicalize_duration_ns = elapsed_ns(canonicalize_start);
            return {CanonicalSnapshotStatus::InvalidCell, std::nullopt,
                    cells_validation.diagnostic, timing};
        }
        if(snapshot.cells.size()
                   > std::numeric_limits<std::size_t>::max()
                             / sizeof(PerceptionMapUpdate::CanonicalCell)
           || snapshot.cells.size() * sizeof(PerceptionMapUpdate::CanonicalCell)
                      > limits_.max_retained_snapshot_bytes) {
            timing.canonicalize_duration_ns = elapsed_ns(canonicalize_start);
            return {CanonicalSnapshotStatus::ResourceLimit, std::nullopt,
                    "canonical snapshot retained bytes exceed configured limit", timing};
        }
        timing.canonicalize_duration_ns = elapsed_ns(canonicalize_start);
        const auto hash_start = std::chrono::steady_clock::now();
        snapshot.geometry_fingerprint =
                PerceptionMapUpdate::ContentHasher::geometry_fingerprint(snapshot.geometry);
        snapshot.content_hash = PerceptionMapUpdate::ContentHasher::content_hash(
                snapshot.source, snapshot.geometry_fingerprint, snapshot.cells);
        timing.content_hash_duration_ns = elapsed_ns(hash_start);
        return {CanonicalSnapshotStatus::Ready, std::move(snapshot), {}, timing};
    }

}// namespace PerceptionLocalMap

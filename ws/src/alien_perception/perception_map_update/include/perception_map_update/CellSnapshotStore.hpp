#ifndef PERCEPTION_MAP_UPDATE_CELL_SNAPSHOT_STORE_HPP
#define PERCEPTION_MAP_UPDATE_CELL_SNAPSHOT_STORE_HPP

#include "perception_map_update/MapUpdateTypes.hpp"
#include "perception_map_update/SpatialChunkLayout.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

namespace PerceptionMapUpdate {

    class CanonicalCellCursor;

    enum class CellStorageMode : std::uint8_t
    {
        Vector,
        Chunked
    };

    struct CellStorageConfig {
        CellStorageMode mode = CellStorageMode::Vector;
        std::uint32_t chunk_edge = 16U;
        std::size_t bucket_count = 256U;
    };

    struct CellStorageMetrics {
        std::uint64_t total_chunks = 0U;
        std::uint64_t touched_chunks = 0U;
        std::uint64_t shared_chunks = 0U;
        std::uint64_t copied_cells = 0U;
        std::uint64_t copied_bucket_entries = 0U;
        std::size_t committed_live_bytes = 0U;
        std::size_t candidate_owned_bytes = 0U;
        std::size_t traversal_scratch_bytes = 0U;
    };

    class CanonicalCellView
    {
    public:
        using Visitor = std::function<void(const CanonicalCell &)>;
        struct State;

        CanonicalCellView();

        std::size_t size() const noexcept;
        bool empty() const noexcept;
        const CanonicalCell & front() const;
        CanonicalCellCursor cursor() const;
        void for_each(const Visitor & visitor) const;
        std::vector<CanonicalCell> materialize() const;
        bool equals(const std::vector<CanonicalCell> & cells) const;

        CanonicalCellView & operator=(const std::vector<CanonicalCell> & cells);
        CanonicalCellView & operator=(std::vector<CanonicalCell> && cells);
        CanonicalCellView & operator=(std::initializer_list<CanonicalCell> cells);

    private:
        friend class CellSnapshotStore;
        explicit CanonicalCellView(std::shared_ptr<const State> state);
        std::shared_ptr<const State> state_;
    };

    class CanonicalCellCursor
    {
    public:
        struct State;

        CanonicalCellCursor(const CanonicalCellCursor &) = delete;
        CanonicalCellCursor & operator=(const CanonicalCellCursor &) = delete;
        CanonicalCellCursor(CanonicalCellCursor &&) noexcept = default;
        CanonicalCellCursor & operator=(CanonicalCellCursor &&) noexcept = default;

        bool done() const noexcept;
        const CanonicalCell & value() const;
        void advance();

    private:
        friend class CanonicalCellView;
        explicit CanonicalCellCursor(
                std::shared_ptr<const CanonicalCellView::State> snapshot);
        std::shared_ptr<State> state_;
    };

    bool operator==(
            const CanonicalCellView & left,
            const CanonicalCellView & right);
    bool operator!=(
            const CanonicalCellView & left,
            const CanonicalCellView & right);

    bool operator==(
            const CanonicalCellView & view,
            const std::vector<CanonicalCell> & cells);
    bool operator==(
            const std::vector<CanonicalCell> & cells,
            const CanonicalCellView & view);
    bool operator!=(
            const CanonicalCellView & view,
            const std::vector<CanonicalCell> & cells);
    bool operator!=(
            const std::vector<CanonicalCell> & cells,
            const CanonicalCellView & view);

    struct CellStoreResult {
        bool success = false;
        std::string diagnostic;
        CellStorageMetrics metrics;
    };

    class CellSnapshotStore
    {
    public:
        using ChunkVisitor = std::function<void(
                const ChunkCoordinate &,
                const std::vector<CanonicalCell> &)>;

        explicit CellSnapshotStore(CellStorageConfig config = {});

        CellStoreResult estimate_replace_upper_bound(std::size_t cell_count) const noexcept;
        CellStoreResult estimate_replace_upper_bound(
                std::size_t cell_count,
                std::size_t chunk_count) const noexcept;
        CellStoreResult estimate_apply_upper_bound(
                const std::vector<DeltaOperation> & operations) const noexcept;
        CellStoreResult replace(std::vector<CanonicalCell> cells);
        CellStoreResult apply(const std::vector<DeltaOperation> & operations);
        void clear();

        std::size_t size() const noexcept;
        CanonicalCellView view() const;
        // Read-only research seam for chunk-aware consumers. The visitor sees
        // deterministic signed chunk-coordinate order and never receives a
        // mutable bucket/chunk handle.
        void for_each_chunk(const ChunkVisitor & visitor) const;
        // Copies one committed chunk into caller-owned storage. A missing or
        // empty chunk returns false; the committed state is never exposed.
        bool copy_chunk(
                const ChunkCoordinate & coordinate,
                std::vector<CanonicalCell> & cells) const;
        const CellStorageMetrics & metrics() const noexcept;
        const void * chunk_identity(const VoxelIndex & index) const noexcept;
        CellStorageMode mode() const noexcept;

    private:
        CellStorageConfig config_;
        std::shared_ptr<const CanonicalCellView::State> state_;
        CellStorageMetrics metrics_;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_CELL_SNAPSHOT_STORE_HPP

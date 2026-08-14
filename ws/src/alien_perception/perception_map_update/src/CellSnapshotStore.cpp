#include "perception_map_update/CellSnapshotStore.hpp"

#include "perception_map_update/SpatialChunkLayout.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <new>
#include <queue>
#include <stdexcept>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        using CellChunk = std::vector<CanonicalCell>;

        struct ChunkEntry {
            ChunkCoordinate coordinate;
            std::shared_ptr<const CellChunk> chunk;
        };

        using ChunkBucket = std::vector<ChunkEntry>;

        bool entry_less(const ChunkEntry & left, const ChunkEntry & right)
        {
            return left.coordinate < right.coordinate;
        }

        CellState operation_state(DeltaOperationKind kind)
        {
            return kind == DeltaOperationKind::UpsertFree ? CellState::Free
                                                          : CellState::Occupied;
        }

        bool checked_add(std::size_t left, std::size_t right, std::size_t & result)
        {
            if(right > std::numeric_limits<std::size_t>::max() - left) {
                return false;
            }
            result = left + right;
            return true;
        }

        bool checked_bytes(std::size_t count, std::size_t size, std::size_t & result)
        {
            if(size != 0U && count > std::numeric_limits<std::size_t>::max() / size) {
                return false;
            }
            result = count * size;
            return true;
        }

        bool is_known_state(CellState state) noexcept
        {
            return state == CellState::Free || state == CellState::Occupied;
        }

        bool is_known_operation(DeltaOperationKind kind) noexcept
        {
            return kind == DeltaOperationKind::UpsertFree
                   || kind == DeltaOperationKind::UpsertOccupied
                   || kind == DeltaOperationKind::RemoveToUnknown;
        }

        bool validate_cells(
                const std::vector<CanonicalCell> & cells,
                std::string & diagnostic)
        {
            for(std::size_t index = 0U; index < cells.size(); ++index) {
                if(!is_known_state(cells[index].state)) {
                    diagnostic = "cell storage replacement has an invalid state";
                    return false;
                }
                if(index > 0U && !(cells[index - 1U].index < cells[index].index)) {
                    diagnostic = "cell storage replacement is not strictly ordered";
                    return false;
                }
            }
            return true;
        }

        bool validate_operations(
                const std::vector<DeltaOperation> & operations,
                std::string & diagnostic)
        {
            for(std::size_t index = 0U; index < operations.size(); ++index) {
                if(!is_known_operation(operations[index].kind)) {
                    diagnostic = "cell storage delta has an invalid operation";
                    return false;
                }
                if(index > 0U
                   && !(operations[index - 1U].index < operations[index].index)) {
                    diagnostic = "cell storage delta is not strictly ordered";
                    return false;
                }
            }
            return true;
        }

        struct LocatedOperation {
            ChunkCoordinate chunk;
            const DeltaOperation * operation = nullptr;
        };

        bool located_operation_less(
                const LocatedOperation & left,
                const LocatedOperation & right)
        {
            if(left.chunk != right.chunk) {
                return left.chunk < right.chunk;
            }
            return left.operation->index < right.operation->index;
        }

        struct DeltaPlan {
            bool success = false;
            std::vector<LocatedOperation> operations;
            std::string diagnostic;
        };

        DeltaPlan make_delta_plan(
                const std::vector<DeltaOperation> & operations,
                std::uint32_t chunk_edge)
        {
            DeltaPlan result;
            if(!validate_operations(operations, result.diagnostic)) {
                return result;
            }
            result.operations.reserve(operations.size());
            for(const auto & operation : operations) {
                const auto address = locate_chunk(operation.index, chunk_edge);
                if(!address) {
                    result.diagnostic = address.diagnostic;
                    return result;
                }
                result.operations.push_back({address.address.chunk, &operation});
            }
            std::sort(
                    result.operations.begin(),
                    result.operations.end(),
                    located_operation_less);
            result.success = true;
            return result;
        }

        struct MergeResult {
            bool success = false;
            std::vector<CanonicalCell> cells;
            std::string diagnostic;
        };

        template<typename OperationAt>
        MergeResult merge_cells_impl(
                const std::vector<CanonicalCell> & current,
                std::size_t operation_count,
                OperationAt operation_at)
        {
            MergeResult result;
            std::size_t reserve_count = 0U;
            if(!checked_add(current.size(), operation_count, reserve_count)) {
                result.diagnostic = "delta candidate size overflows";
                return result;
            }
            result.cells.reserve(reserve_count);
            std::size_t cell_index = 0U;
            std::size_t operation_index = 0U;
            while(cell_index < current.size() || operation_index < operation_count) {
                if(operation_index == operation_count) {
                    result.cells.insert(
                            result.cells.end(), current.begin() + cell_index, current.end());
                    break;
                }
                const auto & operation = operation_at(operation_index);
                if(cell_index == current.size()
                   || operation.index < current[cell_index].index) {
                    if(operation.kind == DeltaOperationKind::RemoveToUnknown) {
                        return {false, {}, "delta removes an unknown cell"};
                    }
                    result.cells.push_back({operation.index, operation_state(operation.kind)});
                    ++operation_index;
                    continue;
                }
                const auto & cell = current[cell_index];
                if(cell.index < operation.index) {
                    result.cells.push_back(cell);
                    ++cell_index;
                    continue;
                }
                if(operation.kind == DeltaOperationKind::RemoveToUnknown) {
                    ++cell_index;
                    ++operation_index;
                    continue;
                }
                const auto state = operation_state(operation.kind);
                if(state == cell.state) {
                    return {false, {}, "delta contains a redundant upsert"};
                }
                result.cells.push_back({operation.index, state});
                ++cell_index;
                ++operation_index;
            }
            result.success = true;
            return result;
        }

        MergeResult merge_cells(
                const std::vector<CanonicalCell> & current,
                const std::vector<DeltaOperation> & operations)
        {
            return merge_cells_impl(
                    current,
                    operations.size(),
                    [&operations](std::size_t index) -> const DeltaOperation & {
                        return operations[index];
                    });
        }

        MergeResult merge_cells(
                const std::vector<CanonicalCell> & current,
                const std::vector<LocatedOperation> & operations,
                std::size_t begin,
                std::size_t end)
        {
            return merge_cells_impl(
                    current,
                    end - begin,
                    [&operations, begin](std::size_t index) -> const DeltaOperation & {
                        return *operations[begin + index].operation;
                    });
        }

    }// namespace

    struct CanonicalCellView::State {
        CellStorageMode mode = CellStorageMode::Vector;
        std::vector<CanonicalCell> vector_cells;
        std::vector<std::shared_ptr<const ChunkBucket>> buckets;
        std::size_t cell_count = 0U;
        std::size_t chunk_count = 0U;
        std::uint32_t chunk_edge = 16U;
    };

    struct CanonicalCellCursor::State {
        struct ChunkSource {
            ChunkCoordinate coordinate;
            const CellChunk * chunk = nullptr;
        };

        struct ChunkCursor {
            const CellChunk * chunk = nullptr;
            std::size_t index = 0U;
        };

        struct Later {
            bool operator()(const ChunkCursor & left, const ChunkCursor & right) const
            {
                return (*right.chunk)[right.index].index
                       < (*left.chunk)[left.index].index;
            }
        };

        std::shared_ptr<const CanonicalCellView::State> snapshot;
        std::size_t vector_index = 0U;
        std::vector<ChunkSource> chunk_sources;
        std::size_t next_chunk_source = 0U;
        std::priority_queue<ChunkCursor, std::vector<ChunkCursor>, Later> chunks;

        void load_next_x_stripe()
        {
            if(!chunks.empty() || next_chunk_source >= chunk_sources.size()) {
                return;
            }
            const auto chunk_x = chunk_sources[next_chunk_source].coordinate.x;
            while(next_chunk_source < chunk_sources.size()
                  && chunk_sources[next_chunk_source].coordinate.x == chunk_x) {
                chunks.push({chunk_sources[next_chunk_source].chunk, 0U});
                ++next_chunk_source;
            }
        }
    };

    namespace {

        std::shared_ptr<const CanonicalCellView::State> empty_vector_state()
        {
            static const auto state = std::make_shared<const CanonicalCellView::State>();
            return state;
        }

        std::shared_ptr<const CanonicalCellView::State> vector_state(
                std::vector<CanonicalCell> cells)
        {
            auto state = std::make_shared<CanonicalCellView::State>();
            state->cell_count = cells.size();
            state->vector_cells = std::move(cells);
            return state;
        }

        const ChunkEntry * find_entry(
                const CanonicalCellView::State & state,
                const ChunkCoordinate & coordinate) noexcept
        {
            if(state.buckets.empty()) {
                return nullptr;
            }
            const auto & bucket = state.buckets[chunk_bucket_index(
                    coordinate, state.buckets.size())];
            if(!bucket) {
                return nullptr;
            }
            const ChunkEntry key {coordinate, {}};
            const auto found = std::lower_bound(
                    bucket->begin(), bucket->end(), key, entry_less);
            return found != bucket->end() && found->coordinate == coordinate
                           ? &*found
                           : nullptr;
        }

        bool traversal_scratch_upper_bound(
                std::size_t chunk_count,
                std::size_t & result) noexcept
        {
            return checked_bytes(
                    chunk_count,
                    sizeof(CanonicalCellCursor::State::ChunkSource)
                            + 2U * sizeof(CanonicalCellCursor::State::ChunkCursor),
                    result);
        }

        CellStorageMetrics state_metrics(const CanonicalCellView::State & state)
        {
            CellStorageMetrics metrics;
            if(state.mode == CellStorageMode::Vector) {
                metrics.committed_live_bytes =
                        state.vector_cells.capacity() * sizeof(CanonicalCell);
                return metrics;
            }
            metrics.total_chunks = state.chunk_count;
            metrics.committed_live_bytes =
                    state.buckets.capacity() * sizeof(std::shared_ptr<const ChunkBucket>);
            for(const auto & bucket : state.buckets) {
                if(!bucket) {
                    continue;
                }
                metrics.committed_live_bytes += bucket->capacity() * sizeof(ChunkEntry);
                for(const auto & entry : *bucket) {
                    metrics.committed_live_bytes +=
                            entry.chunk->capacity() * sizeof(CanonicalCell);
                }
            }
            if(!traversal_scratch_upper_bound(
                       state.chunk_count,
                       metrics.traversal_scratch_bytes)) {
                metrics.traversal_scratch_bytes =
                        std::numeric_limits<std::size_t>::max();
            }
            return metrics;
        }

    }// namespace

    CanonicalCellView::CanonicalCellView() : state_(empty_vector_state()) {}

    CanonicalCellView::CanonicalCellView(std::shared_ptr<const State> state)
        : state_(state ? std::move(state) : empty_vector_state())
    {
    }

    std::size_t CanonicalCellView::size() const noexcept
    {
        return state_->cell_count;
    }

    bool CanonicalCellView::empty() const noexcept
    {
        return size() == 0U;
    }

    const CanonicalCell & CanonicalCellView::front() const
    {
        if(empty()) {
            throw std::out_of_range("canonical cell view is empty");
        }
        if(state_->mode == CellStorageMode::Vector) {
            return state_->vector_cells.front();
        }
        const CanonicalCell * first = nullptr;
        for(const auto & bucket : state_->buckets) {
            if(!bucket) {
                continue;
            }
            for(const auto & entry : *bucket) {
                if(!entry.chunk->empty()
                   && (first == nullptr || entry.chunk->front().index < first->index)) {
                    first = &entry.chunk->front();
                }
            }
        }
        return *first;
    }

    CanonicalCellCursor CanonicalCellView::cursor() const
    {
        return CanonicalCellCursor(state_);
    }

    void CanonicalCellView::for_each(const Visitor & visitor) const
    {
        if(!visitor) {
            throw std::invalid_argument("canonical cell visitor must not be empty");
        }
        if(state_->mode == CellStorageMode::Vector) {
            for(const auto & cell : state_->vector_cells) {
                visitor(cell);
            }
            return;
        }
        auto cell = cursor();
        while(!cell.done()) {
            visitor(cell.value());
            cell.advance();
        }
    }

    std::vector<CanonicalCell> CanonicalCellView::materialize() const
    {
        if(state_->mode == CellStorageMode::Vector) {
            return state_->vector_cells;
        }
        std::vector<CanonicalCell> result;
        result.reserve(size());
        for_each([&result](const CanonicalCell & cell) { result.push_back(cell); });
        return result;
    }

    bool CanonicalCellView::equals(const std::vector<CanonicalCell> & cells) const
    {
        if(size() != cells.size()) {
            return false;
        }
        std::size_t index = 0U;
        bool equal = true;
        for_each([&](const CanonicalCell & cell) {
            if(equal && !(cell == cells[index])) {
                equal = false;
            }
            ++index;
        });
        return equal;
    }

    CanonicalCellView & CanonicalCellView::operator=(
            const std::vector<CanonicalCell> & cells)
    {
        state_ = vector_state(cells);
        return *this;
    }

    CanonicalCellCursor::CanonicalCellCursor(
            std::shared_ptr<const CanonicalCellView::State> snapshot)
        : state_(std::make_shared<State>())
    {
        state_->snapshot = snapshot ? std::move(snapshot) : empty_vector_state();
        if(state_->snapshot->mode == CellStorageMode::Chunked) {
            state_->chunk_sources.reserve(state_->snapshot->chunk_count);
            for(const auto & bucket : state_->snapshot->buckets) {
                if(!bucket) {
                    continue;
                }
                for(const auto & entry : *bucket) {
                    if(entry.chunk && !entry.chunk->empty()) {
                        state_->chunk_sources.push_back(
                                {entry.coordinate, entry.chunk.get()});
                    }
                }
            }
            std::sort(
                    state_->chunk_sources.begin(),
                    state_->chunk_sources.end(),
                    [](const State::ChunkSource & left, const State::ChunkSource & right) {
                        return left.coordinate < right.coordinate;
                    });
            state_->load_next_x_stripe();
        }
    }

    bool CanonicalCellCursor::done() const noexcept
    {
        return state_->snapshot->mode == CellStorageMode::Vector
                       ? state_->vector_index >= state_->snapshot->vector_cells.size()
                       : state_->chunks.empty();
    }

    const CanonicalCell & CanonicalCellCursor::value() const
    {
        if(done()) {
            throw std::out_of_range("canonical cell cursor is exhausted");
        }
        if(state_->snapshot->mode == CellStorageMode::Vector) {
            return state_->snapshot->vector_cells[state_->vector_index];
        }
        const auto & current = state_->chunks.top();
        return (*current.chunk)[current.index];
    }

    void CanonicalCellCursor::advance()
    {
        if(done()) {
            throw std::out_of_range("canonical cell cursor is exhausted");
        }
        if(state_->snapshot->mode == CellStorageMode::Vector) {
            ++state_->vector_index;
            return;
        }
        auto current = state_->chunks.top();
        state_->chunks.pop();
        ++current.index;
        if(current.index < current.chunk->size()) {
            state_->chunks.push(current);
        }
        state_->load_next_x_stripe();
    }

    bool operator==(
            const CanonicalCellView & left,
            const CanonicalCellView & right)
    {
        if(left.size() != right.size()) {
            return false;
        }
        auto left_cursor = left.cursor();
        auto right_cursor = right.cursor();
        while(!left_cursor.done()) {
            if(!(left_cursor.value() == right_cursor.value())) {
                return false;
            }
            left_cursor.advance();
            right_cursor.advance();
        }
        return right_cursor.done();
    }

    bool operator!=(
            const CanonicalCellView & left,
            const CanonicalCellView & right)
    {
        return !(left == right);
    }

    CanonicalCellView & CanonicalCellView::operator=(
            std::vector<CanonicalCell> && cells)
    {
        state_ = vector_state(std::move(cells));
        return *this;
    }

    CanonicalCellView & CanonicalCellView::operator=(
            std::initializer_list<CanonicalCell> cells)
    {
        state_ = vector_state(std::vector<CanonicalCell>(cells));
        return *this;
    }

    bool operator==(
            const CanonicalCellView & view,
            const std::vector<CanonicalCell> & cells)
    {
        return view.equals(cells);
    }

    bool operator==(
            const std::vector<CanonicalCell> & cells,
            const CanonicalCellView & view)
    {
        return view.equals(cells);
    }

    bool operator!=(
            const CanonicalCellView & view,
            const std::vector<CanonicalCell> & cells)
    {
        return !view.equals(cells);
    }

    bool operator!=(
            const std::vector<CanonicalCell> & cells,
            const CanonicalCellView & view)
    {
        return !view.equals(cells);
    }

    CellSnapshotStore::CellSnapshotStore(CellStorageConfig config)
        : config_(config)
        , state_(empty_vector_state())
    {
        if(config_.mode != CellStorageMode::Vector
           && config_.mode != CellStorageMode::Chunked) {
            throw std::invalid_argument("cell storage mode is unknown");
        }
        if(config_.chunk_edge == 0U || config_.bucket_count == 0U) {
            throw std::invalid_argument("cell storage chunk edge and bucket count must be positive");
        }
        if(config_.mode == CellStorageMode::Chunked) {
            auto state = std::make_shared<CanonicalCellView::State>();
            state->mode = CellStorageMode::Chunked;
            state->chunk_edge = config_.chunk_edge;
            state->buckets.resize(config_.bucket_count);
            state_ = std::move(state);
        }
        metrics_ = state_metrics(*state_);
    }

    CellStoreResult CellSnapshotStore::estimate_replace_upper_bound(
            std::size_t cell_count) const noexcept
    {
        CellStorageMetrics metrics;
        metrics.committed_live_bytes = metrics_.committed_live_bytes;
        if(config_.mode == CellStorageMode::Vector) {
            if(!checked_bytes(
                       cell_count, sizeof(CanonicalCell), metrics.candidate_owned_bytes)) {
                return {false, "cell storage replacement estimate overflow", {}};
            }
            return {true, {}, metrics};
        }

        std::size_t directory = 0U;
        std::size_t mutable_directory = 0U;
        std::size_t bucket_entries = 0U;
        std::size_t decoded_cells = 0U;
        std::size_t chunk_cells = 0U;
        std::size_t grouping_records = 0U;
        std::size_t ownership_records = 0U;
        std::size_t scratch = 0U;
        std::size_t doubled_cells = 0U;
        std::size_t grouping_record_size = 0U;
        if(!checked_add(cell_count, cell_count, doubled_cells)
           || !checked_bytes(
                   state_->buckets.capacity(),
                   sizeof(std::shared_ptr<const ChunkBucket>),
                   directory)
           || !checked_bytes(
                   config_.bucket_count,
                   sizeof(std::shared_ptr<ChunkBucket>),
                   mutable_directory)
           || !checked_bytes(doubled_cells, sizeof(ChunkEntry), bucket_entries)
           || !checked_bytes(cell_count, sizeof(CanonicalCell), decoded_cells)
           || !checked_bytes(cell_count, sizeof(CanonicalCell), chunk_cells)
           || !checked_add(
                   sizeof(std::pair<const ChunkCoordinate, CellChunk>),
                   4U * sizeof(void *),
                   grouping_record_size)
           || !checked_bytes(cell_count, grouping_record_size, grouping_records)
           || !checked_bytes(
                   doubled_cells,
                   2U * sizeof(void *),
                   ownership_records)
           || !traversal_scratch_upper_bound(cell_count, scratch)
           || !checked_add(directory, mutable_directory, metrics.candidate_owned_bytes)
           || !checked_add(
                   metrics.candidate_owned_bytes,
                   bucket_entries,
                   metrics.candidate_owned_bytes)
           || !checked_add(
                   metrics.candidate_owned_bytes,
                   decoded_cells,
                   metrics.candidate_owned_bytes)
           || !checked_add(
                   metrics.candidate_owned_bytes,
                   chunk_cells,
                   metrics.candidate_owned_bytes)
           || !checked_add(
                   metrics.candidate_owned_bytes,
                   grouping_records,
                   metrics.candidate_owned_bytes)
           || !checked_add(
                   metrics.candidate_owned_bytes,
                   ownership_records,
                   metrics.candidate_owned_bytes)) {
            return {false, "cell storage replacement estimate overflow", {}};
        }
        metrics.total_chunks = cell_count;
        metrics.traversal_scratch_bytes = scratch;
        return {true, {}, metrics};
    }

    CellStoreResult CellSnapshotStore::estimate_apply_upper_bound(
            const std::vector<DeltaOperation> & operations) const noexcept
    {
        if(operations.empty()) {
            auto metrics = state_metrics(*state_);
            metrics.shared_chunks = state_->chunk_count;
            metrics.candidate_owned_bytes = 0U;
            return {true, {}, metrics};
        }
        if(config_.mode == CellStorageMode::Vector) {
            CellStorageMetrics metrics;
            metrics.committed_live_bytes = metrics_.committed_live_bytes;
            metrics.copied_cells = state_->cell_count;
            std::string diagnostic;
            std::size_t candidate_capacity = 0U;
            if(!validate_operations(operations, diagnostic)) {
                return {false, std::move(diagnostic), {}};
            }
            if(!checked_add(state_->cell_count, operations.size(), candidate_capacity)
               || !checked_bytes(
                       candidate_capacity,
                       sizeof(CanonicalCell),
                       metrics.candidate_owned_bytes)) {
                return {false, "cell storage delta estimate overflow", {}};
            }
            return {true, {}, metrics};
        }

        try {
            const auto plan = make_delta_plan(operations, config_.chunk_edge);
            if(!plan.success) {
                return {false, plan.diagnostic, {}};
            }

            CellStorageMetrics metrics;
            metrics.committed_live_bytes = metrics_.committed_live_bytes;
            std::vector<std::size_t> new_chunks_per_bucket(
                    config_.bucket_count, 0U);
            std::vector<bool> touched_buckets(config_.bucket_count, false);
            std::size_t touched_existing_chunks = 0U;
            std::size_t new_chunks = 0U;
            std::size_t chunk_cells_upper = 0U;
            std::size_t begin = 0U;
            while(begin < plan.operations.size()) {
                std::size_t end = begin + 1U;
                while(end < plan.operations.size()
                      && plan.operations[end].chunk == plan.operations[begin].chunk) {
                    ++end;
                }
                ++metrics.touched_chunks;
                const auto bucket_index = chunk_bucket_index(
                        plan.operations[begin].chunk, config_.bucket_count);
                touched_buckets[bucket_index] = true;
                const auto * entry = find_entry(*state_, plan.operations[begin].chunk);
                const std::size_t current_cells = entry == nullptr
                                                          ? 0U
                                                          : entry->chunk->size();
                if(entry == nullptr) {
                    ++new_chunks_per_bucket[bucket_index];
                    ++new_chunks;
                } else {
                    ++touched_existing_chunks;
                    if(!checked_add(
                               metrics.copied_cells,
                               current_cells,
                               metrics.copied_cells)) {
                        return {false, "cell storage delta estimate overflow", {}};
                    }
                }
                std::size_t merged_capacity = 0U;
                std::size_t merged_bytes = 0U;
                if(!checked_add(current_cells, end - begin, merged_capacity)
                   || !checked_bytes(
                           merged_capacity, sizeof(CanonicalCell), merged_bytes)
                   || !checked_add(
                           chunk_cells_upper, merged_bytes, chunk_cells_upper)) {
                    return {false, "cell storage delta estimate overflow", {}};
                }
                begin = end;
            }

            std::size_t candidate_chunk_upper = 0U;
            std::size_t directory_bytes = 0U;
            std::size_t mutable_directory_bytes = 0U;
            std::size_t plan_bytes = 0U;
            std::size_t bucket_plan_bytes = 0U;
            std::size_t bucket_entries_upper = 0U;
            if(!checked_add(state_->chunk_count, new_chunks, candidate_chunk_upper)
               || !checked_bytes(
                       state_->buckets.capacity(),
                       sizeof(std::shared_ptr<const ChunkBucket>),
                       directory_bytes)
               || !checked_bytes(
                       config_.bucket_count,
                       sizeof(std::shared_ptr<ChunkBucket>),
                       mutable_directory_bytes)
               || !checked_bytes(
                       plan.operations.capacity(), sizeof(LocatedOperation), plan_bytes)
               || !checked_bytes(
                       config_.bucket_count,
                       sizeof(std::size_t) + sizeof(bool),
                       bucket_plan_bytes)) {
                return {false, "cell storage delta estimate overflow", {}};
            }

            for(std::size_t index = 0U; index < config_.bucket_count; ++index) {
                if(!touched_buckets[index]) {
                    continue;
                }
                const std::size_t original_entries = state_->buckets[index]
                                                             ? state_->buckets[index]->size()
                                                             : 0U;
                if(!checked_add(
                           metrics.copied_bucket_entries,
                           original_entries,
                           metrics.copied_bucket_entries)) {
                    return {false, "cell storage delta estimate overflow", {}};
                }
                std::size_t max_entries = 0U;
                std::size_t capacity_upper = 0U;
                std::size_t entry_bytes = 0U;
                if(!checked_add(
                           original_entries,
                           new_chunks_per_bucket[index],
                           max_entries)
                   || !checked_add(max_entries, max_entries, capacity_upper)
                   || !checked_bytes(capacity_upper, sizeof(ChunkEntry), entry_bytes)
                   || !checked_add(
                           bucket_entries_upper,
                           entry_bytes,
                           bucket_entries_upper)) {
                    return {false, "cell storage delta estimate overflow", {}};
                }
            }

            metrics.total_chunks = candidate_chunk_upper;
            metrics.shared_chunks = state_->chunk_count - touched_existing_chunks;
            if(!traversal_scratch_upper_bound(
                       candidate_chunk_upper,
                       metrics.traversal_scratch_bytes)
               || !checked_add(
                       directory_bytes,
                       mutable_directory_bytes,
                       metrics.candidate_owned_bytes)
               || !checked_add(
                       metrics.candidate_owned_bytes,
                       plan_bytes,
                       metrics.candidate_owned_bytes)
               || !checked_add(
                       metrics.candidate_owned_bytes,
                       bucket_plan_bytes,
                       metrics.candidate_owned_bytes)
               || !checked_add(
                       metrics.candidate_owned_bytes,
                       bucket_entries_upper,
                       metrics.candidate_owned_bytes)
               || !checked_add(
                       metrics.candidate_owned_bytes,
                       chunk_cells_upper,
                       metrics.candidate_owned_bytes)) {
                return {false, "cell storage delta estimate overflow", {}};
            }
            return {true, {}, metrics};
        }
        catch(const std::bad_alloc &) {
            return {false, "cell storage delta estimate allocation failed", {}};
        }
    }

    CellStoreResult CellSnapshotStore::replace(std::vector<CanonicalCell> cells)
    {
        std::string validation_diagnostic;
        if(!validate_cells(cells, validation_diagnostic)) {
            return {false, std::move(validation_diagnostic), {}};
        }
        try {
            if(config_.mode == CellStorageMode::Vector) {
                state_ = vector_state(std::move(cells));
                metrics_ = state_metrics(*state_);
                metrics_.candidate_owned_bytes = metrics_.committed_live_bytes;
                return {true, {}, metrics_};
            }
            auto candidate = std::make_shared<CanonicalCellView::State>();
            candidate->mode = CellStorageMode::Chunked;
            candidate->chunk_edge = config_.chunk_edge;
            candidate->cell_count = cells.size();
            candidate->buckets.resize(config_.bucket_count);
            std::map<ChunkCoordinate, CellChunk> chunks;
            for(auto & cell : cells) {
                const auto address = locate_chunk(cell.index, config_.chunk_edge);
                if(!address) {
                    return {false, address.diagnostic, {}};
                }
                chunks[address.address.chunk].push_back(std::move(cell));
            }
            std::vector<std::shared_ptr<ChunkBucket>> mutable_buckets(config_.bucket_count);
            for(auto & pair : chunks) {
                const auto bucket_index = chunk_bucket_index(
                        pair.first, config_.bucket_count);
                if(!mutable_buckets[bucket_index]) {
                    mutable_buckets[bucket_index] = std::make_shared<ChunkBucket>();
                }
                mutable_buckets[bucket_index]->push_back({
                        pair.first,
                        std::make_shared<const CellChunk>(std::move(pair.second))});
            }
            for(std::size_t index = 0U; index < mutable_buckets.size(); ++index) {
                if(mutable_buckets[index]) {
                    std::sort(
                            mutable_buckets[index]->begin(),
                            mutable_buckets[index]->end(),
                            entry_less);
                    candidate->buckets[index] = mutable_buckets[index];
                }
            }
            candidate->chunk_count = chunks.size();
            state_ = std::move(candidate);
            metrics_ = state_metrics(*state_);
            metrics_.candidate_owned_bytes = metrics_.committed_live_bytes;
            return {true, {}, metrics_};
        }
        catch(const std::bad_alloc &) {
            return {false, "cell storage replacement allocation failed", {}};
        }
    }

    CellStoreResult CellSnapshotStore::apply(
            const std::vector<DeltaOperation> & operations)
    {
        std::string validation_diagnostic;
        if(!validate_operations(operations, validation_diagnostic)) {
            return {false, std::move(validation_diagnostic), {}};
        }
        if(operations.empty()) {
            metrics_ = state_metrics(*state_);
            metrics_.shared_chunks = state_->chunk_count;
            return {true, {}, metrics_};
        }
        try {
            if(config_.mode == CellStorageMode::Vector) {
                auto merged = merge_cells(state_->vector_cells, operations);
                if(!merged.success) {
                    return {false, merged.diagnostic, {}};
                }
                auto candidate = vector_state(std::move(merged.cells));
                auto metrics = state_metrics(*state_);
                metrics.copied_cells = state_->cell_count;
                metrics.candidate_owned_bytes =
                        candidate->vector_cells.capacity() * sizeof(CanonicalCell);
                state_ = std::move(candidate);
                metrics.committed_live_bytes = state_metrics(*state_).committed_live_bytes;
                metrics_ = metrics;
                return {true, {}, metrics_};
            }

            const auto plan = make_delta_plan(operations, config_.chunk_edge);
            if(!plan.success) {
                return {false, plan.diagnostic, {}};
            }
            auto candidate = std::make_shared<CanonicalCellView::State>(*state_);
            std::vector<std::shared_ptr<ChunkBucket>> copied_buckets(
                    config_.bucket_count);
            CellStorageMetrics metrics = state_metrics(*state_);
            metrics.copied_cells = 0U;
            metrics.copied_bucket_entries = 0U;
            metrics.candidate_owned_bytes =
                    candidate->buckets.capacity()
                            * sizeof(std::shared_ptr<const ChunkBucket>)
                    + copied_buckets.capacity()
                              * sizeof(std::shared_ptr<ChunkBucket>)
                    + plan.operations.capacity() * sizeof(LocatedOperation);

            std::size_t candidate_count = state_->cell_count;
            std::size_t candidate_chunks = state_->chunk_count;
            std::size_t touched_existing_chunks = 0U;
            std::size_t begin = 0U;
            while(begin < plan.operations.size()) {
                std::size_t end = begin + 1U;
                while(end < plan.operations.size()
                      && plan.operations[end].chunk == plan.operations[begin].chunk) {
                    ++end;
                }
                ++metrics.touched_chunks;
                const auto bucket_index = chunk_bucket_index(
                        plan.operations[begin].chunk, candidate->buckets.size());
                auto bucket = copied_buckets[bucket_index];
                if(!bucket) {
                    const auto & original = state_->buckets[bucket_index];
                    bucket = original ? std::make_shared<ChunkBucket>(*original)
                                      : std::make_shared<ChunkBucket>();
                    copied_buckets[bucket_index] = bucket;
                    candidate->buckets[bucket_index] = bucket;
                    metrics.copied_bucket_entries += bucket->size();
                }
                const ChunkEntry key {plan.operations[begin].chunk, {}};
                auto found = std::lower_bound(
                        bucket->begin(), bucket->end(), key, entry_less);
                const bool existed = found != bucket->end()
                                     && found->coordinate == plan.operations[begin].chunk;
                const CellChunk empty;
                const auto & current = existed ? *found->chunk : empty;
                auto merged = merge_cells(current, plan.operations, begin, end);
                if(!merged.success) {
                    return {false, merged.diagnostic, {}};
                }
                if(existed) {
                    ++touched_existing_chunks;
                    metrics.copied_cells += current.size();
                    candidate_count -= current.size();
                }
                candidate_count += merged.cells.size();
                if(merged.cells.empty()) {
                    if(existed) {
                        bucket->erase(found);
                        --candidate_chunks;
                    }
                } else {
                    metrics.candidate_owned_bytes +=
                            merged.cells.capacity() * sizeof(CanonicalCell);
                    auto replacement = std::make_shared<const CellChunk>(
                            std::move(merged.cells));
                    if(existed) {
                        found->chunk = std::move(replacement);
                    } else {
                        bucket->insert(
                                found,
                                {plan.operations[begin].chunk, std::move(replacement)});
                        ++candidate_chunks;
                    }
                }
                begin = end;
            }
            candidate->cell_count = candidate_count;
            candidate->chunk_count = candidate_chunks;
            metrics.total_chunks = candidate_chunks;
            metrics.shared_chunks = state_->chunk_count - touched_existing_chunks;
            for(const auto & copied : copied_buckets) {
                if(copied) {
                    metrics.candidate_owned_bytes +=
                            copied->capacity() * sizeof(ChunkEntry);
                }
            }
            if(!traversal_scratch_upper_bound(
                       candidate_chunks,
                       metrics.traversal_scratch_bytes)) {
                return {false, "cell storage traversal estimate overflow", {}};
            }
            state_ = std::move(candidate);
            metrics.committed_live_bytes = state_metrics(*state_).committed_live_bytes;
            metrics_ = metrics;
            return {true, {}, metrics_};
        }
        catch(const std::bad_alloc &) {
            return {false, "cell storage delta allocation failed", {}};
        }
    }

    void CellSnapshotStore::clear()
    {
        if(config_.mode == CellStorageMode::Vector) {
            state_ = empty_vector_state();
        } else {
            auto state = std::make_shared<CanonicalCellView::State>();
            state->mode = CellStorageMode::Chunked;
            state->chunk_edge = config_.chunk_edge;
            state->buckets.resize(config_.bucket_count);
            state_ = std::move(state);
        }
        metrics_ = state_metrics(*state_);
    }

    std::size_t CellSnapshotStore::size() const noexcept
    {
        return state_->cell_count;
    }

    CanonicalCellView CellSnapshotStore::view() const
    {
        return CanonicalCellView(state_);
    }

    const CellStorageMetrics & CellSnapshotStore::metrics() const noexcept
    {
        return metrics_;
    }

    const void * CellSnapshotStore::chunk_identity(const VoxelIndex & index) const noexcept
    {
        if(state_->mode != CellStorageMode::Chunked) {
            return nullptr;
        }
        const auto address = locate_chunk(index, state_->chunk_edge);
        if(!address) {
            return nullptr;
        }
        const auto * entry = find_entry(*state_, address.address.chunk);
        return entry == nullptr ? nullptr : entry->chunk.get();
    }

    CellStorageMode CellSnapshotStore::mode() const noexcept
    {
        return config_.mode;
    }

}// namespace PerceptionMapUpdate

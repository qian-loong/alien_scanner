#include "swarm_controller/OctoMapMerger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    namespace {

        using SteadyClock = std::chrono::steady_clock;

        double elapsedSeconds(
                const SteadyClock::time_point start,
                const SteadyClock::time_point finish)
        {
            return std::chrono::duration<double>(finish - start).count();
        }

        bool sameKey(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs)
        {
            return lhs == rhs;
        }

    }// namespace

    OctoMapMerger::OctoMapMerger(const OctoMapMergerConfig & config)
        : config_(config),
          tree_(std::isfinite(config.resolution) && config.resolution > 0.0
                        ? config.resolution
                        : 0.1)
    {
        if(!std::isfinite(config_.resolution) || config_.resolution <= 0.0) {
            throw std::invalid_argument("OctoMapMerger: resolution must be positive and finite");
        }
        if(config_.max_voxels_per_source == 0U) {
            throw std::invalid_argument("OctoMapMerger: max_voxels_per_source must be positive");
        }
        if(config_.max_global_voxels == 0U) {
            throw std::invalid_argument("OctoMapMerger: max_global_voxels must be positive");
        }
    }

    SourceUpdateResult OctoMapMerger::updateSource(
            const std::string & source_id, const octomap::OcTree & source)
    {
        if(source_id.empty()) {
            return invalidResult(
                    "source_id must not be empty", SourceUpdateFailureStage::InvalidInput);
        }

        SourceSnapshot snapshot;
        std::string                 reason;
        SourceUpdateFailureStage    failure_stage = SourceUpdateFailureStage::None;
        SourceUpdateStageTiming     timing;
        const auto normalize_start = SteadyClock::now();
        try {
            if(!normalizeSource(source, snapshot, reason, failure_stage)) {
                timing.normalize_seconds = elapsedSeconds(
                        normalize_start, SteadyClock::now());
                return invalidResult(reason, failure_stage, timing);
            }
        } catch(const std::bad_alloc &) {
            timing.normalize_seconds = elapsedSeconds(
                    normalize_start, SteadyClock::now());
            return invalidResult(
                    "source normalization allocation failed",
                    SourceUpdateFailureStage::Resource, timing);
        }
        timing.normalize_seconds = elapsedSeconds(normalize_start, SteadyClock::now());
        return replaceSource(source_id, std::move(snapshot), false, timing);
    }

    SourceUpdateResult OctoMapMerger::removeSource(const std::string & source_id)
    {
        if(source_id.empty()) {
            return invalidResult(
                    "source_id must not be empty", SourceUpdateFailureStage::InvalidInput);
        }
        if(sources_.find(source_id) == sources_.end()) {
            SourceUpdateResult result;
            result.status          = SourceUpdateStatus::AcceptedUnchanged;
            result.source_revision = source_revision_;
            result.global_revision = global_revision_;
            return result;
        }
        return replaceSource(source_id, {}, true);
    }

    const octomap::OcTree & OctoMapMerger::tree() const
    {
        return tree_;
    }

    std::uint64_t OctoMapMerger::sourceRevision() const
    {
        return source_revision_;
    }

    std::uint64_t OctoMapMerger::globalRevision() const
    {
        return global_revision_;
    }

    std::size_t OctoMapMerger::sourceCount() const
    {
        return sources_.size();
    }

    std::size_t OctoMapMerger::sourceVoxelCount(const std::string & source_id) const
    {
        const auto it = sources_.find(source_id);
        return it == sources_.end() ? 0U : it->second.size();
    }

    std::size_t OctoMapMerger::totalSourceVoxelCount() const
    {
        return total_source_voxels_;
    }

    std::size_t OctoMapMerger::knownCount() const
    {
        return contributions_.size();
    }

    std::size_t OctoMapMerger::freeCount() const
    {
        return contributions_.size() - occupied_count_;
    }

    std::size_t OctoMapMerger::occupiedCount() const
    {
        return occupied_count_;
    }

    SourceUpdateResult OctoMapMerger::invalidResult(
            const std::string & reason, const SourceUpdateFailureStage failure_stage,
            const SourceUpdateStageTiming & timing) const
    {
        SourceUpdateResult result;
        result.status          = SourceUpdateStatus::Invalid;
        result.failure_stage   = failure_stage;
        result.source_revision = source_revision_;
        result.global_revision = global_revision_;
        result.timing           = timing;
        result.reason          = reason;
        return result;
    }

    bool OctoMapMerger::normalizeSource(
            const octomap::OcTree & source, SourceSnapshot & snapshot,
            std::string & reason, SourceUpdateFailureStage & failure_stage) const
    {
        failure_stage = SourceUpdateFailureStage::Normalize;
        const double tolerance = 1.0e-6
                                 * std::max(
                                         {1.0, std::abs(config_.resolution),
                                          std::abs(source.getResolution())});
        if(!std::isfinite(source.getResolution())
           || std::abs(source.getResolution() - config_.resolution) > tolerance)
        {
            reason = "source resolution does not match merger resolution";
            return false;
        }

        std::uint64_t expanded_count = 0U;
        for(auto it = source.begin_leafs(), end = source.end_leafs(); it != end; ++it) {
            if(it.getDepth() > source.getTreeDepth()) {
                reason = "source leaf depth exceeds tree depth";
                return false;
            }
            const unsigned level = source.getTreeDepth() - it.getDepth();
            if(level >= std::numeric_limits<std::uint64_t>::digits) {
                reason = "source leaf expansion width overflows";
                failure_stage = SourceUpdateFailureStage::Resource;
                return false;
            }

            const std::uint64_t width = std::uint64_t {1U} << level;
            std::uint64_t       square = 0U;
            std::uint64_t       cube   = 0U;
            std::uint64_t       next   = 0U;
            if(!checkedMultiply(width, width, square)
               || !checkedMultiply(square, width, cube)
               || !checkedAdd(expanded_count, cube, next))
            {
                reason = "source leaf expansion count overflows";
                failure_stage = SourceUpdateFailureStage::Resource;
                return false;
            }
            if(next > config_.max_voxels_per_source) {
                reason = "source snapshot exceeds max_voxels_per_source";
                failure_stage = SourceUpdateFailureStage::Resource;
                return false;
            }
            expanded_count = next;
        }

        snapshot.clear();
        snapshot.reserve(static_cast<std::size_t>(expanded_count));
        for(auto it = source.begin_leafs(), end = source.end_leafs(); it != end; ++it) {
            const unsigned              level = source.getTreeDepth() - it.getDepth();
            const std::uint64_t         width = std::uint64_t {1U} << level;
            const octomap::OcTreeKey    base  = it.getIndexKey();
            const VoxelState state = source.isNodeOccupied(*it)
                                             ? VoxelState::Occupied
                                             : VoxelState::Free;

            for(std::uint64_t x = 0U; x < width; ++x) {
                for(std::uint64_t y = 0U; y < width; ++y) {
                    for(std::uint64_t z = 0U; z < width; ++z) {
                        snapshot.push_back(VoxelRecord {
                                octomap::OcTreeKey {
                                        static_cast<octomap::key_type>(base[0] + x),
                                        static_cast<octomap::key_type>(base[1] + y),
                                        static_cast<octomap::key_type>(base[2] + z),
                                },
                                state,
                        });
                    }
                }
            }
        }

        std::sort(snapshot.begin(), snapshot.end(), [](const VoxelRecord & lhs, const VoxelRecord & rhs) {
            return keyLess(lhs.key, rhs.key);
        });

        SourceSnapshot unique;
        unique.reserve(snapshot.size());
        for(const VoxelRecord & record : snapshot) {
            if(unique.empty() || !sameKey(unique.back().key, record.key)) {
                unique.push_back(record);
            } else if(record.state == VoxelState::Occupied) {
                unique.back().state = VoxelState::Occupied;
            }
        }
        snapshot.swap(unique);
        return true;
    }

    SourceUpdateResult OctoMapMerger::replaceSource(
            const std::string & source_id, SourceSnapshot snapshot,
            const bool remove_entry, SourceUpdateStageTiming timing)
    {
        const auto old_it = sources_.find(source_id);
        const SourceSnapshot * old_snapshot = old_it == sources_.end() ? nullptr : &old_it->second;

        const auto compare_start = SteadyClock::now();
        const bool unchanged = !remove_entry && old_snapshot != nullptr
                               && recordsEqual(*old_snapshot, snapshot);
        timing.snapshot_compare_seconds = elapsedSeconds(
                compare_start, SteadyClock::now());
        if(unchanged) {
            SourceUpdateResult result;
            result.status          = SourceUpdateStatus::AcceptedUnchanged;
            result.source_revision = source_revision_;
            result.global_revision = global_revision_;
            result.timing           = timing;
            return result;
        }

        const auto preflight_start = SteadyClock::now();
        const auto reject_preflight = [this, &timing, preflight_start](
                                              const std::string & reason,
                                              const SourceUpdateFailureStage stage) {
            timing.delta_preflight_seconds = elapsedSeconds(
                    preflight_start, SteadyClock::now());
            return invalidResult(reason, stage, timing);
        };

        const std::size_t old_size = old_snapshot == nullptr ? 0U : old_snapshot->size();
        if(total_source_voxels_ < old_size
           || snapshot.size() > std::numeric_limits<std::size_t>::max()
                                      - (total_source_voxels_ - old_size))
        {
            return reject_preflight(
                    "total source voxel count overflows",
                    SourceUpdateFailureStage::Resource);
        }
        if(snapshot.size() > std::numeric_limits<std::size_t>::max() - old_size) {
            return reject_preflight(
                    "source delta capacity overflows",
                    SourceUpdateFailureStage::Resource);
        }
        const std::size_t projected_source_voxels =
                total_source_voxels_ - old_size + snapshot.size();

        std::vector<PendingDelta> deltas;
        try {
            deltas.reserve(old_size + snapshot.size());
        } catch(const std::bad_alloc &) {
            return reject_preflight(
                    "source delta allocation failed",
                    SourceUpdateFailureStage::Resource);
        }
        SourceUpdateResult result;
        result.status = SourceUpdateStatus::AcceptedChanged;

        const SourceSnapshot empty;
        const SourceSnapshot & old_records = old_snapshot == nullptr ? empty : *old_snapshot;
        std::size_t old_index = 0U;
        std::size_t new_index = 0U;
        std::size_t projected_global_voxels = contributions_.size();

        while(old_index < old_records.size() || new_index < snapshot.size()) {
            const bool has_old = old_index < old_records.size();
            const bool has_new = new_index < snapshot.size();
            const VoxelRecord * old_record = has_old ? &old_records[old_index] : nullptr;
            const VoxelRecord * new_record = has_new ? &snapshot[new_index] : nullptr;

            bool take_old = false;
            bool take_new = false;
            if(has_old && has_new && sameKey(old_record->key, new_record->key)) {
                if(old_record->state == new_record->state) {
                    ++old_index;
                    ++new_index;
                    continue;
                }
                take_old = true;
                take_new = true;
                ++result.flipped_keys;
            } else if(has_old && (!has_new || keyLess(old_record->key, new_record->key))) {
                take_old = true;
                ++result.removed_keys;
            } else {
                take_new = true;
                ++result.added_keys;
            }

            const octomap::OcTreeKey & key = take_old ? old_record->key : new_record->key;
            const auto contribution_it = contributions_.find(key);
            ContributionCounts old_counts;
            if(contribution_it != contributions_.end()) {
                old_counts = contribution_it->second;
            }
            ContributionCounts new_counts = old_counts;

            if(take_old) {
                std::uint32_t & count = old_record->state == VoxelState::Occupied
                                                ? new_counts.occupied_sources
                                                : new_counts.free_sources;
                if(count == 0U) {
                    return reject_preflight(
                            "internal source contribution underflow",
                            SourceUpdateFailureStage::DeltaPreflight);
                }
                --count;
            }
            if(take_new) {
                std::uint32_t & count = new_record->state == VoxelState::Occupied
                                                ? new_counts.occupied_sources
                                                : new_counts.free_sources;
                if(count == std::numeric_limits<std::uint32_t>::max()) {
                    return reject_preflight(
                            "source contribution count overflows",
                            SourceUpdateFailureStage::DeltaPreflight);
                }
                ++count;
            }

            const DerivedState old_global = derivedState(old_counts);
            const DerivedState new_global = derivedState(new_counts);
            if(old_global == DerivedState::Unknown && new_global != DerivedState::Unknown) {
                if(projected_global_voxels == std::numeric_limits<std::size_t>::max()) {
                    return reject_preflight(
                            "global voxel count overflows",
                            SourceUpdateFailureStage::Resource);
                }
                ++projected_global_voxels;
            } else if(old_global != DerivedState::Unknown
                      && new_global == DerivedState::Unknown)
            {
                if(projected_global_voxels == 0U) {
                    return reject_preflight(
                            "internal global voxel count underflow",
                            SourceUpdateFailureStage::DeltaPreflight);
                }
                --projected_global_voxels;
            }

            deltas.push_back(PendingDelta {key, new_counts, old_global, new_global});
            if(take_old) {
                ++old_index;
            }
            if(take_new) {
                ++new_index;
            }
        }

        if(projected_global_voxels > config_.max_global_voxels) {
            return reject_preflight(
                    "merged snapshot exceeds max_global_voxels",
                    SourceUpdateFailureStage::Resource);
        }

        std::size_t global_changed_keys = 0U;
        std::size_t projected_occupied_count = occupied_count_;
        std::size_t applied_global_voxels = contributions_.size();
        std::size_t peak_global_voxels = applied_global_voxels;
        for(const PendingDelta & delta : deltas) {
            if(delta.old_global == delta.new_global) {
                continue;
            }
            ++global_changed_keys;
            if(delta.old_global == DerivedState::Unknown) {
                ++applied_global_voxels;
                peak_global_voxels = std::max(
                        peak_global_voxels, applied_global_voxels);
            } else if(delta.new_global == DerivedState::Unknown) {
                --applied_global_voxels;
            }
            if(delta.old_global == DerivedState::Occupied) {
                if(projected_occupied_count == 0U) {
                    return reject_preflight(
                            "internal occupied voxel count underflow",
                            SourceUpdateFailureStage::DeltaPreflight);
                }
                --projected_occupied_count;
            }
            if(delta.new_global == DerivedState::Occupied) {
                if(projected_occupied_count
                   == std::numeric_limits<std::size_t>::max())
                {
                    return reject_preflight(
                            "occupied voxel count overflows",
                            SourceUpdateFailureStage::Resource);
                }
                ++projected_occupied_count;
            }
        }
        if(applied_global_voxels != projected_global_voxels) {
            return reject_preflight(
                    "internal projected global voxel count mismatch",
                    SourceUpdateFailureStage::DeltaPreflight);
        }

        ContributionMap prepared_new_contributions;
        SourceMap prepared_new_source;
        const bool inserting_new_source =
                !remove_entry && old_it == sources_.end();
        try {
            if(peak_global_voxels > contributions_.size()) {
                contributions_.reserve(peak_global_voxels);
            }
            std::size_t new_contribution_count = 0U;
            for(const PendingDelta & delta : deltas) {
                if(delta.old_global == DerivedState::Unknown
                   && delta.new_global != DerivedState::Unknown)
                {
                    ++new_contribution_count;
                }
            }
            prepared_new_contributions.reserve(new_contribution_count);
            for(const PendingDelta & delta : deltas) {
                if(delta.old_global == DerivedState::Unknown
                   && delta.new_global != DerivedState::Unknown)
                {
                    prepared_new_contributions.emplace(delta.key, delta.counts);
                }
            }

            if(inserting_new_source) {
                sources_.reserve(sources_.size() + 1U);
                prepared_new_source.reserve(1U);
                prepared_new_source.emplace(source_id, std::move(snapshot));
            }
        } catch(const std::bad_alloc &) {
            return reject_preflight(
                    "merger commit preparation allocation failed",
                    SourceUpdateFailureStage::Resource);
        }
        timing.delta_preflight_seconds = elapsedSeconds(
                preflight_start, SteadyClock::now());

        if(config_.commit_failure_hook) {
            const auto hook_start = SteadyClock::now();
            try {
                config_.commit_failure_hook();
            } catch(...) {
                timing.source_commit_seconds = elapsedSeconds(
                        hook_start, SteadyClock::now());
                return invalidResult(
                        "source commit hook failed",
                        SourceUpdateFailureStage::Commit, timing);
            }
            timing.source_commit_seconds = elapsedSeconds(
                    hook_start, SteadyClock::now());
        }

        const auto apply_start = SteadyClock::now();
        // Apply known states before removals so replacing the only key never
        // expands an empty OcTree root carrying the deleted key's old value.
        for(const PendingDelta & delta : deltas) {
            if(delta.old_global == delta.new_global
               || delta.new_global == DerivedState::Unknown)
            {
                continue;
            }
            if(delta.new_global == DerivedState::Free) {
                tree_.setNodeValue(
                        delta.key, tree_.getClampingThresMinLog(), true);
            } else {
                tree_.setNodeValue(
                        delta.key, tree_.getClampingThresMaxLog(), true);
            }
        }
        for(const PendingDelta & delta : deltas) {
            if(delta.old_global == delta.new_global
               || delta.new_global != DerivedState::Unknown)
            {
                continue;
            }
            tree_.deleteNode(delta.key);
        }

        for(const PendingDelta & delta : deltas) {
            if(delta.new_global == DerivedState::Unknown) {
                contributions_.erase(delta.key);
            } else if(delta.old_global == DerivedState::Unknown) {
                auto prepared = prepared_new_contributions.extract(delta.key);
                if(prepared.empty()) {
                    throw std::logic_error(
                            "prepared contribution missing during commit");
                }
                const auto inserted = contributions_.insert(std::move(prepared));
                if(!inserted.inserted) {
                    throw std::logic_error(
                            "prepared contribution already exists during commit");
                }
            } else {
                const auto existing = contributions_.find(delta.key);
                if(existing == contributions_.end()) {
                    throw std::logic_error(
                            "existing contribution missing during commit");
                }
                existing->second = delta.counts;
            }
        }
        timing.contribution_tree_apply_seconds = elapsedSeconds(
                apply_start, SteadyClock::now());

        const auto inner_start = SteadyClock::now();
        if(global_changed_keys > 0U) {
            if(projected_global_voxels == 0U) {
                tree_.clear();
            } else {
                tree_.updateInnerOccupancy();
            }
            ++global_revision_;
            result.global_changed = true;
        }
        timing.update_inner_occupancy_seconds = elapsedSeconds(
                inner_start, SteadyClock::now());

        const auto commit_start = SteadyClock::now();
        if(remove_entry) {
            sources_.erase(source_id);
        } else if(inserting_new_source) {
            auto prepared = prepared_new_source.extract(source_id);
            if(prepared.empty()) {
                throw std::logic_error("prepared source missing during commit");
            }
            const auto inserted = sources_.insert(std::move(prepared));
            if(!inserted.inserted) {
                throw std::logic_error("prepared source already exists during commit");
            }
        } else {
            sources_.find(source_id)->second = std::move(snapshot);
        }
        occupied_count_ = projected_occupied_count;
        total_source_voxels_ = projected_source_voxels;
        ++source_revision_;
        result.source_revision = source_revision_;
        result.global_revision = global_revision_;
        timing.source_commit_seconds += elapsedSeconds(
                commit_start, SteadyClock::now());
        result.timing = timing;
        return result;
    }

    bool OctoMapMerger::keyLess(
            const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs)
    {
        if(lhs[0] != rhs[0]) {
            return lhs[0] < rhs[0];
        }
        if(lhs[1] != rhs[1]) {
            return lhs[1] < rhs[1];
        }
        return lhs[2] < rhs[2];
    }

    bool OctoMapMerger::recordsEqual(
            const SourceSnapshot & lhs, const SourceSnapshot & rhs)
    {
        if(lhs.size() != rhs.size()) {
            return false;
        }
        for(std::size_t i = 0U; i < lhs.size(); ++i) {
            if(!sameKey(lhs[i].key, rhs[i].key) || lhs[i].state != rhs[i].state) {
                return false;
            }
        }
        return true;
    }

    OctoMapMerger::DerivedState OctoMapMerger::derivedState(
            const ContributionCounts & counts)
    {
        if(counts.occupied_sources > 0U) {
            return DerivedState::Occupied;
        }
        if(counts.free_sources > 0U) {
            return DerivedState::Free;
        }
        return DerivedState::Unknown;
    }

    bool OctoMapMerger::checkedAdd(
            const std::uint64_t lhs, const std::uint64_t rhs, std::uint64_t & result)
    {
        if(rhs > std::numeric_limits<std::uint64_t>::max() - lhs) {
            return false;
        }
        result = lhs + rhs;
        return true;
    }

    bool OctoMapMerger::checkedMultiply(
            const std::uint64_t lhs, const std::uint64_t rhs, std::uint64_t & result)
    {
        if(lhs != 0U && rhs > std::numeric_limits<std::uint64_t>::max() / lhs) {
            return false;
        }
        result = lhs * rhs;
        return true;
    }

}// namespace SwarmController

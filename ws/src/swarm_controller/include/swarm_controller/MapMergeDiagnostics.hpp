#ifndef SWARM_CONTROLLER_MAPMERGEDIAGNOSTICS_HPP
#define SWARM_CONTROLLER_MAPMERGEDIAGNOSTICS_HPP

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>

namespace SwarmController {

    struct MergeHeaderAge {
        bool   valid {false};
        double seconds {-1.0};
    };

    inline MergeHeaderAge mergeHeaderAge(
            const std::int64_t stamp_ns, const std::int64_t now_ns,
            const std::uint64_t stamp_epoch, const std::uint64_t now_epoch)
    {
        if(stamp_ns <= 0 || now_ns < stamp_ns || stamp_epoch != now_epoch) {
            return {};
        }
        return MergeHeaderAge {
                true, static_cast<double>(now_ns - stamp_ns) * 1.0e-9};
    }

    inline double mergeElapsedSeconds(const double start, const double finish)
    {
        if(!std::isfinite(start) || !std::isfinite(finish)
           || start < 0.0 || finish < start)
        {
            return -1.0;
        }
        return finish - start;
    }

    struct MergeCycleStageDurations {
        double claim_seconds {};
        double decode_seconds {};
        double decode_cleanup_seconds {};
        double normalize_seconds {};
        double snapshot_compare_seconds {};
        double delta_preflight_seconds {};
        double contribution_tree_apply_seconds {};
        double update_inner_occupancy_seconds {};
        double source_commit_seconds {};
        double bookkeeping_seconds {};
        double serialize_seconds {};
        double output_prepare_seconds {};
        double publish_seconds {};
        double cycle_total_seconds {};
        double accounting_remainder_seconds {};

        double accountedSeconds() const
        {
            return claim_seconds + decode_seconds + decode_cleanup_seconds
                   + normalize_seconds
                   + snapshot_compare_seconds + delta_preflight_seconds
                   + contribution_tree_apply_seconds
                   + update_inner_occupancy_seconds + source_commit_seconds
                   + bookkeeping_seconds + serialize_seconds
                   + output_prepare_seconds + publish_seconds;
        }

        void finishAccounting(const double total_seconds)
        {
            cycle_total_seconds = total_seconds;
            accounting_remainder_seconds = std::max(
                    0.0, cycle_total_seconds - accountedSeconds());
        }
    };

    struct MergeCycleCompletion {
        bool        merge_applied {false};
        bool        published {false};
        const char * outcome {"all_sources_rejected"};
    };

    inline MergeCycleCompletion completeMergeCycle(
            const std::size_t accepted_sources, const std::size_t rejected_sources,
            const bool serialization_succeeded, const bool publish_succeeded)
    {
        if(accepted_sources == 0U) {
            return {};
        }
        if(!serialization_succeeded) {
            return MergeCycleCompletion {true, false, "serialization_failure"};
        }
        if(!publish_succeeded) {
            return MergeCycleCompletion {true, false, "publish_failure"};
        }
        return MergeCycleCompletion {
                true, true,
                rejected_sources > 0U ? "partial_accept" : "accepted"};
    }

}// namespace SwarmController

#endif// SWARM_CONTROLLER_MAPMERGEDIAGNOSTICS_HPP

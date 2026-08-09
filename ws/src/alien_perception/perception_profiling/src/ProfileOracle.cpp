#include "perception_profiling/ProfileOracle.hpp"

#include "perception_profiling/ProfileMapHarness.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace PerceptionProfiling {
    namespace {

        using PerceptionLocalMap::AcquireStatus;
        using PerceptionLocalMap::CommitReceipt;
        using PerceptionLocalMap::InputStatus;
        using PerceptionLocalMap::LocalObservationMapper;
        using PerceptionLocalMap::OccupancyState;

        struct Census {
            VoxelCounts counts;
            std::optional<PerceptionLocalMap::AxisAlignedBounds> bounds;
        };

        struct MilestoneBracket {
            std::uint64_t threshold = 0U;
            std::uint64_t lower_revision = 0U;
            std::uint64_t upper_revision = 0U;
        };

        bool same_point(
                const PerceptionLocalMap::MapPoint & left,
                const PerceptionLocalMap::MapPoint & right) noexcept
        {
            constexpr double kTolerance = 1e-12;
            return std::abs(left.x - right.x) <= kTolerance
                   && std::abs(left.y - right.y) <= kTolerance
                   && std::abs(left.z - right.z) <= kTolerance;
        }

        bool same_bounds(
                const std::optional<PerceptionLocalMap::AxisAlignedBounds> & left,
                const std::optional<PerceptionLocalMap::AxisAlignedBounds> & right) noexcept
        {
            if(left.has_value() != right.has_value()) {
                return false;
            }
            return !left.has_value()
                   || (same_point(left->minimum, right->minimum)
                       && same_point(left->maximum, right->maximum));
        }

        Census census(
                LocalObservationMapper & mapper,
                const CommitReceipt & receipt)
        {
            auto acquired = mapper.acquire_read_transaction(receipt);
            if(acquired.status != AcquireStatus::Ready
               || !acquired.transaction.has_value()) {
                throw std::runtime_error("oracle could not acquire its exact revision");
            }
            const auto metadata = acquired.transaction->metadata();
            if(!metadata.has_value()) {
                throw std::runtime_error("oracle exact revision has no metadata");
            }
            Census result;
            result.bounds = metadata->known_bounds;
            const bool visited = acquired.transaction->for_each_known_cell(
                    [&result](PerceptionLocalMap::OccupancyCell cell) {
                        ++result.counts.known;
                        if(cell.state == OccupancyState::Free) {
                            ++result.counts.free;
                        }
                        else if(cell.state == OccupancyState::Occupied) {
                            ++result.counts.occupied;
                        }
                    });
            acquired.transaction->close();
            if(!visited || result.counts.known != result.counts.free + result.counts.occupied) {
                throw std::runtime_error("oracle exact census is inconsistent");
            }
            return result;
        }

        OracleCheckpoint make_checkpoint(
                const ProfileScenario & scenario,
                const ScenarioSample & sample,
                const CommitReceipt & receipt,
                const Census & exact)
        {
            return {
                    sample.sequence,
                    sample.observation.origin_stamp.nanoseconds,
                    sample.payload_digest,
                    receipt.map_epoch,
                    receipt.revision,
                    exact.counts,
                    exact.bounds,
                    scenario.contract_fingerprint().hex_digest};
        }

        void count_status(OracleRun & run, InputStatus status)
        {
            switch(status) {
                case InputStatus::Accepted:
                    ++run.accepted;
                    break;
                case InputStatus::Applied:
                    ++run.applied;
                    break;
                case InputStatus::NoEvidence:
                    ++run.no_evidence;
                    break;
                case InputStatus::Rejected:
                    ++run.rejected;
                    break;
                case InputStatus::Unavailable:
                    ++run.unavailable;
                    break;
                case InputStatus::BackendFault:
                    ++run.backend_fault;
                    break;
            }
        }

        std::vector<MilestoneBracket> milestone_brackets(
                const std::vector<OracleCheckpoint> & checkpoints,
                const std::vector<std::uint64_t> & thresholds)
        {
            std::vector<MilestoneBracket> result;
            for(const auto threshold : thresholds) {
                std::uint64_t lower = 0U;
                for(const auto & checkpoint : checkpoints) {
                    if(checkpoint.counts.known > threshold) {
                        result.push_back({threshold, lower, checkpoint.revision});
                        break;
                    }
                    lower = checkpoint.revision;
                }
            }
            return result;
        }

        std::vector<MilestoneCrossing> refine_milestones(
                const ProfileScenario & scenario,
                std::uint64_t sequence_count,
                const std::vector<MilestoneBracket> & brackets)
        {
            if(brackets.empty()) {
                return {};
            }
            std::map<std::uint64_t, MilestoneCrossing> crossings;
            const auto last_revision = std::max_element(
                    brackets.begin(), brackets.end(),
                    [](const auto & left, const auto & right) {
                        return left.upper_revision < right.upper_revision;
                    })->upper_revision;
            ProfileMapHarness harness(scenario);
            for(std::uint64_t sequence = 1U; sequence <= sequence_count; ++sequence) {
                const auto sample = scenario.sample(sequence);
                const auto result = harness.submit(sample);
                if(result.status != InputStatus::Applied || !result.receipt.has_value()) {
                    continue;
                }
                const auto revision = result.receipt->revision;
                bool needs_census = false;
                for(const auto & bracket : brackets) {
                    needs_census = needs_census
                                   || (crossings.count(bracket.threshold) == 0U
                                       && revision > bracket.lower_revision
                                       && revision <= bracket.upper_revision);
                }
                if(needs_census) {
                    const auto exact = census(harness.mapper(), result.receipt.value());
                    for(const auto & bracket : brackets) {
                        if(crossings.count(bracket.threshold) == 0U
                           && revision > bracket.lower_revision
                           && revision <= bracket.upper_revision
                           && exact.counts.known > bracket.threshold) {
                            crossings.emplace(
                                    bracket.threshold,
                                    MilestoneCrossing {
                                            bracket.threshold,
                                            sequence,
                                            sample.observation.origin_stamp.nanoseconds,
                                            revision,
                                            exact.counts.known});
                        }
                    }
                }
                if(revision >= last_revision) {
                    break;
                }
            }

            std::vector<MilestoneCrossing> result;
            result.reserve(crossings.size());
            for(auto & entry : crossings) {
                result.push_back(std::move(entry.second));
            }
            return result;
        }

    }// namespace

    bool VoxelCounts::operator==(const VoxelCounts & other) const noexcept
    {
        return known == other.known && free == other.free && occupied == other.occupied;
    }

    bool VoxelCounts::operator!=(const VoxelCounts & other) const noexcept
    {
        return !(*this == other);
    }

    std::uint64_t RevisionBucket::sample_count() const noexcept
    {
        return last >= first ? last - first + 1U : 0U;
    }

    ProfileOracle::ProfileOracle(ProfileScenario scenario)
        : scenario_(std::move(scenario))
    {}

    OracleRun ProfileOracle::run(const OracleOptions & options) const
    {
        if(options.sequence_count == 0U
           || options.checkpoint_period_revisions == 0U
           || options.bounded_plateau_revisions == 0U
           || options.capacity_max_revision == 0U) {
            throw std::invalid_argument("oracle options must be positive");
        }
        if(scenario_.canonical_sequence_limit().has_value()
           && options.sequence_count > scenario_.canonical_sequence_limit().value()) {
            throw std::invalid_argument("oracle sequence count exceeds canonical scenario");
        }

        OracleRun run;
        run.mode = scenario_.mode();
        ProfileMapHarness harness(scenario_);
        std::optional<Census> plateau_value;
        std::uint64_t plateau_candidate_revision = 0U;
        std::optional<OracleCheckpoint> final_checkpoint;

        for(std::uint64_t sequence = 1U; sequence <= options.sequence_count; ++sequence) {
            const auto sample = scenario_.sample(sequence);
            const auto result = harness.submit(sample);
            count_status(run, result.status);
            if(result.status != InputStatus::Applied || !result.receipt.has_value()) {
                continue;
            }

            run.final_revision = result.receipt->revision;
            run.final_map_epoch = result.receipt->map_epoch;
            const bool bounded_census = scenario_.mode() == ScenarioMode::Bounded;
            const bool periodic_checkpoint =
                    result.receipt->revision % options.checkpoint_period_revisions == 0U;
            if(bounded_census || periodic_checkpoint || sequence == options.sequence_count) {
                const auto exact = census(harness.mapper(), result.receipt.value());
                if(bounded_census) {
                    if(!plateau_value.has_value()
                       || plateau_value->counts != exact.counts
                       || !same_bounds(plateau_value->bounds, exact.bounds)) {
                        plateau_value = exact;
                        plateau_candidate_revision = result.receipt->revision;
                    }
                    else if(!run.plateau_start_revision.has_value()
                            && result.receipt->revision - plateau_candidate_revision
                                       >= options.bounded_plateau_revisions) {
                        run.plateau_start_revision = plateau_candidate_revision;
                    }
                }
                const auto checkpoint = make_checkpoint(
                        scenario_, sample, result.receipt.value(), exact);
                final_checkpoint = checkpoint;
                if(periodic_checkpoint) {
                    run.checkpoints.push_back(checkpoint);
                }
            }
        }

        if(final_checkpoint.has_value()
           && (run.checkpoints.empty()
               || run.checkpoints.back().revision != final_checkpoint->revision)) {
            run.checkpoints.push_back(final_checkpoint.value());
        }

        auto thresholds = options.milestone_thresholds;
        thresholds.push_back(500'000U);
        std::sort(thresholds.begin(), thresholds.end());
        thresholds.erase(std::unique(thresholds.begin(), thresholds.end()), thresholds.end());
        const auto brackets = milestone_brackets(run.checkpoints, thresholds);
        const auto refined = refine_milestones(
                scenario_, options.sequence_count, brackets);
        for(const auto & crossing : refined) {
            if(std::find(
                       options.milestone_thresholds.begin(),
                       options.milestone_thresholds.end(), crossing.threshold)
               != options.milestone_thresholds.end()) {
                run.milestones.push_back(crossing);
            }
        }
        const auto crossing_500k = std::find_if(
                refined.begin(), refined.end(),
                [](const auto & crossing) { return crossing.threshold == 500'000U; });
        run.capacity = classify_capacity(
                crossing_500k == refined.end()
                        ? std::optional<std::uint64_t> {}
                        : std::optional<std::uint64_t> {crossing_500k->revision},
                options.capacity_max_revision);
        return run;
    }

    CapacityPlan ProfileOracle::classify_capacity(
            std::optional<std::uint64_t> crossing_revision,
            std::uint64_t maximum_revision)
    {
        CapacityPlan plan;
        plan.crossing_revision = crossing_revision;
        if(!crossing_revision.has_value()) {
            plan.status = CapacityStatus::NotReached;
            return plan;
        }
        if(crossing_revision.value() > std::numeric_limits<std::uint64_t>::max() - 300U) {
            plan.status = CapacityStatus::CrossingWithoutPostWindow;
            return plan;
        }
        plan.required_end_revision = crossing_revision.value() + 300U;
        if(crossing_revision.value() < 300U
           || plan.required_end_revision.value() > maximum_revision) {
            plan.status = CapacityStatus::CrossingWithoutPostWindow;
            return plan;
        }
        plan.status = CapacityStatus::Covered;
        plan.pre = RevisionBucket {
                crossing_revision.value() - 299U,
                crossing_revision.value() - 100U};
        plan.crossing = RevisionBucket {
                crossing_revision.value() - 99U,
                crossing_revision.value() + 100U};
        plan.post = RevisionBucket {
                crossing_revision.value() + 101U,
                crossing_revision.value() + 300U};
        return plan;
    }

    JoinResult ProfileOracle::validate_join(
            const ProductionStateProjection & production,
            const OracleCheckpoint & oracle)
    {
        if(production.map_epoch != oracle.map_epoch) {
            return {false, "map epoch mismatch"};
        }
        if(production.revision != oracle.revision) {
            return {false, "revision mismatch"};
        }
        if(production.observation_stamp_ns != oracle.observation_stamp_ns) {
            return {false, "last observation stamp mismatch"};
        }
        if(production.contract_fingerprint != oracle.contract_fingerprint) {
            return {false, "contract fingerprint mismatch"};
        }
        if(!same_bounds(production.bounds, oracle.bounds)) {
            return {false, "known bounds mismatch"};
        }
        return {true, {}};
    }

    std::string capacity_status_name(CapacityStatus status)
    {
        switch(status) {
            case CapacityStatus::Covered:
                return "covered";
            case CapacityStatus::NotReached:
                return "not_reached";
            case CapacityStatus::CrossingWithoutPostWindow:
                return "crossing_without_post_window";
        }
        throw std::invalid_argument("unknown capacity status");
    }

}// namespace PerceptionProfiling

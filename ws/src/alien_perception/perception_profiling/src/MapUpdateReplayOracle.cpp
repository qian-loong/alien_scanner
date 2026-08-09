#include "perception_profiling/MapUpdateReplayOracle.hpp"

#include "perception_profiling/ProfileMapHarness.hpp"

#include "perception_local_map/CanonicalSnapshotAdapter.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"

#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace PerceptionProfiling {
    namespace {

        void count_input_status(
                MapUpdateReplayRun & run,
                PerceptionLocalMap::InputStatus status)
        {
            switch(status) {
                case PerceptionLocalMap::InputStatus::Accepted:
                    ++run.accepted_input_count;
                    break;
                case PerceptionLocalMap::InputStatus::Applied:
                    ++run.applied_input_count;
                    break;
                case PerceptionLocalMap::InputStatus::NoEvidence:
                    ++run.no_evidence_input_count;
                    break;
                case PerceptionLocalMap::InputStatus::Rejected:
                    ++run.rejected_input_count;
                    break;
                case PerceptionLocalMap::InputStatus::Unavailable:
                    ++run.unavailable_input_count;
                    break;
                case PerceptionLocalMap::InputStatus::BackendFault:
                    ++run.backend_fault_input_count;
                    break;
            }
        }

        bool accepted_apply_status(PerceptionMapUpdate::ApplyUpdateStatus status) noexcept
        {
            return status == PerceptionMapUpdate::ApplyUpdateStatus::AppliedKeyframe
                   || status == PerceptionMapUpdate::ApplyUpdateStatus::AppliedDelta;
        }

        std::string comparison_diagnostic(
                bool source_matches,
                bool geometry_matches,
                bool revision_matches,
                bool hash_matches,
                std::uint64_t missing,
                std::uint64_t unexpected,
                std::uint64_t state_mismatches)
        {
            if(!source_matches) {
                return "source identity mismatch";
            }
            if(!geometry_matches) {
                return "map geometry mismatch";
            }
            if(!revision_matches) {
                return "revision mismatch";
            }
            if(!hash_matches) {
                return "content hash mismatch";
            }
            if(missing != 0U || unexpected != 0U || state_mismatches != 0U) {
                std::ostringstream stream;
                stream << "cell mismatch: missing=" << missing
                       << ", unexpected=" << unexpected
                       << ", state=" << state_mismatches;
                return stream.str();
            }
            return {};
        }

    }// namespace

    bool MapUpdateReplayRun::all_checkpoints_match() const noexcept
    {
        if(checkpoints.empty() || !final_oracle.has_value()
           || !final_reconstructed.has_value() || !final_comparison.matches
           || rejected_input_count != 0U || backend_fault_input_count != 0U) {
            return false;
        }
        for(const auto & checkpoint : checkpoints) {
            if(!checkpoint.comparison.matches
               || !accepted_apply_status(checkpoint.apply_status)) {
                return false;
            }
        }
        return true;
    }

    MapUpdateReplayOracle::MapUpdateReplayOracle(ProfileScenario scenario)
        : scenario_(std::move(scenario))
    {
    }

    MapUpdateReplayRun MapUpdateReplayOracle::run(
            const MapUpdateReplayOptions & options) const
    {
        if(options.sequence_count == 0U) {
            throw std::invalid_argument("replay sequence count must be positive");
        }
        if(scenario_.canonical_sequence_limit().has_value()
           && options.sequence_count > scenario_.canonical_sequence_limit().value()) {
            throw std::invalid_argument("replay sequence count exceeds canonical scenario");
        }

        MapUpdateReplayRun run;
        run.mode = scenario_.mode();
        ProfileMapHarness harness(scenario_);
        PerceptionLocalMap::CanonicalSnapshotAdapter adapter(options.limits);
        PerceptionMapUpdate::MapUpdateProducer producer(options.limits);
        PerceptionMapUpdate::MapUpdateApplier applier(options.limits);
        std::shared_ptr<const PerceptionMapUpdate::CanonicalSnapshot> final_oracle;
        std::optional<PerceptionMapUpdate::CanonicalSnapshot> first_acceptance;
        std::optional<PerceptionMapUpdate::CanonicalSnapshot> middle_acceptance;
        std::optional<PerceptionMapUpdate::CanonicalSnapshot> previous_acceptance;
        std::optional<PerceptionMapUpdate::CanonicalSnapshot> final_acceptance;
        const std::uint64_t middle_sequence = (options.sequence_count + 1U) / 2U;

        for(std::uint64_t sequence = 1U; sequence <= options.sequence_count; ++sequence) {
            const auto sample = scenario_.sample(sequence);
            const auto input = harness.submit(sample);
            count_input_status(run, input.status);
            if(input.status != PerceptionLocalMap::InputStatus::Applied
               || !input.receipt.has_value()) {
                continue;
            }

            auto acquired = harness.mapper().acquire_read_transaction(*input.receipt);
            if(acquired.status != PerceptionLocalMap::AcquireStatus::Ready
               || !acquired.transaction.has_value()) {
                throw std::runtime_error(
                        "replay could not acquire the committed exact revision");
            }
            auto materialized = adapter.materialize(*acquired.transaction);
            acquired.transaction->close();
            if(materialized.status != PerceptionLocalMap::CanonicalSnapshotStatus::Ready
               || !materialized.snapshot.has_value()) {
                throw std::runtime_error(
                        "replay canonical snapshot failed: " + materialized.diagnostic);
            }

            auto target = std::make_shared<const PerceptionMapUpdate::CanonicalSnapshot>(
                    std::move(*materialized.snapshot));
            auto prepared = producer.prepare(target);
            if(!prepared.update.has_value()
               || (prepared.status != PerceptionMapUpdate::ProduceStatus::ProducedKeyframe
                   && prepared.status
                              != PerceptionMapUpdate::ProduceStatus::ProducedDelta)) {
                throw std::runtime_error(
                        "replay producer rejected exact snapshot: " + prepared.diagnostic);
            }
            if(!applier.expected_source().has_value()
               && !applier.admit_source(target->source)) {
                throw std::runtime_error("replay receiver rejected source admission");
            }
            if(!producer.commit_published(prepared)) {
                throw std::runtime_error("replay producer could not commit publication");
            }

            const auto apply = applier.apply(*prepared.update);
            ReplayCheckpoint checkpoint;
            checkpoint.sequence = sequence;
            checkpoint.observation_stamp_ns = sample.observation.origin_stamp.nanoseconds;
            checkpoint.revision = target->revision;
            checkpoint.update_kind = prepared.update->kind;
            checkpoint.apply_status = apply.status;
            checkpoint.source_cell_count = target->cells.size();
            checkpoint.operation_count = prepared.update->operation_count;
            checkpoint.payload_bytes = prepared.update->payload.size();
            checkpoint.content_hash = target->content_hash;
            checkpoint.update_hash = prepared.update->update_hash;
            if(applier.reconstructed_map().has_value()) {
                checkpoint.reconstructed_cell_count =
                        applier.reconstructed_map()->cells.size();
                checkpoint.comparison = compare(
                        *target,
                        *applier.reconstructed_map(),
                        options.max_difference_samples);
            } else {
                checkpoint.comparison.diagnostic =
                        "receiver has no reconstructed map after update";
            }
            if(!accepted_apply_status(apply.status)) {
                checkpoint.comparison.matches = false;
                checkpoint.comparison.diagnostic =
                        "update apply failed: " + apply.diagnostic;
            }
            if(prepared.update->kind == PerceptionMapUpdate::UpdateKind::Keyframe) {
                ++run.keyframe_count;
            } else {
                ++run.delta_count;
            }
            run.checkpoints.push_back(std::move(checkpoint));
            if(!first_acceptance.has_value()) {
                first_acceptance = *target;
            }
            if(sequence >= middle_sequence
               && !middle_acceptance.has_value()
               && target->revision > first_acceptance->revision) {
                middle_acceptance = *target;
            }
            previous_acceptance = std::move(final_acceptance);
            final_acceptance = *target;
            final_oracle = std::move(target);
        }

        if(first_acceptance.has_value() && final_acceptance.has_value()) {
            if(!middle_acceptance.has_value()
               || middle_acceptance->revision >= final_acceptance->revision) {
                middle_acceptance = previous_acceptance;
            }
            if(middle_acceptance.has_value()
               && first_acceptance->revision < middle_acceptance->revision
               && middle_acceptance->revision < final_acceptance->revision) {
                run.acceptance_snapshots = {
                        std::move(*first_acceptance),
                        std::move(*middle_acceptance),
                        std::move(*final_acceptance)};
            }
        }

        if(final_oracle) {
            run.final_oracle = *final_oracle;
        }
        if(applier.reconstructed_map().has_value()) {
            run.final_reconstructed = *applier.reconstructed_map();
        }
        if(run.final_oracle.has_value() && run.final_reconstructed.has_value()) {
            run.final_comparison = compare(
                    *run.final_oracle,
                    *run.final_reconstructed,
                    options.max_difference_samples);
        } else {
            run.final_comparison.diagnostic = "replay produced no final map";
        }
        return run;
    }

    ReplayComparison MapUpdateReplayOracle::compare(
            const PerceptionMapUpdate::CanonicalSnapshot & oracle,
            const PerceptionMapUpdate::ReconstructedMap & reconstructed,
            std::size_t max_difference_samples)
    {
        ReplayComparison comparison;
        const bool source_matches = oracle.source == reconstructed.source;
        const bool geometry_matches = oracle.geometry == reconstructed.geometry;
        const bool revision_matches = oracle.revision == reconstructed.revision;
        const bool hash_matches = oracle.content_hash == reconstructed.content_hash;

        std::size_t oracle_index = 0U;
        std::size_t reconstructed_index = 0U;
        while(oracle_index < oracle.cells.size()
              || reconstructed_index < reconstructed.cells.size()) {
            if(reconstructed_index == reconstructed.cells.size()
               || (oracle_index < oracle.cells.size()
                   && oracle.cells[oracle_index].index
                              < reconstructed.cells[reconstructed_index].index)) {
                ++comparison.missing_cell_count;
                if(comparison.missing_cell_samples.size() < max_difference_samples) {
                    comparison.missing_cell_samples.push_back(oracle.cells[oracle_index]);
                }
                ++oracle_index;
                continue;
            }
            if(oracle_index == oracle.cells.size()
               || reconstructed.cells[reconstructed_index].index
                          < oracle.cells[oracle_index].index) {
                ++comparison.unexpected_cell_count;
                if(comparison.unexpected_cell_samples.size() < max_difference_samples) {
                    comparison.unexpected_cell_samples.push_back(
                            reconstructed.cells[reconstructed_index]);
                }
                ++reconstructed_index;
                continue;
            }
            if(oracle.cells[oracle_index].state
               != reconstructed.cells[reconstructed_index].state) {
                ++comparison.state_mismatch_count;
                if(comparison.state_mismatch_samples.size() < max_difference_samples) {
                    comparison.state_mismatch_samples.push_back(
                            oracle.cells[oracle_index].index);
                }
            }
            ++oracle_index;
            ++reconstructed_index;
        }

        comparison.diagnostic = comparison_diagnostic(
                source_matches,
                geometry_matches,
                revision_matches,
                hash_matches,
                comparison.missing_cell_count,
                comparison.unexpected_cell_count,
                comparison.state_mismatch_count);
        comparison.matches = comparison.diagnostic.empty();
        return comparison;
    }

}// namespace PerceptionProfiling

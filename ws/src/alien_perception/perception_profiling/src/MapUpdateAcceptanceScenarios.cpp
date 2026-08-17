#include "perception_profiling/MapUpdateAcceptanceScenarios.hpp"

#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace PerceptionProfiling {
    namespace {

        using PerceptionMapUpdate::ApplyUpdateResult;
        using PerceptionMapUpdate::ApplyUpdateStatus;
        using PerceptionMapUpdate::CanonicalSnapshot;
        using PerceptionMapUpdate::MapUpdateApplier;
        using PerceptionMapUpdate::MapUpdateProducer;
        using PerceptionMapUpdate::PreparedUpdate;
        using PerceptionMapUpdate::ReceiverState;
        using PerceptionMapUpdate::ReconstructedMap;
        using PerceptionMapUpdate::UpdateKind;

        void validate_snapshots(const std::vector<CanonicalSnapshot> & snapshots)
        {
            if(snapshots.size() < 3U) {
                throw std::invalid_argument(
                        "map-update acceptance scenarios require three snapshots");
            }
            for(std::size_t index = 1U; index < 3U; ++index) {
                if(snapshots[index].source != snapshots.front().source
                   || !(snapshots[index].geometry == snapshots.front().geometry)
                   || snapshots[index].geometry_fingerprint
                              != snapshots.front().geometry_fingerprint
                   || snapshots[index].revision <= snapshots[index - 1U].revision) {
                    throw std::invalid_argument(
                            "acceptance snapshots do not form one advancing source chain");
                }
            }
        }

        PreparedUpdate produce_and_commit(
                MapUpdateProducer & producer,
                const CanonicalSnapshot & snapshot)
        {
            auto prepared = producer.prepare(
                    std::make_shared<const CanonicalSnapshot>(snapshot));
            if(!prepared.update.has_value()) {
                throw std::runtime_error(
                        "acceptance producer did not create an update: "
                        + prepared.diagnostic);
            }
            if(!producer.commit_published(prepared)) {
                throw std::runtime_error(
                        "acceptance producer could not commit its update");
            }
            return prepared;
        }

        MapUpdateAcceptanceStage stage(
                MapUpdateAcceptanceStageKind kind,
                const MapUpdateApplier & receiver,
                std::uint64_t base_revision,
                std::uint64_t new_revision,
                std::optional<ApplyUpdateResult> apply = std::nullopt,
                std::string correlation_id = {},
                std::string diagnostic = {})
        {
            MapUpdateAcceptanceStage result;
            result.kind = kind;
            result.receiver_state = receiver.state();
            result.attempted_base_revision = base_revision;
            result.attempted_new_revision = new_revision;
            result.correlation_id = std::move(correlation_id);
            if(apply.has_value()) {
                result.apply_status = apply->status;
                if(diagnostic.empty()) {
                    diagnostic = apply->diagnostic;
                }
            }
            result.diagnostic = std::move(diagnostic);
            if(receiver.reconstructed_map().has_value()) {
                result.receiver_map = *receiver.reconstructed_map();
            }
            return result;
        }

        bool matches_snapshot(
                const std::optional<ReconstructedMap> & map,
                const CanonicalSnapshot & snapshot)
        {
            const auto flat_hash = PerceptionMapUpdate::ContentHasher::content_hash(
                    snapshot.source,
                    snapshot.geometry_fingerprint,
                    map.has_value() ? map->cells : PerceptionMapUpdate::CanonicalCellView {});
            return map.has_value() && map->source == snapshot.source
                   && map->geometry == snapshot.geometry
                   && map->revision == snapshot.revision
                   && flat_hash == snapshot.content_hash
                   && map->cells == snapshot.cells;
        }

        bool same_reconstructed_map(
                const std::optional<ReconstructedMap> & left,
                const std::optional<ReconstructedMap> & right)
        {
            if(left.has_value() != right.has_value()) {
                return false;
            }
            return !left.has_value()
                   || (left->source == right->source
                       && left->geometry == right->geometry
                       && left->revision == right->revision
                       && left->content_identity == right->content_identity
                       && left->cells == right->cells);
        }

    }// namespace

    MapUpdateAcceptanceRun MapUpdateAcceptanceScenarios::resync_recovery(
            const std::vector<CanonicalSnapshot> & snapshots,
            const PerceptionMapUpdate::MapUpdateLimits & limits)
    {
        validate_snapshots(snapshots);
        MapUpdateAcceptanceRun run;
        run.kind = MapUpdateAcceptanceScenarioKind::ResyncRecovery;
        run.stages.reserve(4U);

        MapUpdateProducer producer(limits);
        MapUpdateApplier receiver(limits);
        if(!receiver.admit_source(snapshots[0].source)) {
            throw std::runtime_error("resync scenario could not admit its source");
        }

        const auto baseline = produce_and_commit(producer, snapshots[0]);
        const auto baseline_apply = receiver.apply(*baseline.update);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::ReadyBaseline,
                receiver,
                baseline.update->base_revision,
                baseline.update->new_revision,
                baseline_apply));

        const auto dropped = produce_and_commit(producer, snapshots[1]);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::DeltaDropped,
                receiver,
                dropped.update->base_revision,
                dropped.update->new_revision,
                std::nullopt,
                {},
                "delta intentionally dropped by the acceptance fixture"));

        const auto future = produce_and_commit(producer, snapshots[2]);
        const auto gap_apply = receiver.apply(*future.update);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::GapRejected,
                receiver,
                future.update->base_revision,
                future.update->new_revision,
                gap_apply));

        const std::string correlation_id = "replay-resync-1";
        if(!producer.request_keyframe(correlation_id)) {
            throw std::runtime_error("resync scenario could not request a keyframe");
        }
        const auto recovery = produce_and_commit(producer, snapshots[2]);
        const auto recovery_apply = receiver.apply(*recovery.update);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::ResyncRecovered,
                receiver,
                recovery.update->base_revision,
                recovery.update->new_revision,
                recovery_apply,
                recovery.update->correlation_id));

        const bool retained_after_drop = same_reconstructed_map(
                run.stages[0].receiver_map, run.stages[1].receiver_map);
        const bool retained_after_gap = same_reconstructed_map(
                run.stages[0].receiver_map, run.stages[2].receiver_map);
        run.passed = baseline.update->kind == UpdateKind::Keyframe
                     && baseline_apply.status == ApplyUpdateStatus::AppliedKeyframe
                     && dropped.update->kind == UpdateKind::Delta
                     && future.update->kind == UpdateKind::Delta
                     && retained_after_drop && retained_after_gap
                     && gap_apply.status == ApplyUpdateStatus::RejectedGap
                     && run.stages[2].receiver_state == ReceiverState::ResyncRequired
                     && recovery.update->kind == UpdateKind::Keyframe
                     && recovery.update->correlation_id == correlation_id
                     && recovery_apply.status == ApplyUpdateStatus::AppliedKeyframe
                     && run.stages[3].receiver_state == ReceiverState::Ready
                     && matches_snapshot(run.stages[3].receiver_map, snapshots[2]);
        if(!run.passed) {
            run.diagnostic =
                    "resync scenario did not retain the last valid map and recover by keyframe";
        }
        return run;
    }

    MapUpdateAcceptanceRun MapUpdateAcceptanceScenarios::epoch_reset(
            const std::vector<CanonicalSnapshot> & snapshots,
            const PerceptionMapUpdate::MapUpdateLimits & limits)
    {
        validate_snapshots(snapshots);
        if(snapshots[2].revision == std::numeric_limits<std::uint64_t>::max()
           || snapshots[2].source.map_epoch
                      == std::numeric_limits<std::uint64_t>::max()) {
            throw std::invalid_argument(
                    "epoch-reset acceptance seed cannot advance revision or epoch");
        }

        MapUpdateAcceptanceRun run;
        run.kind = MapUpdateAcceptanceScenarioKind::EpochReset;
        run.stages.reserve(4U);

        MapUpdateProducer old_producer(limits);
        MapUpdateApplier receiver(limits);
        if(!receiver.admit_source(snapshots[2].source)) {
            throw std::runtime_error("epoch-reset scenario could not admit the old source");
        }
        const auto old_baseline = produce_and_commit(old_producer, snapshots[2]);
        const auto old_apply = receiver.apply(*old_baseline.update);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::EpochBaseline,
                receiver,
                old_baseline.update->base_revision,
                old_baseline.update->new_revision,
                old_apply));

        auto new_epoch_snapshot = snapshots[0];
        ++new_epoch_snapshot.source.map_epoch;
        new_epoch_snapshot.revision = 1U;
        new_epoch_snapshot.content_hash = PerceptionMapUpdate::ContentHasher::content_hash(
                new_epoch_snapshot.source,
                new_epoch_snapshot.geometry_fingerprint,
                new_epoch_snapshot.cells);
        if(!receiver.admit_source(new_epoch_snapshot.source)) {
            throw std::runtime_error("epoch-reset scenario could not admit the new epoch");
        }
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::NewEpochAdmitted,
                receiver,
                0U,
                new_epoch_snapshot.revision,
                std::nullopt,
                {},
                "new map epoch admitted; keyframe required"));

        auto old_next = snapshots[2];
        ++old_next.revision;
        const auto old_delta = produce_and_commit(old_producer, old_next);
        const auto old_rejection = receiver.apply(*old_delta.update);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::OldEpochRejected,
                receiver,
                old_delta.update->base_revision,
                old_delta.update->new_revision,
                old_rejection));

        MapUpdateProducer new_producer(limits);
        const auto new_keyframe = produce_and_commit(new_producer, new_epoch_snapshot);
        const auto new_apply = receiver.apply(*new_keyframe.update);
        run.stages.push_back(stage(
                MapUpdateAcceptanceStageKind::EpochRecovered,
                receiver,
                new_keyframe.update->base_revision,
                new_keyframe.update->new_revision,
                new_apply));

        const bool retained_after_admission = same_reconstructed_map(
                run.stages[0].receiver_map, run.stages[1].receiver_map);
        const bool retained_after_rejection = same_reconstructed_map(
                run.stages[0].receiver_map, run.stages[2].receiver_map);
        run.passed = old_apply.status == ApplyUpdateStatus::AppliedKeyframe
                     && run.stages[0].receiver_state == ReceiverState::Ready
                     && retained_after_admission && retained_after_rejection
                     && run.stages[1].receiver_state == ReceiverState::ResyncRequired
                     && old_delta.update->kind == UpdateKind::Delta
                     && old_rejection.status == ApplyUpdateStatus::RejectedAdmission
                     && new_keyframe.update->kind == UpdateKind::Keyframe
                     && new_apply.status == ApplyUpdateStatus::AppliedKeyframe
                     && run.stages[3].receiver_state == ReceiverState::Ready
                     && matches_snapshot(
                                run.stages[3].receiver_map, new_epoch_snapshot);
        if(!run.passed) {
            run.diagnostic =
                    "epoch-reset scenario did not reject the old chain and rebuild the new epoch";
        }
        return run;
    }

}// namespace PerceptionProfiling

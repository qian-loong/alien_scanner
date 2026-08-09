#include "perception_profiling/ProfileDataSink.hpp"
#include "perception_profiling/ProfileMapHarness.hpp"
#include "perception_profiling/MapUpdateAcceptanceScenarios.hpp"
#include "perception_profiling/MapUpdateReplayOracle.hpp"
#include "perception_profiling/ProfileOracle.hpp"
#include "perception_profiling/ProfileScenario.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

namespace PerceptionProfiling::Test {
    TEST(ProfileScenario, DeterministicUniqueSequenceAndBoundedMotionHaveNoJump)
    {
        const ProfileScenario first(ScenarioMode::Bounded);
        const ProfileScenario second(ScenarioMode::Bounded);
        auto previous = first.sample(1U);
        EXPECT_EQ(previous.payload_digest, second.sample(1U).payload_digest);
        for(std::uint64_t sequence = 2U; sequence <= 450U; ++sequence) {
            const auto sample = first.sample(sequence);
            EXPECT_EQ(
                    sample.observation.origin_stamp.nanoseconds
                            - previous.observation.origin_stamp.nanoseconds,
                    FrozenProfileConfig::kObservationPeriodNs);
            EXPECT_NE(sample.payload_digest, previous.payload_digest);
            EXPECT_LE(
                    std::abs(
                            sample.observation_pose.position.x()
                            - previous.observation_pose.position.x()),
                    0.0500000001);
            EXPECT_GE(sample.observation_pose.position.x(), first.config().start_x_m);
            EXPECT_LE(sample.observation_pose.position.x(), first.config().end_x_m);
            EXPECT_EQ(
                    first.sequence_for_stamp(sample.observation.origin_stamp),
                    std::optional<std::uint64_t>(sequence));
            previous = sample;
        }
        EXPECT_DOUBLE_EQ(first.sample(201U).observation_pose.position.x(), 11.0);
        EXPECT_DOUBLE_EQ(first.sample(401U).observation_pose.position.x(), 1.0);
    }

    TEST(ProfileScenario, EllipseAndFrozenNoReturnSectorAreExactFullRay)
    {
        const ProfileScenario scenario(ScenarioMode::Expanding);
        const auto sample = scenario.sample(1U);
        ASSERT_TRUE(sample.observation.is_2d());
        const auto & scan = sample.observation.as_scan_2d();
        ASSERT_EQ(scan.ranges.size(), FrozenProfileConfig::kBeamCount);
        EXPECT_EQ(sample.observation.ray_evidence, Perception::RayEvidenceCapability::FullRay);
        for(std::size_t index = 0U; index < scan.ranges.size(); ++index) {
            if(index >= scenario.config().no_return_first_index
               && index <= scenario.config().no_return_last_index) {
                EXPECT_TRUE(std::isinf(scan.ranges[index]));
                EXPECT_GT(scan.ranges[index], 0.0F);
            }
            else {
                ASSERT_TRUE(std::isfinite(scan.ranges[index]));
                const double angle = scan.angle_min_rad
                                     + static_cast<double>(index)
                                               * scan.angle_increment_rad;
                const double y = static_cast<double>(scan.ranges[index]) * std::cos(angle);
                const double z = static_cast<double>(scan.ranges[index]) * std::sin(angle);
                EXPECT_NEAR(
                        y * y / std::pow(scenario.config().tunnel_y_radius_m, 2.0)
                                + z * z
                                          / std::pow(
                                                  scenario.config().tunnel_z_radius_m, 2.0),
                        1.0,
                        2e-6);
            }
        }
    }

    TEST(ProfileScenario, CanonicalTreeCaveSequenceIsDeterministicAndNotAnalyticTunnel)
    {
        const ProfileScenario first(ScenarioMode::Canonical);
        const ProfileScenario second(ScenarioMode::Canonical);
        const ProfileScenario analytic(ScenarioMode::Expanding);

        const auto first_sample = first.sample(1U);
        const auto repeated_sample = second.sample(1U);
        const auto analytic_sample = analytic.sample(1U);
        ASSERT_TRUE(first_sample.observation.is_2d());
        EXPECT_EQ(first_sample.payload_digest, repeated_sample.payload_digest);
        EXPECT_EQ(
                first_sample.observation.as_scan_2d().ranges,
                repeated_sample.observation.as_scan_2d().ranges);
        EXPECT_NE(first_sample.payload_digest, analytic_sample.payload_digest);

        const auto & ranges = first_sample.observation.as_scan_2d().ranges;
        ASSERT_EQ(ranges.size(), FrozenProfileConfig::kBeamCount);
        EXPECT_TRUE(std::any_of(
                ranges.begin()
                        + static_cast<std::ptrdiff_t>(first.config().no_return_first_index),
                ranges.begin()
                        + static_cast<std::ptrdiff_t>(
                                first.config().no_return_last_index + 1U),
                [](float range) { return std::isfinite(range); }));

        double previous_x = first_sample.observation_pose.position.x();
        for(std::uint64_t sequence = 2U;
            sequence <= first.canonical_sequence_limit().value();
            ++sequence) {
            const double x = first.sample(sequence).observation_pose.position.x();
            EXPECT_GE(x, previous_x);
            EXPECT_LE(x, first.config().end_x_m);
            previous_x = x;
        }
        EXPECT_GT(previous_x, first.config().start_x_m);
    }

    TEST(ProfileScenario, StaticExtrinsicMapsScanPlaneToTunnelCrossSection)
    {
        const ProfileScenario scenario(ScenarioMode::Bounded);
        const auto rotation = scenario.config().body_from_sensor.normalized();
        const Eigen::Vector3d local_x = rotation * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d local_y = rotation * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d local_z = rotation * Eigen::Vector3d::UnitZ();
        EXPECT_TRUE(local_x.isApprox(Eigen::Vector3d::UnitY(), 1e-12));
        EXPECT_TRUE(local_y.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
        EXPECT_TRUE(local_z.isApprox(Eigen::Vector3d::UnitX(), 1e-12));
        EXPECT_TRUE(scenario.extrinsic().translation.isZero(0.0));
    }

    TEST(ProfileOracle, RealMapperPreservesOriginFreeAndNoReturnBoundaryUnknown)
    {
        const ProfileScenario scenario(ScenarioMode::Expanding);
        ProfileMapHarness harness(scenario, {30'000'000'000ULL, 3U});
        std::optional<PerceptionLocalMap::CommitReceipt> receipt;
        for(std::uint64_t sequence = 1U; sequence <= 6U; ++sequence) {
            const auto result = harness.submit(scenario.sample(sequence));
            if(result.status == PerceptionLocalMap::InputStatus::Applied) {
                receipt = result.receipt;
            }
        }
        ASSERT_TRUE(receipt.has_value());
        auto acquired = harness.mapper().acquire_read_transaction(receipt.value());
        ASSERT_EQ(acquired.status, PerceptionLocalMap::AcquireStatus::Ready);
        ASSERT_TRUE(acquired.transaction.has_value());
        const double x = scenario.sample(6U).observation_pose.position.x();
        EXPECT_EQ(
                acquired.transaction->query({x, 0.0, scenario.config().body_z_m}).state,
                PerceptionLocalMap::OccupancyState::Free);
        EXPECT_EQ(
                acquired.transaction
                        ->query(
                                {x,
                                 0.0,
                                 scenario.config().body_z_m
                                         + scenario.config().range_max_m})
                        .state,
                PerceptionLocalMap::OccupancyState::Unknown);
    }

    TEST(MapUpdateReplayOracle, ExactRevisionProductionPipelineMatchesEveryCheckpoint)
    {
        const MapUpdateReplayOracle oracle(ProfileScenario(ScenarioMode::Canonical));
        MapUpdateReplayOptions options;
        options.sequence_count = 12U;
        const auto run = oracle.run(options);

        ASSERT_TRUE(run.all_checkpoints_match()) << run.final_comparison.diagnostic;
        EXPECT_EQ(run.rejected_input_count, 0U);
        EXPECT_EQ(run.unavailable_input_count, 1U);
        EXPECT_EQ(run.backend_fault_input_count, 0U);
        ASSERT_EQ(run.checkpoints.size(), run.applied_input_count);
        ASSERT_FALSE(run.checkpoints.empty());
        EXPECT_EQ(run.keyframe_count, 1U);
        EXPECT_EQ(run.delta_count + run.keyframe_count, run.checkpoints.size());
        EXPECT_EQ(
                run.checkpoints.front().update_kind,
                PerceptionMapUpdate::UpdateKind::Keyframe);
        for(const auto & checkpoint : run.checkpoints) {
            EXPECT_TRUE(checkpoint.comparison.matches)
                    << "revision " << checkpoint.revision << ": "
                    << checkpoint.comparison.diagnostic;
            EXPECT_EQ(checkpoint.source_cell_count, checkpoint.reconstructed_cell_count);
            EXPECT_TRUE(
                    checkpoint.apply_status
                            == PerceptionMapUpdate::ApplyUpdateStatus::AppliedKeyframe
                    || checkpoint.apply_status
                               == PerceptionMapUpdate::ApplyUpdateStatus::AppliedDelta);
        }
        ASSERT_TRUE(run.final_oracle.has_value());
        ASSERT_TRUE(run.final_reconstructed.has_value());
        EXPECT_EQ(run.final_oracle->revision, run.checkpoints.back().revision);
        EXPECT_EQ(run.final_oracle->cells, run.final_reconstructed->cells);
    }

    TEST(MapUpdateReplayOracle, FixedInputRepeatsRevisionHashAndUpdateKind)
    {
        const MapUpdateReplayOptions options {8U, 16U, {}};
        const auto first = MapUpdateReplayOracle(
                                   ProfileScenario(ScenarioMode::Bounded))
                                   .run(options);
        const auto second = MapUpdateReplayOracle(
                                    ProfileScenario(ScenarioMode::Bounded))
                                    .run(options);

        ASSERT_TRUE(first.all_checkpoints_match());
        ASSERT_TRUE(second.all_checkpoints_match());
        ASSERT_EQ(first.checkpoints.size(), second.checkpoints.size());
        for(std::size_t index = 0U; index < first.checkpoints.size(); ++index) {
            const auto & left = first.checkpoints[index];
            const auto & right = second.checkpoints[index];
            EXPECT_EQ(left.sequence, right.sequence);
            EXPECT_EQ(left.revision, right.revision);
            EXPECT_EQ(left.update_kind, right.update_kind);
            EXPECT_EQ(left.operation_count, right.operation_count);
            EXPECT_EQ(left.payload_bytes, right.payload_bytes);
            EXPECT_EQ(left.content_hash, right.content_hash);
            EXPECT_EQ(left.update_hash, right.update_hash);
        }
        ASSERT_TRUE(first.final_oracle.has_value());
        ASSERT_TRUE(second.final_oracle.has_value());
        EXPECT_EQ(first.final_oracle->cells, second.final_oracle->cells);
        EXPECT_EQ(first.final_oracle->content_hash, second.final_oracle->content_hash);
    }

    TEST(MapUpdateAcceptanceScenarios, GapRetainsMapUntilCorrelatedKeyframeRecovers)
    {
        MapUpdateReplayOptions options;
        options.sequence_count = 8U;
        const auto replay = MapUpdateReplayOracle(
                                    ProfileScenario(ScenarioMode::Bounded))
                                    .run(options);
        ASSERT_TRUE(replay.all_checkpoints_match());
        ASSERT_EQ(replay.acceptance_snapshots.size(), 3U);

        const auto run = MapUpdateAcceptanceScenarios::resync_recovery(
                replay.acceptance_snapshots);
        ASSERT_TRUE(run.passed) << run.diagnostic;
        ASSERT_EQ(run.stages.size(), 4U);
        EXPECT_EQ(run.stages[0].kind, MapUpdateAcceptanceStageKind::ReadyBaseline);
        EXPECT_EQ(run.stages[0].receiver_state,
                  PerceptionMapUpdate::ReceiverState::Ready);
        EXPECT_EQ(run.stages[1].kind, MapUpdateAcceptanceStageKind::DeltaDropped);
        EXPECT_EQ(run.stages[2].kind, MapUpdateAcceptanceStageKind::GapRejected);
        EXPECT_EQ(run.stages[2].apply_status,
                  PerceptionMapUpdate::ApplyUpdateStatus::RejectedGap);
        EXPECT_EQ(run.stages[2].receiver_state,
                  PerceptionMapUpdate::ReceiverState::ResyncRequired);
        ASSERT_TRUE(run.stages[0].receiver_map.has_value());
        ASSERT_TRUE(run.stages[2].receiver_map.has_value());
        EXPECT_EQ(run.stages[0].receiver_map->revision,
                  run.stages[2].receiver_map->revision);
        EXPECT_EQ(run.stages[0].receiver_map->cells,
                  run.stages[2].receiver_map->cells);
        EXPECT_EQ(run.stages[3].kind, MapUpdateAcceptanceStageKind::ResyncRecovered);
        EXPECT_EQ(run.stages[3].correlation_id, "replay-resync-1");
        EXPECT_EQ(run.stages[3].receiver_state,
                  PerceptionMapUpdate::ReceiverState::Ready);
        ASSERT_TRUE(run.stages[3].receiver_map.has_value());
        EXPECT_EQ(run.stages[3].receiver_map->revision,
                  replay.acceptance_snapshots[2].revision);
        EXPECT_EQ(run.stages[3].receiver_map->cells,
                  replay.acceptance_snapshots[2].cells);
    }

    TEST(MapUpdateAcceptanceScenarios, EpochResetRejectsOldChainAndBuildsNewBaseline)
    {
        MapUpdateReplayOptions options;
        options.sequence_count = 8U;
        const auto replay = MapUpdateReplayOracle(
                                    ProfileScenario(ScenarioMode::Bounded))
                                    .run(options);
        ASSERT_TRUE(replay.all_checkpoints_match());

        const auto run = MapUpdateAcceptanceScenarios::epoch_reset(
                replay.acceptance_snapshots);
        ASSERT_TRUE(run.passed) << run.diagnostic;
        ASSERT_EQ(run.stages.size(), 4U);
        EXPECT_EQ(run.stages[0].kind, MapUpdateAcceptanceStageKind::EpochBaseline);
        EXPECT_EQ(run.stages[1].kind,
                  MapUpdateAcceptanceStageKind::NewEpochAdmitted);
        EXPECT_EQ(run.stages[1].receiver_state,
                  PerceptionMapUpdate::ReceiverState::ResyncRequired);
        EXPECT_EQ(run.stages[2].kind,
                  MapUpdateAcceptanceStageKind::OldEpochRejected);
        EXPECT_EQ(run.stages[2].apply_status,
                  PerceptionMapUpdate::ApplyUpdateStatus::RejectedAdmission);
        ASSERT_TRUE(run.stages[0].receiver_map.has_value());
        ASSERT_TRUE(run.stages[2].receiver_map.has_value());
        EXPECT_EQ(run.stages[0].receiver_map->source,
                  run.stages[2].receiver_map->source);
        EXPECT_EQ(run.stages[0].receiver_map->revision,
                  run.stages[2].receiver_map->revision);
        EXPECT_EQ(run.stages[3].kind,
                  MapUpdateAcceptanceStageKind::EpochRecovered);
        EXPECT_EQ(run.stages[3].receiver_state,
                  PerceptionMapUpdate::ReceiverState::Ready);
        ASSERT_TRUE(run.stages[3].receiver_map.has_value());
        EXPECT_EQ(run.stages[3].receiver_map->source.map_epoch,
                  replay.acceptance_snapshots[0].source.map_epoch + 1U);
        EXPECT_EQ(run.stages[3].receiver_map->revision, 1U);
        EXPECT_EQ(run.stages[3].receiver_map->cells,
                  replay.acceptance_snapshots[0].cells);
    }

    TEST(MapUpdateReplayOracle, ReportsBoundedMissingUnexpectedAndStateDifferences)
    {
        MapUpdateReplayOptions options;
        options.sequence_count = 3U;
        const auto run = MapUpdateReplayOracle(
                                 ProfileScenario(ScenarioMode::Expanding))
                                 .run(options);
        ASSERT_TRUE(run.all_checkpoints_match());
        ASSERT_TRUE(run.final_oracle.has_value());
        ASSERT_TRUE(run.final_reconstructed.has_value());
        ASSERT_GE(run.final_reconstructed->cells.size(), 3U);

        auto changed = *run.final_reconstructed;
        changed.cells[0].state = changed.cells[0].state
                == PerceptionMapUpdate::CellState::Free
                ? PerceptionMapUpdate::CellState::Occupied
                : PerceptionMapUpdate::CellState::Free;
        changed.cells.erase(changed.cells.begin() + 1);
        auto unexpected = changed.cells.back();
        ++unexpected.index.z;
        changed.cells.push_back(unexpected);

        const auto comparison = MapUpdateReplayOracle::compare(
                *run.final_oracle, changed, 1U);
        EXPECT_FALSE(comparison.matches);
        EXPECT_EQ(comparison.missing_cell_count, 1U);
        EXPECT_EQ(comparison.unexpected_cell_count, 1U);
        EXPECT_EQ(comparison.state_mismatch_count, 1U);
        EXPECT_EQ(comparison.missing_cell_samples.size(), 1U);
        EXPECT_EQ(comparison.unexpected_cell_samples.size(), 1U);
        EXPECT_EQ(comparison.state_mismatch_samples.size(), 1U);
        EXPECT_EQ(comparison.diagnostic, "cell mismatch: missing=1, unexpected=1, state=1");
    }

    TEST(ProfileOracle, CheckpointsGrowAndMilestonesUseFirstExactCrossing)
    {
        const ProfileOracle oracle(ProfileScenario(ScenarioMode::Expanding));
        OracleOptions options;
        options.sequence_count = 80U;
        options.checkpoint_period_revisions = 20U;
        options.milestone_thresholds = {100U, 500U, 1000U};
        const auto run = oracle.run(options);
        EXPECT_EQ(run.rejected, 0U);
        EXPECT_EQ(run.backend_fault, 0U);
        ASSERT_GE(run.checkpoints.size(), 2U);
        EXPECT_GT(run.checkpoints.back().counts.known, run.checkpoints.front().counts.known);
        ASSERT_TRUE(run.checkpoints.front().bounds.has_value());
        ASSERT_TRUE(run.checkpoints.back().bounds.has_value());
        EXPECT_GT(
                run.checkpoints.back().bounds->maximum.x,
                run.checkpoints.front().bounds->maximum.x);
        ASSERT_FALSE(run.milestones.empty());
        for(const auto & milestone : run.milestones) {
            EXPECT_GT(milestone.known, milestone.threshold);
            EXPECT_GT(milestone.revision, 0U);
        }
    }

    TEST(ProfileOracle, BoundedPlateauRequiresContinuousExactCensus)
    {
        const ProfileOracle oracle(ProfileScenario(ScenarioMode::Bounded));
        OracleOptions options;
        options.sequence_count = 450U;
        options.checkpoint_period_revisions = 50U;
        options.bounded_plateau_revisions = 20U;
        options.milestone_thresholds.clear();
        const auto run = oracle.run(options);
        ASSERT_TRUE(run.plateau_start_revision.has_value());
        EXPECT_LE(
                run.plateau_start_revision.value() + options.bounded_plateau_revisions,
                run.final_revision);
        ASSERT_GE(run.checkpoints.size(), 2U);
        EXPECT_EQ(run.checkpoints.back().counts, run.checkpoints[run.checkpoints.size() - 2U].counts);
        EXPECT_EQ(run.checkpoints.back().bounds.has_value(), true);
        EXPECT_EQ(run.rejected, 0U);
        EXPECT_EQ(run.backend_fault, 0U);
    }

    TEST(ProfileOracle, CapacityBucketsAreMutuallyExclusiveAndExactlyTwoHundredSamples)
    {
        const auto covered = ProfileOracle::classify_capacity(1200U, 6000U);
        ASSERT_EQ(covered.status, CapacityStatus::Covered);
        ASSERT_TRUE(covered.pre.has_value());
        ASSERT_TRUE(covered.crossing.has_value());
        ASSERT_TRUE(covered.post.has_value());
        EXPECT_EQ(covered.pre->sample_count(), 200U);
        EXPECT_EQ(covered.crossing->sample_count(), 200U);
        EXPECT_EQ(covered.post->sample_count(), 200U);
        EXPECT_LT(covered.pre->last, covered.crossing->first);
        EXPECT_LT(covered.crossing->last, covered.post->first);
        EXPECT_EQ(covered.required_end_revision, std::optional<std::uint64_t>(1500U));
        EXPECT_EQ(
                ProfileOracle::classify_capacity(std::nullopt, 6000U).status,
                CapacityStatus::NotReached);
        EXPECT_EQ(
                ProfileOracle::classify_capacity(5800U, 6000U).status,
                CapacityStatus::CrossingWithoutPostWindow);
    }

    TEST(ProfileOracle, ProductionJoinRejectsEveryAuthoritativeKeyMismatch)
    {
        const ProfileOracle oracle(ProfileScenario(ScenarioMode::Expanding));
        OracleOptions options;
        options.sequence_count = 25U;
        options.checkpoint_period_revisions = 10U;
        options.milestone_thresholds.clear();
        const auto run = oracle.run(options);
        ASSERT_FALSE(run.checkpoints.empty());
        const auto & checkpoint = run.checkpoints.front();
        ProductionStateProjection production {
                1,
                1U,
                checkpoint.map_epoch,
                checkpoint.revision,
                checkpoint.observation_stamp_ns,
                checkpoint.contract_fingerprint,
                checkpoint.bounds,
                0U};
        EXPECT_TRUE(ProfileOracle::validate_join(production, checkpoint).matches);
        ++production.revision;
        const auto mismatch = ProfileOracle::validate_join(production, checkpoint);
        EXPECT_FALSE(mismatch.matches);
        EXPECT_EQ(mismatch.diagnostic, "revision mismatch");
    }

    TEST(ProfileDataSink, WritesSchemaDigestAndTransportManifestWithoutOverwrite)
    {
        const auto directory = std::filesystem::temp_directory_path()
                               / "alien-profile-sink-gtest";
        std::filesystem::remove_all(directory);
        const ProfileScenario scenario(ScenarioMode::Bounded);
        {
            ProfileDataSink sink(directory, scenario, C3ProfileMode::Enabled);
            const auto sample = scenario.sample(1U);
            sink.record_observation(sample.observation, 1);
            auto changed = sample.observation;
            std::get<Perception::Scan2D>(changed.data).ranges[0] += 0.25F;
            sink.record_observation(changed, 2);
            sink.record_observation(scenario.sample(3U).observation, 3);
            sink.record_observation(scenario.sample(2U).observation, 4);
            sink.record_state(
                    {5,
                     1U,
                     1U,
                     1U,
                     sample.observation.origin_stamp.nanoseconds,
                     scenario.contract_fingerprint().hex_digest,
                     std::nullopt,
                     10U});
            sink.record_state(
                    {6,
                     2U,
                     1U,
                     1U,
                     sample.observation.origin_stamp.nanoseconds,
                     scenario.contract_fingerprint().hex_digest,
                     std::nullopt,
                     10U});
            sink.record_state(
                    {7,
                     4U,
                     1U,
                     3U,
                     scenario.sample(3U).observation.origin_stamp.nanoseconds,
                     scenario.contract_fingerprint().hex_digest,
                     std::nullopt,
                     10U});
            sink.record_state(
                    {8,
                     3U,
                     1U,
                     2U,
                     scenario.sample(2U).observation.origin_stamp.nanoseconds,
                     scenario.contract_fingerprint().hex_digest,
                     std::nullopt,
                     10U});
            sink.record_health(
                    {9,
                     sample.observation.origin_stamp.nanoseconds,
                     0U,
                     scenario.config().producer_source_id,
                     scenario.config().producer_session,
                     scenario.contract_fingerprint().hex_digest,
                     true,
                     1U});
            sink.record_diagnostic({10, 0, 1U, "target", "expected warning"});
            DiagnosticProjection producer_diagnostic {
                    11,
                    0,
                    0U,
                    "/profile_local_map: map_update_producer",
                    "running"};
            producer_diagnostic.values = {
                    {"pending", "0"},
                    {"in_flight", "0"},
                    {"pending_revision", "0"},
                    {"in_flight_revision", "0"},
                    {"published_revision", "2"},
                    {"coalesced_receipts", "1"},
                    {"superseded_receipts", "0"},
                    {"published_keyframes", "1"},
                    {"published_deltas", "1"},
                    {"revision_only_deltas", "1"},
                    {"publish_failures", "0"},
                    {"resource_rejections", "0"},
                    {"snapshot_cells", "25"},
                    {"delta_operations", "0"},
                    {"payload_bytes", "4"},
                    {"acquire_duration_ns", "1"},
                    {"materialize_duration_ns", "2"},
                    {"traversal_duration_ns", "3"},
                    {"canonicalize_duration_ns", "4"},
                    {"content_hash_duration_ns", "5"},
                    {"prepare_duration_ns", "6"},
                    {"validation_duration_ns", "7"},
                    {"diff_duration_ns", "8"},
                    {"encode_duration_ns", "9"},
                    {"update_hash_duration_ns", "10"},
                    {"publish_duration_ns", "11"},
            };
            sink.record_diagnostic(producer_diagnostic);
            sink.record_snapshot({11, 0, 1U, true, 0.2, 100U});
            MapUpdateProjection keyframe;
            keyframe.receipt_monotonic_ns = 12;
            keyframe.update_kind = 1U;
            keyframe.vehicle_id = "profile-vehicle";
            keyframe.mapper_session = {1U, 2U};
            keyframe.map_epoch = 1U;
            keyframe.new_revision = 1U;
            keyframe.revision_span = 1U;
            keyframe.known_cell_count = 25U;
            keyframe.canonical_payload_bytes = 404U;
            keyframe.content_hash[0] = 1U;
            keyframe.update_hash[0] = 2U;
            sink.record_map_update(keyframe);
            auto delta = keyframe;
            delta.receipt_monotonic_ns = 13;
            delta.update_kind = 2U;
            delta.base_revision = 1U;
            delta.new_revision = 2U;
            delta.base_content_hash = keyframe.content_hash;
            delta.revision_span = 1U;
            delta.canonical_payload_bytes = 4U;
            delta.update_hash[0] = 3U;
            sink.record_map_update(delta);
            sink.finalize(true);
            EXPECT_EQ(sink.summary().observation_count, 4U);
            EXPECT_EQ(sink.summary().unique_observation_count, 3U);
            EXPECT_EQ(sink.summary().duplicate_observation_count, 1U);
            EXPECT_EQ(sink.summary().observation_sequence_gaps, 1U);
            EXPECT_EQ(sink.summary().out_of_order_observation_count, 1U);
            EXPECT_EQ(sink.summary().observation_digest_mismatches, 1U);
            EXPECT_EQ(sink.summary().state_sequence_gaps, 1U);
            EXPECT_EQ(sink.summary().state_sequence_out_of_order, 1U);
            EXPECT_EQ(sink.summary().revision_heartbeat_count, 1U);
            EXPECT_EQ(sink.summary().revision_gaps, 1U);
            EXPECT_EQ(sink.summary().revision_regressions, 1U);
            EXPECT_EQ(sink.summary().producer_diagnostic_count, 1U);
            EXPECT_EQ(sink.summary().map_update_count, 2U);
            EXPECT_EQ(sink.summary().map_update_keyframe_count, 1U);
            EXPECT_EQ(sink.summary().map_update_delta_count, 1U);
            EXPECT_EQ(sink.summary().map_update_revision_only_delta_count, 1U);
            EXPECT_TRUE(sink.summary().normal_completion);
        }
        EXPECT_TRUE(std::filesystem::exists(directory / "observations.csv"));
        EXPECT_TRUE(std::filesystem::exists(directory / "states.csv"));
        EXPECT_TRUE(std::filesystem::exists(directory / "map_updates.csv"));
        EXPECT_TRUE(std::filesystem::exists(
                directory / "map_update_producer_diagnostics.csv"));
        EXPECT_TRUE(std::filesystem::exists(directory / "sink_manifest.yaml"));
        EXPECT_THROW(ProfileDataSink(directory, scenario), std::invalid_argument);
        EXPECT_THROW(parse_c3_profile_mode("invalid"), std::invalid_argument);
        std::filesystem::remove_all(directory);
    }

}// namespace PerceptionProfiling::Test

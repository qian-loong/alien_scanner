#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MerklePrototypeProtocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace PerceptionMapUpdate::Test {

    namespace {

        CanonicalCell cell(
                std::int64_t x,
                std::int64_t y,
                std::int64_t z,
                CellState state)
        {
            return {{x, y, z}, state};
        }

        SourceIdentity source()
        {
            return {"prototype-protocol", {42U, 7U}, 3U};
        }

        Hash256 geometry_hash()
        {
            return ContentHasher::geometry_fingerprint(
                    {0.1, {0.0, 0.0, 0.0}, "map"});
        }

        std::vector<CanonicalCell> initial_cells()
        {
            return {
                    cell(-1, 0, 0, CellState::Free),
                    cell(0, 0, 0, CellState::Free),
                    cell(16, 0, 0, CellState::Occupied)};
        }

    }// namespace

    TEST(MerklePrototypeProtocolTest, ReceiverRecomputesKeyframeAndDeltaRoots)
    {
        const auto keyframe = MerklePrototypeProtocol::prepare_keyframe(
                source(), geometry_hash(), 1U, initial_cells());
        ASSERT_TRUE(keyframe) << keyframe.diagnostic;
        const auto repeated_keyframe = MerklePrototypeProtocol::prepare_keyframe(
                source(), geometry_hash(), 1U, initial_cells());
        ASSERT_TRUE(repeated_keyframe) << repeated_keyframe.diagnostic;
        EXPECT_EQ(repeated_keyframe.update.update_hash, keyframe.update.update_hash);
        const auto received_keyframe = MerklePrototypeProtocol::verify_keyframe(keyframe.update);
        ASSERT_TRUE(received_keyframe) << received_keyframe.diagnostic;

        const std::vector<DeltaOperation> operations {
                {{0, 0, 0}, DeltaOperationKind::UpsertOccupied},
                {{1, 0, 0}, DeltaOperationKind::UpsertFree}};
        const auto delta = MerklePrototypeProtocol::prepare_delta(
                *keyframe.candidate, 1U, 3U, operations);
        ASSERT_TRUE(delta) << delta.diagnostic;
        const auto received_delta = MerklePrototypeProtocol::verify_delta(
                *received_keyframe.candidate, 1U, delta.update);
        ASSERT_TRUE(received_delta) << received_delta.diagnostic;
        EXPECT_EQ(
                received_delta.candidate->tree().versioned_digest(),
                delta.candidate->tree().versioned_digest());
        EXPECT_EQ(
                received_delta.candidate->store().view(),
                delta.candidate->store().view());
        EXPECT_EQ(received_keyframe.candidate->store().view(), initial_cells());
    }

    TEST(MerklePrototypeProtocolTest, RevisionOnlyAndRemoveHaveDistinctRoots)
    {
        const auto keyframe = MerklePrototypeProtocol::prepare_keyframe(
                source(), geometry_hash(), 5U, initial_cells());
        ASSERT_TRUE(keyframe) << keyframe.diagnostic;
        const auto revision_only = MerklePrototypeProtocol::prepare_delta(
                *keyframe.candidate, 5U, 6U, {});
        ASSERT_TRUE(revision_only) << revision_only.diagnostic;
        EXPECT_EQ(
                revision_only.update.result_content,
                keyframe.candidate->tree().versioned_digest());
        const auto verified_revision = MerklePrototypeProtocol::verify_delta(
                *keyframe.candidate, 5U, revision_only.update);
        ASSERT_TRUE(verified_revision) << verified_revision.diagnostic;

        const auto remove = MerklePrototypeProtocol::prepare_remove(
                *verified_revision.candidate, 6U, 7U);
        ASSERT_TRUE(remove) << remove.diagnostic;
        EXPECT_TRUE(is_zero_hash(remove.update.result_content.digest));
        EXPECT_FALSE(is_zero_hash(verified_revision.candidate->tree().content_root()));
        const auto verified_remove = MerklePrototypeProtocol::verify_remove(
                *verified_revision.candidate, 6U, remove.update);
        EXPECT_TRUE(verified_remove) << verified_remove.diagnostic;
        EXPECT_TRUE(verified_remove.removed);
        EXPECT_FALSE(verified_remove.candidate);
    }

    TEST(MerklePrototypeProtocolTest, TamperingAndWrongBaseAreAtomicRejections)
    {
        const auto keyframe = MerklePrototypeProtocol::prepare_keyframe(
                source(), geometry_hash(), 1U, initial_cells());
        ASSERT_TRUE(keyframe) << keyframe.diagnostic;
        const auto original_root = keyframe.candidate->tree().content_root();
        const auto original_cells = keyframe.candidate->store().view().materialize();
        const auto delta = MerklePrototypeProtocol::prepare_delta(
                *keyframe.candidate,
                1U,
                2U,
                {{{0, 0, 0}, DeltaOperationKind::UpsertOccupied}});
        ASSERT_TRUE(delta) << delta.diagnostic;

        auto tampered = delta.update;
        tampered.result_content.digest[0] ^= 0xffU;
        EXPECT_FALSE(MerklePrototypeProtocol::verify_delta(
                *keyframe.candidate, 1U, tampered));

        auto wrong_base = delta.update;
        wrong_base.base_content.digest[0] ^= 0xffU;
        wrong_base.update_hash = MerklePrototypeProtocol::update_hash(wrong_base);
        EXPECT_FALSE(MerklePrototypeProtocol::verify_delta(
                *keyframe.candidate, 1U, wrong_base));

        auto wrong_descriptor = delta.update;
        wrong_descriptor.descriptor.chunk_edge = 8U;
        wrong_descriptor.base_content.descriptor = wrong_descriptor.descriptor;
        wrong_descriptor.result_content.descriptor = wrong_descriptor.descriptor;
        wrong_descriptor.update_hash = MerklePrototypeProtocol::update_hash(wrong_descriptor);
        EXPECT_FALSE(MerklePrototypeProtocol::verify_delta(
                *keyframe.candidate, 1U, wrong_descriptor));

        EXPECT_EQ(keyframe.candidate->tree().content_root(), original_root);
        EXPECT_EQ(keyframe.candidate->store().view(), original_cells);
    }

    TEST(MerklePrototypeProtocolTest, UnknownKindAndCrossRevisionDeltaFailClosed)
    {
        const auto keyframe = MerklePrototypeProtocol::prepare_keyframe(
                source(), geometry_hash(), 1U, initial_cells());
        ASSERT_TRUE(keyframe) << keyframe.diagnostic;
        const auto delta = MerklePrototypeProtocol::prepare_delta(
                *keyframe.candidate,
                1U,
                2U,
                {{{0, 0, 0}, DeltaOperationKind::UpsertOccupied}});
        ASSERT_TRUE(delta) << delta.diagnostic;

        auto unknown = delta.update;
        unknown.kind = static_cast<MerklePrototypeUpdateKind>(99U);
        unknown.update_hash = MerklePrototypeProtocol::update_hash(unknown);
        EXPECT_FALSE(MerklePrototypeProtocol::verify_delta(
                *keyframe.candidate, 1U, unknown));
        EXPECT_FALSE(MerklePrototypeProtocol::verify_delta(
                *keyframe.candidate, 2U, delta.update));
    }

}// namespace PerceptionMapUpdate::Test

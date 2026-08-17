#include "perception_map_update/MerklePrototypeProtocol.hpp"

#include "CanonicalEncoding.hpp"
#include "Sha256DigestSink.hpp"

#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        using DigestSink = Internal::Sha256DigestSink;

        void write_descriptor(DigestSink & sink, const ContentIdentityDescriptor & descriptor)
        {
            Encoding::write_u16(sink, static_cast<std::uint16_t>(descriptor.scheme));
            Encoding::write_u32(sink, descriptor.chunk_edge);
            Encoding::write_u16(sink, descriptor.coordinate_key_version);
            Encoding::write_u16(sink, descriptor.node_encoding_version);
        }

        void write_digest(DigestSink & sink, const VersionedContentDigest & digest)
        {
            write_descriptor(sink, digest.descriptor);
            Encoding::write_hash(sink, digest.digest);
        }

        VersionedContentDigest tombstone(const ContentIdentityDescriptor & descriptor)
        {
            return {descriptor, {}};
        }

        bool valid_revision_span(
                std::uint64_t base_revision,
                std::uint64_t result_revision) noexcept
        {
            return base_revision > 0U && result_revision > base_revision;
        }

        bool validate_common(
                const MerklePrototypeUpdate & update,
                std::string & diagnostic)
        {
            if(!update.descriptor.valid()
               || update.base_content.descriptor != update.descriptor
               || update.result_content.descriptor != update.descriptor) {
                diagnostic = "prototype update descriptor is invalid or inconsistent";
                return false;
            }
            if(MerklePrototypeProtocol::update_hash(update) != update.update_hash) {
                diagnostic = "prototype update hash does not match its canonical envelope";
                return false;
            }
            return true;
        }

        bool validate_against_committed(
                const MerklePrototypeApplier & committed,
                std::uint64_t committed_revision,
                const MerklePrototypeUpdate & update,
                std::string & diagnostic)
        {
            if(update.descriptor != committed.tree().descriptor()
               || update.source != committed.tree().source()
               || update.geometry_fingerprint != committed.tree().geometry_fingerprint()) {
                diagnostic = "prototype update identity does not match committed state";
                return false;
            }
            if(update.base_revision != committed_revision
               || !valid_revision_span(update.base_revision, update.result_revision)) {
                diagnostic = "prototype update revision span is invalid";
                return false;
            }
            if(update.base_content != committed.tree().versioned_digest()) {
                diagnostic = "prototype update base content does not match committed root";
                return false;
            }
            return true;
        }

    }// namespace

    Hash256 MerklePrototypeProtocol::update_hash(const MerklePrototypeUpdate & update)
    {
        DigestSink sink;
        Encoding::write_string(sink, "alien-scanner/map-update/merkle-prototype-v2");
        Encoding::write_u16(sink, kMerkleContentIdentityVersion);
        Encoding::write_u8(sink, static_cast<std::uint8_t>(update.kind));
        write_descriptor(sink, update.descriptor);
        Encoding::write_identity(sink, update.source);
        Encoding::write_hash(sink, update.geometry_fingerprint);
        Encoding::write_u64(sink, update.base_revision);
        Encoding::write_u64(sink, update.result_revision);
        write_digest(sink, update.base_content);
        write_digest(sink, update.result_content);
        Encoding::write_u64(sink, static_cast<std::uint64_t>(update.keyframe_cells.size()));
        for(const auto & cell : update.keyframe_cells) {
            Encoding::write_cell(sink, cell);
        }
        Encoding::write_u64(sink, static_cast<std::uint64_t>(update.delta_operations.size()));
        for(const auto & operation : update.delta_operations) {
            Encoding::write_i64(sink, operation.index.x);
            Encoding::write_i64(sink, operation.index.y);
            Encoding::write_i64(sink, operation.index.z);
            Encoding::write_u8(sink, static_cast<std::uint8_t>(operation.kind));
        }
        return sink.finish();
    }

    MerklePrototypeTransition MerklePrototypeProtocol::prepare_keyframe(
            const SourceIdentity & source,
            const Hash256 & geometry_fingerprint,
            std::uint64_t result_revision,
            const std::vector<CanonicalCell> & cells,
            CellStorageConfig storage,
            ContentIdentityDescriptor descriptor)
    {
        MerklePrototypeTransition result;
        if(result_revision == 0U) {
            result.diagnostic = "prototype keyframe revision must be positive";
            return result;
        }
        const auto candidate = MerklePrototypeApplier::build(
                source, geometry_fingerprint, cells, storage, descriptor);
        if(!candidate || !candidate.candidate) {
            result.diagnostic = candidate.diagnostic;
            return result;
        }
        result.update.kind = MerklePrototypeUpdateKind::Keyframe;
        result.update.source = source;
        result.update.geometry_fingerprint = geometry_fingerprint;
        result.update.result_revision = result_revision;
        result.update.descriptor = descriptor;
        result.update.base_content = tombstone(descriptor);
        result.update.result_content = candidate.candidate->tree().versioned_digest();
        result.update.keyframe_cells = cells;
        result.update.update_hash = update_hash(result.update);
        result.candidate = candidate.candidate;
        result.success = true;
        return result;
    }

    MerklePrototypeTransition MerklePrototypeProtocol::prepare_delta(
            const MerklePrototypeApplier & base,
            std::uint64_t base_revision,
            std::uint64_t result_revision,
            const std::vector<DeltaOperation> & operations)
    {
        MerklePrototypeTransition result;
        if(!valid_revision_span(base_revision, result_revision)) {
            result.diagnostic = "prototype delta revision span is invalid";
            return result;
        }
        const auto candidate = base.apply(operations);
        if(!candidate || !candidate.candidate) {
            result.diagnostic = candidate.diagnostic;
            return result;
        }
        result.update.kind = MerklePrototypeUpdateKind::Delta;
        result.update.source = base.tree().source();
        result.update.geometry_fingerprint = base.tree().geometry_fingerprint();
        result.update.base_revision = base_revision;
        result.update.result_revision = result_revision;
        result.update.descriptor = base.tree().descriptor();
        result.update.base_content = base.tree().versioned_digest();
        result.update.result_content = candidate.candidate->tree().versioned_digest();
        result.update.delta_operations = operations;
        result.update.update_hash = update_hash(result.update);
        result.candidate = candidate.candidate;
        result.success = true;
        return result;
    }

    MerklePrototypeTransition MerklePrototypeProtocol::prepare_remove(
            const MerklePrototypeApplier & base,
            std::uint64_t base_revision,
            std::uint64_t result_revision)
    {
        MerklePrototypeTransition result;
        if(!valid_revision_span(base_revision, result_revision)) {
            result.diagnostic = "prototype remove revision span is invalid";
            return result;
        }
        result.update.kind = MerklePrototypeUpdateKind::Remove;
        result.update.source = base.tree().source();
        result.update.geometry_fingerprint = base.tree().geometry_fingerprint();
        result.update.base_revision = base_revision;
        result.update.result_revision = result_revision;
        result.update.descriptor = base.tree().descriptor();
        result.update.base_content = base.tree().versioned_digest();
        result.update.result_content = tombstone(result.update.descriptor);
        result.update.update_hash = update_hash(result.update);
        result.removed = true;
        result.success = true;
        return result;
    }

    MerklePrototypeTransition MerklePrototypeProtocol::verify_keyframe(
            const MerklePrototypeUpdate & update,
            CellStorageConfig storage)
    {
        MerklePrototypeTransition result;
        if(!validate_common(update, result.diagnostic)) {
            return result;
        }
        if(update.kind != MerklePrototypeUpdateKind::Keyframe
           || update.base_revision != 0U
           || update.result_revision == 0U
           || !is_zero_hash(update.base_content.digest)
           || !update.delta_operations.empty()) {
            result.diagnostic = "prototype keyframe envelope is invalid";
            return result;
        }
        const auto candidate = MerklePrototypeApplier::build(
                update.source,
                update.geometry_fingerprint,
                update.keyframe_cells,
                storage,
                update.descriptor);
        if(!candidate || !candidate.candidate) {
            result.diagnostic = candidate.diagnostic;
            return result;
        }
        if(candidate.candidate->tree().versioned_digest() != update.result_content) {
            result.diagnostic = "prototype keyframe result root does not match local rebuild";
            return result;
        }
        result.success = true;
        result.update = update;
        result.candidate = candidate.candidate;
        return result;
    }

    MerklePrototypeTransition MerklePrototypeProtocol::verify_delta(
            const MerklePrototypeApplier & committed,
            std::uint64_t committed_revision,
            const MerklePrototypeUpdate & update)
    {
        MerklePrototypeTransition result;
        if(!validate_common(update, result.diagnostic)) {
            return result;
        }
        if(update.kind != MerklePrototypeUpdateKind::Delta
           || !update.keyframe_cells.empty()
           || is_zero_hash(update.result_content.digest)
           || !validate_against_committed(
                   committed, committed_revision, update, result.diagnostic)) {
            if(result.diagnostic.empty()) {
                result.diagnostic = "prototype delta envelope is invalid";
            }
            return result;
        }
        const auto candidate = committed.apply(update.delta_operations);
        if(!candidate || !candidate.candidate) {
            result.diagnostic = candidate.diagnostic;
            return result;
        }
        if(candidate.candidate->tree().versioned_digest() != update.result_content) {
            result.diagnostic = "prototype delta result root does not match local apply";
            return result;
        }
        result.success = true;
        result.update = update;
        result.candidate = candidate.candidate;
        return result;
    }

    MerklePrototypeTransition MerklePrototypeProtocol::verify_remove(
            const MerklePrototypeApplier & committed,
            std::uint64_t committed_revision,
            const MerklePrototypeUpdate & update)
    {
        MerklePrototypeTransition result;
        if(!validate_common(update, result.diagnostic)) {
            return result;
        }
        if(update.kind != MerklePrototypeUpdateKind::Remove
           || !update.keyframe_cells.empty()
           || !update.delta_operations.empty()
           || !is_zero_hash(update.result_content.digest)
           || !validate_against_committed(
                   committed, committed_revision, update, result.diagnostic)) {
            if(result.diagnostic.empty()) {
                result.diagnostic = "prototype remove envelope is invalid";
            }
            return result;
        }
        result.success = true;
        result.update = update;
        result.removed = true;
        return result;
    }

}// namespace PerceptionMapUpdate

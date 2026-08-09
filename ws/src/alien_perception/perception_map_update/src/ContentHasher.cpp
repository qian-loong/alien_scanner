#include "perception_map_update/ContentHasher.hpp"

#include "CanonicalEncoding.hpp"

#include <openssl/evp.h>

#include <array>
#include <cstring>
#include <memory>
#include <stdexcept>

namespace PerceptionMapUpdate {

    namespace {

        class DigestSink
        {
        public:
            DigestSink()
                    : context_(EVP_MD_CTX_new(), &EVP_MD_CTX_free)
            {
                if(!context_ || EVP_DigestInit_ex(context_.get(), EVP_sha256(), nullptr) != 1) {
                    throw std::runtime_error("failed to initialize SHA-256 context");
                }
            }

            void append(const std::uint8_t * data, std::size_t size)
            {
                if(size == 0U) {
                    return;
                }
                if(size >= kBufferBytes) {
                    flush();
                    digest(data, size);
                    return;
                }
                if(buffered_ + size > kBufferBytes) {
                    flush();
                }
                std::memcpy(buffer_.data() + buffered_, data, size);
                buffered_ += size;
            }

            Hash256 finish()
            {
                flush();
                Hash256 result {};
                unsigned int size = 0U;
                if(EVP_DigestFinal_ex(context_.get(), result.data(), &size) != 1
                   || size != result.size()) {
                    throw std::runtime_error("failed to finalize SHA-256 context");
                }
                return result;
            }

        private:
            // Canonical encoding emits many tiny appends (a few bytes per field, four
            // per cell); batching them keeps the EVP_DigestUpdate call count bounded.
            static constexpr std::size_t kBufferBytes = 64U * 1024U;

            void digest(const std::uint8_t * data, std::size_t size)
            {
                if(EVP_DigestUpdate(context_.get(), data, size) != 1) {
                    throw std::runtime_error("failed to update SHA-256 context");
                }
            }

            void flush()
            {
                if(buffered_ != 0U) {
                    digest(buffer_.data(), buffered_);
                    buffered_ = 0U;
                }
            }

            std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context_;
            std::array<std::uint8_t, kBufferBytes> buffer_ {};
            std::size_t buffered_ = 0U;
        };

        void write_domain(DigestSink & sink, const char * domain)
        {
            Encoding::write_string(sink, domain);
            Encoding::write_u16(sink, kCanonicalEncodingVersion);
            Encoding::write_u8(sink, static_cast<std::uint8_t>(HashAlgorithm::Sha256));
        }

    }// namespace

    Hash256 ContentHasher::geometry_fingerprint(const MapGeometry & geometry)
    {
        DigestSink sink;
        write_domain(sink, "alien-scanner/map-geometry/v1");
        Encoding::write_geometry(sink, geometry);
        return sink.finish();
    }

    Hash256 ContentHasher::content_hash(
            const SourceIdentity & source,
            const Hash256 & geometry_fingerprint,
            const std::vector<CanonicalCell> & cells)
    {
        DigestSink sink;
        write_domain(sink, "alien-scanner/map-content/v1");
        Encoding::write_identity(sink, source);
        Encoding::write_hash(sink, geometry_fingerprint);
        Encoding::write_u64(sink, static_cast<std::uint64_t>(cells.size()));
        for(const auto & cell : cells) {
            Encoding::write_cell(sink, cell);
        }
        return sink.finish();
    }

    Hash256 ContentHasher::update_hash(const MapUpdate & update)
    {
        DigestSink sink;
        write_domain(sink, "alien-scanner/map-update/v1");
        Encoding::write_u16(sink, update.protocol_version);
        Encoding::write_u16(sink, update.canonical_encoding_version);
        Encoding::write_u8(sink, static_cast<std::uint8_t>(update.hash_algorithm));
        Encoding::write_u8(sink, static_cast<std::uint8_t>(update.kind));
        Encoding::write_identity(sink, update.source);
        Encoding::write_geometry(sink, update.geometry);
        Encoding::write_u64(sink, update.base_revision);
        Encoding::write_u64(sink, update.new_revision);
        Encoding::write_u64(sink, update.revision_span);
        Encoding::write_u64(sink, update.observed_coalesced_receipt_count);
        Encoding::write_hash(sink, update.geometry_fingerprint);
        Encoding::write_hash(sink, update.base_content_hash);
        Encoding::write_hash(sink, update.content_hash);
        Encoding::write_provenance(sink, update.latest_commit);
        Encoding::write_u64(sink, update.known_cell_count);
        Encoding::write_u64(sink, update.operation_count);
        Encoding::write_u64(sink, update.canonical_payload_bytes);
        Encoding::write_u64(sink, static_cast<std::uint64_t>(update.payload.size()));
        if(!update.payload.empty()) {
            sink.append(update.payload.data(), update.payload.size());
        }
        return sink.finish();
    }

}// namespace PerceptionMapUpdate

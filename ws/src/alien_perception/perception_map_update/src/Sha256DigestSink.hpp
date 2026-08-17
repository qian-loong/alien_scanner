#ifndef PERCEPTION_MAP_UPDATE_SHA256_DIGEST_SINK_HPP
#define PERCEPTION_MAP_UPDATE_SHA256_DIGEST_SINK_HPP

#include "perception_map_update/MapUpdateTypes.hpp"

#include <openssl/evp.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace PerceptionMapUpdate::Internal {

    // Unbuffered sink for small Merkle node/envelope digests. The flat map
    // hasher keeps its separate large-write buffer because its workload is a
    // single digest with many tiny canonical fields.
    class Sha256DigestSink
    {
    public:
        Sha256DigestSink()
                : context_(EVP_MD_CTX_new(), &EVP_MD_CTX_free)
        {
            if(!context_ || EVP_DigestInit_ex(context_.get(), EVP_sha256(), nullptr) != 1) {
                throw std::runtime_error("failed to initialize SHA-256 context");
            }
        }

        void append(const std::uint8_t * data, std::size_t size)
        {
            if(size > 0U && EVP_DigestUpdate(context_.get(), data, size) != 1) {
                throw std::runtime_error("failed to update SHA-256 context");
            }
        }

        Hash256 finish()
        {
            Hash256 result {};
            unsigned int size = 0U;
            if(EVP_DigestFinal_ex(context_.get(), result.data(), &size) != 1
               || size != result.size()) {
                throw std::runtime_error("failed to finalize SHA-256 context");
            }
            return result;
        }

    private:
        std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context_;
    };

}// namespace PerceptionMapUpdate::Internal

#endif// PERCEPTION_MAP_UPDATE_SHA256_DIGEST_SINK_HPP

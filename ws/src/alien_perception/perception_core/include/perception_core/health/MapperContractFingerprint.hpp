#ifndef PERCEPTION_CORE_HEALTH_MAPPER_CONTRACT_FINGERPRINT_HPP
#define PERCEPTION_CORE_HEALTH_MAPPER_CONTRACT_FINGERPRINT_HPP

#include "perception_core/health/mapper_input_contract.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace Perception {

    struct MapperContractFingerprint {
        static constexpr std::uint32_t kSchemaVersion = 1;

        std::uint32_t schema_version = kSchemaVersion;
        std::string   hex_digest;

        bool is_well_formed() const noexcept;

        static MapperContractFingerprint compute(
                const std::vector<SensorDescriptor> & descriptors,
                const MapperInputContract &           contract);
    };

}// namespace Perception

#endif// PERCEPTION_CORE_HEALTH_MAPPER_CONTRACT_FINGERPRINT_HPP

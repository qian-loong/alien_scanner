#include "perception_core/health/MapperContractFingerprint.hpp"

#include "perception_core/observation/ray_evidence_capability.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace Perception {

    namespace {

        constexpr std::array<std::uint32_t, 64> kSha256RoundConstants {
                0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
                0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
                0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
                0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
                0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
                0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
                0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
                0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
                0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
                0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
                0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
                0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
                0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
                0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
                0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
                0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

        std::uint32_t rotate_right(std::uint32_t value, std::uint32_t count) noexcept
        {
            return (value >> count) | (value << (32U - count));
        }

        std::array<std::uint8_t, 32> sha256(std::vector<std::uint8_t> bytes)
        {
            const std::uint64_t bit_length = static_cast<std::uint64_t>(bytes.size()) * 8U;
            bytes.push_back(0x80U);
            while((bytes.size() % 64U) != 56U) {
                bytes.push_back(0U);
            }
            for(int shift = 56; shift >= 0; shift -= 8) {
                bytes.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xffU));
            }

            std::array<std::uint32_t, 8> state {
                    0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};

            for(std::size_t offset = 0; offset < bytes.size(); offset += 64U) {
                std::array<std::uint32_t, 64> schedule {};
                for(std::size_t index = 0; index < 16U; ++index) {
                    const std::size_t base = offset + index * 4U;
                    schedule[index] = (static_cast<std::uint32_t>(bytes[base]) << 24U)
                                      | (static_cast<std::uint32_t>(bytes[base + 1U]) << 16U)
                                      | (static_cast<std::uint32_t>(bytes[base + 2U]) << 8U)
                                      | static_cast<std::uint32_t>(bytes[base + 3U]);
                }
                for(std::size_t index = 16U; index < schedule.size(); ++index) {
                    const auto s0 = rotate_right(schedule[index - 15U], 7U)
                                    ^ rotate_right(schedule[index - 15U], 18U)
                                    ^ (schedule[index - 15U] >> 3U);
                    const auto s1 = rotate_right(schedule[index - 2U], 17U)
                                    ^ rotate_right(schedule[index - 2U], 19U)
                                    ^ (schedule[index - 2U] >> 10U);
                    schedule[index] = schedule[index - 16U] + s0
                                      + schedule[index - 7U] + s1;
                }

                auto a = state[0];
                auto b = state[1];
                auto c = state[2];
                auto d = state[3];
                auto e = state[4];
                auto f = state[5];
                auto g = state[6];
                auto h = state[7];
                for(std::size_t index = 0; index < schedule.size(); ++index) {
                    const auto sum1 = rotate_right(e, 6U) ^ rotate_right(e, 11U)
                                      ^ rotate_right(e, 25U);
                    const auto choice = (e & f) ^ ((~e) & g);
                    const auto temporary1 = h + sum1 + choice
                                            + kSha256RoundConstants[index] + schedule[index];
                    const auto sum0 = rotate_right(a, 2U) ^ rotate_right(a, 13U)
                                      ^ rotate_right(a, 22U);
                    const auto majority = (a & b) ^ (a & c) ^ (b & c);
                    const auto temporary2 = sum0 + majority;
                    h = g;
                    g = f;
                    f = e;
                    e = d + temporary1;
                    d = c;
                    c = b;
                    b = a;
                    a = temporary1 + temporary2;
                }
                state[0] += a;
                state[1] += b;
                state[2] += c;
                state[3] += d;
                state[4] += e;
                state[5] += f;
                state[6] += g;
                state[7] += h;
            }

            std::array<std::uint8_t, 32> digest {};
            for(std::size_t index = 0; index < state.size(); ++index) {
                digest[index * 4U] = static_cast<std::uint8_t>(state[index] >> 24U);
                digest[index * 4U + 1U] = static_cast<std::uint8_t>(state[index] >> 16U);
                digest[index * 4U + 2U] = static_cast<std::uint8_t>(state[index] >> 8U);
                digest[index * 4U + 3U] = static_cast<std::uint8_t>(state[index]);
            }
            return digest;
        }

        class CanonicalWriter
        {
        public:
            void write_u8(std::uint8_t value) { bytes_.push_back(value); }

            void write_u32(std::uint32_t value)
            {
                for(int shift = 24; shift >= 0; shift -= 8) {
                    bytes_.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
                }
            }

            void write_u64(std::uint64_t value)
            {
                for(int shift = 56; shift >= 0; shift -= 8) {
                    bytes_.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
                }
            }

            void write_i64(std::int64_t value)
            {
                write_u64(static_cast<std::uint64_t>(value));
            }

            void write_bool(bool value) { write_u8(value ? 1U : 0U); }

            void write_string(const std::string & value)
            {
                if(value.size() > std::numeric_limits<std::uint32_t>::max()) {
                    throw std::invalid_argument("contract string is too long");
                }
                write_u32(static_cast<std::uint32_t>(value.size()));
                bytes_.insert(bytes_.end(), value.begin(), value.end());
            }

            void write_double(double value)
            {
                if(!std::isfinite(value)) {
                    throw std::invalid_argument("contract contains a non-finite number");
                }
                if(value == 0.0) {
                    value = 0.0;
                }
                std::uint64_t bits = 0;
                static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
                std::memcpy(&bits, &value, sizeof(bits));
                write_u64(bits);
            }

            const std::vector<std::uint8_t> & bytes() const noexcept { return bytes_; }

        private:
            std::vector<std::uint8_t> bytes_;
        };

        std::uint8_t sensor_type_value(SensorType type)
        {
            switch(type) {
                case SensorType::LIDAR_2D:
                    return 0U;
                case SensorType::LIDAR_3D:
                    return 1U;
            }
            throw std::invalid_argument("contract contains an unknown sensor type");
        }

        std::uint8_t ray_evidence_value(RayEvidenceCapability capability)
        {
            if(!is_valid_ray_evidence(capability)) {
                throw std::invalid_argument("contract contains unknown ray evidence");
            }
            return static_cast<std::uint8_t>(capability);
        }

        std::vector<SensorID> sorted_sensor_ids(
                const std::optional<std::vector<SensorID>> & values)
        {
            auto result = values.value_or(std::vector<SensorID> {});
            std::sort(result.begin(), result.end());
            return result;
        }

        void write_requirement(CanonicalWriter & writer, const SensorRequirement & requirement)
        {
            writer.write_u8(sensor_type_value(requirement.type));
            writer.write_u64(static_cast<std::uint64_t>(requirement.min_count));
            writer.write_u8(ray_evidence_value(requirement.minimum_ray_evidence));
            const auto ids = sorted_sensor_ids(requirement.specific_sensors);
            writer.write_u32(static_cast<std::uint32_t>(ids.size()));
            for(const auto & id : ids) {
                writer.write_string(id.value);
            }
        }

        auto requirement_sort_key(const SensorRequirement & requirement)
        {
            std::string ids;
            for(const auto & id : sorted_sensor_ids(requirement.specific_sensors)) {
                ids.append(id.value);
                ids.push_back('\0');
            }
            return std::make_tuple(
                    sensor_type_value(requirement.type), requirement.min_count,
                    ray_evidence_value(requirement.minimum_ray_evidence), ids);
        }

        void write_requirements(
                CanonicalWriter & writer,
                std::vector<SensorRequirement> requirements)
        {
            std::sort(
                    requirements.begin(), requirements.end(),
                    [](const auto & left, const auto & right) {
                        return requirement_sort_key(left) < requirement_sort_key(right);
                    });
            writer.write_u32(static_cast<std::uint32_t>(requirements.size()));
            for(const auto & requirement : requirements) {
                write_requirement(writer, requirement);
            }
        }

        void write_descriptor(CanonicalWriter & writer, const SensorDescriptor & descriptor)
        {
            if(descriptor.sensor_id.value.empty() || descriptor.frame_id.empty()) {
                throw std::invalid_argument("descriptor identity must not be empty");
            }
            writer.write_string(descriptor.sensor_id.value);
            writer.write_u8(sensor_type_value(descriptor.type));
            writer.write_string(descriptor.frame_id);
            writer.write_double(descriptor.mounting_position.x());
            writer.write_double(descriptor.mounting_position.y());
            writer.write_double(descriptor.mounting_position.z());
            writer.write_double(descriptor.mounting_orientation.w());
            writer.write_double(descriptor.mounting_orientation.x());
            writer.write_double(descriptor.mounting_orientation.y());
            writer.write_double(descriptor.mounting_orientation.z());
            writer.write_double(descriptor.fov.horizontal_min_rad);
            writer.write_double(descriptor.fov.horizontal_max_rad);
            writer.write_double(descriptor.fov.vertical_min_rad);
            writer.write_double(descriptor.fov.vertical_max_rad);
            writer.write_double(descriptor.angular_resolution_rad);
            writer.write_double(descriptor.range_min_m);
            writer.write_double(descriptor.range_max_m);
            writer.write_u8(ray_evidence_value(descriptor.ray_evidence));
        }

        std::string to_hex(const std::array<std::uint8_t, 32> & digest)
        {
            std::ostringstream stream;
            stream << std::hex << std::setfill('0');
            for(const auto byte : digest) {
                stream << std::setw(2) << static_cast<unsigned int>(byte);
            }
            return stream.str();
        }

    }// namespace

    bool MapperContractFingerprint::is_well_formed() const noexcept
    {
        if(schema_version != kSchemaVersion || hex_digest.size() != 64U) {
            return false;
        }
        return std::all_of(
                hex_digest.begin(), hex_digest.end(),
                [](char value) {
                    return (value >= '0' && value <= '9') || (value >= 'a' && value <= 'f');
                });
    }

    MapperContractFingerprint MapperContractFingerprint::compute(
            const std::vector<SensorDescriptor> & descriptors,
            const MapperInputContract &           contract)
    {
        CanonicalWriter writer;
        writer.write_string("alien-scanner/mapper-input-contract");
        writer.write_u32(kSchemaVersion);

        auto sorted_descriptors = descriptors;
        std::sort(
                sorted_descriptors.begin(), sorted_descriptors.end(),
                [](const auto & left, const auto & right) {
                    return left.sensor_id < right.sensor_id;
                });
        writer.write_u32(static_cast<std::uint32_t>(sorted_descriptors.size()));
        for(const auto & descriptor : sorted_descriptors) {
            write_descriptor(writer, descriptor);
        }

        write_requirements(writer, contract.minimum_viable);
        writer.write_u32(static_cast<std::uint32_t>(contract.degraded_combinations.size()));
        for(const auto & combination : contract.degraded_combinations) {
            write_requirements(writer, combination.requirements);
        }
        writer.write_bool(contract.requires_pose);
        writer.write_i64(contract.pose_freshness_threshold.nanoseconds);
        writer.write_bool(contract.expected_pose_frame.has_value());
        if(contract.expected_pose_frame.has_value()) {
            writer.write_string(contract.expected_pose_frame.value());
        }
        writer.write_double(contract.minimum_pose_quality);
        writer.write_u64(static_cast<std::uint64_t>(contract.recovery_stability_samples));

        return MapperContractFingerprint {kSchemaVersion, to_hex(sha256(writer.bytes()))};
    }

}// namespace Perception

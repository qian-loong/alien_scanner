#include "perception_local_map/SensorExtrinsicRegistry.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace PerceptionLocalMap {

    namespace {

        bool is_finite(const Eigen::Vector3d & value)
        {
            return value.allFinite();
        }

        bool is_finite(const Eigen::Quaterniond & value)
        {
            return std::isfinite(value.w()) && std::isfinite(value.x())
                   && std::isfinite(value.y()) && std::isfinite(value.z());
        }

        Eigen::Isometry3d make_transform(
                const Eigen::Vector3d & translation,
                const Eigen::Quaterniond & orientation)
        {
            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            result.translation() = translation;
            result.linear() = orientation.toRotationMatrix();
            return result;
        }

    }// namespace

    SensorExtrinsicRegistry::SensorExtrinsicRegistry(
            std::vector<Perception::SensorDescriptor> descriptors,
            double position_tolerance_m,
            double orientation_tolerance_rad)
        : position_tolerance_m_(position_tolerance_m)
        , orientation_tolerance_rad_(orientation_tolerance_rad)
    {
        if(!std::isfinite(position_tolerance_m_) || position_tolerance_m_ < 0.0
           || !std::isfinite(orientation_tolerance_rad_) || orientation_tolerance_rad_ < 0.0) {
            throw std::invalid_argument("extrinsic tolerances must be finite and non-negative");
        }
        for(auto & descriptor : descriptors) {
            if(descriptor.sensor_id.value.empty() || descriptor.frame_id.empty()
               || !descriptors_.emplace(descriptor.sensor_id.value, descriptor).second) {
                throw std::invalid_argument("extrinsic registry requires unique sensor identities");
            }
        }
    }

    ExtrinsicResult SensorExtrinsicRegistry::validate_and_freeze(
            const SensorExtrinsicSample & sample)
    {
        const auto descriptor = descriptors_.find(sample.sensor_id.value);
        if(descriptor == descriptors_.end()) {
            return {ExtrinsicStatus::Rejected, Eigen::Isometry3d::Identity(), "unknown sensor"};
        }
        if(sample.frame_id != descriptor->second.frame_id) {
            return {ExtrinsicStatus::Rejected, Eigen::Isometry3d::Identity(), "sensor frame mismatch"};
        }
        if(sample.sensor_session.boot_time_ns == 0 || !is_finite(sample.translation)
           || !is_finite(sample.orientation) || sample.orientation.norm() < 1e-12) {
            return {ExtrinsicStatus::Rejected, Eigen::Isometry3d::Identity(), "invalid extrinsic sample"};
        }

        const Eigen::Quaterniond normalized = sample.orientation.normalized();
        const auto descriptor_orientation = descriptor->second.mounting_orientation.normalized();
        if(!is_finite(descriptor->second.mounting_position)
           || !is_finite(descriptor_orientation)
           || (descriptor->second.mounting_position - sample.translation).norm()
                      > position_tolerance_m_
           || descriptor_orientation.angularDistance(normalized)
                      > orientation_tolerance_rad_) {
            return {ExtrinsicStatus::Rejected, Eigen::Isometry3d::Identity(),
                    "TF sample does not match frozen descriptor mounting"};
        }
        const auto existing = frozen_.find(sample.sensor_id.value);
        if(existing == frozen_.end()) {
            frozen_.emplace(
                    sample.sensor_id.value,
                    FrozenExtrinsic {sample.frame_id, sample.translation, normalized});
            return {ExtrinsicStatus::Accepted,
                    make_transform(sample.translation, normalized), {}};
        }
        if(existing->second.frame_id != sample.frame_id
           || (existing->second.translation - sample.translation).norm() > position_tolerance_m_
           || existing->second.orientation.angularDistance(normalized)
                      > orientation_tolerance_rad_) {
            return {ExtrinsicStatus::Rejected, Eigen::Isometry3d::Identity(), "frozen extrinsic drift"};
        }
        return {ExtrinsicStatus::Accepted,
                make_transform(existing->second.translation, existing->second.orientation), {}};
    }

    std::optional<Eigen::Isometry3d> SensorExtrinsicRegistry::lookup(
            const Perception::SensorID & sensor_id) const
    {
        const auto value = frozen_.find(sensor_id.value);
        if(value == frozen_.end()) {
            return std::nullopt;
        }
        return make_transform(value->second.translation, value->second.orientation);
    }

    void SensorExtrinsicRegistry::clear()
    {
        frozen_.clear();
    }

}// namespace PerceptionLocalMap

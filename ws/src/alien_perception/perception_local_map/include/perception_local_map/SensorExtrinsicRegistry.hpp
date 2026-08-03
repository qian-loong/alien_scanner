#ifndef PERCEPTION_LOCAL_MAP_SENSOR_EXTRINSIC_REGISTRY_HPP
#define PERCEPTION_LOCAL_MAP_SENSOR_EXTRINSIC_REGISTRY_HPP

#include "perception_core/observation/sensor_descriptor.hpp"
#include "perception_core/types/identity.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace PerceptionLocalMap {

    struct SensorExtrinsicSample {
        Perception::SensorID  sensor_id;
        Perception::SessionID sensor_session {0, 0};
        std::string           frame_id;
        Eigen::Vector3d       translation = Eigen::Vector3d::Zero();
        Eigen::Quaterniond    orientation = Eigen::Quaterniond::Identity();
    };

    enum class ExtrinsicStatus
    {
        Accepted,
        Rejected
    };

    struct ExtrinsicResult {
        ExtrinsicStatus            status = ExtrinsicStatus::Rejected;
        Eigen::Isometry3d          body_from_sensor = Eigen::Isometry3d::Identity();
        std::string                diagnostic;
    };

    class SensorExtrinsicRegistry
    {
    public:
        SensorExtrinsicRegistry(
                std::vector<Perception::SensorDescriptor> descriptors,
                double position_tolerance_m,
                double orientation_tolerance_rad);

        ExtrinsicResult validate_and_freeze(const SensorExtrinsicSample & sample);
        std::optional<Eigen::Isometry3d> lookup(const Perception::SensorID & sensor_id) const;
        void clear();

    private:
        struct FrozenExtrinsic {
            std::string        frame_id;
            Eigen::Vector3d    translation;
            Eigen::Quaterniond orientation;
        };

        std::unordered_map<std::string, Perception::SensorDescriptor> descriptors_;
        std::unordered_map<std::string, FrozenExtrinsic>              frozen_;
        double position_tolerance_m_;
        double orientation_tolerance_rad_;
    };

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_SENSOR_EXTRINSIC_REGISTRY_HPP

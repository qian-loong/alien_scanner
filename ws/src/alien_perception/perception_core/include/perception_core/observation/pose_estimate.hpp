#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_POSE_ESTIMATE_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_POSE_ESTIMATE_HPP

#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>

namespace Perception {

    /// Pose estimate from external source
    struct PoseEstimate {
        SourceID    source_id;
        SessionID   session_id;
        std::string frame_id;    // Reference frame
        std::string clock_domain;// Clock domain identifier
        Timestamp   stamp;

        // Pose
        Eigen::Vector3d    position;
        Eigen::Quaterniond orientation;

        // Quality metrics
        using Matrix6d = Eigen::Matrix<double, 6, 6>;
        std::optional<Matrix6d> covariance;// 6x6 covariance matrix (position + orientation)
        double                  quality;   // Quality score [0, 1]

        // Freshness
        Duration freshness;// Time since last update

        // Reset epoch (relocation/coordinate system change counter)
        uint64_t reset_epoch;

        // Check if fresh (freshness < threshold)
        bool is_fresh(Duration threshold) const
        {
            return freshness < threshold;
        }
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_POSE_ESTIMATE_HPP

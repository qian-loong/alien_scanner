#ifndef SWARM_DATA_PLANE_ROS_VEHICLE_IDENTITY_CONVERSIONS_HPP
#define SWARM_DATA_PLANE_ROS_VEHICLE_IDENTITY_CONVERSIONS_HPP

#include "swarm_data_interfaces/msg/vehicle_identity.hpp"
#include "swarm_data_plane/TopologyTypes.hpp"

namespace SwarmDataPlane::Ros::Detail {

    inline void encode_vehicle_identity(
            const VehicleIdentity & identity,
            swarm_data_interfaces::msg::VehicleIdentity & message)
    {
        message.fleet_id = identity.fleet_id;
        message.vehicle_id = identity.vehicle_id;
        message.session_boot_time_ns = identity.session.boot_time_ns;
        message.session_random_suffix = identity.session.random_suffix;
    }

    inline VehicleIdentity decode_vehicle_identity(
            const swarm_data_interfaces::msg::VehicleIdentity & message)
    {
        return {
                message.fleet_id,
                message.vehicle_id,
                {message.session_boot_time_ns, message.session_random_suffix}};
    }

}// namespace SwarmDataPlane::Ros::Detail

#endif// SWARM_DATA_PLANE_ROS_VEHICLE_IDENTITY_CONVERSIONS_HPP

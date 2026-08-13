#ifndef SWARM_DATA_PLANE_ROUTED_MAP_VALIDATOR_HPP
#define SWARM_DATA_PLANE_ROUTED_MAP_VALIDATOR_HPP

#include "swarm_data_plane/DataPlaneTypes.hpp"

namespace SwarmDataPlane {

    EnvelopeValidationResult validate_routed_map_update(
            const RoutedMapUpdate & message,
            const DataPlaneLimits & limits = {});

    ForwardResult forward_routed_map_update(
            const RoutedMapUpdate & message,
            std::uint64_t local_elapsed_ns,
            const DataPlaneLimits & limits = {});

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_ROUTED_MAP_VALIDATOR_HPP

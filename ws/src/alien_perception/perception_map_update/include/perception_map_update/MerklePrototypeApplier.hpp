#ifndef PERCEPTION_MAP_UPDATE_MERKLE_PROTOTYPE_APPLIER_HPP
#define PERCEPTION_MAP_UPDATE_MERKLE_PROTOTYPE_APPLIER_HPP

#include "perception_map_update/MerkleMapState.hpp"

namespace PerceptionMapUpdate {

    // Compatibility aliases for the isolated C4.2 oracle and benchmark.
    // Production and test code share the single MerkleMapState implementation.
    using MerklePrototypeTimings = MerkleMapTimings;
    using MerklePrototypeResult = MerkleMapStateResult;
    using MerklePrototypeApplier = MerkleMapState;

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MERKLE_PROTOTYPE_APPLIER_HPP

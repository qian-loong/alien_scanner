#ifndef PERCEPTION_MAP_UPDATE_CONTENT_HASHER_HPP
#define PERCEPTION_MAP_UPDATE_CONTENT_HASHER_HPP

#include "perception_map_update/MapUpdateTypes.hpp"

namespace PerceptionMapUpdate {

    class ContentHasher
    {
    public:
        static Hash256 geometry_fingerprint(const MapGeometry & geometry);
        static Hash256 content_hash(
                const SourceIdentity & source,
                const Hash256 & geometry_fingerprint,
                const std::vector<CanonicalCell> & cells);
        static Hash256 update_hash(const MapUpdate & update);
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_CONTENT_HASHER_HPP

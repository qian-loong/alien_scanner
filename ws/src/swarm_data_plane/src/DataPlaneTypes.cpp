#include "swarm_data_plane/DataPlaneTypes.hpp"

namespace SwarmDataPlane {

    bool ProducerIdentity::operator==(const ProducerIdentity & other) const noexcept
    {
        return producer_id == other.producer_id && session == other.session;
    }

    bool ProducerIdentity::operator!=(const ProducerIdentity & other) const noexcept
    {
        return !(*this == other);
    }

    bool ProducerIdentity::operator<(const ProducerIdentity & other) const noexcept
    {
        if(producer_id != other.producer_id) {
            return producer_id < other.producer_id;
        }
        return session < other.session;
    }

}// namespace SwarmDataPlane

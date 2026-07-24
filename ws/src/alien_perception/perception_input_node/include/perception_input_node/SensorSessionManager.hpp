#ifndef PERCEPTION_INPUT_NODE_INCLUDE_PERCEPTION_INPUT_NODE_SENSOR_SESSION_MANAGER_HPP
#define PERCEPTION_INPUT_NODE_INCLUDE_PERCEPTION_INPUT_NODE_SENSOR_SESSION_MANAGER_HPP

#include "perception_core/observation/sensor_descriptor.hpp"
#include "perception_core/types/identity.hpp"
#include <string>
#include <unordered_map>
#include <vector>

namespace Perception::Input {

    class SensorSessionManager
    {
    public:
        struct RegistrationResult {
            bool        accepted;
            std::string diagnostic;
        };

        explicit SensorSessionManager(SessionID session_id);

        RegistrationResult register_sensor(const SensorDescriptor & descriptor);
        bool               can_reconnect(
                              const SensorDescriptor & descriptor,
                              std::string *            diagnostic = nullptr) const;

        void freeze() { frozen_ = true; }
        bool is_frozen() const { return frozen_; }

        const SessionID &             session_id() const { return session_id_; }
        const SensorDescriptor *      find_descriptor(const SensorID & sensor_id) const;
        std::vector<SensorDescriptor> descriptors() const;

    private:
        SessionID                                      session_id_;
        bool                                           frozen_ = false;
        std::unordered_map<SensorID, SensorDescriptor> descriptors_;

        static std::string describe_descriptor(const SensorDescriptor & descriptor);
        std::string        inventory_summary() const;
    };

}// namespace Perception::Input

#endif// PERCEPTION_INPUT_NODE_INCLUDE_PERCEPTION_INPUT_NODE_SENSOR_SESSION_MANAGER_HPP

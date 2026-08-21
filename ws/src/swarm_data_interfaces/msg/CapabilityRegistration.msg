uint16 CURRENT_PROTOCOL_VERSION = 1

uint8 CAPABILITY_EXPLORATION = 1
uint8 CAPABILITY_RELAY_FORWARDING = 2
uint8 CAPABILITY_MAP_AGGREGATION = 3

uint16 protocol_version
swarm_data_interfaces/VehicleIdentity identity
uint64 registration_generation
uint8[<=8] declared_capabilities

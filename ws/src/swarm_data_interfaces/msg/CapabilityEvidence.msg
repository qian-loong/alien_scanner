uint16 CURRENT_PROTOCOL_VERSION = 1

uint8 CAPABILITY_EXPLORATION = 1
uint8 CAPABILITY_RELAY_FORWARDING = 2
uint8 CAPABILITY_MAP_AGGREGATION = 3

uint8 VEHICLE_HEALTH_UNKNOWN = 1
uint8 VEHICLE_HEALTH_HEALTHY = 2
uint8 VEHICLE_HEALTH_DEGRADED = 3
uint8 VEHICLE_HEALTH_LOW_POWER = 4
uint8 VEHICLE_HEALTH_FAILSAFE = 5

uint8 RESOURCE_HEALTH_UNKNOWN = 1
uint8 RESOURCE_HEALTH_HEALTHY = 2
uint8 RESOURCE_HEALTH_COMPUTE_OVER_BUDGET = 3
uint8 RESOURCE_HEALTH_MEMORY_OVER_BUDGET = 4
uint8 RESOURCE_HEALTH_LINK_DEGRADED = 5

uint16 protocol_version
swarm_data_interfaces/VehicleIdentity identity
uint64 evidence_revision
uint8[<=8] effective_capabilities
uint8 vehicle_health
uint8 resource_health
swarm_data_interfaces/ServiceHealthEvidence[<=2] service_health

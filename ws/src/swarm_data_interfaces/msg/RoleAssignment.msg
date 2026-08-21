uint8 PRIMARY_EXPLORER = 1
uint8 PRIMARY_RELAY = 2
uint8 PRIMARY_EDGE_AGGREGATOR = 3
uint8 PRIMARY_RESERVE = 4

uint8 LIFECYCLE_ACTIVE = 1
uint8 LIFECYCLE_DRAINING = 2

swarm_data_interfaces/VehicleIdentity identity
uint64 capability_revision
uint8 primary_role
uint8 lifecycle
swarm_data_interfaces/ServiceAssignment[<=2] services

uint16 CURRENT_PROTOCOL_VERSION = 1

uint16 protocol_version
string<=128 fleet_id
uint64 topology_epoch
uint64 role_epoch
swarm_data_interfaces/RoleAssignment[<=64] assignments

uint16 CURRENT_PROTOCOL_VERSION = 1

uint8 STATE_PREPARED = 1
uint8 STATE_QUIESCING = 2
uint8 STATE_HANDOFF_READY = 3
uint8 STATE_COMMITTED = 4
uint8 STATE_ROLLED_BACK = 5

uint16 protocol_version
string<=128 transition_id
uint8 state
uint64 base_role_epoch
uint64 topology_epoch
swarm_data_interfaces/RoleAssignment[<=64] target_assignments
swarm_data_interfaces/VehicleIdentity[<=64] changed_members
swarm_data_interfaces/RoleTransitionAck[<=128] acknowledgements

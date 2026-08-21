uint16 CURRENT_PROTOCOL_VERSION = 1

uint16 protocol_version
string<=128 fleet_id
uint64 topology_epoch
swarm_data_interfaces/MemberRecord[<=64] members
swarm_data_interfaces/LinkDescriptor[<=256] links
swarm_data_interfaces/GraphEdge[<=512] edges
swarm_data_interfaces/RouteDescriptor[<=256] routes

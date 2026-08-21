uint8 GRAPH_CONTROL = 2
uint8 GRAPH_MAP = 3

string<=128 route_id
uint8 graph
swarm_data_interfaces/VehicleIdentity source
swarm_data_interfaces/VehicleIdentity target
uint64 topology_epoch
uint64 route_epoch
uint16 ttl_hops
uint64 validity_budget_ns
swarm_data_interfaces/RouteHop[<=64] hops

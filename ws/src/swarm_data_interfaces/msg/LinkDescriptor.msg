uint8 HEALTH_UNKNOWN = 1
uint8 HEALTH_UP = 2
uint8 HEALTH_DEGRADED = 3
uint8 HEALTH_DOWN = 4

string<=128 link_id
uint64 link_epoch
swarm_data_interfaces/VehicleIdentity source
swarm_data_interfaces/VehicleIdentity target
uint8 health
uint64 latency_budget_ns
uint64 bandwidth_budget_bps
uint32 loss_budget_ppm

uint8 STATE_ABSENT = 1
uint8 STATE_JOINING = 2
uint8 STATE_RESYNCING = 3
uint8 STATE_READY = 4
uint8 STATE_DRAINING = 5
uint8 STATE_LOST = 6
uint8 STATE_QUARANTINED = 7

uint8 AVAILABILITY_REMOVED = 1
uint8 AVAILABILITY_LIVE = 2
uint8 AVAILABILITY_FROZEN = 3

swarm_data_interfaces/VehicleIdentity identity
uint64 registration_generation
swarm_data_interfaces/ComponentRegistration[<=32] components
swarm_data_interfaces/SensorDescriptorIdentity[<=32] sensors
uint8 state
uint8 availability
bool link_ready
bool clock_ready
bool alignment_ready
bool map_ready

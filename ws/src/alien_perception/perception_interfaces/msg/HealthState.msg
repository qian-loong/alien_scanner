# HealthState message
std_msgs/Header header

# Health state enumeration
uint8 STATE_HEALTHY = 0
uint8 STATE_DEGRADED = 1
uint8 STATE_UNAVAILABLE = 2

uint8 state
string degradation_reason

# Capability set
bool has_2d_lidar
bool has_3d_lidar
bool has_fresh_pose
uint32 active_sensor_count

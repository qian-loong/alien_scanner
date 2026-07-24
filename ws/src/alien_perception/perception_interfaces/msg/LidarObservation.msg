# LidarObservation message
std_msgs/Header header

# Identity
string sensor_id
uint64 session_boot_time_ns
uint32 session_random_suffix
string clock_domain

# Data type indicator
uint8 DATA_TYPE_SCAN_2D = 0
uint8 DATA_TYPE_CLOUD_3D = 1
uint8 data_type

# 2D scan data (used if data_type == SCAN_2D)
float64 angle_min
float64 angle_max
float64 angle_increment
float64 range_min
float64 range_max
float32[] ranges
float32[] intensities

# 3D cloud data (used if data_type == CLOUD_3D)
geometry_msgs/Point[] points
float32[] point_intensities

# PoseEstimate message
std_msgs/Header header

# Source identity
string source_id
uint64 session_boot_time_ns
uint32 session_random_suffix
string clock_domain

# Pose
geometry_msgs/Pose pose

# Quality metrics
float64[36] covariance  # 6x6 covariance matrix
float64 quality  # Quality score [0, 1]

# Freshness (nanoseconds since last update)
int64 freshness_ns

# Reset epoch
uint64 reset_epoch

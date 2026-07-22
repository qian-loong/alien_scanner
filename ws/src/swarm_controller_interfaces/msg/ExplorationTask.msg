uint8 MODE_LOCAL_FALLBACK=0
uint8 MODE_ASSIGNED=1
uint8 MODE_STANDBY=2

std_msgs/Header header
uint64 allocator_epoch
uint64 revision
uint64 task_id
uint8 mode
geometry_msgs/Pose target
builtin_interfaces/Duration lease

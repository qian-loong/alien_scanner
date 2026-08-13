uint8 STATUS_DELIVERED = 1
uint8 STATUS_DUPLICATE = 2
uint8 STATUS_REJECTED = 3
uint8 STATUS_EXPIRED = 4

uint16 protocol_version
string<=128 message_id
string<=128 receiver_id
uint64 receiver_session_boot_time_ns
uint32 receiver_session_random_suffix
uint8 status
bool resync_required
string<=128 correlation_id
string<=128 source_vehicle_id
uint64 source_mapper_session_boot_time_ns
uint32 source_mapper_session_random_suffix
uint64 source_map_epoch
uint64 source_revision
uint64 receive_monotonic_ns
string<=256 diagnostic

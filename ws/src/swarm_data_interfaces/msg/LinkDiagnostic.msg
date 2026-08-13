uint8 EVENT_DELIVERED = 1
uint8 EVENT_DROPPED = 2
uint8 EVENT_REJECTED = 3
uint8 EVENT_EXPIRED = 4
uint8 EVENT_LINK_DOWN = 5
uint8 EVENT_LINK_UP = 6

uint16 protocol_version
string<=128 message_id
string<=128 endpoint_id
uint64 endpoint_session_boot_time_ns
uint32 endpoint_session_random_suffix
uint8 event
uint64 route_epoch
uint16 hop_count
uint64 local_receive_monotonic_ns
uint64 local_send_monotonic_ns
uint64 queue_depth
uint64 queued_bytes
string<=64 fault_reason
string<=256 diagnostic

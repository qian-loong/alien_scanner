uint16 CURRENT_PROTOCOL_VERSION = 1

uint8 PRIORITY_RESYNC = 1
uint8 PRIORITY_MAP_KEYFRAME = 2
uint8 PRIORITY_MAP_DELTA = 3
uint8 PRIORITY_SUMMARY = 4
uint8 PRIORITY_DIAGNOSTIC = 5

uint16 protocol_version
string<=128 message_id
string<=128 producer_id
uint64 producer_session_boot_time_ns
uint32 producer_session_random_suffix
uint64 sequence
string<=128 correlation_id
uint8 priority

string<=64 origin_clock_domain
uint64 origin_clock_session_boot_time_ns
uint32 origin_clock_session_random_suffix
uint64 origin_time_ns
uint64 validity_budget_ns
uint64 accumulated_forwarding_ns

uint64 route_epoch
uint16 hop_count
uint16 ttl_hops

uint64 payload_bytes
uint8[32] payload_hash
perception_interfaces/MapUpdate map_update

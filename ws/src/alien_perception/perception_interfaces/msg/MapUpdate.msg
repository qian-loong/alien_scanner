# Versioned source-local occupancy state update. Header stamp is transport time only.
std_msgs/Header header

uint8 HASH_SHA256 = 1

uint8 KIND_KEYFRAME = 1
uint8 KIND_DELTA = 2
uint8 KIND_SUMMARY = 3
uint8 KIND_REMOVE = 4

uint16 protocol_version
uint16 canonical_encoding_version
uint8 hash_algorithm
perception_interfaces/ContentIdentityDescriptor content_identity
uint8 update_kind

string vehicle_id
uint64 mapper_session_boot_time_ns
uint32 mapper_session_random_suffix
uint64 map_epoch

string source_map_frame
float64 resolution_m
geometry_msgs/Point lattice_origin

uint64 base_revision
uint64 new_revision
uint64 revision_span
uint64 observed_coalesced_receipt_count

uint8[32] geometry_fingerprint
uint8[32] base_content_hash
uint8[32] content_hash
uint8[32] update_hash

string latest_sensor_id
uint64 latest_sensor_session_boot_time_ns
uint32 latest_sensor_session_random_suffix
builtin_interfaces/Time latest_observation_stamp
string latest_observation_clock_domain
uint32 latest_commit_changed_cell_count

uint64 known_cell_count
uint64 operation_count
uint64 canonical_payload_bytes
string correlation_id
uint8[] payload

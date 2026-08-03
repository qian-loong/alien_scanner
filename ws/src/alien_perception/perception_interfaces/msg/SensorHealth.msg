# SensorHealth message
std_msgs/Header header

# Sensor identity
string sensor_id

# Health status enumeration
uint8 STATUS_ACTIVE = 0
uint8 STATUS_STALE = 1
uint8 STATUS_LOST = 2

uint8 status

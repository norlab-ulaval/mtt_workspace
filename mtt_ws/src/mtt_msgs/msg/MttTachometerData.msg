# MTT-154 Tachometer Data Message (CANBus_Specification.md v1.1)
# Contains complete telemetry data from vehicle CAN 0x2FF

# Standard ROS header
std_msgs/Header header

# Temperature sensors in Celsius
float64 main_sensor_temp_a
float64 main_sensor_temp_b

# Raw tachometer values (MSB-first from CAN)
uint16 tachometer_instant     # Instantaneous speed in raw encoder ticks per second (RPS)
uint32 tachometer_cumulative  # Cumulative distance in raw encoder ticks

# Calculated speed values
float64 speed_ms    # Speed in meters per second
float64 speed_kmh   # Speed in kilometers per hour

# Calculated distance
float64 distance_km # Cumulative distance in kilometers

# Vehicle direction state
string direction    # Current direction: "Forward" or "Reverse"

# Articulation joint angle for steering odometry
float64 articulation_angle_rad    # Current articulation joint angle in radians

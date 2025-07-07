# MTT Tachometer Data Message
# Contains speed, distance, and temperature information from vehicle

# Speed in km/h calculated from tachometer instant reading
float64 speed_kmh

# Cumulative distance in kilometers
float64 distance_km

# Temperature sensors in Celsius
float64 temperature_a
float64 temperature_b

# Raw tachometer values
uint16 tachimeter_instant
uint32 tachimeter_cumulative

# Timestamp when data was received
builtin_interfaces/Time stamp

### Kuon kuon_control/Units

# Units supported by the Kuon

uint8   enum   # the units enum value

### units enumeration
uint8 UNITS_UNDEF     = 0     # units undefined (maybe defaulted)
uint8 UNITS_RAW       = 1     # raw native sensor/actuator/robot units

# relative
uint8 UNITS_PERCENT   = 2     # percent of maximum [-100.0,100.0] or [0.0,100.0]
uint8 UNITS_NORM      = 3     # normalize [-1.0,1.0] or [0.0,1.0]

# position
uint8 UNITS_M         = 4     # meters

# velocity (linear and angular)
uint8 UNITS_M_PER_S   = 5     # meters/second
uint8 UNITS_RAD_PER_S = 6     # radians/second
uint8 UNITS_RPM       = 7     # revolutions/minute


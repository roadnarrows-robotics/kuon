### Kuon kuon_control/JointStateExtended message

# Joints in the case of Kuon are the wheels (4 joints) or the robot center of 
# rotation (1 joint), depending on context.

# Base sensor_msgs/JointState
Header    header        # standard header

string[]  name          # motor name
float64[] position      # robot/wheel position (radians)
float64[] velocity      # robot/wheel velocity w (radians/second)
float64[] effort        # motor torque T = N-m (newton-meters)

# Kuon extensions
float64[] odometer      # robot/wheel odometer m (meters)
int32[]   encoder       # motor encoder position (ticks)
float64[] velocity_mps  # robot/wheel velocity v (meters/second)
int32[]   speed         # raw speed (unitless)
float64[] power_elec    # motor input electrical power Pe (watts)
float64[] power_mech    # motor output mechanical power Pm (watts)
float32[] brake         # motor brake setting (scale 0-1)
float32[] slew          # motor power slew setting (scale 0-1)

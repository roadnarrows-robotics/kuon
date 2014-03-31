### Kuon kuon_control/JointStateExtended message

# Joints in the case of Kuon are motors.

# Base sensor_msgs/JointState
Header    header              # standard header

string[]        name          # motor name
float64[]       position      # motor position (radians)
float64[]       velocity      # motor velocity w (radians/second)
float64[]       effort        # motor torque T = N-m (newton-meters)

# Kuon extensions
float64[]       odometer      # motor odometer m (meters)
int32[]         encoder       # motor encoder position (ticks)
float64[]       velocity_mps  # motor velocity v (meters/second)
float64[]       power_elec    # motor input electrical power Pe (watts)
float64[]       power_mech    # motor output mechanical power Pm (watts)

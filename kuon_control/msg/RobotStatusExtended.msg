### Kuon kuon_control/RobotStatusExtended message

# Base industrial_msgs/RobotStatus
Header    header    # standard header

industrial_msgs/RobotMode mode              # robot operation mode: AUTO MANUAL
industrial_msgs/TriState  e_stopped         # emergency stopped: TRUE FALSE
industrial_msgs/TriState  drives_powered    # servos powered: TRUE FALSE
industrial_msgs/TriState  motion_possible   # motion possbile: TRUE FALSE
industrial_msgs/TriState  in_motion         # in motion: TRUE FALSE
industrial_msgs/TriState  in_error          # in error condition: TRUE FALSE
int32 error_code                            # kuon error code

# Kuon extensions
float32       governor                      # 0 - 1 of maximum speed
float32       battery                       # battery estimated energy (Wh)
MotorHealth[] motor_health                  # motor health

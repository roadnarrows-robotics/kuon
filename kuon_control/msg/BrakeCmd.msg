### Kuon kuon_control/BrakeCmd

# Motor braking is either rheostatic, if the generated electrical power during
# braking is dissipated as heat, and regenerative (future Kuon designs).
# In the current implementation, braking applies to all motors.

float32 brake     # motor dynamic braking intensity [0.0 - 1.0]

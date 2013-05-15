# ArmMovement.msg
# [ml] Message type for controlling the arm and reporting arm status.

# Set true to pick a mode. One must be true, one must be false.
bool position
bool velocity

# Each of the following has a range of [-1,1].
# 1 => {max_left, max_up, max_open}, -1 => {max_right, max_down, max_close}. 
float64 pan_motor_position
float64 tilt_motor_position
float64 cable_motor_position

# Each of the following has a range of [-1,1]
# positive => {left, up, open}, negative => {right, down, close}.
float64 pan_motor_velocity
float64 tilt_motor_velocity
float64 cable_motor_velocity

# These will be set true if the motor has hit its maximum or minimum position.
bool pan_at_max
bool tilt_at_max
bool cable_at_max

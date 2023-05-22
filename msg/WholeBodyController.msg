# This message describes the whole-body state of controller.
#
# A whole-body controller state is defined by the desired, actual, and
# error of the whole-body state of the system. The description of
# every element is defined in their respective messages.
#
# The header specifies the time at which the trajectory is published.


std_msgs/Header header

# Desired whole-body state
whole_body_state_msgs/WholeBodyState desired

# Actual whole-body state
whole_body_state_msgs/WholeBodyState actual

# Error whole-body state
whole_body_state_msgs/WholeBodyState error

# Command state
whole_body_state_msgs/JointCommand[] command

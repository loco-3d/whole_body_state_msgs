# This message describes a whole-body trajectory.
#
# A whole-body trajectory is defined by the actual state of the system,
# and a sequence of states.
# The description of every element is defined in their respective
# messages. This state is described in respective time.
#
# The header specifies the time at which the trajectory is published.


Header header

# The actual whole-body state
state_msgs/WholeBodyState actual

# The whole-body trajectory
state_msgs/WholeBodyState[] trajectory

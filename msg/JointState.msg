# This message describes the state of an actuated joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the joint name
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s),
#  * the acceleration of the joint (rad/s^2 or m/s^2), and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name


# Name of the joint
string name

# States of the joint
float64 position
float64 velocity
float64 acceleration
float64 effort

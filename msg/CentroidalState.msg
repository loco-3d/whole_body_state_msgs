# This message describes the states of a centroidal state.
#
# The centroidal state is defined by:
#  * the position, velocity and acceleration of the CoM,
#  * the base orientation and angular velocity, and
#  * the linear and angular momentum and their rates.
# where each quantity is expressed in the world frame.

# Center of mass
geometry_msgs/Vector3 com_position
geometry_msgs/Vector3 com_velocity

# Base orientation
geometry_msgs/Quaternion base_orientation
geometry_msgs/Vector3 base_angular_velocity

# Linear and angular momentum
geometry_msgs/Twist momenta
geometry_msgs/Twist momenta_rate

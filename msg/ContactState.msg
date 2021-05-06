# This message describes the state of contact or end-effector body.
#
# The contact state is expressed in the world frame. A contact state is
# defined by:
#  * type of contact
#  * status of the contact
#  * the frame name
#  * the pose of the contact,
#  * the velocity of the contact,
#  * the wrench of the contact
#  * the normal vector that defines the surface
#  * the friction coefficient of the surface
#  * its classified state, if available

# Type of contact identifiers
uint8 LOCOMOTION   = 0
uint8 MANIPULATION = 1

# Type of contact states
uint8 UNKNOWN  = 0  # 0 because it's the default for int8
uint8 INACTIVE = 1
uint8 ACTIVE   = 2
uint8 SLIPPING = 3

# Name of the contact body
string name

# Type of contact
uint8 type

# Contact status (e.g. from a contact state estimator)
uint8 status

# State of the contact body
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Wrench wrench
geometry_msgs/Vector3 surface_normal
float64 friction_coefficient

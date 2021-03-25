# This message describes the state of contact or end-effector body.
#
# The contact state is expressed in the world frame. A contact state is
# defined by:
#  * type of contact
#  * the frame name
#  * the pose of the contact,
#  * the velocity of the contact,
#  * the wrench of the contact
#  * the normal vector that defines the surface
#  * the friction coefficient of the surface
#  * its classified state, if available

# Type of contact identifiers
uint8 locomotion=0
uint8 manipulation=1

# Name of the contact body
string name
uint8 type

# Type of contact states
int8 UNKNOWN  = 0  # 0 because it's the default for int8
int8 ACTIVE   = 1
int8 INACTIVE = 2
int8 SLIPPING = 3

# Contact state (e.g. from a contact state estimator)
int8 contact_state

# State of the contact body
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Wrench wrench
geometry_msgs/Vector3 surface_normal
float64 friction_coefficient

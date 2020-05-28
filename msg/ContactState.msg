# This message describes the state of contact or end-effector body.
#
# The contact state is expressed in the world frame. A contact state is
# defined by:
#  * the frame name
#  * the pose of the contact,
#  * the velocity of the contact,
#  * the wrench of the contact


# Name of the contact body
string name

# State of the contact body
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Wrench wrench

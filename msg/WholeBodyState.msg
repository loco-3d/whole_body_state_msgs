# This message describes the whole-body state of rigid body system.
#
# A whole-body state is defined by the centroidal, joints and contact states.
# The description of every element is defined in their respective
# messages. This state is described in respective time.
#
# The centroidal state describes
#  * the position, velocity and acceleration of the CoM,
#  * the base orientation and angular velocity, and
#  * the linear and angular momentum and their rates.
#
# The joint states describes:
#  * the joint name
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s),
#  * the effort that is applied in the joint (Nm or N).
# The contact state describes
#  * the frame name
#  * the pose of the contact,
#  * the velocity of the contact, and
#  * the wrench of the contact
# The header specifies the time at which the state is published.


std_msgs/Header header

# This represent the time state
float64 time

# This represents the base state (CoM motion, angular motion and centroidal momenta)
whole_body_state_msgs/CentroidalState centroidal

# This represents the joint state (position, velocity, acceleration and effort)
whole_body_state_msgs/JointState[] joints

# This represents the end-effector state (cartesian position and contact forces)
whole_body_state_msgs/ContactState[] contacts

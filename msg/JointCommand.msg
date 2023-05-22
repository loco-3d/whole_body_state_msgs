# This message describes the state of joint command.
#
# The state of each command (revolute or prismatic) is defined by:
#  * the total command to the joint (Nm or N),
#  * the feedforward command component (Nm or N), and
#  * the feedback command component (Nm or N)
#
# Each commanded joint is uniquely identified by its name


string name
float64 total
float64 feedforward
float64 feedback

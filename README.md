State ROS messages
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

A state describes the whole-body information of an arbitrary robotic system. The state is broken in three pieces: centroidal, joint and contact state.
The centroidal state describes:
  1. the position and velocity of the center of mass,
  2. base orientation and its angular velocity, and
  3. whole-body momenta and its rate.
  
Instead, the contact state describes:
  1. Type of contact (locomotion / manipulation)
  2. Name of the contact frame
  3. Pose of the contact,
  4. Twist of the contact,
  5. Wrench of the contact,
  6. Surface normal and friction coefficient.

With these basic states, the package describes whole-body states and trajectories. Additionally, it defines a whole-body control message which contains desired and actual states.

## :penguin: Building

The whole_body_state_msgs is a catkin project which can be built as:

	cd your_ros_ws/
	catkin build #catkin_make

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://cmastalli.github.io/), The University of Edinburgh :uk:

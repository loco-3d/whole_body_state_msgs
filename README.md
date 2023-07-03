# Whole-body state ROS messages

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

A whole-body state describes the entire information of an arbitrary robotic system. This state is broken in three pieces: centroidal, joint and contact state.
The centroidal state describes:

1. the position and velocity of the center of mass,
1. base orientation and its angular velocity, and
1. whole-body momenta and its rate.

Instead, the contact state describes:

1. Type of contact (locomotion / manipulation)
1. Name of the contact frame
1. Pose of the contact,
1. Twist of the contact,
1. Wrench of the contact,
1. Surface normal and friction coefficient.

With these basic states, the package also describes a whole-body trajectory. Additionally, it defines a whole-body control message which contains desired and actual states.

## :penguin: Building

The whole_body_state_msgs is a catkin/colcon project which can be built as:

```bash
    cd your_ros_ws/
    catkin build # ROS1
    colcon build # ROS2
```

Note that this package supports ROS1 and ROS2.

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://romilab.org), Heriot-Watt University :uk:

### :construction_worker: With contributions from

- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/), University of Oxford :uk:

and maintained by the [Robot Motor Intelligence (RoMI)](https://romilab.org) lab.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(state_msgs)

# Set up project properties
SET(PROJECT_NAME state_msgs)
SET(PROJECT_DESCRIPTION "State ROS messages")

# Print initial message - using STATUS so it doesn't trigger as a warning
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE(STATUS "Copyright (C) 2020 University of Edinburgh")
MESSAGE(STATUS "All rights reserved.")
MESSAGE(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

# Define messages to be generated
add_message_files(
  DIRECTORY msg
  FILES
    JointCommand.msg
    CentroidalState.msg
    JointState.msg
    ContactState.msg
    WholeBodyState.msg
    WholeBodyTrajectory.msg
    WholeBodyController.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Define catkin dependencies
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)

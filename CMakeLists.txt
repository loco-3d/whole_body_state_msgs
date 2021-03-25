CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(whole_body_state_msgs)

# Set up project properties
SET(PROJECT_NAME whole_body_state_msgs)
SET(PROJECT_DESCRIPTION "Whole-body State ROS messages")

# Print initial message - using STATUS so it doesn't trigger as a warning
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE(STATUS "Copyright (C) 2020 University of Edinburgh, University of Oxford")
MESSAGE(STATUS "All rights reserved.")
MESSAGE(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

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

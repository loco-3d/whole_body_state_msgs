CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(state_msgs)

# Set up project properties
SET(PROJECT_NAME state_msgs)
SET(PROJECT_DESCRIPTION "State ROS messages")

# Print initial message
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE("State ROS messages")
MESSAGE("Copyright (C) 2020-2022 University of Edinburgh")
MESSAGE("All rights reserved.")
MESSAGE("Released under the BSD 3-Clause License.")

# Set a default build type to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# Find required packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation)

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
    WholeBodyController.msg)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs)


# Define catkin dependencies
catkin_package(
    LIBRARIES  ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp message_runtime std_msgs geometry_msgs)


include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Installation rule
INSTALL(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

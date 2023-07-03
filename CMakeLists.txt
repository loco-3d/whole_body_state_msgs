cmake_minimum_required(VERSION 3.3)
project(whole_body_state_msgs)

add_compile_options(-Wall -Wextra -Wpedantic)

# Set up project properties
set(PROJECT_NAME whole_body_state_msgs)
set(PROJECT_DESCRIPTION "Whole-body State ROS messages")

# Print initial message - using STATUS so it doesn't trigger as a warning
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message(
  STATUS
    "Copyright (C) 2020-2023 Heriot-Watt University, University of Edinburgh")
message(STATUS "                        University of Oxford, IHMC")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

if($ENV{ROS_VERSION} EQUAL 2) # ROS 2
  # Find required packages
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  # Define messages to be generated
  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/CentroidalState.msg"
    "msg/ContactState.msg"
    "msg/JointCommand.msg"
    "msg/JointState.msg"
    "msg/WholeBodyController.msg"
    "msg/WholeBodyState.msg"
    "msg/WholeBodyTrajectory.msg"
    DEPENDENCIES
    std_msgs
    geometry_msgs)

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else() # ROS 1
  # Find required packages
  find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs
                                          message_generation)

  # Define messages to be generated
  add_message_files(
    DIRECTORY
    msg
    FILES
    JointCommand.msg
    CentroidalState.msg
    JointState.msg
    ContactState.msg
    WholeBodyState.msg
    WholeBodyTrajectory.msg
    WholeBodyController.msg)

  generate_messages(DEPENDENCIES std_msgs geometry_msgs)

  # Define catkin dependencies
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
endif()

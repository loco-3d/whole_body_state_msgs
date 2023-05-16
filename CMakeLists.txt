CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(whole_body_state_msgs)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Set up project properties
SET(PROJECT_NAME whole_body_state_msgs)
SET(PROJECT_DESCRIPTION "Whole-body State ROS messages")

# Print initial message - using STATUS so it doesn't trigger as a warning
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE(STATUS "Copyright (C) 2020 University of Edinburgh, University of Oxford")
MESSAGE(STATUS "All rights reserved.")
MESSAGE(STATUS "Released under the BSD 3-Clause License.")

if($ENV{ROS_VERSION} EQUAL 2)  # ROS 2
  # Find required packages
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  # Define messages to be generated
  rosidl_generate_interfaces(${PROJECT_NAME}
          "msg/CentroidalState.msg"
          "msg/ContactState.msg"
          "msg/JointCommand.msg"
          "msg/JointState.msg"
          "msg/WholeBodyController.msg"
          "msg/WholeBodyState.msg"
          "msg/WholeBodyTrajectory.msg"
          DEPENDENCIES std_msgs geometry_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()  # ROS 1
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
endif()
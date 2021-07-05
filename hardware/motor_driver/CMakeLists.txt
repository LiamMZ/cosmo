cmake_minimum_required(VERSION 3.0.2)
project(motor_driver)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  cosmo
  pluginlib
  nodelet
)

add_message_files(
  FILES
  MotorCommand.msg
  MovementState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motor_driver
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_definitions( ${EIGEN3_DEFINITIONS} )
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library( ${PROJECT_NAME}
    src/kinematics.cpp
    src/gait_controller.cpp
    src/stance_controller.cpp
    src/swing_controller.cpp
    src/driver.cpp
  )

add_library(MotorDriverNodelet src/driver_nodelet.cpp)
target_link_libraries(MotorDriverNodelet ${catkin_LIBRARIES} ${PROJECT_NAME} -lwiringPi)
## Mark the nodelet library for installations
install(TARGETS MotorDriverNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(FILES nodelet_plugin.xml motor_actuator_nodelet.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
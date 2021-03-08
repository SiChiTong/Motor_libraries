cmake_minimum_required(VERSION 3.0.2)
project(agv_define)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  message_generation
  geometry_msgs
  actionlib_msgs
  actionlib
)

add_message_files(
  FILES
  agv_action.msg
  agv_flexisoft.msg
  agv_status.msg
  charger_goal.msg
  nod_goal.msg
)

add_service_files(
  FILES
  lift.srv
  convertJsonToRos_srv.srv
)

add_action_files(
  DIRECTORY action
  FILES line.action charger.action nod.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/robot_dynparam.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(agvlib src/agvlib.cpp)
target_link_libraries(agvlib ${catkin_LIBRARIES})


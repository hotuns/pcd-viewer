cmake_minimum_required(VERSION 3.0.2)
project(mission_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  WaypointPosition.msg
  PositionCommand.msg
  MinTraj.msg
  Control.msg
  MissionList.msg
  MissionStatus.msg
  HangarChargeStatus.msg
  TaskOpt.msg
  )

# add_action_files(
#   FILES
#   ExecuteMission.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  # actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs #actionlib_msgs
)

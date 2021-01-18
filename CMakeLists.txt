cmake_minimum_required(VERSION 2.8.3)
project(kamaz_test_task)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  rospy
  std_msgs
  nav_msgs)

add_message_files(
  FILES
  KamazPath.msg
)

add_service_files(
  FILES
  StartRecord.srv
  StopRecord.srv
  ClearTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs genmsg message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

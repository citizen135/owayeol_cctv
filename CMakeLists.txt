cmake_minimum_required(VERSION 3.0.2)
project(owayeol_cctv)

find_package(catkin REQUIRED COMPONENTS
  rospy
  move_base_msgs
  geometry_msgs
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

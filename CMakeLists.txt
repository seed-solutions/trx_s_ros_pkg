cmake_minimum_required(VERSION 2.8.3)
project(trx_s_ros_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)

 add_service_files(
  FILES
  HandController.srv
 )
  generate_messages(
    DEPENDENCIES
    std_msgs
  )

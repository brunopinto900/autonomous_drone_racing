cmake_minimum_required(VERSION 2.8.3)
project(core)

find_package(catkin REQUIRED
roscpp
rospy
message_generation
std_msgs
geometry_msgs
)

add_message_files( 
    FILES 
    State.msg
    Trajectory.msg
)


 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package( CATKIN_DEPENDS roscpp std_msgs message_runtime )


set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)


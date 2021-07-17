cmake_minimum_required(VERSION 3.0)
project(controller)

find_package(catkin REQUIRED
nav_msgs
roscpp
rospy
std_msgs
geometry_msgs
visualization_msgs
trajectory_msgs
nav_msgs
core
airsim_ros_pkgs
mavros_msgs
)

find_package (Eigen3 REQUIRED)

catkin_package(  INCLUDE_DIRS include CATKIN_DEPENDS rospy roscpp std_msgs core airsim_ros_pkgs mavros_msgs )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

add_executable(PID_position_controller
        src/position_controller.cpp
)

target_link_libraries(PID_position_controller ${Eigen_LIBRARIES} )
target_link_libraries(PID_position_controller ${catkin_LIBRARIES})

add_dependencies(PID_position_controller ${catkin_EXPORTED_TARGETS})

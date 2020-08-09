cmake_minimum_required(VERSION 2.8.3)
project(reusnake_visualize)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs
  
  # added to supprt robot_markers
  kdl_parser
  tf2_kdl
  tf2_ros
  transform_graph
  urdf
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES titan6_cpp
  CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
get_filename_component(ROOT_DIR "${ROOT_DIR}" REALPATH)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  # added to supprt robot_markers
  ${orocos_kdl_INCLUDE_DIRS}
)

SET(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_markers/builder.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_markers/forward_kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mr/modern_robotics.cpp
)

add_executable(visualizer
  ${SOURCES} )
target_link_libraries( visualizer
                          m pthread
                          ${orocos_kdl_LIBRARIES}
                          ${catkin_LIBRARIES} 
                          )

add_executable(gazebo_vo_compare src/gazebo_vo_compare.cpp)
target_link_libraries(gazebo_vo_compare ${catkin_LIBRARIES})

add_executable(gazebo_vio_compare src/gazebo_vio_compare.cpp)
target_link_libraries(gazebo_vio_compare ${catkin_LIBRARIES})

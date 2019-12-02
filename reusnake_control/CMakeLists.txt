cmake_minimum_required(VERSION 2.8.3)
project(reusnake_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_FLAGS "-O3")

# Instruct CMake to run moc automatically when needed.
# this has to be set for QT projects, or compile will pop up error
# set(CMAKE_AUTOMOC ON)
set(Debug true)

LIST( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/cmake )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Doxygen)
# added to supprt robot_markers, this one should be included with ros-version-orocos-kdl
find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs

  hebi_cpp_api # hebi cpp api to replace code  
  tf2_kdl
  tf2_ros
  
  snake_control_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES titan6_cpp
  CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs hebi_cpp_api snake_control_bridge
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

# TODO: seperate robot core and hebi robot core
add_library(hebi_robot_core OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/hebi_robot_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/robot_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mr/modern_robotics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot/reusnake.cpp
)

add_library(estimation OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/ekf.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/models.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/VirtualChassisKinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/SnakeKinematics.cpp
)

SET(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_control.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/input/input_manager_mobile_io.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/util/util.cpp
)

add_executable(control 
  ${SOURCES} 
  $<TARGET_OBJECTS:hebi_robot_core>)
target_link_libraries( control
                          m pthread
                          ${orocos_kdl_LIBRARIES}
                          ${catkin_LIBRARIES} 
                          )
add_executable(estimator_node src/estimation/estimator_node.cpp)
target_link_libraries(estimator_node ${catkin_LIBRARIES} estimation)

add_executable(dummy_snake src/estimation/dummy_snake.cpp)
target_link_libraries(dummy_snake ${catkin_LIBRARIES})

if (Debug) # add test

endif (Debug)

# Add ultra-conservative warnings.
if ( CMAKE_COMPILER_IS_GNUCC )
  set_property( TARGET control APPEND_STRING PROPERTY COMPILE_FLAGS " -Wall -Wno-int-in-bool-context " )
endif ( CMAKE_COMPILER_IS_GNUCC )


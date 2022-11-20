cmake_minimum_required(VERSION 2.8.3)
project(stage_first)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  stage
  # stage_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_kdl
  kdl_parser
  trajectory_msgs
  sensor_msgs
  rosgraph_msgs
  urdf
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stage_first
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(Eigen_INCLUDE_DIRS /usr/local/include/eigen3)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main
    orocos-kdl
        ${catkin_LIBRARIES})
add_dependencies(main stage_first_generate_messages_cpp)



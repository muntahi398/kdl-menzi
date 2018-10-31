cmake_minimum_required(VERSION 2.8.3)
project(kdl_menzi)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  orocos_kdl
  roscpp
)






catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kdl_menzi
#  CATKIN_DEPENDS kdl_parser orocos_kdl roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable (kdl_menzi src/kdl_menzi.cpp)
target_link_libraries (kdl_menzi ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})

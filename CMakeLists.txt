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
  gazebo_ros
)


find_package(gazebo REQUIRED)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kdl_menzi
#  CATKIN_DEPENDS kdl_parser orocos_kdl roscpp
#  DEPENDS system_lib
#gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})


#add_executable (kdl_menzi src/kdl_menzi.cpp)
#target_link_libraries (kdl_menzi ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_executable (kdl_menzi_param src/kdl_menzi_param.cpp)
target_link_libraries (kdl_menzi_param ${catkin_LIBRARIES} )

#add_executable (contact_pub src/contact_pub.cpp)
#target_link_libraries (contact_pub ${catkin_LIBRARIES} ${GAZEBO_libraries})


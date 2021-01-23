cmake_minimum_required(VERSION 3.0.2)
project(scale_frds_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS}
                     ${Boost_INCLUDE_DIR}
                     ${GAZEBO_INCLUDE_DIRS}
)

add_executable(gazebo_transport_to_ros_topic_frds src/gazebo_transport_to_ros_topic_frds.cpp)
add_dependencies(gazebo_transport_to_ros_topic_frds ${gazebo_transport_to_ros_topic_frds_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_transport_to_ros_topic_frds
	${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)










cmake_minimum_required(VERSION 2.8.3)
project(scale_controller)  # <-- this is the project name, should match in the package.xml

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS	# This goes out and finds the packages 
  std_msgs
  geometry_msgs                             # we are listening to turtlesim/Pose messages and we
  roscpp                                # need std_msgs because we are publishing a std_msgs/Bool
                                        # Often, we need Eigen, tf, and other packages that we use 
)                                       # the node

catkin_package(CATKIN_DEPENDS geometry_msgs) # This declares our node as a catkin package, so it gets picked up by a catkin_make

include_directories(include) # This line makes sure the compiler looks in the include folder for our headers
include_directories(
  ${catkin_INCLUDE_DIRS}     # This line adds in the headers already added by catkin (ros/ros.h)
)

## Declare a cpp executable 		# These lines create the executable (node).  You can make more than
add_executable(scale_cont_node
	src/scale_cont_node.cpp     
	src/scale_cont.cpp           
)


## Add cmake target dependencies of the executable/library
add_dependencies(scale_cont_node ${scale_cont_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scale_cont_node 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}                   
)

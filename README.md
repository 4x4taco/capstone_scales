# CPP Udacity Capstone Project
This project replicates a set of vehicle corner scales to approximate the weight distribution of automobiles.  A Gazebo environment has been created with 4 physical scales and 4 force torque sensors connected to these scales.

4 ROS nodes were created to listen to the Gazebo transport layer and republishes the force torque sensor data via a ros topic.  The main node scale controller subscribes to each of the 4 topics, reads the sensor information and modifies it producing a weight distrubution output for this unique vehicle.  

The scale world file has been created to display the location of the vehicle scales via simplified geometry and displayed in red.  The main node scale controller sums the total weights of each scale and provides an textual output for the weight distributions of the vehicles normalized to the total weight.

## Dependencies for Running Locally
* cmake >= 2.8.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* rosdistro: Kinetic
* rosversion: 1.12.16
* gazebo: version 7.16.1

## Project Rubric Points

1. All points of the Readme and compiling/testing sections have been addressed.
2. Project demonstrates and understanding of C++ functions and control structures.  See line(s) 38, 47, 55, 63, 73, 90, 102, and 119 of src/scale_cont.cpp
3. Project reads data from a file and processes the data. In each of the gazebo_transport_to_ros files the ros::param::get function is called to retrieve labels from the            launch file.  The string values are used to setup the labels for the sensors within Gazebo.  See line(s) 57, 58, and 58 within                                                    src/scale_frds_pkg/src/gazebo_transport_to_ros_topic_frds.cpp and line(s) 4,5, and 6 within src/scale_frds_pkg/launch/gaz_to_ros_relay_frds.launch
4. The Project uses Object Oriented Programming techniques.  The Gazebo transport to ROS source files were not organized using classes, however src/scale_cont.cpp made use of      object oriented programming.
5. Class constructors utilize member initialization lists.  src/scale_cont.cpp line(s) 5, 6, and 7 initialize a public and private ros nodehandle.
6. The project makes use of references in function declarations.  line(s) 11-17 of src/scale_cont.cpp use references to call the callback functions when a message is                detected on the suscribed topic.
7. Classes encapslate behavior.  Data and fucntions that pertain to to displaying and calculating the scale weights were kept within the scaleController class                      src/scale_cont.cpp.


## Basic Build Instructions

1. Clone this repo.  git clone https://github.com/4x4taco/capstone_scales
2. Copy all subfolder of src folder into catkin_ws/src folder with file browser
3. Copy simulation_ws folder to same directory as catkin_ws
4. Compile using catkin_make within catkin_ws directory
5. Source contents of my simulations using source simulation_ws/devel/setup.bash
6. Launch gazebo world within simulation_ws using roslaunch my_simulations scales_world.launch
7. Source contents of catkin_ws using source catkin_ws/devel/setup.bash
8. Launch rosnodes from within catkin_ws/src using roslaunch scale_controller scale_controller.launch

## Project Description
Scales_world.world contains all of the inititial settings for the scale layout in Gazebo.  Revolute joints were created and attached to the world and link coordinate systems. Force torque sensors were then attached to the joints.  These sensors measure the angular torque and force for all of the degrees of freedom.  Only the weight vector for the scales was used for the rest of the project.  The mass parameter of the .world file was overidden to represent masses proportional to measurements taken in a real world measurement obtained with a set of Long Acre corner scales.  See picture of scale layout in Gazebo below.

### Scale Layout in Gazebo
![Screenshot](/capstone_scale_pics/gazebo_scale_layout.PNG)

A small delay was needed between launching the gazebo world and launching the ROS nodes.  This is to establish the gazebo environment and transport layer.  Due to this delay the Gazebo launch file was kept seperate from the the main launch file.  ROS nodes subscribe to the sensor values inside Gazebo.  The ROS nodes then publish to a topic and are started along with the node for the scale controller that handles the calculations and output.  See Gazebo sensor output and ROS architecture below.
### Gazebo Sensor Output
![Screenshot](/capstone_scale_pics/gazebo_sensor_output.PNG)
### Project Node Graph
![Screenshot](/capstone_scale_pics/node_graph.PNG)

Calculations inside the scale controller class and node include summation of each force sensor on each scale.  The values for front, rear, driver and passenger are accumulated and normalized with respect to the total weight of the vehicle.  This represents the weight distribution of the vehicle expressed in percentages for each of the sections.  
### Scale Controller Output
![Screenshot](/capstone_scale_pics/scale_cont_node_output.PNG)

The following columns in the output terminal, represent the labels for the different sections of the vehicle.  (FRDS) Front Driver Side, (FRPS) Front Passenger Side, (RDS) Rear Driver Side, (RPS) Rear Passenger Side.  Pecentages of the weight distributions are displayed as (DR) Driver side, (PS) Passenger side, (FR) Front, (RR) Rear.  These values simulation within ROS and Gazebo compare very closely to measurements taken.

### References
https://magiccvs.byu.edu/wiki/#!ros_tutorials/c++_node_class.md

http://gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors

https://answers.gazebosim.org/question/18715/does-anyone-have-a-working-example-or-a-tutorial-for-a-force_torque-plugin/?answer=18730#post-id-18730


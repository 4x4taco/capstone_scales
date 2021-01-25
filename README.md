# CPP Udacity Capstone Project
This project replicates a set of vehicle corner scales to approximate the weight distribution of automobiles.  A Gazebo environment has been created with 4 physical scales and 4 force torque sensors connected to these scales.

4 ROS nodes were created to listen to the Gazebo transport layer and republishes the force torques sensor data via a ros topic.  The main node scale controller subscribes to each of the 4 topics, reads the sensor information and modifies it producing a weight distrubution output for this unique vehicle.  

The scale world file has been created to display the location of the vehicle scales via simplified geometry and displayed in red.  The mass of each scale has been modified to be equivalent to the mass of a tested vehicle and can be access in the scale world file.  The main node scale controller sums the total weights of each scale and provides an textual output for the weight distributions of the vehicles normalized to the toal weight.

## Dependencies for Running Locally
* cmake >= 2.8.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
*rosdistro: Kinetic
*rosversion: 1.12.16
*gazebo: version 7.16.1



## Basic Build Instructions

1. Clone this repo.
2. Copy src folder into catkin_ws/src
3. Copy simulation_ws to working directory
4. Compile using catkin_make within catkin_ws directory
5. Launch gazebo world within simulation_ws using roslaunch my_simulations scales_world.launch
6. Launch rosnodes from within catkin_ws/src using roslaunch scale_controller scale_controller.launch

## Project Description
File scales_world.launch contains all of the inititial settings for the scale layout.  Revolute joints were created and attached to the world and link coordinate systems.  The mass parameter of the .world file was overidden to represent mass proportional to measurements taken in a real world measurement obtained with a set of long acre corner scales.  See picture of scale layout in Gazebo below.
### Scale Layout in Gazebo
![Screenshot](/capstone_scale_pics/gazebo_scale_layout.PNG

A small delay was added between launching the gazebo world and launching the ROS nodes.  This is to establish the gazebo environment and transport layer.  ROS nodes that subscribe to the sensor values inside Gazebo a publish to a topic are started along with a node for the controller that handles the calucaltions and output.  See Gazebo sensor output and ROS architecture below.
### Gazebo Sensor Output
![Screenshot](/capstone_scale_pics/gazebo_sensor_output.PNG)
### Project Node Graph
![Screenshot](/capstone_scale_pics/node_graph.PNG)

Calculations inside the scale controller class and anode include summation of each force sensor on each scale.  The values for front, rear, driver and passenger are accumulated and normalize with respect to the total weight of the vehicle.  This represents the weight distribution of the vehicle expressed in percentages.  
### Scale Controller Output
![Screenshot](/capstone_scale_pics/scale_cont_node_output.PNG)

The following columns in the output terminal, represent the labels for the different sections of the vehicle.  (FRDS) Front Driver Side, (FRPS) Front Passenger Side, (RDS) Rear Driver Side, (RPS) Rear Passenger Side.  Pecentages of the weight distributions are displayed as (DR) Driver side, (PS) Passenger side, (FR) Front, (RR) Rear.

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
![Screenshot](capstone_scale_pics/gazebo_scale_layout.png)

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


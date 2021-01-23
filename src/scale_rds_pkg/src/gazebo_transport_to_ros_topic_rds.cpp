// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <iostream>

//ros publisher topic for scale on the front drivers side of the vehicle
ros::Publisher scalerds;


void torquesCb_rds(const ConstWrenchStampedPtr &_msg)
{
  std::cout << "Received msg: " << std::endl;
  std::cout << _msg->DebugString() << std::endl;
  geometry_msgs::WrenchStamped msgWrenchedStamped;

  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = _msg->wrench().force().x();
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = _msg->wrench().force().z();
  scalerds.publish(msgWrenchedStamped);
}


int main(int argc, char **argv)
{
  //declare variables for ros parameters
  std::string gazebo_transport_topic_to_sub_rds;

  std::string ros_topic_to_pub_rds;

  double ros_rate;

  ROS_INFO("Starting gazebo");

  // Load Gazebo
  gazebo::client::setup(argc, argv);
  
  ROS_INFO("Starting ROS node");
  // Load ROS
  ros::init(argc, argv, "gazebo_transport_to_ros_topic_rds");

  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ROS node and init
  ros::NodeHandle nh;

  // Get ROS params
  ros::param::get("~gazebo_transport_topic_to_sub_rds", gazebo_transport_topic_to_sub_rds);
  ros::param::get("~ros_topic_to_pub_rds", ros_topic_to_pub_rds);
  ros::param::get("~ros_rate", ros_rate);

  //Create publish object
  scalerds = nh.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub_rds, 100);

  // Listen to Gazebo force_torque sensor topic
  gazebo::transport::SubscriberPtr sub_rds = node->Subscribe(gazebo_transport_topic_to_sub_rds, torquesCb_rds);
  ros::Rate loop_rate(ros_rate); // 100 hz
  
  while(ros::ok())
  {
   
    ros::spinOnce();
    loop_rate.sleep();
  }
  gazebo::shutdown();
  return 0;
}
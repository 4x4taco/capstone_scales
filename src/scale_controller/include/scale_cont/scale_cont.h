#ifndef SCALE_CONT_H
#define SCALE_CONT_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/WrenchStamped.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>


//namespace scale_controller
//{

class scaleController
{

public:

  scaleController();

//private:

  //***************** NODE HANDLES ***************//
  ros::NodeHandle nh_;         // public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; // private node handle for pulling parameter values from the parameter server

  //***************** PUBLISHERS AND SUBSCRIBERS ***************//
  // will end up getting hooked up to the callback for the geometry message from gazebo
  ros::Subscriber frds_subscriber_;

  ros::Subscriber frps_subscriber_;

  ros::Subscriber rds_subscriber_;

  ros::Subscriber rps_subscriber_;

  //int frds_force{0};
  //int frps_force{0};
  //int rds_force{0};
  //int rps_force{0};
  float front_weight{0.0};
  float rear_weight{0.0};
  float driver_side_weight{0.0};
  float passenger_side_weight{0.0};
  float total_weight_N{0.0};
  float total_weight_lbs{0.0};
  std::array<int, 4> force_values_N;
  std::array<int, 4> force_values_lbs;
  std::array<std::string, 8> output_columns {{"FRDS(lbs)", "FRPS(lbs)", "RDS(lbs)", "RPS(lbs)","DR(%)", "PS(%)", "FR(%)", "RR(%)" }};
  std::array<float, 4> cg_values;  //LEFT RIGHT FRONT REAR
  const char space = ' ';
  const int width = 10;

  //std::vector<int16_t> scale_weights;
  /* Copy the vector in a shared pointer */
 //std::shared_ptr<std::vector<int16_t>> scale_weights_shared = std::make_shared<std::vector<int16_t>> (scale_weights);

  //std::vector<std::string> scale_names;
  /* Copy the vector in a shared pointer */
  //std::shared_ptr<std::vector<std::string>> scale_names_shared = std::make_shared<std::vector<std::string>> (scale_names);

  //***************** PARAMETERS ***************//
  double threshold_;
  // a parameter we get from the ROS server, in this case the value below which
  // we consider the turtle as not moving.  This is basically a class variable
  // at this point,

  //***************** STATE VARIABLES ***************//
  // in this node, we don't have any variables.  Often though, we need to remember
  // things between loops, so we could create variables here to hold those values

  //***************** CALLBACKS ***************//

  void frdsCallback(const geometry_msgs::WrenchStampedPtr &_msg);

  void frpsCallback(const geometry_msgs::WrenchStampedPtr &_msg);

  void rdsCallback(const geometry_msgs::WrenchStampedPtr &_msg);

  void rpsCallback(const geometry_msgs::WrenchStampedPtr &_msg);

  //***************** CLASS METHODS ***************//
  void lineFormatNames();
  void lineFormatNumbersN();
  void lineFormatNumbersLbs();
  void calcTotalWeight();
  void cgCalculateWeightDistributions();


};

//} // namespace scale_controller

#endif // 

#include "scale_cont/scale_cont.h"
#include <geometry_msgs/WrenchStamped.h>


scaleController::scaleController() :
  nh_(ros::NodeHandle()),            
  nh_private_(ros::NodeHandle("~"))  
  //***************** NODE HANDLES ***************//

{
  frds_subscriber_ = nh_.subscribe("/gazebo_transport_to_ros_topic_frds", 10, &scaleController::frdsCallback, this);

  frps_subscriber_ = nh_.subscribe("/gazebo_transport_to_ros_topic_frps", 10, &scaleController::frpsCallback, this);

  rds_subscriber_ = nh_.subscribe("/gazebo_transport_to_ros_topic_rds", 10, &scaleController::rdsCallback, this);

  rps_subscriber_ = nh_.subscribe("/gazebo_transport_to_ros_topic_rps", 10, &scaleController::rpsCallback, this);

  int force_values[4];
  
  //array designation for different scale forces
  
  //FRONT DRIVER SIDE== force_values[0];
  //FRONT PASSENGER SIDE== force_values[1];
  //REAR DRIVER SIDE== force_values[2];
  //REAR PASSENGER SIDE == force_values[3];

  //array designation for different scale forces
  //frds_force == force_values[0];
  //frps_force == force_values[1];
  //rds_force == force_values[2];
  //rps_force == force_values[3];

}


  //***************** CALLBACKS ***************//
void scaleController::frdsCallback(const geometry_msgs::WrenchStampedPtr &_msg)
{
  //std::cout << "Received msg: from frds " << std::endl;
  //convert to lbs and save to array
  this->force_values_N[0] = (int)_msg->wrench.force.z;
  this->force_values_lbs[0] = (int)_msg->wrench.force.z*.22481;
}


void scaleController::frpsCallback(const geometry_msgs::WrenchStampedPtr &_msg)
{
  //std::cout << "Received msg : frome frps " << std::endl;
  //convert to lbs and save to array
  this->force_values_N[1] = (int)_msg->wrench.force.z;
  this->force_values_lbs[1] = (int)_msg->wrench.force.z*.22481;
}

void scaleController::rdsCallback(const geometry_msgs::WrenchStampedPtr &_msg)
{
  //std::cout << "Received msg: from rds" << std::endl;
  //convert to lbs and save to array
  this->force_values_N[2] = (int)_msg->wrench.force.z;
  this->force_values_lbs[2] = (int)_msg->wrench.force.z*.22481;
}

void scaleController::rpsCallback(const geometry_msgs::WrenchStampedPtr &_msg)
{
  //std::cout << "Received msg: from rps " << std::endl;
  //convert to lbs and save to array
  this->force_values_N[3] = (int)_msg->wrench.force.z;
  this->force_values_lbs[3] = (int)_msg->wrench.force.z*.22481;
}

  //***************** CLASS FUNCTIONS ***************//

void scaleController::calcTotalWeight()
{
  this->total_weight_N = 0.0;
  this->total_weight_lbs = 0.0;
  //total weight in Newtons
    for (int i = 0; i < this->force_values_N.size(); i++)
  {
    this->total_weight_N += (float)this->force_values_N[i];
  
  }
  //total weight in lbs
    for (int i = 0; i < this->force_values_lbs.size(); i++)
  {
    this->total_weight_lbs += this->force_values_lbs[i];
  }
}

void scaleController::lineFormatNames()
{
  for (int i = 0; i < this->output_columns.size(); i++)
  {
    std::cout << std::left << std::setw(this->width) << std::setfill(' ') << this->output_columns[i];
  }

std::cout << std::endl;
  
}


void scaleController::lineFormatNumbersLbs()
{
  std::cout << std::setprecision(2);
  for (int i = 0; i < this->force_values_lbs.size(); i++)
  {
    std::cout << std::left << std::setw(this->width) << std::setfill(' ') << this->force_values_lbs[i];
  }
  for (int i = 0; i < this->cg_values.size(); i++)
  {
    std::cout << std::left << std::setw(this->width) << std::setfill(' ') << this->cg_values[i];
  }

  std::cout << std::endl;

}


void scaleController::cgCalculateWeightDistributions()
{
  //FRONT DRIVER SIDE== force_values[0];
  //FRONT PASSENGER SIDE== force_values[1];
  //REAR DRIVER SIDE== force_values[2];
  //REAR PASSENGER SIDE == force_values[3];


  int front_weight{0};
  //check to see if values are zeros (callbacks not called)
  if (this->force_values_N[0] == 0)
  {
    //callback functions have not been called return to the main loop
    return;
  }
  else
  {
    this->driver_side_weight = this->force_values_lbs[0] + this->force_values_lbs[2]; 
    this->passenger_side_weight = this->force_values_lbs[1] + this->force_values_lbs[3]; 
    this->front_weight = this->force_values_lbs[0] + this->force_values_lbs[1];
    this->rear_weight = this->force_values_lbs[2] + this->force_values_lbs[3];
    this->cg_values[0] = ((this->driver_side_weight / this->total_weight_lbs));
    this->cg_values[1] = ((this->passenger_side_weight / this->total_weight_lbs));
    this->cg_values[2] = ((this->front_weight / this->total_weight_lbs));
    this->cg_values[3] = ((this->rear_weight / this->total_weight_lbs));
  }
}


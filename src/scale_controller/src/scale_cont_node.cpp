
#include "scale_cont/scale_cont.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "scale_controller_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS
  ros::Rate loop_rate(.1);  //set loop rate
  scaleController controller;  // instatiate our class object

  while(ros::ok())
  {
    ros::spinOnce(); // check for new messages and call the callback if one exists
    controller.calcTotalWeight();
    controller.cgCalculateWeightDistributions();
    controller.lineFormatNames();
    controller.lineFormatNumbersLbs();
    loop_rate.sleep();

  }
  return 0;
}

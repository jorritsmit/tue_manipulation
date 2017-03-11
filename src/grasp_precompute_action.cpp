// Author: Rob Janssen & the two stooges
// Modifications: Janno Lunenburg

#include "tue/manipulation/grasp_precompute.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{

    ROS_INFO("Starting grasp precompute node");

    ros::init(argc, argv, "grasp_precompute_server");
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    GraspPrecompute gp;
    if (gp.initialize())
    {
      ros::waitForShutdown();
    }
    else
    {
      return 1;
    }
    return 0;

}

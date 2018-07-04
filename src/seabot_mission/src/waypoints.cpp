#include <iostream>

#include <ros/ros.h>
#include <cmath>

#include "seabotmission.h"

using namespace std;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);

  ros::Rate loop_rate(frequency);
  SeabotMission m("/home/lemezoth/workspaceFlotteur/mission");

  ROS_INFO("Load mission");
  m.update_mission(); // Update mission file

  while (ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


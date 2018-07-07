#include <iostream>

#include <ros/ros.h>
#include <cmath>
#include "seabot_mission/Waypoint.h"
#include "seabotmission.h"

using namespace std;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle n;

//  piston_position = rospy.Publisher('/regulation/depth_set_point', DepthPose, queue_size=1)

  ros::Publisher waypoint_pub = n.advertise<seabot_mission::Waypoint>("/regulation/depth_set_point", 1);
  seabot_mission::Waypoint waypoint_msg;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);

  ros::Rate loop_rate(frequency);
  SeabotMission m("/home/lemezoth/workspaceFlotteur/mission");

  ROS_INFO("Load mission");
  m.update_mission(); // Update mission file

  double north, east, depth, ratio;

  while (ros::ok()){
    ros::spinOnce();

    m.compute_command(north, east, depth, ratio);

    waypoint_msg.depth = depth;
    waypoint_msg.north = north;
    waypoint_msg.east = east;
    waypoint_pub.publish(waypoint_msg);

    loop_rate.sleep();
  }

  return 0;
}


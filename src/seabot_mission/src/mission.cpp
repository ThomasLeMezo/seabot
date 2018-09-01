#include <iostream>

#include <ros/ros.h>
#include <cmath>
#include "seabot_mission/Waypoint.h"
#include "seabotmission.h"
#include "seabot_power_driver/Battery.h"
#include "seabot_power_driver/FlashCounter.h"

using namespace std;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle n;

  // Publisher
  ros::Publisher waypoint_pub = n.advertise<seabot_mission::Waypoint>("set_point", 1);
  seabot_mission::Waypoint waypoint_msg;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);
  const string mission_file_name = n_private.param<string>("mission_file_name", "mission_test.xml");
  const string mission_path = n_private.param<string>("mission_path", "");
  const int flash_counter = ceil(n_private.param<double>("flash_time_next_waypoint", 3.0));

  ROS_INFO("[Mission] Wait for Flasher counter service");
  ros::service::waitForService("/driver/power/flash_counter");
  ros::ServiceClient service_flash_counter = n.serviceClient<seabot_power_driver::FlashCounter>("/driver/power/flash_counter");

  ros::Rate loop_rate(frequency);
  SeabotMission m(mission_path);

  ROS_INFO("Load mission");
  m.load_mission(mission_file_name); // Update mission file

  double north, east, depth, ratio;

  while (ros::ok()){
    ros::spinOnce();

    bool is_new_waypoint = m.compute_command(north, east, depth, ratio);
    waypoint_msg.depth_only = m.is_depth_only();
    waypoint_msg.depth = depth;
    waypoint_msg.north = north;
    waypoint_msg.east = east;
    waypoint_msg.mission_enable = m.is_mission_enable();
    waypoint_msg.waypoint_number = m.get_current_waypoint();
    waypoint_msg.wall_time = ros::WallTime::now().toSec();
    waypoint_msg.time_to_next_waypoint = m.get_time_to_next_waypoint();
    waypoint_pub.publish(waypoint_msg);

    if(is_new_waypoint){
        seabot_power_driver::FlashCounter srv;
        srv.request.counter = flash_counter;
        if (!service_flash_counter.call(srv)){
          ROS_ERROR("[Safety] Failed to call zero depth");
        }
      }

    loop_rate.sleep();
  }

  return 0;
}


#include <ros/ros.h>
#include <cmath>
#include <array>

#include <seabot_fusion/GnssPose.h>
#include <seabot_power_driver/Battery.h>
#include <gpsd_client/GPSFix.h>
#include <gpsd_client/GPSStatus.h>

#include "iridium.h"

using namespace std;

double east = 0.0;
double north = 0.0;
double speed = 0.0;
double yaw_gnss = 0.0;
array<double, 4> batteries_level ={0.0, 0.0, 0.0, 0.0};

void pose_callback(const seabot_fusion::GnssPose::ConstPtr& msg){
  east = msg->east;
  north = msg->north;
}

void gnss_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  speed = msg->speed;
  yaw_gnss = msg->track * M_PI/180.0; // Degree from north
}

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
    batteries_level[0] = msg->battery1;
    batteries_level[1] = msg->battery2;
    batteries_level[2] = msg->battery3;
    batteries_level[3] = msg->battery4;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "iridium_node");
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber batteries_sub = n.subscribe("/fusion/battery", 1, batteries_callback);
  ros::Subscriber pose_sub = n.subscribe("/fusion/pose", 1, pose_callback);
  ros::Subscriber gnss_sub = n.subscribe("/driver/fix_extended", 1, gnss_callback);

  // Parameters
  ros::NodeHandle n_private("~");

  ros::Duration duration_sleep(n_private.param<int>("duration_sleep", 60*5));

  Iridium iridium;
  iridium.enable_com(false);

  while (ros::ok()){
    ros::spinOnce();

    iridium.m_east = east;
    iridium.m_north = north;

    iridium.add_new_log_file();
//    iridium.send_and_receive_data();

    duration_sleep.sleep();
  }

  return 0;
}


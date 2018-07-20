#include <ros/ros.h>
#include <cmath>
#include <array>

#include <seabot_fusion/GnssPose.h>
#include <seabot_power_driver/Battery.h>
#include <gpsd_client/GPSFix.h>
#include <gpsd_client/GPSStatus.h>
#include <seabot_fusion/DepthPose.h>

#include "iridium.h"

using namespace std;

double east = 0.0;
double north = 0.0;
double speed = 0.0;
double yaw_gnss = 0.0;

double depth_surface_limit = 0.5;
double wait_surface_time = 10.0;
ros::WallTime time_at_surface;
bool is_surface = false;
ros::WallTime time_last_communication;

Iridium iridium;

array<double, 4> batteries_level ={0.0, 0.0, 0.0, 0.0};

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  if(msg->depth < depth_surface_limit){
    if(!is_surface){
      time_at_surface = ros::WallTime::now();
      is_surface = true;
    }
  }
  else
    is_surface = false;
}

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

bool call_iridium(){
  // Test if is at surface for sufficient period of time
  if(is_surface && (ros::WallTime::now()-time_at_surface).toSec()>wait_surface_time){
    iridium.m_east = east;
    iridium.m_north = north;
    iridium.add_new_log_file();

    iridium.send_and_receive_data();
    return true;
  }
  else
    return false;
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
  const double frequency = n_private.param<double>("frequency", 1.0);
  const double duration_between_msg = n_private.param<double>("duration_between_msg", 60*5);
  wait_surface_time = n_private.param<double>("wait_time_surface", 10.0);
  depth_surface_limit = n_private.param<double>("depth_surface_limit", 0.5);

  iridium.uart_init();
  iridium.enable_com(true);

  ros::Rate loop_rate(frequency);

  while (ros::ok()){
    ros::spinOnce();

    ros::WallTime t = ros::WallTime::now();
    if(is_surface && ((t-time_last_communication).toSec()>duration_between_msg)){
      if(call_iridium()){
        time_last_communication = t;
      }
    }

    loop_rate.sleep();
  }

  return 0;
}


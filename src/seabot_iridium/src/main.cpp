#include <ros/ros.h>
#include <cmath>
#include <array>

#include <seabot_fusion/GnssPose.h>
#include <seabot_power_driver/Battery.h>
#include <gpsd_client/GPSFix.h>
#include <gpsd_client/GPSStatus.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_fusion/InternalPose.h>
#include <seabot_safety/SafetyLog.h>
#include <seabot_mission/Waypoint.h>

#include "iridium.h"

using namespace std;

double depth_surface_limit = 0.5;
double wait_surface_time = 10.0;
ros::WallTime time_at_surface;
bool is_surface = false;
ros::WallTime time_last_communication;

Iridium iridium;

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  if(msg->depth < depth_surface_limit){
    if(!is_surface){
      time_at_surface = ros::WallTime::now();
      is_surface = true;
      ROS_INFO("[Iridium] Surface detected");
      iridium.iridium_power(true);
    }
  }
  else{
    is_surface = false;
    iridium.iridium_power(false);
  }
}

void pose_callback(const seabot_fusion::GnssPose::ConstPtr& msg){
  iridium.m_east = msg->east;
  iridium.m_north = msg->north;
}

void safety_callback(const seabot_safety::SafetyLog::ConstPtr& msg){
  iridium.m_seabot_state = 0;
  iridium.m_seabot_state |= (msg->published_frequency & 0b1) << 0;
  iridium.m_seabot_state |= (msg->depth_limit & 0b1) << 1;
  iridium.m_seabot_state |= (msg->batteries_limit & 0b1) << 2;
  iridium.m_seabot_state |= (msg->depressurization & 0b1) << 3;
}

void gnss_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  iridium.m_gnss_speed = msg->speed; // Normaly in m/s
  iridium.m_gnss_heading = msg->track; // Degree from north
}

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
    iridium.m_batteries[0] = msg->battery1;
    iridium.m_batteries[1] = msg->battery2;
    iridium.m_batteries[2] = msg->battery3;
    iridium.m_batteries[3] = msg->battery4;
}

void sensor_internal_callback(const seabot_fusion::InternalPose::ConstPtr& msg){
  iridium.m_internal_pressure = msg->pressure;
  iridium.m_internal_temperature = msg->temperature;
}

void mission_callback(const seabot_mission::Waypoint::ConstPtr &msg){
  iridium.m_current_waypoint = msg->waypoint_number;
}

bool call_iridium(){
  // Test if is at surface for sufficient period of time
  if((ros::WallTime::now()-time_at_surface).toSec()>wait_surface_time){
    ROS_INFO("[Iridium] Call iridium");
    iridium.serialize_log_TDT1();
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
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber safety_sub = n.subscribe("/safety/safety", 1, depth_callback);
  ros::Subscriber sensor_internal_sub = n.subscribe("/fusion/sensor_internal", 1, sensor_internal_callback);
  ros::Subscriber mission_sub = n.subscribe("/mission/set_point", 1, mission_callback);

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);
  const double duration_between_msg = n_private.param<double>("duration_between_msg", 60*5);
  wait_surface_time = n_private.param<double>("wait_time_surface", 2.0);
  depth_surface_limit = n_private.param<double>("depth_surface_limit", 0.5);

  iridium.uart_init();
  iridium.enable_com(true);

  ros::Rate loop_rate(frequency);
  time_last_communication.fromSec(0);

  while (ros::ok()){
    ros::spinOnce();

    ros::WallTime t = ros::WallTime::now();
    if((is_surface || iridium.is_demo_mode()) && ((t-time_last_communication).toSec()>duration_between_msg)){
      if(call_iridium()){
        time_last_communication = t;
      }
    }

    loop_rate.sleep();
  }

  return 0;
}


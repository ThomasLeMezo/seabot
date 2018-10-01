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
#include <seabot_mission/MissionEnable.h>

#include <std_srvs/Empty.h>
#include <seabot_power_driver/SleepModeParam.h>

#include "iridium.h"
#include "missionxml.h"

using namespace std;

double depth_surface_limit = 0.5;
double wait_surface_time = 10.0;
ros::WallTime time_at_surface;
bool is_surface = false;
ros::WallTime time_last_communication;

Iridium iridium;
bool demo_mode=false;

string mission_file_path;

ros::ServiceClient service_sleep_mode, service_sleep_param, service_reload_mission;

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  if(msg->depth < depth_surface_limit){
    if(!is_surface){
      time_at_surface = ros::WallTime::now();
      is_surface = true;
      ROS_DEBUG("[Iridium] Surface detected");
      iridium.iridium_power(true);
    }
  }
  else{
    is_surface = false;
    iridium.iridium_power(false);
  }
}

void pose_callback(const seabot_fusion::GnssPose::ConstPtr& msg){
  iridium.logTDT.m_east = msg->east;
  iridium.logTDT.m_north = msg->north;
}

void safety_callback(const seabot_safety::SafetyLog::ConstPtr& msg){
  iridium.logTDT.m_seabot_state = 0;
  iridium.logTDT.m_seabot_state |= (msg->published_frequency & 0b1) << 0;
  iridium.logTDT.m_seabot_state |= (msg->depth_limit & 0b1) << 1;
  iridium.logTDT.m_seabot_state |= (msg->batteries_limit & 0b1) << 2;
  iridium.logTDT.m_seabot_state |= (msg->depressurization & 0b1) << 3;
}

void gnss_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  iridium.logTDT.m_gnss_speed = msg->speed; // Normaly in m/s
  iridium.logTDT.m_gnss_heading = msg->track; // Degree from north
}

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
  iridium.logTDT.m_batteries[0] = msg->battery1;
  iridium.logTDT.m_batteries[1] = msg->battery2;
  iridium.logTDT.m_batteries[2] = msg->battery3;
  iridium.logTDT.m_batteries[3] = msg->battery4;
}

void sensor_internal_callback(const seabot_fusion::InternalPose::ConstPtr& msg){
  iridium.logTDT.m_internal_pressure = msg->pressure;
  iridium.logTDT.m_internal_temperature = msg->temperature;
  iridium.logTDT.m_internal_temperature = msg->humidity;
}

void mission_callback(const seabot_mission::Waypoint::ConstPtr &msg){
  iridium.logTDT.m_current_waypoint = msg->waypoint_number;
}

void call_sleep_param(const int &hours, const int &min, const int &sec, const int &sec_to_sleep){
  seabot_power_driver::SleepModeParam srv;
  srv.request.sec = sec;
  srv.request.min = min;
  srv.request.hours = hours;
  srv.request.sec_to_sleep = sec_to_sleep;
  if (!service_sleep_param.call(srv)){
    ROS_ERROR("[Iridium] Failed to call sleep param");
  }
}

void call_sleep(){
  std_srvs::Empty srv;
  if(!service_sleep_mode.call(srv)){
    ROS_ERROR("[Iridium] Failed to call sleep mode");
  }
}

void call_reload_mission(){
  std_srvs::Empty srv;
  if(!service_reload_mission.call(srv)){
    ROS_ERROR("[Iridium] Failed to call reload mission");
  }
}

//void call_enable_mission(const bool &enable_mission, const bool &enable depth, const bool &enable_engine){
//  seabot_mission::MissionEnable srv;
//  srv.request.enable_mission = enable_mission;
//  srv.request.enable_engine = enable_engine;
//  srv.request.enable_depth = enable_depth;
//  if(!service_mission_enable.call(srv)){
//    ROS_ERROR("[Iridium] Failed to call reload mission");
//  }
//}

bool call_iridium(){
  // Test if is at surface for sufficient period of time
  if((ros::WallTime::now()-time_at_surface).toSec()>wait_surface_time){
    iridium.get_new_log_files();
    iridium.send_and_receive_data();
    iridium.process_cmd_file();

    // Process cmd files
    for(LogTDT &l:iridium.m_cmd_list){
      switch(l.m_cmd_type){
      case CMD_SLEEP:
      {
        int sleep_time = l.m_sleep_time;
        int hours = floor(sleep_time/60.);
        int min = sleep_time-hours*60;
        call_sleep_param(hours, min, 0, 200);
        call_sleep();
        break;
      }
      case CMD_MISSION:
      {
        // ToDO : write new mission file
        MissionXML m(l);
        m.write(mission_file_path);
        call_reload_mission();
        break;
      }
      case CMD_PARAMETERS:
      {
        // Enable/Diseable
        // safety, flash, mission, sink etc.
        if(!l.m_enable_mission){
        }
        else{

        }
        break;
      }
      default:
        break;
      }
    }

    // Clean cmd list
    iridium.m_cmd_list.clear();

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
  ros::Subscriber safety_sub = n.subscribe("/safety/safety", 1, safety_callback);
  ros::Subscriber sensor_internal_sub = n.subscribe("/fusion/sensor_internal", 1, sensor_internal_callback);
  ros::Subscriber mission_sub = n.subscribe("/mission/set_point", 1, mission_callback);

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);
  const double duration_between_msg = n_private.param<double>("duration_between_msg", 60*5);
  wait_surface_time = n_private.param<double>("wait_time_surface", 2.0);
  depth_surface_limit = n_private.param<double>("depth_surface_limit", 0.5);
  demo_mode = n_private.param<double>("demo", false);

  const string mission_file_name = n.param<string>("mission_file_name", "mission_test.xml");
  const string mission_path = n.param<string>("mission_path", "");
  mission_file_path = mission_path + "/" + mission_file_name;

  // Services
  ros::service::waitForService("/driver/power/sleep_mode");
  ros::service::waitForService("/driver/power/sleep_mode_param");
  ros::service::waitForService("/mission/reload_mission");

  service_sleep_mode = n.serviceClient<std_srvs::Empty>("/driver/power/sleep_mode");
  service_sleep_param = n.serviceClient<seabot_power_driver::SleepModeParam>("/driver/power/sleep_mode_param");
  service_reload_mission = n.serviceClient<std_srvs::Empty>("/mission/reload_mission");

  iridium.uart_init();
  iridium.enable_com(true);
  iridium.iridium_power(true);
  iridium.set_demo_mode(demo_mode);

  ros::Rate loop_rate(frequency);
  time_last_communication.fromSec(0);

  ROS_INFO("[Iridium] Start Ok");
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


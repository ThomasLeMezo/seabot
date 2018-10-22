#include <ros/ros.h>
#include <omp.h>
#include <math.h>
#include <array>

#include <seabot_fusion/GnssPose.h>
#include <seabot_power_driver/Battery.h>
#include <gpsd_client/GPSFix.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_fusion/InternalPose.h>
#include <seabot_safety/SafetyLog.h>
#include <seabot_mission/Waypoint.h>
#include <seabot_mission/MissionEnable.h>

#include <std_srvs/Empty.h>
#include <seabot_power_driver/SleepModeParam.h>

#include "missionxml.h"

#include "seabot_iridium/IridiumStatus.h"
#include "seabot_iridium/IridiumSession.h"
#include "std_msgs/String.h"
#include "sbd.h"

#include <thread>
#include <chrono>

using namespace std;

double duration_between_msg = 180;

double depth_surface_limit = 0.5;
double wait_surface_time = 10.0;
ros::WallTime time_at_surface;
bool test_surface = false;
bool is_surface = false;
ros::WallTime time_last_communication;

string mission_file_path;
ros::ServiceClient service_sleep_mode, service_sleep_param, service_reload_mission, service_enable_mission;
ros::Publisher iridium_pub;

bool valid_fix = false;
double latitude = 0.0;
double longitude = 0.0;
ros::WallTime time_last_gnss;

SBD sbd;
LogData log_state;

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  if(msg->depth < depth_surface_limit){
    if(!test_surface){
      time_at_surface = ros::WallTime::now();
      test_surface = true;
    }
    else{
      if((ros::WallTime::now()-time_at_surface).toSec()>wait_surface_time)
        is_surface = true;
      //        ROS_DEBUG("[Iridium] Surface detected");
      //        sbd.sbd_power(true);
    }
  }
  else{
    test_surface = false;
    is_surface = false;
    //    sbd.sbd_power(false);
  }
}

void pose_callback(const seabot_fusion::GnssPose::ConstPtr& msg){
  log_state.m_east = msg->east;
  log_state.m_north = msg->north;
}

void safety_callback(const seabot_safety::SafetyLog::ConstPtr& msg){
  log_state.m_safety_published_frequency = msg->published_frequency;
  log_state.m_safety_depth_limit = msg->depth_limit;
  log_state.m_safety_batteries_limit = msg->batteries_limit;
  log_state.m_safety_depressurization = msg->depressurization;
}

void gnss_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  if(msg->status >= msg->STATUS_MODE_2D)
    valid_fix = true;
  else
    valid_fix = false;
  log_state.m_gnss_speed = msg->speed; // Normaly in m/s (?)
  log_state.m_gnss_heading = msg->track; // Degree from north
  latitude = msg->latitude;
  longitude = msg->longitude;
  time_last_gnss = ros::WallTime::now();
}

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
  log_state.m_batteries[0] = msg->battery1;
  log_state.m_batteries[1] = msg->battery2;
  log_state.m_batteries[2] = msg->battery3;
  log_state.m_batteries[3] = msg->battery4;
}

void sensor_internal_callback(const seabot_fusion::InternalPose::ConstPtr& msg){
  log_state.m_internal_pressure = msg->pressure;
  log_state.m_internal_temperature = msg->temperature;
  log_state.m_internal_humidity = msg->humidity;
}

void mission_callback(const seabot_mission::Waypoint::ConstPtr &msg){
  log_state.m_current_waypoint = msg->waypoint_number;
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

void call_enable_mission(const bool &enable_mission, const bool &enable_depth, const bool &enable_engine){
  seabot_mission::MissionEnable srv;
  srv.request.enable_mission = enable_mission;
  srv.request.enable_engine = enable_engine;
  srv.request.enable_depth = enable_depth;
  if(!service_enable_mission.call(srv))
    ROS_ERROR("[Iridium] Failed to call reload mission");
}


void call_decode(const string &data_raw){
  // Test if is at surface for sufficient period of time
  LogData log_cmd;
  log_cmd.deserialize_log_CMD(data_raw);

  switch(log_cmd.m_cmd_type){
  case CMD_SLEEP:
  {
    int sleep_time = log_cmd.m_sleep_time;
    int hours = floor(sleep_time/60.);
    int min = sleep_time-hours*60;
    call_sleep_param(hours, min, 0, 200);
    call_sleep();
    break;
  }
  case CMD_MISSION:
  {
    // ToDO : write new mission file
    MissionXML m(log_cmd);
    m.write(mission_file_path);
    call_reload_mission();
    break;
  }
  case CMD_PARAMETERS:
  {
    // Enable/Diseable
    // safety, flash, mission, sink etc.
    duration_between_msg = (log_cmd.m_period_message/10.)*60.;
    call_enable_mission(log_cmd.m_enable_mission, log_cmd.m_enable_depth, log_cmd.m_enable_engine);
    break;
  }
  default:
    break;
  }
  log_state.m_last_cmd_received = log_cmd.m_cmd_type;
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
  duration_between_msg = n_private.param<double>("duration_between_msg", 60*5);
  wait_surface_time = n_private.param<double>("wait_time_surface", 2.0);
  depth_surface_limit = n_private.param<double>("depth_surface_limit", 0.5);

  const string mission_file_name = n.param<string>("mission_file_name", "mission_test.xml");
  const string mission_path = n.param<string>("mission_path", "");
  mission_file_path = mission_path + "/" + mission_file_name;

  const bool debug = n_private.param<bool>("debug", false);

  // Publisher
  ros::Publisher iridium_session_pub = n.advertise<seabot_iridium::IridiumSession>("session", 1);
  ros::Publisher iridium_data_received_pub = n.advertise<std_msgs::String>("received_raw", 1);
  ros::Publisher iridium_status_pub = n.advertise<seabot_iridium::IridiumStatus>("status", 1);

  // Services
  ros::service::waitForService("/driver/power/sleep_mode");
  ros::service::waitForService("/driver/power/sleep_mode_param");
  ros::service::waitForService("/mission/reload_mission");
  ros::service::waitForService("/mission/enable_mission");

  service_sleep_mode = n.serviceClient<std_srvs::Empty>("/driver/power/sleep_mode");
  service_sleep_param = n.serviceClient<seabot_power_driver::SleepModeParam>("/driver/power/sleep_mode_param");
  service_reload_mission = n.serviceClient<std_srvs::Empty>("/mission/reload_mission");
  service_enable_mission = n.serviceClient<seabot_mission::MissionEnable>("/mission/enable_mission");

  ros::Rate loop_rate(frequency);
  time_last_communication.fromSec(0);
  ros::WallTime time_last_log_version;

  sbd.init();
  sbd.set_debug(debug);

//  omp_set_num_threads(2);
#pragma omp parallel num_threads(2)
  {
#pragma omp single
    {
#pragma omp task
      {
        // Read Serial Com
        while(ros::ok())
          sbd.read();
      }

#pragma omp task
      {
        // Write Serial
        bool send_data_required = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        sbd.cmd_enable_indicator_reporting(true);
        sbd.cmd_enable_alert(true);

        ROS_INFO("[Iridium] Start Ok");
        while(ros::ok()){
          loop_rate.sleep();
          ros::spinOnce();

          // State machine
          ros::WallTime t = ros::WallTime::now();
          if(is_surface
             && ((t-time_last_communication).toSec()>duration_between_msg)
             && sbd.get_indicator_service() == 1){
            send_data_required = true;

            // Update LogData
            if((t-time_last_log_version).toSec()>30.){
              string log_sentence = log_state.serialize_log_state(ros::WallTime::now().toSec());
              sbd.cmd_write_message(log_sentence);
              if(valid_fix && (t-time_last_gnss).toSec()<10.)
                sbd.set_gnss(latitude, longitude);
            }
          }

          // Request a session if ring alert received or waiting data
          if(sbd.get_indicator_service()==1 && (send_data_required || sbd.get_ring_alert() || sbd.get_waiting()>0)){
            int result = sbd.cmd_session();

            if(result==1)
              ROS_INFO("[Iridium] Session not finished yet");

            seabot_iridium::IridiumSession session_msg;
            session_msg.mo = sbd.get_session_mo();
            session_msg.momsn = sbd.get_session_momsn();
            session_msg.mt = sbd.get_session_mt();
            session_msg.mtmsn = sbd.get_session_mtmsn();
            session_msg.waiting = sbd.get_waiting();
            iridium_session_pub.publish(session_msg);

            // Analyse success
            bool flush_mo = false;
            int session_mo_result = sbd.get_session_mo();
            if(session_mo_result>=0 && session_mo_result <= 4){
              time_last_communication = ros::WallTime::now();
              flush_mo = true;
              send_data_required = false;
              log_state.m_last_cmd_received = 0;
            }

            // Look at received message
            bool flush_mt = false;
            if(sbd.get_session_mt()==1){
              std::string message_data = sbd.cmd_read_message();

              // LogData Raw
              std_msgs::String msg_raw;
              msg_raw.data = message_data;
              iridium_data_received_pub.publish(msg_raw);

              // Decode
              call_decode(message_data);
              flush_mt = true;
            }

            sbd.cmd_flush_message(flush_mo, flush_mt); // Flush data to avoid re-sending it
          }

          // LogData Data
          if(is_surface){
            seabot_iridium::IridiumStatus status_msg;
            status_msg.service = sbd.get_indicator_service();
            status_msg.signal_strength = sbd.get_indicator_signal();
            status_msg.antenna = sbd.get_indicator_antenna();
            iridium_status_pub.publish(status_msg);
          }
        }
      }
    }
  }

  return 0;
}

/*
 * ToDo :
 *  -> msg to set the frequency of state msg
*/


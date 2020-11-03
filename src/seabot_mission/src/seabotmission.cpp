#include "seabotmission.h"
#include <fstream>
//#include "boost/filesystem.hpp"
//#include "boost/regex.hpp"
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <ctime>

using namespace std;
//using namespace boost;
namespace pt = boost::property_tree;

SeabotMission::SeabotMission(std::string folder_path){
  m_folder_path = folder_path;
}

bool SeabotMission::compute_command(double &north, double &east, double &depth, double &limit_velocity, double &approach_velocity, bool &enable_thrusters, double &ratio, bool &seafloor_landing){
  // Test if last waypoint
  bool is_new_waypoint = false;
  if(m_current_waypoint < m_waypoints.size()){
    // Test if next time
    ros::WallTime t_now = ros::WallTime::now();

    if(t_now < m_time_start){
      m_mission_enable = false;
      depth = 0.0;
      north = m_waypoints[m_current_waypoint].north;
      east = m_waypoints[m_current_waypoint].east;

      limit_velocity = m_waypoints[m_current_waypoint].limit_velocity;
      approach_velocity = m_waypoints[m_current_waypoint].approach_velocity;
      enable_thrusters = m_waypoints[m_current_waypoint].enable_thrusters;
      seafloor_landing = m_waypoints[m_current_waypoint].seafloor_landing;
      ratio = 0;
      m_duration_next_waypoint = m_time_start-t_now;
    }
    else{
      m_mission_enable = true;
      if(t_now>=m_waypoints[m_current_waypoint].time_end){
        m_current_waypoint++;
        is_new_waypoint = true;
      }
      if(m_old_waypoint != m_current_waypoint){
        m_old_waypoint = m_current_waypoint;
        ROS_DEBUG("[Mission] Start following waypoint %lu (ending at %f)", m_current_waypoint, round(m_waypoints[m_current_waypoint].time_end.toSec()));
      }

      depth = m_waypoints[m_current_waypoint].depth;

      limit_velocity = m_waypoints[m_current_waypoint].limit_velocity;
      approach_velocity = m_waypoints[m_current_waypoint].approach_velocity;
      enable_thrusters = m_waypoints[m_current_waypoint].enable_thrusters;
      seafloor_landing = m_waypoints[m_current_waypoint].seafloor_landing;

      ros::WallTime t1;
      if(m_current_waypoint==0)
        t1 = m_time_start;
      else
        t1 = m_waypoints[m_current_waypoint-1].time_end;
      ros::WallTime t2 = m_waypoints[m_current_waypoint].time_end;
      ratio = (t_now-t1).toSec()/(t2-t1).toSec(); // Should be between [0, 1]

      if(m_current_waypoint>0){
        double d_north = m_waypoints[m_current_waypoint].north - m_waypoints[m_current_waypoint-1].north;
        double d_east = m_waypoints[m_current_waypoint].east - m_waypoints[m_current_waypoint-1].east;

        north = m_waypoints[m_current_waypoint-1].north + d_north*ratio;
        east = m_waypoints[m_current_waypoint-1].east + d_east*ratio;
      }
      else{
        north = m_waypoints[m_current_waypoint].north;
        east = m_waypoints[m_current_waypoint].east;
      }

      m_duration_next_waypoint = m_waypoints[m_current_waypoint].time_end-t_now;
    }
  }
  else{
    if(m_mission_enable == true)
      ROS_INFO("[Mission] End of waypoints");
    depth = 0.0;
    limit_velocity = 0.0;
    approach_velocity = 1.0;
    north = m_waypoints[m_waypoints.size()-1].north;
    east = m_waypoints[m_waypoints.size()-1].east;
    ratio = 1;
    enable_thrusters = false;
    seafloor_landing = false;
    m_mission_enable = false;
    m_duration_next_waypoint = ros::WallDuration(0);
  }

  return is_new_waypoint;
}


void SeabotMission::load_mission(const std::string &file_xml){
  if(m_folder_path.empty())
    m_file_name = file_xml;
  else
    m_file_name = m_folder_path + "/" + file_xml;
  pt::ptree tree;
  ROS_DEBUG("[Seabot_Mission] Read xml file : %s", m_file_name.c_str());
  try {
    pt::read_xml(m_file_name, tree);
  } catch (std::exception const&  ex) {
    ROS_FATAL("[Seabot_Mission] %s", ex.what());
    exit(1);
  }

  m_waypoints.clear();

  try{
    m_offset_north = tree.get_child("mission.offset.north").get_value<double>();
  } catch (std::exception const&  ex){
    ROS_DEBUG("[Seabot_Mission] No north offset defined %s", ex.what());
  }

  try{
    m_offset_east = tree.get_child("mission.offset.east").get_value<double>();
  } catch (std::exception const&  ex){
    ROS_DEBUG("[Seabot_Mission] No east offset defined %s", ex.what());
  }

  m_time_start = ros::WallTime::now() + ros::WallDuration(60);
  // Read special offset time
  try{
    const int year = tree.get_child("mission.offset.start_time_utc.year").get_value<int>();
    const int month = tree.get_child("mission.offset.start_time_utc.month").get_value<int>();
    const int day = tree.get_child("mission.offset.start_time_utc.day").get_value<int>();
    const int hour = tree.get_child("mission.offset.start_time_utc.hour").get_value<int>();
    const int min = tree.get_child("mission.offset.start_time_utc.min").get_value<int>();

    struct tm time = { 0 };
    time.tm_year = year - 1900;
    time.tm_mon  = month - 1;
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min  = min;
    time.tm_sec  = 0.0;

    m_time_start = ros::WallTime(mktime(&time));
    ROS_DEBUG("[Seabot_Mission] Start time = %f", m_time_start.toSec());
  } catch (std::exception const&  ex){
    ROS_DEBUG("[Seabot_Mission] No time offset defined %s - Set now + 60s", ex.what());
  }

  ros::WallTime last_time = m_time_start;
  BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("mission.paths")){
    decode_waypoint(v, last_time, 0.0);
  }
}

void SeabotMission::decode_waypoint(pt::ptree::value_type &v, ros::WallTime &last_time, const double &depth_offset){
  if(v.first == "waypoint"){
    Waypoint w;
    try{

      // North & East position
      boost::optional<double> north_local = v.second.get_optional<double>("north");
      boost::optional<double> east_local = v.second.get_optional<double>("east");
      if(north_local.is_initialized() && east_local.is_initialized()){
        w.north = north_local.value();
        w.east = north_local.value();
      }
      else{
        w.enable_thrusters = false;
      }

      boost::optional<double> depth = v.second.get_optional<double>("depth");
      if(depth.is_initialized()){
        w.depth = depth.value() + depth_offset;
      }
      else{
        w.depth = 0.0;
      }

      boost::optional<bool> landing = v.second.get_optional<bool>("seafloor_landing");
      if(landing.is_initialized()){
        w.seafloor_landing = landing.value();
      }
      else{
        w.seafloor_landing = false;
      }


      // Duration
      boost::optional<double> t = v.second.get_optional<double>("duration_since_start");
      boost::optional<double> d = v.second.get_optional<double>("duration");
      if(t.is_initialized()) // End_time
        w.time_end = m_time_start + ros::WallDuration(t.value());
      else if(d.is_initialized()){ // Duration
        w.time_end = last_time + ros::WallDuration(d.value());
      }
      else
        throw(std::runtime_error("(No time or duration founded for a waypoint)"));

      // Regulation parameters
      boost::optional<double> vel = v.second.get_optional<double>("limit_velocity");
      if(vel.is_initialized())
        w.limit_velocity = vel.value();
      else
        w.limit_velocity = m_limit_velocity_default;

      boost::optional<double> approach = v.second.get_optional<double>("approach_velocity");
      if(vel.is_initialized())
        w.approach_velocity = approach.value();
      else
        w.approach_velocity = m_approach_velocity_default;

      boost::optional<bool> enable_thrusters = v.second.get_optional<bool>("enable_thrusters");
      if(enable_thrusters.is_initialized())
        w.enable_thrusters = enable_thrusters.value();
    }
    catch(std::exception const&  ex) {
      ROS_FATAL("[Seabot_Mission] Wrong xml file %s", ex.what());
      exit(1);
    }
    last_time = w.time_end;
    m_waypoints.push_back(w);

    ROS_INFO("[Seabot_Mission] Load Waypoint %zu (t_end=%li, d=%lf, E=%lf, N=%lf, vel=%f, app=%f)", m_waypoints.size(), (long int)w.time_end.toSec(), w.depth, w.east, w.north, w.limit_velocity, w.approach_velocity);
  }
  else if(v.first == "loop"){
    const int nb_loop = v.second.get<int>("<xmlattr>.number", 1);
    const double depth_increment = v.second.get<double>("<xmlattr>.depth_increment", 0.0);

    double depth_offset_tmp = depth_offset;
    for(int i=0; i<nb_loop; i++){
      ROS_INFO("[Seabot_Mission] Loop %i/%i", i+1, nb_loop);
      BOOST_FOREACH(pt::ptree::value_type &v_loop,v.second){
        decode_waypoint(v_loop, last_time, depth_offset_tmp);
      }
      depth_offset_tmp += depth_increment;
    }
  }
}


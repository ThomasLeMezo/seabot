#include "seabotmission.h"
#include <fstream>
//#include "boost/filesystem.hpp"
//#include "boost/regex.hpp"
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

using namespace std;
//using namespace boost;
namespace pt = boost::property_tree;

SeabotMission::SeabotMission(std::string folder_path){
  m_folder_path = folder_path;
}

void SeabotMission::compute_command(double &north, double &east, double &depth, double &ratio){
  // Test if last waypoint
  if(m_current_waypoint < m_waypoints.size()-1){
    // Test if next time
    ros::WallTime t_now = ros::WallTime::now();

    if(t_now < m_waypoints[m_current_waypoint].time_start){
      m_mission_enable = false;
      depth = 0.0;
      north = m_waypoints[m_current_waypoint].north;
      east = m_waypoints[m_current_waypoint].east;
      ratio = 0;
    }
    else{
      m_mission_enable = true;

      depth = m_waypoints[m_current_waypoint].depth;

      ros::WallTime t1 = m_waypoints[m_current_waypoint].time_start;
      ros::WallTime t2 = m_waypoints[m_current_waypoint+1].time_start;
      ratio = (t_now-t1).toSec()/(t2-t1).toSec(); // Should be between [0, 1]

      double d_north = m_waypoints[m_current_waypoint+1].north - m_waypoints[m_current_waypoint].north;
      double d_east = m_waypoints[m_current_waypoint+1].east - m_waypoints[m_current_waypoint].east;

      north = m_waypoints[m_current_waypoint].north + d_north*ratio;
      east = m_waypoints[m_current_waypoint].east + d_east*ratio;

      if(t_now>=m_waypoints[m_current_waypoint+1].time_start){
        m_current_waypoint++;
        ROS_INFO("[Seabot_mission] Start following waypoint %lu (%f)", m_current_waypoint, m_waypoints[m_current_waypoint].time_start.toSec());
      }
    }
  }
  else{
    if(m_mission_enable == true)
      ROS_INFO("[Seabot_mission] End of waypoints");
    depth = 0.0;
    north = m_waypoints[m_current_waypoint].north;
    east = m_waypoints[m_current_waypoint].east;
    ratio = 1;
    m_mission_enable = false;
  }
}


void SeabotMission::load_mission(const std::string &file_xml){
  m_file_name = m_folder_path + "/" + file_xml;
  pt::ptree tree;
  ROS_INFO("[Seabot_Mission] Read xml file : %s", m_file_name.c_str());
  try {
    pt::read_xml(m_file_name, tree);
  } catch (std::exception const&  ex) {
    ROS_FATAL("[Seabot_Mission] %s", ex.what());
    exit(1);
  }

  m_waypoints.clear();
  int nb = 0;

  double offset_north = 0.0;
  double offset_east = 0.0;
  double offset_time = 0.0;

  try{
    offset_north = tree.get_child("offset.north").get_value<double>();
  } catch (std::exception const&  ex){
      ROS_INFO("[Seabot_Mission] No north offset defined %s", ex.what());
  }

  try{
  offset_east = tree.get_child("offset.east").get_value<double>();
  } catch (std::exception const&  ex){
      ROS_INFO("[Seabot_Mission] No east offset defined %s", ex.what());
  }

  try{
  offset_time = tree.get_child("offset.time").get_value<int>();
  } catch (std::exception const&  ex){
    ROS_INFO("[Seabot_Mission] No time offset defined %s", ex.what());
  }

  int mission_type = tree.get_child("paths.time").get_value<int>();
  if(mission_type == 1)
    m_depth_only = true;

  BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("paths")){
    if(v.first == "waypoint"){
      double north, east, depth, posix_time;
      try{
        if(!m_depth_only){
          north = v.second.get_child("north").get_value<double>() + offset_north;
          east = v.second.get_child("east").get_value<double>() + offset_east;
        }
        depth = v.second.get_child("depth").get_value<double>();
        posix_time = v.second.get_child("time").get_value<double>() + offset_time;
      }
      catch (std::exception const&  ex) {
        ROS_FATAL("[Seabot_Mission] Wrong xml file %s", ex.what());
        exit(1);
      }
      ROS_INFO("[Seabot_Mission] Waypoint %i (%f, %f, %f, %f)", nb, posix_time, depth, east, north);
      ros::WallTime wt(posix_time);

      Waypoint w(wt, depth, north, east); // ToDo: convert lambert !!
      m_waypoints.push_back(w);
      nb++;
    }
  }
}


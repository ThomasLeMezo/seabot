#include "missionxml.h"
#include "logData.h"

#include <ctime>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>
namespace pt = boost::property_tree;

MissionXML::MissionXML(LogData &log){
  m_log = log;
}

void MissionXML::write(const std::string &filename) const{
  // Create an empty property tree object.
  pt::ptree m_tree;

  /// ******************** HEADER ******************** ///


  // TODO !!! WRONG
  if(m_log.m_msg_type == CMD_MISSION_NEW){
      time_t gtime = static_cast<time_t>(m_log.m_start_time);
      struct tm *timeinfo = gmtime(&gtime);

      pt::ptree tree_offset;
      tree_offset.put("year", timeinfo->tm_year+1900);
      tree_offset.put("month", timeinfo->tm_mon+1);
      tree_offset.put("day", timeinfo->tm_mday);
      tree_offset.put("hour", timeinfo->tm_hour);
      tree_offset.put("min", timeinfo->tm_min);

      m_tree.add_child("mission.offset.start_time_utc", tree_offset);
      m_tree.put("mission.paths.<xmlattr>.type", "0");
  }
  else{
      // Load xml
      pt::read_xml(filename, m_tree, 4);
  }

  for(const Waypoint &w:m_log.m_waypoint_list){
    pt::ptree sub_tree_wp;
    sub_tree_wp.put("waypoint.duration", w.duration);
    if(w.enable_thrusters){
        sub_tree_wp.put("waypoint.east", w.east);
        sub_tree_wp.put("waypoint.north", w.north);
    }
    sub_tree_wp.put("waypoint.depth", w.depth);

    if(w.seafloor_landing)
        sub_tree_wp.put("waypoint.seafloor_landing", true);

    m_tree.add_child("mission.paths.waypoint", sub_tree_wp.get_child("waypoint"));
  }

  std::stringstream ss;
  boost::property_tree::xml_parser::write_xml(ss, m_tree);
  std::cout << ss.str() << std::endl;

  pt::xml_writer_settings<std::string> settings(' ', 4);
  pt::write_xml(filename, m_tree, std::locale(), settings);
}

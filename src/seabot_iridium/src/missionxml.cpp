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

      m_tree.put("offset.start_time_utc.year", timeinfo->tm_year+1900);
      m_tree.put("offset.start_time_utc.month", timeinfo->tm_mon+1);
      m_tree.put("offset.start_time_utc.day", timeinfo->tm_mday);
      m_tree.put("offset.start_time_utc.hour", timeinfo->tm_hour);
      m_tree.put("offset.start_time_utc.min", timeinfo->tm_min);

      m_tree.put("paths.<xmlattr>.type", "0");
  }
  else{
      // Load xml
      pt::read_xml(filename, m_tree, 4);
  }

  for(const Waypoint &w:m_log.m_waypoint_list){
    pt::ptree sub_tree;
    sub_tree.put("waypoint.duration", w.duration);
    if(w.enable_thrusters){
        sub_tree.put("waypoint.east", w.east);
        sub_tree.put("waypoint.north", w.north);
    }
    sub_tree.put("waypoint.depth", w.depth);

    if(w.seafloor_landing)
        sub_tree.put("waypoint.seafloor_landing", true);

    m_tree.add_child("paths.waypoint", sub_tree.get_child("waypoint"));
  }

  std::stringstream ss;
  boost::property_tree::xml_parser::write_xml(ss, m_tree);
  std::cout << ss.str() << std::endl;

  pt::xml_writer_settings<std::string> settings(' ', 4);
  pt::write_xml(filename, m_tree, std::locale(), settings);
}

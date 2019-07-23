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
  pt::ptree tree;

  /// ******************** HEADER ******************** ///
  tree.put("offset.north", m_log.m_offset_north);
  tree.put("offset.east", m_log.m_offset_east);

  time_t rawtime = m_log.m_offset_time;
  time(&rawtime);
  struct tm *timeinfo = localtime(&rawtime);

  tree.put("offset.start_time_utc.year", timeinfo->tm_year+1900);
  tree.put("offset.start_time_utc.month", timeinfo->tm_mon+1);
  tree.put("offset.start_time_utc.day", timeinfo->tm_mday);
  tree.put("offset.start_time_utc.hour", timeinfo->tm_hour);
  tree.put("offset.start_time_utc.min", timeinfo->tm_min);

  for(const Waypoint &w:m_log.m_waypoint_list){
    pt::ptree sub_tree;
    sub_tree.put("waypoint.duration_since_start", w.time_end);
    sub_tree.put("waypoint.east", w.east);
    sub_tree.put("waypoint.north", w.north);
    sub_tree.put("waypoint.depth", w.depth);

    tree.add_child("paths.waypoint", sub_tree.get_child("waypoint"));
  }
  tree.put("paths.<xmlattr>.type", "0");

  pt::xml_writer_settings<std::string> settings(' ', 4);
  pt::write_xml(filename, tree, std::locale(), settings);
}

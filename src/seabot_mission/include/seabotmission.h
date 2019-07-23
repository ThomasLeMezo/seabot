#ifndef SEABOTMISSION_H
#define SEABOTMISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <ros/ros.h>

#include <boost/property_tree/ptree.hpp>

class Waypoint{

public:
  Waypoint(){}

  Waypoint(const ros::WallTime &time_end_param, const double &depth_param, const double &north_param, const double &east_param, const double&velocity_depth_param, const bool &enable_thrusters_param=true){
    time_end = time_end_param;
    depth = depth_param;
    east = east_param;
    north = north_param;
    velocity_depth = velocity_depth_param;
    enable_thrusters = enable_thrusters_param;
  }
public:
  double north = 0.0;
  double east = 0.0;
  double depth = 0.0;
  double velocity_depth = 0.0;
  bool  enable_thrusters = true;
  ros::WallTime time_end;
};

class SeabotMission
{
public:
  /**
     * @brief SeabotMission
     */
  SeabotMission(std::string folder_path);

  //    /**
  //     * @brief load_mission
  //     * @param filename
  //     */
  //    void load_mission(const std::string &file_name);

  /**
     * @brief update_mission
     * @return
     */
  bool update_mission();

  /**
     * @brief compute_command
     * @param north
     * @param east
     * @param depth
     */
  bool compute_command(double &north, double &east, double &depth, double &velocity_depth, bool &enable_engine, double &ratio);

  /**
     * @brief load_mission
     * @param file_xml
     */
  void load_mission(const std::string &file_xml);

  /**
   * @brief is_mission_enable
   * @return
   */
  bool is_mission_enable() const;

  /**
   * @brief is_depth_only
   * @return
   */
  bool is_depth_only() const;

  /**
   * @brief get_current_waypoint
   * @return
   */
  size_t get_current_waypoint() const;

  /**
   * @brief get_time_to_next_waypoint
   * @return
   */
  double get_time_to_next_waypoint() const;

  /**
   * @brief set_velocity_depth_default
   * @param vel
   */
  void set_velocity_depth_default(const double &vel);

private:
  void decode_waypoint(boost::property_tree::ptree::value_type &v, ros::WallTime &last_time, const double &depth_offset);

private:
  std::string m_folder_path = "";
  std::string m_file_name = "";
  std::vector<Waypoint> m_waypoints;
  int m_current_waypoint = 0;
  int m_old_waypoint = -1;
  bool    m_mission_enable = false;
  bool    m_update_mission = true;
  bool    m_depth_only = false;
  ros::WallDuration m_duration_next_waypoint = ros::WallDuration(0.0);

  ros::WallTime m_time_start;
  double m_offset_north = 0.0;
  double m_offset_east = 0.0;
  double m_velocity_depth_default = 0.02;
};

inline bool SeabotMission::is_mission_enable() const{
  return m_mission_enable;
}

inline bool SeabotMission::is_depth_only() const{
  return m_depth_only;
}

inline size_t SeabotMission::get_current_waypoint() const{
  return m_current_waypoint;
}

inline double SeabotMission::get_time_to_next_waypoint() const{
  return m_duration_next_waypoint.toSec();
}

inline void SeabotMission::set_velocity_depth_default(const double &vel){
  m_velocity_depth_default = vel;
}


#endif // SEABOTMISSION_H

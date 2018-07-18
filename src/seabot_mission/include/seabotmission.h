#ifndef SEABOTMISSION_H
#define SEABOTMISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <ros/ros.h>

class Waypoint{

public:
  Waypoint();

  Waypoint(const ros::WallTime &time_start_param, const double &depth_param, const double &north_param, const double &east_param){
    time_start = time_start_param;
    depth = depth_param;
    east = east_param;
    north = north_param;
  }
public:
  double north = 0.0;
  double east = 0.0;
  double depth = 0.0;
  ros::WallTime time_start;
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
  void compute_command(double &north, double &east, double &depth, double &ratio);

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

private:
  std::string m_folder_path = "";
  std::string m_file_name = "";
  std::vector<Waypoint> m_waypoints;
  size_t m_current_waypoint = 0;
  bool    m_mission_enable = false;
  bool    m_update_mission = true;
  bool    m_depth_only = false;
};

inline bool SeabotMission::is_mission_enable() const{
  return m_mission_enable;
}

inline bool SeabotMission::is_depth_only() const{
  return m_depth_only;
}

#endif // SEABOTMISSION_H

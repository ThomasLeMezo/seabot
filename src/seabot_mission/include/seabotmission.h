#ifndef SEABOTMISSION_H
#define SEABOTMISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <ros/ros.h>

class Waypoint{

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

    /**
     * @brief load_mission
     * @param filename
     */
    bool load_mission();

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

private:
    std::string m_folder_path = "";
    std::string m_file_name = "";
    std::vector<Waypoint> m_waypoints;
    size_t m_current_waypoint = 0;
    bool    m_mission_enable = false;
};

#endif // SEABOTMISSION_H

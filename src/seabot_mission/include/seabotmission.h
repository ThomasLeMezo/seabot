#ifndef SEABOTMISSION_H
#define SEABOTMISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>

class Waypoint{
public:
    double north = 0.0;
    double east = 0.0;
    double depth = 0.0;
};

class SeabotMission
{
public:
  SeabotMission();

private:
  std::string m_file_name;
  std::vector<Waypoint> m_waypoints;

  long int m_offset_east, m_offset_north;
  long int m_offset_time;

};

#endif // SEABOTMISSION_H

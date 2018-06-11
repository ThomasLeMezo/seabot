#ifndef SEABOTMISSION_H
#define SEABOTMISSION_H

#include <string>
#include <vector>
#include <array>
#include <fstream>

struct Waypoint{
    unsigned int time:18;
    unsigned int depth:9;
    int east:16;
    int north:16;
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

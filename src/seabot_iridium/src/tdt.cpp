#include "iridium.h"
#include <iostream>
#include <fstream>

using namespace std;

Iridium iridium;

int main(int argc, char *argv[]){
  std::ofstream outfile("log_TDT1.txt");
  string file_name(argv[1]);
  iridium.deserialize_log_TDT1(file_name);
  cout << "file open : " << outfile.is_open() << endl;

  outfile << "file_name = " << file_name << endl;
  
  outfile << "east = " << iridium.m_east << endl;
  outfile << "north = " << iridium.m_north << endl;
  outfile << "gnss_speed = " << iridium.m_gnss_speed << endl;
  outfile << "gnss_heading = " << iridium.m_gnss_heading << endl;

  outfile << "seabot_state = " << iridium.m_seabot_state << endl;

  outfile << "batteries[0] = " << iridium.m_batteries[0] << endl;
  outfile << "batteries[1] = " << iridium.m_batteries[1] << endl;
  outfile << "batteries[2] = " << iridium.m_batteries[2] << endl;
  outfile << "batteries[3] = " << iridium.m_batteries[3] << endl;

  outfile << "internal_pressure = " << iridium.m_internal_pressure << endl;
  outfile << "internal_temperature = " << iridium.m_internal_temperature << endl;

  outfile << "current_waypoint = " << iridium.m_current_waypoint << endl;

  outfile.close();

  return 0;
}


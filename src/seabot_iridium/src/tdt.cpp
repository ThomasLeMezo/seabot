#include "logtdt.h"
#include <iostream>
#include <fstream>

using namespace std;

LogTDT logTDT;

int main(int argc, char *argv[]){
  std::ofstream outfile("/mnt/webperso/iridium/log_TDT1.txt");
  string file_name(argv[1]);
  logTDT.deserialize_log_TDT1(file_name);
  cout << "file open : " << outfile.is_open() << endl;

  outfile << "file_name = " << file_name << endl;
  
  outfile << "east = " << std::fixed << logTDT.m_east << endl;
  outfile << "north = " << std::fixed << logTDT.m_north << endl;
  outfile << "gnss_speed = " << logTDT.m_gnss_speed << endl;
  outfile << "gnss_heading = " << logTDT.m_gnss_heading << endl;

  outfile << "seabot_state = " << logTDT.m_seabot_state << endl;

  outfile << "batteries[0] = " << logTDT.m_batteries[0] << endl;
  outfile << "batteries[1] = " << logTDT.m_batteries[1] << endl;
  outfile << "batteries[2] = " << logTDT.m_batteries[2] << endl;
  outfile << "batteries[3] = " << logTDT.m_batteries[3] << endl;

  outfile << "internal_pressure = " << logTDT.m_internal_pressure << endl;
  outfile << "internal_temperature = " << logTDT.m_internal_temperature << endl;

  outfile << "current_waypoint = " << logTDT.m_current_waypoint << endl;

  outfile.close();

  return 0;
}


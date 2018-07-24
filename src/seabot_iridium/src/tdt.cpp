#include "iridium.h"

using namespace std;

Iridium iridium;

int main(int argc, char *argv[]){
  string file_name(argv[1]);
  iridium.deserialize_log_TDT1(file_name);

  cout << "file_name = " << file_name << endl;
  
  cout << "east = " << iridium.m_east << endl;
  cout << "north = " << iridium.m_north << endl;
  cout << "gnss_speed = " << iridium.m_gnss_speed << endl;
  cout << "gnss_heading = " << iridium.m_gnss_heading << endl;

  cout << "seabot_state = " << iridium.m_seabot_state << endl;

  cout << "batteries[0] = " << iridium.m_batteries[0] << endl;
  cout << "batteries[1] = " << iridium.m_batteries[1] << endl;
  cout << "batteries[2] = " << iridium.m_batteries[2] << endl;
  cout << "batteries[3] = " << iridium.m_batteries[3] << endl;

  cout << "internal_pressure = " << iridium.m_internal_pressure << endl;
  cout << "internal_temperature = " << iridium.m_internal_temperature << endl;

  cout << "current_waypoint = " << iridium.m_current_waypoint << endl;

  return 0;
}


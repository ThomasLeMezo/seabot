#include "logtdt.h"
#include "missionxml.h"
#include <iostream>
#include <fstream>
#include <array>
#include <cstring>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;

LogTDT logTDT;

void test1(){
  LogTDT log;
  Waypoint w1(100, 5, 1, 2);
  Waypoint w2(100, 5, 1, 2);
  log.m_waypoint_list.push_back(w1);
  log.m_waypoint_list.push_back(w2);
  log.m_offset_time = 100;
  log.m_offset_east = 3;
  log.m_offset_north = 3;

  MissionXML m(log);
  m.write("/home/lemezoth/mission_test.xml");
}

bool sortCommandAbs (double i,double j) { return (abs(i)<abs(j)); }

void test2(){
  array<double, 4> u_tab;
  u_tab[3] = -1;
  u_tab[2] = 2;
  u_tab[1] = -3;
  u_tab[0] = 4;
  cout << u_tab[0] << endl;
  sort(u_tab.begin(), u_tab.end(), &sortCommandAbs);
  cout << u_tab[0] << endl;
}

int main(int argc, char *argv[]){
  string file_name(argv[1]);
  logTDT.deserialize_log_TDT1(file_name);

  stringstream data;
  data << "file_name: " << file_name << endl;
  
  data << "east: " << std::fixed << logTDT.m_east << endl;
  data << "north: " << std::fixed << logTDT.m_north << endl;
  data << "gnss_speed: " << logTDT.m_gnss_speed << endl;
  data << "gnss_heading: " << logTDT.m_gnss_heading << endl;

  data << "seabot_state: " << logTDT.m_seabot_state << endl;

  data << "batteries[0]: " << logTDT.m_batteries[0] << endl;
  data << "batteries[1]: " << logTDT.m_batteries[1] << endl;
  data << "batteries[2]: " << logTDT.m_batteries[2] << endl;
  data << "batteries[3]: " << logTDT.m_batteries[3] << endl;

  data << "internal_pressure: " << logTDT.m_internal_pressure << endl;
  data << "internal_temperature: " << logTDT.m_internal_temperature << endl;

  data << "current_waypoint: " << logTDT.m_current_waypoint << endl;


  string output_filename1 = string(argv[1]) + "_decode.yaml";
  std::ofstream outfile(output_filename1);
  outfile << data.str();
  outfile.close();

  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  string output_filename2 = string(homedir) + "/last_tdt1.yaml";
  std::ofstream outfile2(output_filename2);
  outfile2 << data.str();
  outfile2.close();

  return 0;
}


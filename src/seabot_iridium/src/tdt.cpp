#include "logtdt.h"
#include "missionxml.h"
#include <iostream>
#include <fstream>
#include <array>

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
  test2();


//  std::ofstream outfile("/mnt/webperso/iridium/log_TDT1.txt");
//  string file_name(argv[1]);
//  logTDT.deserialize_log_TDT1(file_name);
//  cout << "file open : " << outfile.is_open() << endl;

//  outfile << "file_name = " << file_name << endl;
  
//  outfile << "east = " << std::fixed << logTDT.m_east << endl;
//  outfile << "north = " << std::fixed << logTDT.m_north << endl;
//  outfile << "gnss_speed = " << logTDT.m_gnss_speed << endl;
//  outfile << "gnss_heading = " << logTDT.m_gnss_heading << endl;

//  outfile << "seabot_state = " << logTDT.m_seabot_state << endl;

//  outfile << "batteries[0] = " << logTDT.m_batteries[0] << endl;
//  outfile << "batteries[1] = " << logTDT.m_batteries[1] << endl;
//  outfile << "batteries[2] = " << logTDT.m_batteries[2] << endl;
//  outfile << "batteries[3] = " << logTDT.m_batteries[3] << endl;

//  outfile << "internal_pressure = " << logTDT.m_internal_pressure << endl;
//  outfile << "internal_temperature = " << logTDT.m_internal_temperature << endl;

//  outfile << "current_waypoint = " << logTDT.m_current_waypoint << endl;

//  outfile.close();

  return 0;
}


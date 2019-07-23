#include "logData.h"
#include "missionxml.h"
#include <iostream>
#include <fstream>
#include <array>
#include <cstring>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

//#include "TimeoutSerial.h"
#include "sbd.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "ros/ros.h"

#include "omp.h"

using namespace std;

LogData logData;

void test1(){
  LogData log;
  Waypoint w1(100, 5, 1, 2);
  Waypoint w2(100, 5, 1, 2);
  logData.m_waypoint_list.push_back(w1);
  logData.m_waypoint_list.push_back(w2);
  logData.m_offset_time = 100;
  logData.m_offset_east = 3;
  logData.m_offset_north = 3;

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

void test4(){
  SBD sbd("/dev/ttyAMA0", 19200);

  omp_set_num_threads(2);
#pragma omp parallel
  {
#pragma omp single
    {
#pragma omp task
      {
        while(1)
          sbd.read();
      }

#pragma omp task
      {
        sleep(1);
//        cout << "=> CSQ = " << sbd.cmd_CSQ() << endl;
//        cin.get();
//        cout << "=> imei = " << sbd.cmd_get_imei() << endl;
//        cin.get();
        cout << "=> write = " << sbd.cmd_write_message("AZERTYUIOPQSDFGHJ") << endl;
        cin.get();
//        cout << "=> Copy Mo Mt = " << sbd.cmd_copy_MO_MT() << endl;
//        cin.get();
//        cout << "=> read = " << sbd.cmd_read_message() << endl;
//        cin.get();
//        sbd.cmd_enable_indicator_reporting();
//        sbd.cmd_enable_alert();

//        cin.get();
//        sbd.set_gnss(48.39475416, -4.48271749);
//        sbd.cmd_status();
//        cout << "=> status = " << sbd.get_status_mo() << " " << sbd.get_status_mt() << " " << sbd.get_waiting() << endl;

//        cin.get();
//        sbd.cmd_session();

//        sleep(10);
//        cout << "MO=" << sbd.get_session_mo() << endl;
//        cout << "MOMSN=" << sbd.get_session_momsn() << endl;
//        cout << "MT=" << sbd.get_session_mt() << endl;
//        cout << "MTMSN=" << sbd.get_session_mtmsn() << endl;
//        cout << "waiting=" << sbd.get_waiting() << endl;

      }
    }
  }
}

void test5(){
  string separator = "\r\n";
  string sentence = "00012392134";
  vector<string> fields;
  boost::split(fields, sentence, boost::is_any_of(separator), boost::token_compress_on);
  cout << "fields size = " << fields.size() << endl;
  for(string &s:fields)
    cout << "=> " << s << endl;

  bool fast = true;
  string test2 = "AT+CSQ" + string(fast?"F":"");
  cout << test2 << endl;

  double m_latitude = -4.39475416;
  int lat_deg = int(m_latitude);
  double lat_min = int(abs(m_latitude - lat_deg)*60000.)/1000.;
  cout << lat_deg << lat_min << endl;

  std::ostringstream lat_string, lon_string;
  if(lat_deg<0)
    lat_string << "-";
  lat_string << setfill('0') << setw(2) << abs(lat_deg);
  lat_string << std::noshowpos << std::fixed << std::setprecision(3) << lat_min;

  cout << lat_string.str() << endl;

}

void test6(){
  LogData l;

  cout << "time: " << std::fixed << l.m_time_now << endl;
  cout << "east: " << std::fixed << l.m_east << endl;
  cout << "north: " << std::fixed << l.m_north << endl;
  cout << "gnss_speed: " << l.m_gnss_speed << endl;
  cout << "gnss_heading: " << l.m_gnss_heading << endl;

  cout << "batteries[0]: " << l.m_batteries[0] << endl;
  cout << "batteries[1]: " << l.m_batteries[1] << endl;
  cout << "batteries[2]: " << l.m_batteries[2] << endl;
  cout << "batteries[3]: " << l.m_batteries[3] << endl;

  cout << "internal_pressure: " << l.m_internal_pressure << endl;
  cout << "internal_temperature: " << l.m_internal_temperature << endl;

  cout << "current_waypoint: " << l.m_current_waypoint << endl;
  cout << "last_cmd_received: " << l.m_last_cmd_received << endl;

  string raw = l.serialize_log_state(1234);
  l.write_file("Test.raw", raw);

  LogData d;
  string draw = d.read_file("Test.raw");
  d.deserialize_log_state(draw);

  cout << endl << endl;
  cout << "time: " << std::fixed << d.m_time_now << endl;
  cout << "east: " << std::fixed << d.m_east << endl;
  cout << "north: " << std::fixed << d.m_north << endl;
  cout << "gnss_speed: " << d.m_gnss_speed << endl;
  cout << "gnss_heading: " << d.m_gnss_heading << endl;

  cout << "batteries[0]: " << d.m_batteries[0] << endl;
  cout << "batteries[1]: " << d.m_batteries[1] << endl;
  cout << "batteries[2]: " << d.m_batteries[2] << endl;
  cout << "batteries[3]: " << d.m_batteries[3] << endl;

  cout << "internal_pressure: " << d.m_internal_pressure << endl;
  cout << "internal_temperature: " << d.m_internal_temperature << endl;

  cout << "current_waypoint: " << d.m_current_waypoint << endl;
  cout << "last_cmd_received: " << d.m_last_cmd_received << endl;
}

void test7(){
  ros::Rate loop_rate(0.5);

#pragma omp parallel
  {
#pragma omp single
    {
#pragma omp task
      {
        // Read Serial Com
        while(1){
          cout << "Test2" << endl;
          loop_rate.sleep();
        }
      }

#pragma omp task
      {
        while(1){
          sleep(7);
          cout << "Test7" << endl;

        }
      }
    }
  }
}

int main(int argc, char *argv[]){
//  test6();

//  string data = "AZERTYUIOPQSDFGHJ";
//  uint16_t checksum = 0;

////  vector<uint8_t> t = {0x4b, 0x7f, 0x88, 0x0, 0x80, 0x12, 0x00, 0xc0, 0x66, 0x05, 0xff, 0x20, 0x7c, 0x40, 0x07, 0x00, 0x00};
////  vector<uint8_t> t = {0x53, 0x7f, 0x88, 0x0, 0x80, 0x12, 0x0, 0xc0, 0x66, 0x5, 0xf, 0x21, 0x7c, 0x40, 0x7, 0x0, 0x0};

//  for(char &c:data){
//    checksum += static_cast<uint8_t>(c);
//    cout << c << " | " << static_cast<uint8_t>(c) << " " << checksum << endl;
//  }

////  string data_checksum = data;
////  data_checksum += uint8_t(checksum>>8);
////  data_checksum += uint8_t(checksum);

//  cout << std::hex << (int)(checksum >> 8) << endl;
//  cout << std::hex << (int)(checksum & 0xFF) << endl;

////  for(int i=0; i<data_checksum.size(); i++){
////    cout << std::hex << (int)data_checksum.c_str()[i] << endl;
////  }
////  cout << data_checksum << endl;

//  data += static_cast<char>(checksum >> 8);
//  data += static_cast<char>(checksum & 0xFF);
//  cout << data << endl;

//  test4();

  vector<double> heading_memory = {0., 180.};
  double sum_sin=0.0, sum_cos=0.0;
  for(double &h:heading_memory){
    sum_sin += sin(h*M_PI/180.);
    sum_cos += cos(h*M_PI/180.);
  }
  cout << "heading = " << atan2(sum_sin, sum_cos)*180./M_PI << endl;


  return 0;
}


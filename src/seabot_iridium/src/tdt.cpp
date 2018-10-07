#include "logtdt.h"
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

#include "omp.h"

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
        cout << "=> CSQ = " << sbd.cmd_CSQ() << endl;
        cin.get();
        cout << "=> imei = " << sbd.cmd_get_imei() << endl;
        cin.get();
        cout << "=> write = " << sbd.cmd_write_message("Test") << endl;
        cin.get();
        cout << "=> Copy Mo Mt = " << sbd.cmd_copy_MO_MT() << endl;
        cin.get();
        cout << "=> read = " << sbd.cmd_read_message() << endl;
        cin.get();
        sbd.cmd_enable_indicator_reporting();
        sbd.cmd_enable_alert();

        cin.get();
        sbd.set_gnss(48.39475416, -4.48271749);
        sbd.cmd_status();
        cout << "=> status = " << sbd.get_status_mo() << " " << sbd.get_status_mt() << " " << sbd.get_waiting() << endl;

        cin.get();
        sbd.cmd_session();

        sleep(10);
        cout << "MO=" << sbd.get_session_mo() << endl;
        cout << "MOMSN=" << sbd.get_session_momsn() << endl;
        cout << "MT=" << sbd.get_session_mt() << endl;
        cout << "MTMSN=" << sbd.get_session_mtmsn() << endl;
        cout << "waiting=" << sbd.get_waiting() << endl;

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

int main(int argc, char *argv[]){

  test4();

  //  string file_name(argv[1]);
  //  logTDT.deserialize_log_TDT1(file_name);

  //  stringstream data;
  //  data << "file_name: " << file_name << endl;
  
  //  data << "east: " << std::fixed << logTDT.m_east << endl;
  //  data << "north: " << std::fixed << logTDT.m_north << endl;
  //  data << "gnss_speed: " << logTDT.m_gnss_speed << endl;
  //  data << "gnss_heading: " << logTDT.m_gnss_heading << endl;

  //  data << "seabot_state: " << logTDT.m_seabot_state << endl;

  //  data << "batteries[0]: " << logTDT.m_batteries[0] << endl;
  //  data << "batteries[1]: " << logTDT.m_batteries[1] << endl;
  //  data << "batteries[2]: " << logTDT.m_batteries[2] << endl;
  //  data << "batteries[3]: " << logTDT.m_batteries[3] << endl;

  //  data << "internal_pressure: " << logTDT.m_internal_pressure << endl;
  //  data << "internal_temperature: " << logTDT.m_internal_temperature << endl;

  //  data << "current_waypoint: " << logTDT.m_current_waypoint << endl;


  //  string output_filename1 = string(argv[1]) + "_decode.yaml";
  //  std::ofstream outfile(output_filename1);
  //  outfile << data.str();
  //  outfile.close();

  //  struct passwd *pw = getpwuid(getuid());
  //  const char *homedir = pw->pw_dir;
  //  string output_filename2 = string(homedir) + "/last_tdt1.yaml";
  //  std::ofstream outfile2(output_filename2);
  //  outfile2 << data.str();
  //  outfile2.close();

  return 0;
}


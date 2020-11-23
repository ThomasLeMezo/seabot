#include "logData.h"
#include "sbd.h"
#include "missionxml.h"

using namespace std;

int main(int argc, char *argv[]){
  string file_path(argv[1]);
  cout << file_path << endl;

  // Test if is at surface for sufficient period of time
  LogData log_cmd;
  string data_raw = log_cmd.read_file(file_path);

  cout << data_raw << endl;
  log_cmd.deserialize_log_CMD(data_raw);

  cout << log_cmd.m_waypoint_list.size() << endl;

  // ToDO : write new mission file
  MissionXML m(log_cmd);
  m.write("/home/lemezoth/Downloads/mission_iridium.xml");

  return 0;
}




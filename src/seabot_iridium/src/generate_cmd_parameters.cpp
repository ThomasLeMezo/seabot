#include "logData.h"
#include "sbd.h"

using namespace std;

int main(int argc, char *argv[]){
  string file_path(argv[1]);

  LogData logData;
  logData.m_enable_mission = (bool) strtoul(argv[2], NULL, 0);
  logData.m_enable_flash = (bool) strtoul(argv[3], NULL, 0);
  logData.m_enable_depth = (bool) strtoul(argv[4], NULL, 0);
  logData.m_enable_engine = (bool) strtoul(argv[5], NULL, 0);
  logData.m_period_message = (bool) strtoul(argv[6], NULL, 0);

  string cmd_raw = logData.serialize_log_CMD_parameters();
  logData.write_file(file_path, cmd_raw, NB_BITS_CMD_PARAMETERS);
  return 0;
}


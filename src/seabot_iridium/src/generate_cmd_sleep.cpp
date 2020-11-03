#include "logData.h"
#include "sbd.h"

using namespace std;

int main(int argc, char *argv[]){
    string file_path(argv[1]);
    unsigned int sleep_time = strtoul(argv[2], NULL, 0);

    LogData logData;
    logData.m_sleep_time = sleep_time; // in min

    string cmd_raw = logData.serialize_log_CMD_sleep();
    logData.write_file(file_path, cmd_raw, NB_BITS_CMD_SLEEP);
  return 0;
}

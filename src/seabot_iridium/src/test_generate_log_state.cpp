#include "logData.h"
#include "sbd.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;

int main(int argc, char *argv[]){
    LogData logData;

    logData.m_mean_east = 937052;
    logData.m_mean_north = 6227185;
    logData.m_gnss_speed = 3;
    logData.m_mean_heading = 150;

    logData.m_safety_published_frequency = false;
    logData.m_safety_depth_limit = false;
    logData.m_safety_batteries_limit = false;
    logData.m_safety_depressurization = false;
    logData.m_enable_mission = true;
    logData.m_enable_depth = true;
    logData.m_enable_engine = true;
    logData.m_enable_flash = true;

    logData.m_batteries[0] = 12.1;
    logData.m_batteries[1] = 12.1;
    logData.m_batteries[2] = 12.1;
    logData.m_batteries[3] = 12.1;

    logData.m_internal_pressure = 690;
    logData.m_internal_temperature = 23;
    logData.m_internal_humidity = 50;

    logData.m_current_waypoint = 40;
    logData.m_last_cmd_received = 1;

    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    string filename;
    filename.append(homedir);
    filename.append("/test_log_state.sbd");

    string data = logData.serialize_log_state(1605976656);
    logData.write_file(filename, data, data.size()*8);
  return 0;
}

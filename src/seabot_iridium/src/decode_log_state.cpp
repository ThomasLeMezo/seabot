#include "logData.h"
#include "missionxml.h"
#include <iostream>
#include <fstream>
#include <array>
#include <cstring>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "sbd.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

using namespace std;

int main(int argc, char *argv[]){
    std::string login_name(std::getenv("USER"));

    string path_iridium("/home/" + login_name + "/iridium/received/");
    string path_archive(path_iridium + "archive/");

    boost::filesystem::path dir_archive(path_archive);
    boost::filesystem::create_directory(dir_archive);

    string dir_path(argv[1]);
    string file_name(argv[2]);
    string imei(argv[3]);

    LogData logData;

    string file_path = dir_path+file_name;
    string data_raw = logData.read_file(file_path);
    logData.deserialize_log_state(data_raw);

    stringstream data;
    data << "file_name: " << file_name << endl;

    data << "east: " << std::fixed << logData.m_east << endl;
    data << "north: " << std::fixed << logData.m_north << endl;
    data << "gnss_speed: " << logData.m_gnss_speed << endl;
    data << "gnss_heading: " << logData.m_gnss_heading << endl;

    data << "safety_published_frequency: " << logData.m_safety_published_frequency << endl;
    data << "safety_depth_limit: " << logData.m_safety_depth_limit << endl;
    data << "safety_batteries_limit: " << logData.m_safety_batteries_limit << endl;
    data << "safety_depressurization: " << logData.m_safety_depressurization << endl;
    data << "enable_mission: " << logData.m_enable_mission << endl;
    data << "enable_depth: " << logData.m_enable_depth << endl;
    data << "enable_engine: " << logData.m_enable_engine << endl;
    data << "enable_flash: " << logData.m_enable_flash << endl;

    data << "batteries[0]: " << logData.m_batteries[0] << endl;
    data << "batteries[1]: " << logData.m_batteries[1] << endl;
    data << "batteries[2]: " << logData.m_batteries[2] << endl;
    data << "batteries[3]: " << logData.m_batteries[3] << endl;

    data << "internal_pressure: " << logData.m_internal_pressure << endl;
    data << "internal_temperature: " << logData.m_internal_temperature << endl;

    data << "current_waypoint: " << logData.m_current_waypoint << endl;
    data << "last_cmd_received: " << logData.m_last_cmd_received << endl;


    string path_dir = path_archive + imei + "/";
    boost::filesystem::path dir(path_dir);
    boost::filesystem::create_directory(dir);

    string output_filename1 = path_dir + file_name + "_decode.yaml";
    std::ofstream outfile(output_filename1);
    outfile << data.str();
    outfile.close();

    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    string output_filename2 = path_iridium + "/" + "last_received.yaml";
    std::ofstream outfile2(output_filename2);
    outfile2 << data.str();
    outfile2.close();

  return 0;
}

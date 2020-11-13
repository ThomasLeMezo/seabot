#include "logData.h"

#include "boost/filesystem.hpp"
#include <boost/multiprecision/cpp_int.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <iterator>

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

#include <unistd.h>
#include <sys/types.h>

#include <time.h>

using namespace std;
using boost::multiprecision::cpp_int;

bool LogData::deserialize_log_CMD_sleep(const string &message){
  cout << "Deserialize log sleep" << endl;

  uint_cmd_sleep_t data = (uint_cmd_sleep_t(1) << NB_BITS_CMD_SLEEP) - 1;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  unsigned int bit_position = 4;
  bit_position += deserialize_data<uint_cmd_sleep_t>(data, 12, bit_position, m_sleep_time);
  return true;
}

bool LogData::deserialize_log_CMD_parameters(const string &message){
  cout << "Deserialize log parameters" << endl;

  uint_cmd_parameters_t data;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  unsigned int bit_position = 4;
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_mission);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_flash);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_depth);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_engine);

  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 8, bit_position, m_period_message);

  return true;
}

unsigned int LogData::deserialize_log_CMD_waypoint(const std::string &message, const unsigned int &bit_position){
  string message_wp_type = message.substr(static_cast<size_t>(bit_position/8), static_cast<size_t>(NB_BITS_CMD_WAYPOINT_TYPE/8));
  uint_cmd_waypoint_type_t data_type= (uint_cmd_waypoint_type_t(1)<<NB_BITS_CMD_WAYPOINT_TYPE) -1;
  memcpy(data_type.backend().limbs(), message_wp_type.c_str(), message_wp_type.size());

   unsigned int bit_position_local = 1;
   unsigned int duration;

  if((data_type & 0x1) == 1){
      string message_wp_type = message.substr(static_cast<size_t>(bit_position/8), static_cast<size_t>(NB_BITS_CMD_WAYPOINT_TRAJ/8));
      uint_cmd_waypoint_traj_t data_traj = (uint_cmd_waypoint_traj_t(1)<<NB_BITS_CMD_WAYPOINT_TRAJ) -1;
      memcpy(data_traj.backend().limbs(), message_wp_type.c_str(), message_wp_type.size());

      int east, north;
      bit_position_local += deserialize_data<uint_cmd_waypoint_traj_t>(data_traj, 9, bit_position_local, duration);
      bit_position_local += deserialize_data<uint_cmd_waypoint_traj_t>(data_traj, 15, bit_position_local, east);
      bit_position_local += deserialize_data<uint_cmd_waypoint_traj_t>(data_traj, 15, bit_position_local, north);

      Waypoint w(duration*60.0,0.0, east*4.0+m_mean_east, north*4.0+m_mean_north, true);
      m_waypoint_list.push_back(w);
  }
  else{
      string message_wp_type = message.substr(static_cast<size_t>(bit_position/8), static_cast<size_t>(NB_BITS_CMD_WAYPOINT_DEPTH/8));
      uint_cmd_waypoint_depth_t data_depth = (uint_cmd_waypoint_depth_t(1)<<NB_BITS_CMD_WAYPOINT_DEPTH) -1;
      memcpy(data_depth.backend().limbs(), message_wp_type.c_str(), message_wp_type.size());

      unsigned int depth;
      bool seafloor_landing=false;
      bit_position_local += deserialize_data<uint_cmd_waypoint_depth_t>(data_depth, 9, bit_position_local, duration);
      bit_position_local += deserialize_data<uint_cmd_waypoint_depth_t>(data_depth, 11, bit_position_local, depth);
      bit_position_local += deserialize_data<uint_cmd_waypoint_depth_t>(data_depth, 1, bit_position_local, seafloor_landing);
      bit_position_local += 2;

      Waypoint w(duration*60.0,depth/4.0, 0.0, 0.0, false, seafloor_landing);
      m_waypoint_list.push_back(w);
  }

  return bit_position_local;
}

bool LogData::deserialize_log_CMD_mission(const string &message){
  cout << "Deserialize log mission" << endl;
  m_waypoint_list.clear();

  uint_cmd_mission_header_t data = (uint_cmd_mission_header_t(1) << NB_BITS_CMD_MISSION_HEADER) - 1;
  memcpy(data.backend().limbs(), message.c_str(), NB_BITS_CMD_MISSION_HEADER/8);

  unsigned int bit_position = 4;
  unsigned int nb_waypoints, start_time;

  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 8, bit_position, nb_waypoints);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 22, bit_position, start_time);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 15, bit_position, m_mean_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 15, bit_position, m_mean_north, L93_NORTH_MIN, L93_NORTH_MAX);

  m_start_time = REF_POSIX_TIME + start_time*60;

  cout << "nb_waypoints = " << nb_waypoints << endl;
  cout << "m_start_time time = " << m_start_time << endl;
  cout << "start_time = " << start_time << endl;
  cout << "m_mean_north = " << m_mean_north << endl;
  cout << "m_mean_east = " << m_mean_east << endl;

  // Deserialize waypoint list and add to list
  for(size_t i=0; i<nb_waypoints; i++)
      bit_position += deserialize_log_CMD_waypoint(message, bit_position);

  return true;
}

std::string LogData::serialize_log_state(const long long &time){
  // Set all bits to 1 (enable add option)
  uint_log1_t data = (uint_log1_t(1) << NB_BITS_LOG1) - 1;

  unsigned int bit_position = 0;
  bit_position += serialize_data<uint_log1_t>(data, 4, bit_position, LOG_STATE);

  // Get timestamped of the day
  struct tm * timeinfo;
  time_t t1 = time;
  timeinfo = gmtime(&t1);
  timeinfo->tm_hour = 0;
  timeinfo->tm_min = 0;
  timeinfo->tm_sec = 0;
  time_t t2 = mktime(timeinfo);

  int time_sec_day = t1-t2;
  int time_LQ = static_cast<int>(round(time_sec_day/3.0));
  bit_position += serialize_data<uint_log1_t>(data, 14, bit_position, time_LQ, 0, ((1<<14) -1));
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, m_mean_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, m_mean_north, L93_NORTH_MIN, L93_NORTH_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_mean_heading, 0, 359.0);

  unsigned char state = 0;
  state |= (m_safety_published_frequency & 0x1) << 0;
  state |= (m_safety_depth_limit & 0x1) << 1;
  state |= (m_safety_batteries_limit & 0x1) << 2;
  state |= (m_safety_depressurization & 0x1) << 3;
  state |= (m_enable_mission & 0x1) << 4;
  state |= (m_enable_depth & 0x1) << 5;
  state |= (m_enable_engine & 0x1) << 6;
  state |= (m_enable_flash & 0x1) << 7;
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, state);

  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[0], 9, 12.4);
  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[1], 9, 12.4);
  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[2], 9, 12.4);
  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[3], 9, 12.4);

  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_internal_humidity, 50.0, 100.0);

  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_current_waypoint);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_last_cmd_received);
  return string((char*)data.backend().limbs(), NB_BITS_LOG1/8);
}

bool LogData::write_file(const string &file_name, const string &data, const unsigned int nb_bits){
  ofstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cerr << "Unable to open " << file_name << " new log file : " << errno << endl;
    return false;
  }

  save_file.write(data.c_str(), nb_bits/8);
  save_file.close();

  return true;
}

std::string LogData::read_file(const std::string &file_name){
  std::ifstream save_file;
  save_file.open(file_name);
  if(!save_file.is_open()){
    cout << "File cannot be open" << strerror(errno) << endl;
    cout << "file_name = " << file_name << endl;
    return "";
  }
  std::string data((std::istreambuf_iterator<char>(save_file)),
                   std::istreambuf_iterator<char>());

  save_file.close();
  return data;
}

bool LogData::deserialize_log_CMD(const string &raw_data){
  unsigned char message_type = raw_data[0] & 0xF;

  switch(message_type){
  case CMD_SLEEP:
    m_msg_type = CMD_SLEEP;
    cout << "CMD Sleep" << endl;
    deserialize_log_CMD_sleep(raw_data);
    break;
  case CMD_PARAMETERS:
    cout << "CMD Parameters" << endl;
    m_msg_type = CMD_PARAMETERS;
    deserialize_log_CMD_parameters(raw_data);
      break;
  case CMD_MISSION_NEW:
    cout << "CMD Mission NEW" << endl;
    m_msg_type = CMD_MISSION_NEW;
    deserialize_log_CMD_mission(raw_data);
    break;
  case CMD_MISSION_KEEP:
    cout << "CMD Mission" << endl;
    m_msg_type = CMD_MISSION_KEEP;
    deserialize_log_CMD_mission(raw_data);
    break;

  default:
      cout << "MESSAGE TYPE NOT FOUND : " << static_cast<int>(message_type) << endl;
    break;
  }
  return true;
}

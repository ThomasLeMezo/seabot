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

using namespace std;
using boost::multiprecision::cpp_int;

std::string LogData::serialize_log_CMD_sleep(){
  //  size_t nb_bits = 5*4; // must be a multiple of 8
  uint_cmd_sleep_t data;

  int bit_position = 0;
  bit_position += serialize_data<uint_cmd_sleep_t>(data, 4, bit_position, CMD_SLEEP);
  bit_position += serialize_data<uint_cmd_sleep_t>(data, 12, bit_position, min(m_sleep_time, (unsigned int)(1<<12)-1));

  return string((char*)data.backend().limbs(), NB_BITS_CMD_SLEEP/8);
}

bool LogData::deserialize_log_CMD_sleep(const string &message){
  cout << "Deserialize log sleep" << endl;

  uint_cmd_sleep_t data = (uint_cmd_sleep_t(1) << NB_BITS_CMD_SLEEP) - 1;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  int bit_position = 0;
  unsigned int tmp;
  bit_position += deserialize_data<uint_cmd_sleep_t>(data, 4, bit_position, tmp);
  m_cmd_type = (CMD_TYPE)tmp;
  bit_position += deserialize_data<uint_cmd_sleep_t>(data, 12, bit_position, m_sleep_time);
  return true;
}

std::string LogData::serialize_log_CMD_parameters(){
  //  size_t nb_bits = 5*4; // must be a multiple of 4
  uint_cmd_parameters_t data;

  int bit_position = 0;
  bit_position += serialize_data<uint_cmd_parameters_t>(data, 4, bit_position, CMD_PARAMETERS);

  bit_position += serialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_mission);
  bit_position += serialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_flash);
  bit_position += serialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_depth);
  bit_position += serialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_engine);

  bit_position += serialize_data<uint_cmd_parameters_t>(data, 8, bit_position, m_period_message);

  return string((char*)data.backend().limbs(), NB_BITS_CMD_PARAMETERS/8);
}

bool LogData::deserialize_log_CMD_parameters(const string &message){
  cout << "Deserialize log parameters" << endl;

  uint_cmd_parameters_t data;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  int bit_position = 0;
  unsigned int tmp;
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 4, bit_position, tmp);
  m_cmd_type = (CMD_TYPE)tmp;
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_mission);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_flash);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_depth);
  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 1, bit_position, m_enable_engine);

  bit_position += deserialize_data<uint_cmd_parameters_t>(data, 8, bit_position, m_period_message);

  return true;
}

std::string LogData::serialize_log_CMD_waypoint(const Waypoint &w){
  uint_cmd_waypoint_t data = (uint_cmd_waypoint_t(1)<<NB_BITS_CMD_WAYPOINT) -1;

  int bit_position = 0;
  unsigned int time_end = (unsigned int)min(round((w.time_end-m_offset_time)/60.), (1<<12)-1.);
  if(time_end==0)
    cout << "TIME END is not correctly set" << endl;
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 12, bit_position, time_end);
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 16, bit_position, w.east, -50e3, 50e3);
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 16, bit_position, w.north, -50e3, 50e3);
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 8, bit_position, w.depth, 0.0, 51.2);

  return string((char*)data.backend().limbs(), NB_BITS_CMD_WAYPOINT/8);
}

bool LogData::deserialize_log_CMD_waypoint(const std::string &message){
  uint_cmd_waypoint_t data = (uint_cmd_waypoint_t(1)<<NB_BITS_CMD_WAYPOINT) -1;
  memcpy(data.backend().limbs(), message.c_str(), message.size());

  int bit_position = 0;
  double east, north, depth;
  unsigned int time_end;
  bit_position += deserialize_data<uint_cmd_waypoint_t>(data, 12, bit_position, time_end);
  bit_position += deserialize_data<uint_cmd_waypoint_t>(data, 16, bit_position, east, -50e3, 50e3);
  bit_position += deserialize_data<uint_cmd_waypoint_t>(data, 16, bit_position, north, -50e3, 50e3);
  bit_position += deserialize_data<uint_cmd_waypoint_t>(data, 8, bit_position, depth, 0.0, 51.2);

  Waypoint w(time_end*60.+m_offset_time,depth, north+m_offset_east, east+m_offset_north);
  m_waypoint_list.push_back(w);
  return true;
}

std::string LogData::serialize_log_CMD_mission(){
  // Write Header
  uint_cmd_mission_header_t data = (uint_cmd_mission_header_t(1)<<NB_BITS_CMD_MISSION_HEADER) -1;

  int bit_position = 0;
  unsigned int nb_waypoint = min(m_waypoint_list.size(), (size_t)(1<<8)-1);
  unsigned int offset_time = max(0.,round((m_offset_time-TIME_POSIX_START)/60.));
  if(offset_time==0)
    cout << "WARN offset time for mission is 0"<< endl;
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 4, bit_position, CMD_MISSION);
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 8, bit_position, nb_waypoint);
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 18, bit_position, offset_time, 0, (1<<18 -1));
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 21, bit_position, m_offset_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 21, bit_position, m_offset_north, L93_NORTH_MIN, L93_NORTH_MAX);

  string message = string((char*)data.backend().limbs(), NB_BITS_CMD_MISSION_HEADER/8);

  // Write waypoint list
  for(size_t i = 0; i<nb_waypoint; i++){
    message += serialize_log_CMD_waypoint(m_waypoint_list[i]);
  }

  return message;
}

bool LogData::deserialize_log_CMD_mission(const string &message){
  cout << "Deserialize log mission" << endl;

  uint_cmd_mission_header_t data = (uint_cmd_mission_header_t(1) << NB_BITS_CMD_MISSION_HEADER) - 1;
  memcpy(data.backend().limbs(), message.c_str(), NB_BITS_CMD_MISSION_HEADER/8);

  int bit_position = 0;
  unsigned int cmd_type_tmp, nb_waypoints, offset_time;

  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 4, bit_position, cmd_type_tmp);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 8, bit_position, nb_waypoints);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 18, bit_position, offset_time);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 21, bit_position, m_offset_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += deserialize_data<uint_cmd_mission_header_t>(data, 21, bit_position, m_offset_north, L93_NORTH_MIN, L93_NORTH_MAX);
  m_offset_time = offset_time*60.+TIME_POSIX_START;

  // Write waypoint list
  for(size_t i=0; i<nb_waypoints; i++){
    size_t s_bit = NB_BITS_CMD_MISSION_HEADER/8+i*NB_BITS_CMD_WAYPOINT;
    deserialize_log_CMD_waypoint(message.substr(s_bit, NB_BITS_CMD_WAYPOINT));
  }

  return true;
}

std::string LogData::serialize_log_state(const long long &time){
  size_t nb_bits = 32*4; // must be a multiple of 4
  uint_log1_t data = (uint_log1_t(1)<<nb_bits) -1;

  int bit_position = 0;

  unsigned int time_now = max(0.,round((time-TIME_POSIX_START)/60.));
  bit_position += serialize_data<uint_log1_t>(data, 18, bit_position, time_now, 0, (1<<18 -1));
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, m_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, m_north, L93_NORTH_MIN, L93_NORTH_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_heading, 0, 359.0);

  unsigned char state = 0;
  state |= (m_safety_published_frequency & 0b1) << 0;
  state |= (m_safety_depth_limit & 0b1) << 1;
  state |= (m_safety_batteries_limit & 0b1) << 2;
  state |= (m_safety_depressurization & 0b1) << 3;
  state |= (m_enable_mission & 0b1) << 4;
  state |= (m_enable_depth & 0b1) << 5;
  state |= (m_enable_engine & 0b1) << 6;
  state |= (m_enable_flash & 0b1) << 7;
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

bool LogData::deserialize_log_state(const string &data_raw){

  uint_log1_t data = (uint_log1_t(1) << NB_BITS_LOG1) - 1;
  memcpy(data.backend().limbs(), data_raw.c_str(), data_raw.size()); // To be checked !
  cout << data << endl;

  cout << "Start deserializing data" << endl;
  int bit_position = 0;
  unsigned int time_now;
  bit_position += deserialize_data<uint_log1_t>(data, 18, bit_position, time_now);
  bit_position += deserialize_data<uint_log1_t>(data, 21, bit_position, m_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += deserialize_data<uint_log1_t>(data, 21, bit_position, m_north, L93_NORTH_MIN, L93_NORTH_MAX);
  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_heading, 0, 359.0);

  unsigned int state = 0;
  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, state);
  m_safety_published_frequency = (state >> 0) & 0b1;
  m_safety_depth_limit = (state >> 1) & 0b1;
  m_safety_batteries_limit = (state >> 2) & 0b1;
  m_safety_depressurization = (state >> 3) & 0b1;
  m_enable_mission = (state >> 4) & 0b1;
  m_enable_depth = (state >> 5) & 0b1;
  m_enable_engine = (state >> 6) & 0b1;
  m_enable_flash = (state >> 7) & 0b1;

  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[0], 9, 12.4);
  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[1], 9, 12.4);
  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[2], 9, 12.4);
  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[3], 9, 12.4);

  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);
  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_internal_humidity,50.0, 100.0);

  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_current_waypoint);
  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_last_cmd_received);

  m_time_now = time_now*60.+TIME_POSIX_START;

  return true;
}

bool LogData::deserialize_log_CMD(const string &raw_data){
  unsigned char message_type = raw_data[0] & 0xF;

  switch(message_type){
  case CMD_SLEEP:
    m_cmd_type = CMD_SLEEP;
    cout << "CMD Sleep" << endl;
    deserialize_log_CMD_sleep(raw_data);
    break;
  case CMD_MISSION:
    cout << "CMD Mission" << endl;
    m_cmd_type = CMD_MISSION;
    deserialize_log_CMD_mission(raw_data);
    break;
  default:
    break;
  }
}

#include "logtdt.h"

#include "boost/filesystem.hpp"

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

#include <unistd.h>
#include <sys/types.h>

using namespace std;
using boost::multiprecision::cpp_int;

bool LogTDT::serialize_log_CMD_sleep(const string &file_name){
  ofstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cerr << "Unable to open " << file_name << " new cmd file : " << errno << endl;
    return false;
  }

  //  size_t nb_bits = 5*4; // must be a multiple of 4
  uint_cmd_sleep_t data = (uint_cmd_sleep_t(1)<<NB_BITS_CMD_SLEEP) -1;

  int bit_position = 0;
  bit_position += serialize_data<uint_cmd_sleep_t>(data, 4, bit_position, CMD_SLEEP);
  bit_position += serialize_data<uint_cmd_sleep_t>(data, 12, bit_position, min(m_sleep_time, (unsigned int)(1<<12)-1));

  save_file.write((char*)&data, NB_BITS_CMD_SLEEP/8);
  save_file.close();

  return true;
}

bool LogTDT::deserialize_log_CMD_sleep(const string &file_name){
  cout << "Deserialize log sleep" << endl;
  ifstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cout << "File cannot be open" << strerror(errno) << endl;
    return false;
  }

  uint_cmd_sleep_t data = (uint_cmd_sleep_t(1) << NB_BITS_CMD_SLEEP) - 1;

  try{
    save_file.read((char*)data.backend().limbs(), NB_BITS_CMD_SLEEP/8);
  }
  catch(std::ios_base::failure& e){
    cout << "ERROR Reading : " << e.what() << endl;
  }
  save_file.close();

  cout << "Start deserializing data" << endl;
  int bit_position = 0;
  unsigned int tmp;
  bit_position += deserialize_data<uint_cmd_sleep_t>(data, 4, bit_position, tmp);
  m_cmd_type = (CMD_TYPE)tmp;
  bit_position += deserialize_data<uint_cmd_sleep_t>(data, 12, bit_position, m_sleep_time);
  return true;
}

bool LogTDT::serialize_log_CMD_waypoint(ofstream &save_file, const Waypoint &w){
  if(!save_file.is_open()){
    cerr << "Unable to open file : " << errno << endl;
    return false;
  }

  uint_cmd_waypoint_t data = (uint_cmd_waypoint_t(1)<<NB_BITS_CMD_WAYPOINT) -1;

  int bit_position = 0;
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 12, bit_position, min(w.time_end, (unsigned long)(1<<12)-1));
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 16, bit_position, w.east, -50e3, 50e3);
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 16, bit_position, w.north, -50e3, 50e3);
  bit_position += serialize_data<uint_cmd_waypoint_t>(data, 8, bit_position, w.depth, 0.0, 51.2);

  save_file.write((char*)&data, NB_BITS_CMD_WAYPOINT/8);

  return true;
}

bool LogTDT::serialize_log_CMD_mission(const string &file_name){
  ofstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cerr << "Unable to open " << file_name << " new cmd file : " << errno << endl;
    return false;
  }

  // Write Header
  uint_cmd_mission_header_t data = (uint_cmd_mission_header_t(1)<<NB_BITS_CMD_MISSION_HEADER) -1;

  int bit_position = 0;
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 4, bit_position, CMD_MISSION);
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 8, bit_position, min(m_waypoint_list.size(), (size_t)(1<<8)-1));
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 18, bit_position, m_offset_time/60., TIME_POSIX_START, TIME_POSIX_START+60.*(1<<18 -1));
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 21, bit_position, m_offset_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += serialize_data<uint_cmd_mission_header_t>(data, 21, bit_position, m_offset_north, L93_NORTH_MIN, L93_NORTH_MAX);

  save_file.write((char*)&data, NB_BITS_CMD_MISSION_HEADER/8);

  // Write waypoint list
  for(Waypoint &w:m_waypoint_list){
    serialize_log_CMD_waypoint(save_file, w);
  }

  save_file.close();

  return true;
}

bool LogTDT::serialize_log_TDT1(const string &file_name){
  ofstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cerr << "Unable to open " << file_name << " new log file : " << errno << endl;
    return false;
  }

  size_t nb_bits = 26*4; // must be a multiple of 4
  uint_log1_t data = (uint_log1_t(1)<<nb_bits) -1;

  int bit_position = 0;
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, m_east, L93_EAST_MIN, L93_EAST_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 21, bit_position, m_north, L93_NORTH_MIN, L93_NORTH_MAX);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_heading, 0, 359.0);

  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_seabot_state);

  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[0], 9, 12.4);
  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[1], 9, 12.4);
  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[2], 9, 12.4);
  bit_position += serialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[3], 9, 12.4);

  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);
  bit_position += serialize_data<uint_log1_t>(data, 6, bit_position, m_internal_humidity, 50.0, 100.0);

  bit_position += serialize_data<uint_log1_t>(data, 8, bit_position, m_current_waypoint);

  save_file.write((char*)&data, NB_BITS_LOG1/8);
  save_file.close();

  return true;
}

bool LogTDT::deserialize_log_TDT1(const string &file_name){
  cout << "Deserialize log TDT1" << endl;
  ifstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cout << "File cannot be open" << strerror(errno) << endl;
    return false;
  }

  uint_log1_t data = (uint_log1_t(1) << NB_BITS_LOG1) - 1;

  try{
    save_file.read((char*)data.backend().limbs(), NB_BITS_LOG1/8);
  }
  catch(std::ios_base::failure& e){
    cout << "ERROR Reading : " << e.what() << endl;
  }
  save_file.close();

  cout << data << endl;

  cout << "Start deserializing data" << endl;
  int bit_position = 0;
  bit_position += deserialize_data<uint_log1_t>(data, 21, bit_position, m_east, 0, 1300000);
  bit_position += deserialize_data<uint_log1_t>(data, 21, bit_position, m_north, 6000000, 7200000);
  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_gnss_heading, 0, 359.0);

  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_seabot_state);

  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[0], 9, 12.4);
  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[1], 9, 12.4);
  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[2], 9, 12.4);
  bit_position += deserialize_data<uint_log1_t>(data, 5, bit_position, m_batteries[3], 9, 12.4);

  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);
  bit_position += deserialize_data<uint_log1_t>(data, 6, bit_position, m_internal_humidity,50.0, 100.0);

  bit_position += deserialize_data<uint_log1_t>(data, 8, bit_position, m_current_waypoint);

  return true;
}

bool LogTDT::deserialize_log_CMD(const string &file_name, CMD_TYPE &type){
  cout << "Deserialize log TDT1" << endl;
  ifstream save_file;
  save_file.open(file_name);

  if(!save_file.is_open()){
    cout << "File cannot be open" << strerror(errno) << endl;
    return false;
  }

  // Read heading message type
  char data;
  try{
    save_file.read(&data, 1);
  }
  catch(std::ios_base::failure& e){
    cout << "ERROR Reading : " << e.what() << endl;
  }
  save_file.close();

  unsigned char message_type = data & (1<<4 -1);

  switch(message_type){
  case CMD_SLEEP:
    m_cmd_type = CMD_SLEEP;

    break;
  case CMD_MISSION:
    m_cmd_type = CMD_MISSION;

    break;
  default:
    break;
  }
}

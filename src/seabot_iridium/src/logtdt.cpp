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

int LogTDT::serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double &value_max){
  double scale = (double)(1<<nb_bit-1)/(value_max-value_min);
  double bit_max = (1<<nb_bit-1);
  long unsigned int v = (long unsigned int)(min(max(round((value-value_min)*scale), 0.0), bit_max));
  long unsigned int v_max = (1<<nb_bit)-1;
  v = min(v, v_max);

  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  bits &= ~mask;
  bits |= (uint_log1_t(v) & ((uint_log1_t(1)<<nb_bit)-1)) << start_bit;
  return nb_bit;
}

int LogTDT::deserialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, double &value, const double &value_min, const double &value_max){
  double scale = (double)(1<<nb_bit-1)/(value_max-value_min);
  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  uint_log1_t v = (bits & mask)>>start_bit;
//  cout << v << endl;
  value = (double)v;
  value /= scale;
  value += value_min;

  return nb_bit;
}

int LogTDT::serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const unsigned int &value){
  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  bits &= ~mask;
  bits |= (uint_log1_t(value) & ((uint_log1_t(1)<<nb_bit)-1)) << start_bit;
  return nb_bit;
}

int LogTDT::deserialize_data(const uint_log1_t &bits, const int &nb_bit, const int &start_bit, unsigned int &value){
  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  uint_log1_t v = (bits & mask)>>start_bit;

  value = (double)v;
  return nb_bit;
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
  bit_position += serialize_data(data, 21, bit_position, m_east, 0, 1300000);
  bit_position += serialize_data(data, 21, bit_position, m_north, 6000000, 7200000);
  bit_position += serialize_data(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += serialize_data(data, 8, bit_position, m_gnss_heading, 0, 359.0);

  bit_position += serialize_data(data, 8, bit_position, m_seabot_state);

  bit_position += serialize_data(data, 5, bit_position, m_batteries[0], 9, 12.4);
  bit_position += serialize_data(data, 5, bit_position, m_batteries[1], 9, 12.4);
  bit_position += serialize_data(data, 5, bit_position, m_batteries[2], 9, 12.4);
  bit_position += serialize_data(data, 5, bit_position, m_batteries[3], 9, 12.4);

  bit_position += serialize_data(data, 6, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += serialize_data(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);
  bit_position += serialize_data(data, 6, bit_position, m_internal_humidity, 50.0, 100.0);

  bit_position += serialize_data(data, 8, bit_position, m_current_waypoint);

  save_file.write((char*)&data, NB_BITS/8);
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
  else{
    cout << "File is open" << endl;
  }

  uint_log1_t data = uint_log1_t(1) << NB_BITS - 1;

  try{
    save_file.read((char*)data.backend().limbs(), NB_BITS/8);
  }
  catch(std::ios_base::failure& e){
    cout << "ERROR Reading : " << e.what() << endl;
  }

  cout << data << endl;

  cout << "Start deserializing data" << endl;
  int bit_position = 0;
  bit_position += deserialize_data(data, 21, bit_position, m_east, 0, 1300000);
  bit_position += deserialize_data(data, 21, bit_position, m_north, 6000000, 7200000);
  bit_position += deserialize_data(data, 8, bit_position, m_gnss_speed, 0, 5.0);
  bit_position += deserialize_data(data, 8, bit_position, m_gnss_heading, 0, 359.0);

  bit_position += deserialize_data(data, 8, bit_position, m_seabot_state);

  bit_position += deserialize_data(data, 5, bit_position, m_batteries[0], 9, 12.4);
  bit_position += deserialize_data(data, 5, bit_position, m_batteries[1], 9, 12.4);
  bit_position += deserialize_data(data, 5, bit_position, m_batteries[2], 9, 12.4);
  bit_position += deserialize_data(data, 5, bit_position, m_batteries[3], 9, 12.4);

  bit_position += deserialize_data(data, 6, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += deserialize_data(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);
  bit_position += deserialize_data(data, 6, bit_position, m_internal_humidity,50.0, 100.0);

  bit_position += deserialize_data(data, 8, bit_position, m_current_waypoint);

  return true;
}

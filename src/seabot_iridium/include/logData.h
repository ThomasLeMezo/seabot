#ifndef LOGDATA_H
#define LOGDATA_H

#include <string>
#include <vector>
#include <boost/multiprecision/cpp_int.hpp>

using boost::multiprecision::cpp_int;

//#define TIME_POSIX_START 1570529969 // 12 Juin 2019 => (18bit in minutes => ???)
#define L93_EAST_MIN 0
#define L93_EAST_MAX 1300000
#define L93_NORTH_MIN 6000000
#define L93_NORTH_MAX 7200000

// Guerledan
// 253502,6805671

enum MSG_TYPE:unsigned int {LOG_STATE=0, CMD_SLEEP=1, CMD_MISSION=2, CMD_PARAMETERS=3};

#define NB_BITS_LOG1 136
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_LOG1, NB_BITS_LOG1, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_log1_t;

#define NB_BITS_CMD_SLEEP 16
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_SLEEP, NB_BITS_CMD_SLEEP, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_sleep_t;

#define NB_BITS_CMD_WAYPOINT 52
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_WAYPOINT, NB_BITS_CMD_WAYPOINT, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_waypoint_t;

#define NB_BITS_CMD_MISSION_HEADER 72
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_MISSION_HEADER, NB_BITS_CMD_MISSION_HEADER, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_mission_header_t;

#define NB_BITS_CMD_PARAMETERS 16
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS_CMD_PARAMETERS, NB_BITS_CMD_PARAMETERS, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_cmd_parameters_t;


class Waypoint{

public:
  Waypoint(){}

  Waypoint(const unsigned long &time_end_param, const double &depth_param, const double &north_param, const double &east_param){
    time_end = time_end_param;
    depth = depth_param;
    east = east_param;
    north = north_param;
  }
public:
  double north = 0.0;
  double east = 0.0;
  double depth = 0.0;
  unsigned long time_end = 0;
};

class LogData
{
public:
  /**
   * @brief LogData
   */
  LogData(){}

private:
  /**
   * @brief serialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @param min
   * @param max
   */
  template<typename _T>
  int serialize_data(_T &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double&value_max);

  /**
   * @brief deserialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @param value_min
   * @param value_max
   * @return
   */
  template<typename _T>
  int deserialize_data(_T &bits, const int &nb_bit, const int &start_bit, double &value, const double &value_min, const double &value_max);

  /**
   * @brief serialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @return
   */
  template<typename _T>
  int serialize_data(_T &bits, const int &nb_bit, const int &start_bit, const unsigned int &value);

  /**
   * @brief deserialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @return
   */
  template<typename _T>
  int deserialize_data(const _T &bits, const int &nb_bit, const int &start_bit, unsigned int &value);  

  /**
   * @brief deserialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @return
   */
  template<typename _T>
  int deserialize_data(const _T &bits, const int &nb_bit, const int &start_bit, bool &value);

public:
  /**
   * @brief deserialize_log_state
   * @param file_name
   */
  bool deserialize_log_state(const std::string &data_raw);

  /**
   * @brief deserialize_log_CMD
   * @param file_name
   * @param type
   * @return
   */
  bool deserialize_log_CMD(const std::string &raw_data);

  /**
   * @brief serialize_log_state
   * @param file_name
   * @return
   */
  std::string serialize_log_state(const long long &time);

  /**
   * @brief deserialize_log_CMD_sleep
   * @param file_name
   * @return
   */
  bool deserialize_log_CMD_sleep(const std::string &data);

  /**
   * @brief serialize_log_CMD_sleep
   * @param file_name
   * @param min
   * @return
   */
  std::string serialize_log_CMD_sleep();

  /**
   * @brief serialize_log_CMD_mission
   * @param file_name
   * @return
   */
  std::string serialize_log_CMD_mission();

  /**
   * @brief deserialize_log_CMD_mission
   * @param file_name
   * @return
   */
  bool deserialize_log_CMD_mission(const std::string &data);

  /**
   * @brief serialize_log_CMD_parameters
   * @param file_name
   * @return
   */
  std::string serialize_log_CMD_parameters();

  /**
   * @brief deserialize_log_CMD_parameters
   * @param file_name
   * @return
   */
  bool deserialize_log_CMD_parameters(const std::string &message);

  /**
   * @brief write_file
   * @param file_name
   * @param data
   * @return
   */
  bool write_file(const std::string &file_name, const std::string &data, const unsigned int nb_bits = NB_BITS_LOG1);

  /**
   * @brief read_file
   * @param file_name
   * @return
   */
  std::string read_file(const std::string &file_name);

private:

  /**
   * @brief serialize_log_CMD_waypoint
   * @param save_file
   * @param w
   * @return
   */
  std::string serialize_log_CMD_waypoint(const Waypoint &w);

  /**
   * @brief LogData::deserialize_log_CMD_waypoint
   * @param save_file
   * @return
   */
  bool deserialize_log_CMD_waypoint(const std::string &message);

public:
  double m_time_now = 0.0;

  double m_east = 42.0; // 2^21-1
  double m_north = 6000042.0; // 2^21-1 + 6e6
  double m_gnss_speed = 4.2;
  double m_gnss_heading = 242.0;
  double m_batteries[4] = {10.0, 10.0, 10.0, 10.0};

  double m_mean_east = 42.0;
  double m_mean_north = 6000042.0;
  double m_mean_heading = 242.0;

  double m_internal_pressure = 742.0;
  double m_internal_temperature = 42.0;
  double m_internal_humidity = 0.0;

  unsigned int  m_current_waypoint = 42; // 0 to 255 max

  unsigned int m_sleep_time = 0;

  std::vector<Waypoint> m_waypoint_list;
  double m_offset_east = 0.;
  double m_offset_north = 0.;
//  long long m_offset_time = TIME_POSIX_START; // in sec

  bool m_enable_mission = true;
  bool m_enable_flash = true;
  bool m_enable_depth = true;
  bool m_enable_engine = true;

  unsigned int m_last_cmd_received = 0;
  unsigned int m_period_message = 30; // in 10*min

  bool m_safety_published_frequency = false;
  bool m_safety_depth_limit = false;
  bool m_safety_batteries_limit = false;
  bool m_safety_depressurization = false;

  MSG_TYPE m_msg_type;
};

template<typename _T>
int LogData::serialize_data(_T &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double &value_max){
  double scale = (double)(1<<nb_bit-1)/(value_max-value_min);
  double bit_max = (1<<nb_bit-1);
  long unsigned int v = (long unsigned int)(std::min(std::max(round((value-value_min)*scale), 0.0), bit_max));
  long unsigned int v_max = (1<<nb_bit)-1;
  v = std::min(v, v_max);

  _T mask = ((_T(1)<<nb_bit)-1) << start_bit;
  bits &= ~mask;
  bits |= (_T(v) & ((_T(1)<<nb_bit)-1)) << start_bit;
  return nb_bit;
}

template<typename _T>
int LogData::deserialize_data(_T &bits, const int &nb_bit, const int &start_bit, double &value, const double &value_min, const double &value_max){
  double scale = (double)(1<<nb_bit-1)/(value_max-value_min);
  _T mask = ((_T(1)<<nb_bit)-1) << start_bit;
  _T v = (bits & mask)>>start_bit;
//  cout << v << endl;
  value = (double)v;
  value /= scale;
  value += value_min;

  return nb_bit;
}

template<typename _T>
int LogData::serialize_data(_T &bits, const int &nb_bit, const int &start_bit, const unsigned int &value){
  _T mask = ((_T(1)<<nb_bit)-1) << start_bit;
  bits &= ~mask;
  bits |= (_T(value) & ((_T(1)<<nb_bit)-1)) << start_bit;
  return nb_bit;
}

template<typename _T>
int LogData::deserialize_data(const _T &bits, const int &nb_bit, const int &start_bit, unsigned int &value){
  _T mask = ((_T(1)<<nb_bit)-1) << start_bit;
  _T v = (bits & mask)>>start_bit;

  value = (unsigned int)v;
  return nb_bit;
}

template<typename _T>
int LogData::deserialize_data(const _T &bits, const int &nb_bit, const int &start_bit, bool &value){
  _T mask = ((_T(1)<<nb_bit)-1) << start_bit;
  _T v = (bits & mask)>>start_bit;

  value = (bool)v;
  return nb_bit;
}

#endif // LOGDATA_H

#ifndef LOGTDT_H
#define LOGTDT_H

#include <string>
#include <vector>
#include <boost/multiprecision/cpp_int.hpp>

using boost::multiprecision::cpp_int;

#define NB_BITS 112
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS, NB_BITS, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_log1_t;


class LogTDT
{
public:
  /**
   * @brief LogTDT
   */
  LogTDT(){}

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
  int serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double&value_max);

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
  int deserialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, double &value, const double &value_min, const double &value_max);

  /**
   * @brief serialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @return
   */
  int serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const unsigned int &value);

  /**
   * @brief deserialize_data
   * @param bits
   * @param nb_bit
   * @param start_bit
   * @param value
   * @return
   */
  int deserialize_data(const uint_log1_t &bits, const int &nb_bit, const int &start_bit, unsigned int &value);

public:
  /**
   * @brief deserialize_log_TDT1
   * @param file_name
   */
  bool deserialize_log_TDT1(const std::string &file_name);

  /**
   * @brief serialize_log_TDT1
   * @param file_name
   * @return
   */
  bool serialize_log_TDT1(const std::string &file_name);

public:
  double m_east = 42.0; // 2^21-1
  double m_north = 6000042.0; // 2^21-1 + 6e6
  double m_gnss_speed = 4.2;
  double m_gnss_heading = 242.0;
  double m_batteries[4] = {10.0, 10.0, 10.0, 10.0};

  double m_internal_pressure = 742.0;
  double m_internal_temperature = 42.0;
  double m_internal_humidity = 0.0;

  unsigned int m_seabot_state = 0;
  unsigned int  m_current_waypoint = 42; // 0 to 255 max

};

#endif // LOGTDT_H

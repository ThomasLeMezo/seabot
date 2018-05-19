#ifndef NEOM8L_H
#define NEOM8L_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>

#include <iostream>     // std::cout, std::right, std::endl
#include <iomanip>      // std::setw

#include <ros/ros.h>

#include <nmea/info.h>
#include <nmea/context.h>
#include <nmea/parser.h>
#include <nmea/gmath.h>

#include <proj_api.h>

class NeoM8L
{
public:
  NeoM8L();
  ~NeoM8L();

  /**
   * @brief i2c_open
   * @return
   */
  int i2c_open();

  /**
   * @brief read_data
   * @return
   */
  int read_data();

  /**
   * @brief get_nmea_info
   * @return
   */
  const nmeaINFO& get_nmea_info() const;

  /**
   * @brief convert_local_frame
   */
  void convert_local_frame();

  /**
   * @brief get_east
   * @return
   */
  const double& get_east() const;

  /**
   * @brief get_north
   * @return
   */
  const double& get_north() const;

  /**
   * @brief get_lat
   * @return
   */
  const double& get_lat() const;

  /**
   * @brief get_lon
   * @return
   */
  const double& get_lon() const;

  /**
   * @brief get_time_data
   * @return
   */
  const int &get_time_data() const;

  /**
   * @brief update_time_data
   */
  void update_time_data();

  /**
   * @brief convert_data (Lat, Long to degree + Proj update)
   */
  void convert_data();

  /**
   * @brief get_time_month
   * @return
   */
  unsigned int get_time_month() const;

private:
  int m_file;
  const int m_i2c_addr = 0x42;
  const char* m_i2c_periph = "/dev/i2c-1";

  nmeaINFO m_info;
  nmeaPARSER m_parser;

  std::ostringstream m_string_buff;

  projPJ pj_lambert, pj_latlong;
  double m_east, m_north;
  double m_lat, m_lon, m_elv;

  unsigned int m_time_month=0;
  int m_time_data = 0; // =sec of the gnss time
};

inline const nmeaINFO& NeoM8L::get_nmea_info() const{
  return m_info;
}

inline const double& NeoM8L::get_north() const{
  return m_north;
}

inline const double& NeoM8L::get_east() const{
  return m_east;
}

inline const double& NeoM8L::get_lat() const{
  return m_lat;
}

inline const double& NeoM8L::get_lon() const{
  return m_lon;
}

inline const int& NeoM8L::get_time_data() const{
  return m_time_data;
}

inline void NeoM8L::update_time_data(){
  m_time_data = m_info.utc.sec;
}

inline unsigned int NeoM8L::get_time_month() const{
  return m_time_month;
}

#endif // NEOM8L_H

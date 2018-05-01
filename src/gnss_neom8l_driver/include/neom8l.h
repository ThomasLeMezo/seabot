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

private:
  int m_file;
  const int m_i2c_addr = 0x42;
  const char* m_i2c_periph = "/dev/i2c-1";

  nmeaINFO m_info;
  nmeaPARSER m_parser;

  std::ostringstream m_string_buff;

  projPJ pj_lambert, pj_latlong;
  double m_east, m_north;
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

#endif // NEOM8L_H

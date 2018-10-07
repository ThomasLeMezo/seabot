#ifndef SBD_H
#define SBD_H

#include <string>
//#include "TimeoutSerial.h"
#include "BufferedAsyncSerial.h"
#include "omp.h"

#define SBD_TOKEN_OK "\r\nOK\r\n"
#define SBD_TOKEN_READY "\r\nREADY\r\n"
#define SBD_TOKEN_SEPARATOR "\r\n"

class SBD
{
public:

  /**
   * @brief SBD
   * @param serial_port
   * @param baud_rate
   * @param timeout (in ms)
   */
  SBD(const std::string &serial_port_name="/dev/ttyAMA0", const unsigned int &baud_rate=19200);

  ~SBD();

  /**
   * @brief disable_echo
   */
  void disable_echo();

  /**
   * @brief cmd_CSQ
   */
  int cmd_CSQ(bool fast=true);

  /**
   * @brief cmd_get_imei
   * @return
   */
  long long cmd_get_imei();

  /**
   * @brief cmd_copy_MO_MT
   */
  int cmd_copy_MO_MT();

  /**
   * @brief cmd_write_message
   * @param data
   * @param data_size
   * @return
   */
  int cmd_write_message(const std::string &data);

  /**
   * @brief cmd_read_message
   * @param data
   * @return
   */
  std::string cmd_read_message();

  /**
   * @brief SBD::cmd_status
   */
  int cmd_status();

  /**
   * @brief cmd_flush_message
   * @param MO
   * @param MT
   * @return
   */
  int cmd_flush_message(const bool &MO, const bool &MT);

  /**
   * @brief SBD::cmd_session
   * @param answer
   * @return
   */
  int cmd_session(const bool &answer=false);

  /**
   * @brief cmd_enable_alert
   * @param enable
   * @return
   */
  int cmd_enable_alert(const bool &enable=true);

  /**
   * @brief cmd_enable_indicator_reporting
   * @param enable
   * @return
   */
  int cmd_enable_indicator_reporting(const bool &enable=true);

  /**
   * @brief cmd_trafic_management_status
   * @return
   */
  int cmd_trafic_management_status();

  /**
   * @brief set_gnss
   * @param latitude
   * @param longitude
   */
  void set_gnss(const double &latitude, const double &longitude);

  /**
   * @brief decode_sentence
   */
  void read();

private:

  /**
   * @brief write_AT
   * @param cmd
   * @return
   */
  void write(const std::string &at_cmd);

private:
  BufferedAsyncSerial m_serial;
  bool m_valid_gnss = false;
  double m_latitude = 48.39475416;
  double m_longitude = -4.48271749;

  bool m_OK = false;
  bool m_ERROR = false;
  bool m_READY = false;
  bool m_read_msg = false;

public:
  int m_STATUS_MO = -2;
  int m_STATUS_MOMSN = -2;
  int m_STATUS_MT = -2;
  int m_STATUS_MTMSN = -2;
  int m_STATUS_RA = -2;
  int m_waiting = -2;

  int m_SESSION_MO = -2;
  int m_SESSION_MOMSN = -2;
  int m_SESSION_MT = -2;
  int m_SESSION_MTMSN = -2;

  int m_CSQ = -1;
  int m_copy_MO_MT_size = -1;

  long long m_imei = 0;

  int m_ready_return = -1;

  std::string m_read_msg_data = "";

  int m_indicator_signal = -1;
  int m_indicator_service = -1;
  int m_indicator_antenna = -1;

  int m_ring_alert = 0;

  int m_trafic_management_status = -1;
  int m_trafic_management_time = -1;

};

inline void SBD::set_gnss(const double &latitude, const double &longitude){
  m_latitude = latitude;
  m_longitude = longitude;
  m_valid_gnss = true;
}

#endif // SBD_H

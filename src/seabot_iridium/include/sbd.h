#ifndef SBD_H
#define SBD_H

#include <string>
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
  SBD(const std::string &serial_port_name, const unsigned int &baud_rate);
  SBD();

  ~SBD();

  /**
   * @brief init
   */
  void init(const std::string &serial_port_name="/dev/ttyAMA0", const unsigned int &baud_rate=19200);

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
  int cmd_session();

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
   * @brief cmd_set_registration_mode
   * @param mode
   * @return
   */
  int cmd_set_registration_mode(const int &mode);

  /**
   * @brief decode_sentence
   */
  void read();

  /**
   * @brief sbd_power
   * @param enable
   * @return
   */
  bool sbd_power(const bool &enable);

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

  bool m_in_session = false;

  omp_lock_t lock_data;


  int m_STATUS_MO = -2;
  int m_STATUS_MOMSN = -2;
  int m_STATUS_MT = -2;
  int m_STATUS_MTMSN = -2;
  int m_STATUS_RA = -2;
  int m_waiting = -2;

  int m_SESSION_MO = 5;
  int m_SESSION_MOMSN = -2;
  int m_SESSION_MT = 2;
  int m_SESSION_MTMSN = -2;

  int m_CSQ = -1;
  int m_copy_MO_MT_size = -1;

  long long m_imei = 0;

  int m_ready_return = -1;

  std::string m_read_msg_data = "";

  int m_indicator_signal = -1;
  int m_indicator_service = -1;
  int m_indicator_antenna = -1;

  bool m_ring_alert = false;
  int m_ring_alert_code = -1;

  int m_trafic_management_status = -1;
  int m_trafic_management_time = -1;

  bool m_areg_new_event = false;
  int m_areg_event = -1;
  int m_areg_error_code = -1;

  unsigned int m_gpio_power = 5;
  bool m_iridium_power_state = false;

public:
  int get_status_mo();
  int get_status_momsn();
  int get_status_mt();
  int get_status_mtmsn();
  int get_status_ra();
  int get_waiting();
  int get_session_mo();
  int get_session_momsn();
  int get_session_mt();
  int get_session_mtmsn();
  int get_csq();
  int get_copy_mo_mt_size();
  long long get_imei();
  int get_write_return();
  std::string get_read_msg_data();
  int get_indicator_signal();
  int get_indicator_service();
  int get_indicator_antenna();
  bool get_ring_alert();
  int get_ring_alert_code();
  int get_trafic_management_status();
  int get_trafic_management_time();
  bool get_areg_new_event();
  int get_areg_event();
  int get_areg_error_code();
  bool is_in_session();

};

inline void SBD::set_gnss(const double &latitude, const double &longitude){
  m_latitude = latitude;
  m_longitude = longitude;
  m_valid_gnss = true;
}

inline int SBD::get_status_mo(){
  omp_set_lock(&lock_data);
  int result = m_STATUS_MO;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_momsn(){
  omp_set_lock(&lock_data);
  int result = m_STATUS_MOMSN;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_mt(){
  omp_set_lock(&lock_data);
  int result = m_STATUS_MT;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_mtmsn(){
  omp_set_lock(&lock_data);
  int result = m_STATUS_MTMSN;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_status_ra(){
  omp_set_lock(&lock_data);
  int result = m_STATUS_RA;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_waiting(){
  omp_set_lock(&lock_data);
  int result = m_waiting;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_mo(){
  omp_set_lock(&lock_data);
  int result = m_SESSION_MO;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_momsn(){
  omp_set_lock(&lock_data);
  int result = m_SESSION_MOMSN;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_mt(){
  omp_set_lock(&lock_data);
  int result = m_SESSION_MT;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_session_mtmsn(){
  omp_set_lock(&lock_data);
  int result = m_SESSION_MTMSN;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_csq(){
  omp_set_lock(&lock_data);
  int result = m_CSQ;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_copy_mo_mt_size(){
  omp_set_lock(&lock_data);
  int result = m_copy_MO_MT_size;
  omp_unset_lock(&lock_data);
  return result;
}

inline long long SBD::get_imei(){
  omp_set_lock(&lock_data);
  long result = m_imei;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_write_return(){
  omp_set_lock(&lock_data);
  int result = m_ready_return;
  omp_unset_lock(&lock_data);
  return result;
}

inline std::string SBD::get_read_msg_data(){
  omp_set_lock(&lock_data);
  std::string result = m_read_msg_data;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_indicator_signal(){
  omp_set_lock(&lock_data);
  int result = m_indicator_signal;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_indicator_service(){
  omp_set_lock(&lock_data);
  int result = m_indicator_service;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_indicator_antenna(){
  omp_set_lock(&lock_data);
  int result = m_indicator_antenna;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::get_ring_alert(){
  omp_set_lock(&lock_data);
  bool result = m_ring_alert;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_ring_alert_code(){
  omp_set_lock(&lock_data);
  int result = m_ring_alert_code;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_trafic_management_status(){
  omp_set_lock(&lock_data);
  int result = m_trafic_management_status;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_trafic_management_time(){
  omp_set_lock(&lock_data);
  int result = m_trafic_management_time;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::get_areg_new_event(){
  omp_set_lock(&lock_data);
  bool result = m_areg_new_event;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_areg_event(){
  omp_set_lock(&lock_data);
  int result = m_areg_event;
  omp_unset_lock(&lock_data);
  return result;
}

inline int SBD::get_areg_error_code(){
  omp_set_lock(&lock_data);
  int result = m_areg_error_code;
  omp_unset_lock(&lock_data);
  return result;
}

inline bool SBD::is_in_session(){
  omp_set_lock(&lock_data);
  bool result = m_in_session;
  omp_unset_lock(&lock_data);
  return result;
}


#endif // SBD_H

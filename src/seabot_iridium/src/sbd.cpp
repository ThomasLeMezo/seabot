#include "sbd.h"
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <ros/ros.h>

#include <thread>
#include <chrono>

using namespace boost;
using namespace std;

SBD::SBD(){
  omp_init_lock(&lock_data);
}

void SBD::init(const string &serial_port_name, const unsigned int &baud_rate){
  try{
    m_serial.open(serial_port_name, baud_rate);
  }
  catch(boost::system::system_error& e){
    cout<<"Error: "<<e.what()<<endl;
  }

  disable_echo(); // Disable echo from SBD
}

SBD::SBD(const string &serial_port_name, const unsigned int &baud_rate){
  omp_init_lock(&lock_data);
  init();
}

SBD::~SBD(){
  m_serial.close();
  omp_destroy_lock(&lock_data);
}

void SBD::read(){
  string result = m_serial.readStringUntil(SBD_TOKEN_SEPARATOR);

  if(result != ""){
    //    cout << result << endl;
    ROS_INFO("[Iridium_raw] %s", result.c_str());
    if(boost::starts_with(result, "OK")){
      m_OK = true;
      m_READY = false;
    }
    else if(boost::starts_with(result, "READY")){
      m_READY = true;
    }
    else if(boost::starts_with(result, "ERROR")){
      m_ERROR = true;
      m_READY = false;
      m_OK = false;
      ROS_WARN("[Iridium] Received ERROR");
    }
    else if(boost::starts_with(result, "SBDRING")){
      m_ring_alert = true;
    }
    else if(boost::starts_with(result, "+SBDREG:")){
      vector<string> fields;
      string result0 = result.substr(8);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      m_reg_status = stoi(fields[0]);
      m_reg_error_code = stoi(fields[1]);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CRIS:")){
      vector<string> fields;
      string result0 = result.substr(6);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      m_ring_alert_code = stoi(fields[1]);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CIEV:")){
      vector<string> fields;
      string result0 = result.substr(6);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      int indicator = stoi(fields[0]);
      omp_set_lock(&lock_data);
      switch(indicator){
      case 0:
        m_indicator_signal = stoi(fields[1]);
        //        cout << "=> Signal = " << m_indicator_signal << endl;
        break;
      case 1:
        m_indicator_service = stoi(fields[1]);
        //        cout << "=> Service = " << m_indicator_service << endl;
        break;
      case 2:
        m_indicator_antenna = stoi(fields[1]);
        //        cout << "=> Antenna = " << (m_indicator_antenna?"Fault":"OK") << endl;
        break;
      default:
        break;
      }
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "300234")){
      omp_set_lock(&lock_data);
      m_imei = stoll(result);
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CSQ:")){
      omp_set_lock(&lock_data);
      m_CSQ = stoi(result.substr(6));
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+CSQF:")){
      omp_set_lock(&lock_data);
      m_CSQ = stoi(result.substr(6));
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "SBDTC")){
      omp_set_lock(&lock_data);
      m_copy_MO_MT_size = stoi(result.substr(49));
      omp_unset_lock(&lock_data);
    }
    else if(m_READY){
      // result after a READY
      omp_set_lock(&lock_data);
      m_ready_return = stoi(result);
      m_READY = false;
      omp_unset_lock(&lock_data);

    }
    else if(boost::starts_with(result, "+SBDSX:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      if(fields.size()>=6){
        omp_set_lock(&lock_data);
        m_STATUS_MO = stoi(fields[0]);
        m_STATUS_MOMSN = stoi(fields[1]);
        m_STATUS_MT = stoi(fields[2]);
        m_STATUS_MTMSN = stoi(fields[3]);
        m_STATUS_RA = stoi(fields[4]);
        m_waiting = stoi(fields[5]);

        if(m_ring_alert && m_STATUS_MT==1)
          m_ring_alert = false;
        omp_unset_lock(&lock_data);
      }
    }
    else if(boost::starts_with(result, "+SBDLOE:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      m_trafic_management_status = stoi(fields[0]);
      m_trafic_management_time = stoi(fields[1]); // Check format
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+SBDIX:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);

      omp_set_lock(&lock_data);
      if(fields.size()>=6){
        m_SESSION_MO = stoi(fields[0]);
        m_SESSION_MOMSN = stoi(fields[1]);
        m_SESSION_MT = stoi(fields[2]);
        m_SESSION_MTMSN = stoi(fields[3]);
        m_waiting = stoi(fields[5]);
      }
      m_in_session = false;
      ROS_INFO("[Iridium] Session result");
      omp_unset_lock(&lock_data);
    }
    else if(boost::starts_with(result, "+AREG:")){
      vector<string> fields;
      string result0 = result.substr(6);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      omp_set_lock(&lock_data);
      m_areg_new_event = true;
      m_areg_event = stoi(fields[0]);
      m_areg_error_code = stoi(fields[1]);
      omp_unset_lock(&lock_data);
    }
    else if(m_read_msg){ // Keep at the end
      omp_set_lock(&lock_data);
      m_read_msg_data = result.substr(2, result.size()-4); // checksum + ending token
      m_read_msg = false;
      omp_unset_lock(&lock_data);
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

/**
 * Note : each line fillful this format \r\n *** \r\n
 */
void SBD::write(const std::string &at_cmd){
  omp_set_lock(&lock_data);
  m_OK = false;
  omp_unset_lock(&lock_data);

  string cmd = at_cmd + '\r';
  m_serial.writeString(cmd);
  ROS_INFO("[Iridium_raw] send = %s", cmd.c_str());
}

void SBD::disable_echo(){
  write("ATE0");
}

int SBD::cmd_CSQ(bool fast){
  omp_set_lock(&lock_data);
  m_CSQ = -1;
  omp_unset_lock(&lock_data);

  write("AT+CSQ" + string(fast?"F":""));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  omp_set_lock(&lock_data);
  int result = m_CSQ;
  omp_unset_lock(&lock_data);
  return result;
}

long long SBD::cmd_get_imei(){
  write("AT+CGSN");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  omp_set_lock(&lock_data);
  long long result = m_imei;
  omp_unset_lock(&lock_data);
  return result;
}

int SBD::cmd_copy_MO_MT(){
  omp_set_lock(&lock_data);
  m_copy_MO_MT_size = -1;
  omp_unset_lock(&lock_data);
  write("AT+SBDTC");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  omp_set_lock(&lock_data);
  int result = m_copy_MO_MT_size;
  omp_unset_lock(&lock_data);
  return result;
}

int SBD::cmd_write_message(const std::string &data){
  set_ready(false);

  if(data.size()>340)
    return 4;
  string cmd = "AT+SBDWB=" + std::to_string(data.size());
  write(cmd);

  string data_checksum(data);
  // Compute checksum
  uint16_t checksum = 0;
  for(size_t i=0; i < data.size(); i++)
    checksum += (uint8_t) data.c_str()[i];

  data_checksum += (uint8_t) checksum>>8;
  data_checksum += (uint8_t) checksum;

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  write(data_checksum);

  for(int i=0; i<data_checksum.size(); i++){
    ROS_INFO("[Iridium_sbdwb] Send = %x", data_checksum.c_str()[i]);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool valid = false;
  for(size_t i=0; i<20; i++){
    if(is_ready()){
      valid = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return (valid)?0:-1;
}

std::string SBD::cmd_read_message(){
  string cmd = "AT+SBDRB";
  omp_set_lock(&lock_data);
  m_read_msg = true;
  omp_unset_lock(&lock_data);

  write(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  omp_set_lock(&lock_data);
  string result = m_read_msg_data;
  omp_unset_lock(&lock_data);

  return result;
}

int SBD::cmd_flush_message(const bool &MO, const bool &MT){
  string cmd = "AT+SBDD";
  if(MO && MT)
    cmd += to_string(2);
  else if(MO)
    cmd += to_string(0);
  else
    cmd += to_string(1);
  write(cmd);
  return 0;
}

int SBD::cmd_status(){
  string cmd = "AT+SBDSX";
  write(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return 0;
}

int SBD::cmd_session(){
  omp_set_lock(&lock_data);
  bool start_mission = m_in_session;
  bool answer = m_ring_alert;
  m_SESSION_MO = -2;
  m_SESSION_MT = -2;
  omp_unset_lock(&lock_data);

  if(!start_mission){
    omp_set_lock(&lock_data);
    m_in_session = true;
    omp_unset_lock(&lock_data);

    ROS_INFO("[Iridium] Start a session");
    string cmd = "AT+SBDIX" + string(answer?"A":"");
    if(m_valid_gnss){
      int lat_deg = int(m_latitude);
      double lat_min = int(abs(m_latitude - lat_deg)*60000.)/1000.;
      int lon_deg = int(m_longitude);
      double lon_min = int(abs(m_longitude - lon_deg)*60000.)/1000.;
      cmd += "=";

      std::ostringstream lat_string, lon_string;
      if(lat_deg<0)
        lat_string << "-";
      lat_string << setfill('0') << setw(2) << abs(lat_deg);
      lat_string << std::fixed << std::setprecision(3) << lat_min;
      cmd += lat_string.str();

      cmd += ",";

      if(lon_deg<0)
        lon_string << "-";
      lon_string << setfill('0') << setw(3) << abs(lon_deg);
      lon_string << std::fixed << std::setprecision(3) << lon_min;
      cmd += lon_string.str();
    }

    write(cmd);

    for(size_t i=0; i<4*60; i++){
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      if(is_in_session()==false){
        return 0;
      }
    }

    ROS_INFO("[Iridium] End of session not received");
    return 1;
  }
  else{
    return 1;
  }
}

int SBD::cmd_enable_alert(const bool &enable){
  string cmd = "AT+SBDMTA=";
  cmd += string(enable?"1":"0");
  write(cmd);
  return 0;
}

int SBD::cmd_enable_indicator_reporting(const bool &enable){
  string cmd = "AT+CIER=";
  cmd += string(enable?"1":"0");
  cmd += ",1,1,1,0";
  write(cmd);
  return 0;
}

int SBD::cmd_trafic_management_status(){
  string cmd = "AT+SBDLOE";
  write(cmd);
  return 0;
}

int SBD::cmd_set_registration_mode(const int &mode){
  string cmd = "AT+SBDAREG=";
  cmd += to_string(mode);
  write(cmd);
  return 0;
}

bool SBD::sbd_power(const bool &enable){
  if(enable != m_iridium_power_state){

    string gpio_file = "/sys/class/gpio/gpio" + to_string(m_gpio_power) + "/value";

    ofstream setvalgpio(gpio_file.c_str()); // open value file for gpio
    if (!setvalgpio.is_open()){
      ROS_WARN("[IRIDIUM] Unable to write power on on GPIO %u", m_gpio_power);
      return false;
    }

    setvalgpio << enable?1:0;//write value to value file
    setvalgpio.close();// close value file

    m_iridium_power_state = enable;
  }
  return true;
}



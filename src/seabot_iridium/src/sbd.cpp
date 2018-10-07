#include "sbd.h"
#include <string>
#include <iostream>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>

#include <sstream>
#include <iomanip>

using namespace boost;
using namespace std;

SBD::SBD(const string &serial_port_name, const unsigned int &baud_rate){
  try{
    m_serial.open(serial_port_name, baud_rate);
  }
  catch(boost::system::system_error& e){
    cout<<"Error: "<<e.what()<<endl;
  }

  disable_echo(); // Disable echo from SBD
}

SBD::~SBD(){
  m_serial.close();
}

void SBD::read(){
  string result = m_serial.readStringUntil(SBD_TOKEN_SEPARATOR);

  if(result != ""){
//    cout << result << endl;
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
    }
    else if(boost::starts_with(result, "+CRIS:")){
      vector<string> fields;
      boost::split(fields, result, boost::is_any_of(","), boost::token_compress_on);
      m_ring_alert = stoi(fields[1]);
    }
    else if(boost::starts_with(result, "+CIEV=")){
      vector<string> fields;
      boost::split(fields, result, boost::is_any_of(","), boost::token_compress_on);
      int indicator = stoi(fields[0]);
      switch(indicator){
      case 0:
        m_indicator_signal = stoi(fields[1]);
        cout << "=> Signal = " << m_indicator_signal << endl;
        break;
      case 1:
        m_indicator_service = stoi(fields[1]);
        cout << "=> Service = " << m_indicator_service << endl;
        break;
      case 2:
        m_indicator_antenna = stoi(fields[1]);
        cout << "=> Antenna = " << (m_indicator_antenna?"Fault":"OK") << endl;
        break;
      default:
        break;
      }
    }
    else if(boost::starts_with(result, "300234")){
      m_imei = stoll(result);
    }
    else if(boost::starts_with(result, "+CSQ:")){
      m_CSQ = stoi(result.substr(6));
    }
    else if(boost::starts_with(result, "+CSQF:")){
      m_CSQ = stoi(result.substr(6));
    }
    else if(boost::starts_with(result, "SBDTC")){
      m_copy_MO_MT_size = stoi(result.substr(49));
    }
    else if(m_READY){
      // result after a READY
      m_ready_return = stoi(result);
    }
    else if(boost::starts_with(result, "+SBDSX:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      if(fields.size()>=6){
        m_STATUS_MO = stoi(fields[0]);
        m_STATUS_MOMSN = stoi(fields[1]);
        m_STATUS_MT = stoi(fields[2]);
        m_STATUS_MTMSN = stoi(fields[3]);
        m_STATUS_RA = stoi(fields[4]);
        m_waiting = stoi(fields[5]);
      }
    }
    else if(boost::starts_with(result, "+SBDLOE:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);
      m_trafic_management_status = stoi(fields[0]);
      m_trafic_management_time = stoi(fields[1]); // Check format
    }
    else if(boost::starts_with(result, "+SBDIX:")){
      vector<string> fields;
      string result0 = result.substr(7);
      boost::split(fields, result0, boost::is_any_of(","), boost::token_compress_on);

      if(fields.size()>=6){
        m_SESSION_MO = stoi(fields[0]);
        m_SESSION_MOMSN = stoi(fields[1]);
        m_SESSION_MT = stoi(fields[2]);
        m_SESSION_MTMSN = stoi(fields[3]);
        m_waiting = stoi(fields[5]);
      }
    }
    else if(m_read_msg){ // Keep at the end
      m_read_msg_data = result.substr(2, result.size()-4); // checksum + ending token
      m_read_msg = false;
    }
  }

  usleep(10000);
}

/**
 * Note : each line fillful this format \r\n *** \r\n
 */
void SBD::write(const std::string &at_cmd){
  m_OK = false;
  string cmd = at_cmd + '\r';
  m_serial.writeString(cmd);

}

void SBD::disable_echo(){
  write("ATE0");
}

int SBD::cmd_CSQ(bool fast){
  m_CSQ = -1;
  write("AT+CSQ" + string(fast?"F":""));
  usleep(500000);
  return m_CSQ;
}

long long SBD::cmd_get_imei(){
  write("AT+CGSN");
  usleep(500000);
  return m_imei;
}

int SBD::cmd_copy_MO_MT(){
  m_copy_MO_MT_size = -1;
  write("AT+SBDTC");
  usleep(500000);
  return m_copy_MO_MT_size;
}

int SBD::cmd_write_message(const std::string &data){
  m_READY = false;
  if(data.size()>340)
    return 4;
  string cmd = "AT+SBDWB=" + std::to_string(data.size());
  write(cmd);

  for(size_t i=0; i<100; i++){
    if(m_READY)
      break;
    usleep(10000);
  }

  if(m_READY){
    // Compute checksum
    uint16_t checksum = 0;
    for(size_t i=0; i < data.size(); i++)
      checksum += data.c_str()[i];

    string data_checksum = data;
    data_checksum += uint8_t(checksum>>8);
    data_checksum += uint8_t(checksum);
    write(data_checksum);

    usleep(100000);
    return m_ready_return;
  }
  else
    return -1;
}

std::string SBD::cmd_read_message(){
  string cmd = "AT+SBDRB";
  write(cmd);
  usleep(100000);
  return m_read_msg_data;
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
  usleep(100000);
  return 0;
}

int SBD::cmd_session(const bool &answer){
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
  usleep(100000);
  return 0;
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



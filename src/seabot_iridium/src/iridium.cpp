#include "iridium.h"

#include "ros/ros.h"

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <sys/ioctl.h>

#include "boost/filesystem.hpp"

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;
using boost::multiprecision::cpp_int;

Iridium::Iridium(){
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  m_path_received_full = string(homedir) + "/" + m_path_received;

  boost::filesystem::path p(m_path_received_full);
  if (boost::filesystem::create_directories(p))
    ROS_INFO("[Iridium] Directory to store msg received was not found, create a new one");
}

int32_t Iridium::uart_init(){
  m_uart_fd = open( "/dev/ttyAMA0", O_RDWR|O_NOCTTY|O_NDELAY);
  /* Error Handling */
  if ( m_uart_fd < 0 ){
    ROS_WARN("[Iridium] ERROR %i opening /dev/ttyAMA0 : %s", errno, strerror(errno));
    ROS_WARN("[Iridium] Demo mode");
    m_demo_mode = true;
    return TIS_ERROR_SERIAL_ERROR;
  }
  fcntl(m_uart_fd, F_SETFL, 0);
  int DTR_bit = TIOCM_DTR;
  ioctl(m_uart_fd, TIOCMBIC, &DTR_bit);

  //CONFIGURE THE UART (http://www.cplusplus.com/forum/general/219754/)
  //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
  //	CSIZE:- CS5, CS6, CS7, CS8
  //	CLOCAL - Ignore modem status lines
  //	CREAD - Enable receiver
  //	IGNPAR = Ignore characters with parity errors
  //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
  //	PARENB - Parity enable
  //	PARODD - Odd parity (else even)
  struct termios options;
  tcgetattr(m_uart_fd, &options);
  cfmakeraw(&options);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag |= CS8;
  options.c_cflag |= B19200;

  // NOPARITY
  options.c_cflag &= ~CMSPAR;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~PARODD;
  // ONESTOPBIT
  options.c_cflag &= ~CSTOPB;

  options.c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHONL | ISIG);
  // Raw input.
  options.c_iflag &= ~(IGNBRK | BRKINT | INLCR | IGNCR | ICRNL);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~ISTRIP;
  options.c_iflag &= ~PARMRK;
  options.c_iflag |= IGNPAR;
  // Raw output.
  options.c_oflag &= ~(OPOST | ONLCR | ONOCR | ONLRET | OCRNL);

  options.c_cc[VMIN] = 0; // Minimum number of characters to read.
  options.c_cc[VTIME] = 4000/100; // Time to wait for every character read in tenths of seconds. (ms/100)

  tcsetattr(m_uart_fd, TCSADRAIN, &options);

  // ToDo : read the IMEI value

  return TIS_ERROR_SUCCESS;
}

int32_t uart_send_data(void *serial_struct, uint8_t *data, int32_t count){
  ssize_t error = 0;
  while(error<count){
    ssize_t tmp = write(*((int *)serial_struct), data+error, count-error);
    //    ROS_INFO("[Iridium] send %i - %li", (int)tmp, count-error);
    if(tmp<0){
      ROS_WARN("[Iridium] ERROR %i writing /dev/ttyAMA0 : %s", errno, strerror(errno));
      return TIS_ERROR_SERIAL_ERROR;
    }
    else if(tmp==0){
      return TIS_ERROR_TIMEOUT;
    }
    error += tmp;
  }

  return TIS_ERROR_SUCCESS;
}

int32_t uart_receive_data(void *serial_struct, uint8_t *data, int32_t count){
  ssize_t error = 0;
  while(error<count){
    ssize_t tmp = read(*((int *)serial_struct), data+error, count-error);
    //    ROS_INFO("[Iridium] read %i - %li", (int)tmp, (count-error));
    if(tmp<0){
      ROS_WARN("[Iridium] ERROR %i reading /dev/ttyAMA0 : %s", errno, strerror(errno));
      return TIS_ERROR_SERIAL_ERROR;
    }
    else if(tmp==0){
      return TIS_ERROR_TIMEOUT;
    }
    error += tmp;
  }

  return TIS_ERROR_SUCCESS;
}

int32_t uart_wait_data(void *serial_struct, uint32_t timeout){
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(*((int *)serial_struct), &fds);
  struct timeval timeout_struct; /* 10 seconds */
  timeout_struct.tv_sec = timeout;
  timeout_struct.tv_usec = 0;
  int error = select(*((int *)serial_struct)+1, &fds, NULL, NULL, &timeout_struct);

  if(error<0){
    ROS_WARN("[Iridium] ERROR %i waiting data from /dev/ttyAMA0 : %s", errno, strerror(errno));
    return TIS_ERROR_SERIAL_ERROR;
  }
  else if(error == 0){
    ROS_INFO("[Iridium] ERROR %i timeout waiting data from /dev/ttyAMA0 : %s", errno, strerror(errno));
    return TIS_ERROR_TIMEOUT;
  }
  else
    return TIS_ERROR_SUCCESS;
}

int32_t uart_flush_TX(void *serial_struct){
  int error = tcflush(*((int *)serial_struct),TCOFLUSH);
  if(error<0){
    ROS_WARN("[Iridium] ERROR %i flushing TX (TCOFLUSH) /dev/ttyAMA0 : %s", errno, strerror(errno));
    return TIS_ERROR_SERIAL_ERROR;
  }
  else
    return TIS_ERROR_SUCCESS;
}

int32_t uart_flush_RX(void *serial_struct){
  int error = tcflush(*((int *)serial_struct),TCIFLUSH);
  if(error<0){
    ROS_WARN("[Iridium] ERROR %i flushing RX (TCIFLUSH) /dev/ttyAMA0 : %s", errno, strerror(errno));
    return TIS_ERROR_SERIAL_ERROR;
  }
  else
    return TIS_ERROR_SUCCESS;
}

int32_t uart_release(void *serial_struct){
  int error = close(*((int *)serial_struct));
  if(error<0){
    ROS_WARN("[Iridium] ERROR %i closing /dev/ttyAMA0 : %s", errno, strerror(errno));
    return TIS_ERROR_SERIAL_ERROR;
  }
  else
    return TIS_ERROR_SUCCESS;
}

bool Iridium::iridium_power(const bool &enable){
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

////Cete fonction prend en paramètre un tableau contenant des pointeurs vers le nom des fichiers à envoyer ainsi que le nombre de fichier à renvoyer.
bool Iridium::send_and_receive_data(){
  if(m_files_to_send.size()>0  && m_enable_iridium && !m_demo_mode){
    ROS_INFO("[Iridium] Start send/receive data");
    if (TIS_init(&m_tis,
                 reinterpret_cast<uint8_t *>(&m_path_received_full[0]),				//"CMD" est le chemin du dossier qui contiendra les fichier reçus.
                 TIS_MODEM_9602,	//Modèle du modem
                 m_imei,	//Numéro IMEI du modem
                 TIS_SERVICE_SBD,	//Service SBD
                 NULL,				//Inutile avec un modem sans carte SIM
                 0,					//Inutile en modem SBD
                 NULL,				//Inutile en modem SBD
                 (uint8_t)m_files_to_send.size(),		//Nombre de fichier envoyés
                 (void*)(&m_uart_fd),			//Un pointeur vers la structure décrivant la liaison série
                 uart_send_data,	//Fonction utilisant les appelles système de la plateforme pour envoyer des données sur la liaison série
                 uart_receive_data, //Fonction utilisant les appelles système de la plateforme pour recevoir des données sur la liaison série
                 uart_wait_data,	//Fonction utilisant les appelles système de la plateforme pour attendre des données sur la liaison série
                 uart_flush_TX,		//Fonction utilisant les appelles système de la plateforme pour vider le tampon de sortie de la liaison série
                 uart_flush_RX		//Fonction utilisant les appelles système de la plateforme pour vider le tampon d'entrée de la liaison série
                 ) != TIS_ERROR_SUCCESS) {
      return false;
    }

    // Add files to transmit
    for(string file:m_files_to_send)
      TIS_add_file_to_send(&m_tis, reinterpret_cast<uint8_t *>(&file[0]));

    int enable_transmission = m_transmission_number_attempt;
    while(enable_transmission>0){
//      iridium_power(true); // Power On Iridium
      int result = TIS_transmission(&m_tis, 10); // Launch transmission

      ///***** Diagnostic Result *****///
      if (result != TIS_ERROR_SUCCESS) // Get diagnostic info
        ROS_WARN("[Iridium] Error while transmitting : %i", result);
      ROS_INFO("[Iridium] Send messages : (%i / %i)\n", m_tis.SBD_sent_without_error, m_tis.SBD_sent_without_error + m_tis.SBD_sent_with_error);
      ROS_INFO("[Iridium] Received messages : (%i / %i)\n", m_tis.SBD_received_without_error, m_tis.SBD_received_without_error + m_tis.SBD_received_with_error);
      ROS_INFO("[Iridium] Waiting to receive messages : %i\n", m_tis.SBD_waiting_messages);
      for (size_t i = 0; i < m_files_to_send.size(); i++)
        ROS_INFO("[Iridium] File %s was send at %i/100", m_files_to_send[i].c_str(), TIS_get_file_progress(&m_tis, i));

      // Test if tranmission is over
      if ((TIS_remaining_file_to_send(&m_tis) != 0) || (TIS_waiting_incoming_data(&m_tis) == true)){
        sleep(m_transmission_sleep_time); //La durée dépend de votre application, la fonction depend de votre plateforme et non de la librairie)
        enable_transmission--;
      }
      else{
        enable_transmission = 0;
//        iridium_power(false); // Power Off Iridium
      }
    }

    TIS_clean(&m_tis);
    m_files_to_send.clear();
  }
  return true;
}

const std::string Iridium::get_new_tdt_file(){
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  long int wall_time_now = round(ros::WallTime::now().toSec());

  std::stringstream sstream;
  sstream << homedir << "/" << m_path_send << "/";

  boost::filesystem::path p(sstream.str());
  if (boost::filesystem::create_directories(p))
    ROS_INFO("[Iridium] Directory to store msg send was not found, create a new one");

  sstream << std::hex << wall_time_now; // Print in hex
  sstream << ".tdt";
  return sstream.str();
}

int Iridium::serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double &value_max){
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

int Iridium::deserialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, double &value, const double &value_min, const double &value_max){
  double scale = (double)(1<<nb_bit-1)/(value_max-value_min);
  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  uint_log1_t v = (bits & mask)>>start_bit;
//  cout << v << endl;
  value = (double)v;
  value /= scale;
  value += value_min;

  return nb_bit;
}

int Iridium::serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const unsigned int &value){
  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  bits &= ~mask;
  bits |= (uint_log1_t(value) & ((uint_log1_t(1)<<nb_bit)-1)) << start_bit;
  return nb_bit;
}

int Iridium::deserialize_data(const uint_log1_t &bits, const int &nb_bit, const int &start_bit, unsigned int &value){
  uint_log1_t mask = ((uint_log1_t(1)<<nb_bit)-1) << start_bit;
  uint_log1_t v = (bits & mask)>>start_bit;

  value = (double)v;
  return nb_bit;
}

bool Iridium::serialize_log_TDT1(){
  ofstream save_file;
  std::string name_file = get_new_tdt_file();
  save_file.open(name_file);

  if(!save_file.is_open()){
    ROS_WARN("[Iridium] Unable to open (%s) new log file %i : %s", name_file.c_str(), errno, strerror(errno));
    return false;
  }

  size_t nb_bits = 26*4; // must be a multiple of 4
  uint_log1_t data = (uint_log1_t(1)<<nb_bits) -1;

//  cout << "data.str().size() = " << data.str().size() << endl;
//  auto count = data.backend().size();
//  auto tsize = sizeof(boost::multiprecision::limb_type);
//  auto copy_count = count * tsize;

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

  bit_position += serialize_data(data, 4, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += serialize_data(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);

  bit_position += serialize_data(data, 8, bit_position, m_current_waypoint);

  if(m_demo_mode){
    std::cout << data << std::endl;
  }

  save_file.write((char*)&data, NB_BITS/8);
  save_file.close();

  m_files_to_send.push_back(name_file);
  ROS_INFO("[Iridium] Add new file to queue %s", name_file.c_str());
}

bool Iridium::deserialize_log_TDT1(const string &file_name){
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

  cout << "Start deserialize data" << endl;
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

  bit_position += deserialize_data(data, 4, bit_position, m_internal_pressure, 680.0, 800.0);
  bit_position += deserialize_data(data, 6, bit_position, m_internal_temperature, 8.0, 50.0);

  bit_position += deserialize_data(data, 8, bit_position, m_current_waypoint);

  return true;
}

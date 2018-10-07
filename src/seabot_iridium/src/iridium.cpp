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
#include <boost/range/iterator_range.hpp>

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "logtdt.h"

#include "seabot_iridium/IridiumLog.h"

using namespace std;
using boost::multiprecision::cpp_int;

Iridium::Iridium(){
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  m_homedir = string(homedir);

  m_path_received = m_homedir + "/iridium/received";
  m_path_received_tmp = m_homedir + "/iridium/tmp";
  m_path_send = m_homedir + "/iridium/send";

  boost::filesystem::path p_received(m_path_received);
  boost::filesystem::path p_received_tmp(m_path_received_tmp);
  boost::filesystem::path p_send(m_path_send);

  boost::filesystem::create_directories(p_received);
  boost::filesystem::create_directories(p_received_tmp);
  boost::filesystem::create_directories(p_send);
}


////Cete fonction prend en paramètre un tableau contenant des pointeurs vers le nom des fichiers à envoyer ainsi que le nombre de fichier à renvoyer.
bool Iridium::send_and_receive_data(ros::Publisher &iridium_pub){
  bool transmission_successful = false;
  if(m_enable_iridium && !m_demo_mode){
    ROS_INFO("[Iridium] Start send/receive data");
//    if (TIS_init(&m_tis,
//                 reinterpret_cast<uint8_t *>(&m_path_received_tmp[0]),				//"CMD" est le chemin du dossier qui contiendra les fichier reçus.
//                 TIS_MODEM_9602,	//Modèle du modem
//                 m_imei,	//Numéro IMEI du modem
//                 TIS_SERVICE_SBD,	//Service SBD
//                 NULL,				//Inutile avec un modem sans carte SIM
//                 0,					//Inutile en modem SBD
//                 NULL,				//Inutile en modem SBD
//                 (uint8_t)m_files_to_send.size(),		//Nombre de fichier envoyés
//                 (void*)(&m_uart_fd),			//Un pointeur vers la structure décrivant la liaison série
//                 uart_send_data,	//Fonction utilisant les appelles système de la plateforme pour envoyer des données sur la liaison série
//                 uart_receive_data, //Fonction utilisant les appelles système de la plateforme pour recevoir des données sur la liaison série
//                 uart_wait_data,	//Fonction utilisant les appelles système de la plateforme pour attendre des données sur la liaison série
//                 uart_flush_TX,		//Fonction utilisant les appelles système de la plateforme pour vider le tampon de sortie de la liaison série
//                 uart_flush_RX		//Fonction utilisant les appelles système de la plateforme pour vider le tampon d'entrée de la liaison série
//                 ) != TIS_SUCCESS) {
//      return false;
//    }

    // Add files to transmit
    for(string file:m_files_to_send)
      TIS_add_file_to_send(&m_tis, reinterpret_cast<uint8_t *>(&file[0]));

    int enable_transmission = m_transmission_number_attempt;
    while(enable_transmission>0){
      //      iridium_power(true); // Power On Iridium
      int signal_strenght = TIS_signal_strenght(&m_tis);
      int result = TIS_transmission(&m_tis, 10); // Launch transmission

      ///***** Diagnostic Result *****///
      seabot_iridium::IridiumLog msg;
      msg.error_code = result;
      msg.message_sent = m_tis.SBD_sent_without_error;
      msg.message_sent_total = m_tis.SBD_sent_without_error + m_tis.SBD_sent_with_error;
      msg.message_receive = m_tis.SBD_received_without_error;
      msg.message_receive_total = m_tis.SBD_received_without_error + m_tis.SBD_received_with_error;
      msg.message_waiting = m_tis.SBD_waiting_messages;
      msg.signal_strength = signal_strenght;

      for (size_t i = 0; i < m_files_to_send.size(); i++){
        msg.message_sent_name.push_back(m_files_to_send[i]);
        msg.message_sent_progress.push_back(TIS_get_file_progress(&m_tis, i));
      }
      iridium_pub.publish(msg);

      // Test if transmission is over
      if ((TIS_remaining_file_to_send(&m_tis) != 0) || (TIS_waiting_incoming_data(&m_tis) == true)){
        sleep(m_transmission_sleep_time); //La durée dépend de votre application, la fonction depend de votre plateforme et non de la librairie)
        enable_transmission--;
      }
      else{
        enable_transmission = 0;
        transmission_successful = true;
        //        iridium_power(false); // Power Off Iridium
      }
    }

    TIS_clean(&m_tis);
    m_files_to_send.clear();
  }
  return transmission_successful;
}

void Iridium::get_new_log_files(){
//  string file_name = get_new_tdt_filename();
//  if(logTDT.serialize_log_TDT1(file_name))
//    m_files_to_send.push_back(file_name);
}

const std::string Iridium::get_new_tdt_filename(){
//  long int wall_time_now = round(ros::WallTime::now().toSec());

//  std::stringstream sstream;
//  sstream << m_path_send << "/";

//  sstream << std::hex << wall_time_now; // Print in hex
//  sstream << ".tdt";
//  return sstream.str();
}

void Iridium::process_cmd_file(){
//  // List files received
//  boost::filesystem::path p(m_path_received_tmp);
//  for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {})){
//    if(boost::filesystem::is_regular_file(entry.path())){
//      ROS_INFO("[Iridium] Received %s", entry.path().c_str());
//      deserialize_cmd_file(entry.path().string());

//      // Move file to archive
//      string file_name = entry.path().filename().string();
//      boost::filesystem::path p_old(entry.path());
//      boost::filesystem::path p_new(m_path_received + "/" + file_name);
//      boost::filesystem::rename(p_old, p_new);
//    }
//  }
}

void Iridium::deserialize_cmd_file(const string &file_name){
  LogTDT l;
  l.deserialize_log_CMD(file_name);
  m_cmd_list.push_back(l);
}

void Iridium::test(){
//int32_t uart_send_data(void *serial_struct, uint8_t *data, int32_t count)
}

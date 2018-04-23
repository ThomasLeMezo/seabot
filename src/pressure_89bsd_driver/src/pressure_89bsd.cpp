#include "pressure_89bsd.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>

#include <bitset>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#define CMD_RESET 0x1E // reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV_D1_4096 0x48 // ADC conversion command
#define CMD_ADC_CONV_D2_4096 0x58 // ADC conversion command
#define CMD_PROM 0xA0 // Coefficient location

#define Q0 9.0
#define Q1 11.0
#define Q2 9.0
#define Q3 15.0
#define Q4 15.0
#define Q5 16.0
#define Q6 16.0

#define T_MIN -20.0
#define T_MAX 85.0

using namespace std;

int16_t bin2decs(u_int16_t val, size_t nb_bit){
    if((val & 0b1<<(nb_bit-1))==0)
        return val;
    else
        return (val-(1<<(nb_bit)));
}

Pressure_89BSD::Pressure_89BSD()
{
}

Pressure_89BSD::~Pressure_89BSD(){
  close(m_file);
}

int Pressure_89BSD::reset(){
  int res = i2c_smbus_write_byte(m_file, CMD_RESET);
  usleep(30000); // 28ms reload for the sensor (?)
  if (res < 0)
    ROS_INFO("[Pressure_89BSD] error reset sensor");
  else
    ROS_INFO("[Pressure_89BSD] reset ok");
  return 0;
}

int Pressure_89BSD::i2c_open(){
  if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
    ROS_WARN("Failed to open the I2C bus");
    exit(1);
  }

  if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
    ROS_WARN("Failed to acquire bus access and/or talk to slave");
    exit(1);
  }
  return 0;
}

int Pressure_89BSD::init_sensor(){
  ROS_INFO("[Pressure_89BSD] Sensor initialization");
  reset();

  for(int i=0; i<7; i++){
    const char add = CMD_PROM + (char) 2*(i+1);
    if (write(m_file,&add,1) != 1)
      ROS_WARN("[Pressure_89BSD] Error Writing");
    u_int8_t buff[2] = {0};
    if (read(m_file,&buff,2) != 2)
      ROS_WARN("[Pressure_89BSD] Error Reading");
    m_prom[i] = (buff[0] << 8) | buff[1] << 0;
  }
  ROS_INFO("[Pressure_89BSD] Sensor Read PROM OK");

  m_C0 = bin2decs(m_prom[0]>>2,14);
  m_C1 = bin2decs(((m_prom[0] & 0x3)<<12) | (m_prom[1] >> 4), 14);
  m_C2 = bin2decs(((m_prom[1] & 0xF)<<6) | (m_prom[2] >> 10), 10);
  m_C3 = bin2decs((m_prom[2] & 0x3FF), 10);
  m_C4 = bin2decs((m_prom[3] >> 6), 10);
  m_C5 = bin2decs(((m_prom[3] & 0x1F) << 4) | (m_prom[4]>>12), 10);
  m_C6 = bin2decs(((m_prom[4] >> 2) & 0x3FF), 10);
  m_A0 = bin2decs(((m_prom[4] & 0x3) << 8) | (m_prom[5]>>8), 10);
  m_A1 = bin2decs(((m_prom[5] & 0xFF) << 2) | (m_prom[6]>>14), 10);
  m_A2 = bin2decs(((m_prom[5] >> 3) & 0x3FF), 10);

  ROS_INFO("[Pressure_89BSD] Sensor Compute PROM OK");
  ROS_INFO("[Pressure_89BSD] C0 = %d", m_C0);
  ROS_INFO("[Pressure_89BSD] C1 = %d", m_C1);
  ROS_INFO("[Pressure_89BSD] C2 = %d", m_C2);
  ROS_INFO("[Pressure_89BSD] C3 = %d", m_C3);
  ROS_INFO("[Pressure_89BSD] C4 = %d", m_C4);
  ROS_INFO("[Pressure_89BSD] C5 = %d", m_C5);
  ROS_INFO("[Pressure_89BSD] C6 = %d", m_C6);

  ROS_INFO("[Pressure_89BSD] A0 = %d", m_A0);
  ROS_INFO("[Pressure_89BSD] A1 = %d", m_A1);
  ROS_INFO("[Pressure_89BSD] A2 = %d", m_A2);

  return 0;
}

int Pressure_89BSD::get_value(){

  return 0;
}

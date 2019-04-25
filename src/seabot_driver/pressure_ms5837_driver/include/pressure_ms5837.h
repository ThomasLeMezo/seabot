#ifndef PRESSURE_H
#define PRESSURE_H

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

#include <ros/ros.h>
#include <deque>


#define CMD_RESET 0x1E // reset command
#define CMD_ADC_READ 0x00 // ADC read command

#define CMD_ADC_CONV_D1_4096 0x48 // ADC conversion command
#define CMD_ADC_CONV_D2_4096 0x58 // ADC conversion command

#define CMD_ADC_CONV_D1_8192 0x4A // ADC conversion command
#define CMD_ADC_CONV_D2_8192 0x5A // ADC conversion command
#define CMD_PROM 0xA0 // Coefficient location

#define SLEEP_8192 20000

class Pressure_ms5837
{
public:
  Pressure_ms5837();
  ~Pressure_ms5837();

  int i2c_open();
  int init_sensor();
  int get_value();

  bool measure();

  int reset();

  int get_D1();
  int get_D2();

  double get_pression();
  double get_temperature();

  void set_pressure_memory_size(const size_t &pressure_memory_size);

  void estimate_pressure_speed();

private:

  int m_file = 0;
  const int m_i2c_addr = 0x76;
  const char* m_i2c_periph = "/dev/i2c-1";
  const float m_p_min = 0.0; // In Bar
  const float m_p_max = 30.0; // In Bar

  int16_t m_C[5];

  unsigned long m_D1, m_D2;
  bool m_valid_data = false;

  double m_pressure, m_temperature;

};

inline int Pressure_ms5837::get_D1(){
  i2c_smbus_write_byte(m_file, CMD_ADC_CONV_D1_8192);
  ros::Duration(0.03).sleep();
  unsigned char buff[3] = {0, 0, 0};
  if (i2c_smbus_read_i2c_block_data(m_file, CMD_ADC_READ, 3, buff)!=3){
      ROS_WARN("[Pressure_ms5837] Error Reading D1");
      m_valid_data = false;
      return -1;
  }
  m_valid_data = true;
  m_D1 = (buff[0] << 16) | (buff[1] << 8) | buff[2];
  return 0;
}

inline int Pressure_ms5837::get_D2(){
  i2c_smbus_write_byte(m_file, CMD_ADC_CONV_D2_8192);
  ros::Duration(0.03).sleep();
  unsigned char buff[3] = {0, 0, 0};
  if (i2c_smbus_read_i2c_block_data(m_file, CMD_ADC_READ, 3, buff)!=3){
      ROS_WARN("[Pressure_ms5837] Error Reading D2");
      m_valid_data = false;
      return -1;
  }
  m_valid_data = true;
  m_D2 = (buff[0] << 16) | (buff[1] << 8) | buff[2];
  return 0;
}

inline double Pressure_ms5837::get_pression(){
  return m_pressure;
}

inline double Pressure_ms5837::get_temperature(){
  return m_temperature;
}

int16_t bin2decs(u_int16_t val, size_t nb_bit);

#endif // PRESSURE_H

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

extern "C" {
    #include <linux/i2c-dev.h>
}
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
extern "C" {
    #include <i2c/smbus.h>
}
#endif

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
#define CMD_ADC_CONV 0x48 // ADC conversion command
#define CMD_PROM 0xA0 // Coefficient location

#define CONVERSION_TIME 10000 // 10ms

class Temperature_TSYS01
{
public:
  Temperature_TSYS01();
  ~Temperature_TSYS01();

  int i2c_open();
  int init_sensor();
  int reset();

  bool measure();

  double get_temperature();

  void set_pressure_memory_size(const size_t &pressure_memory_size);

private:

  int m_file = 0;
  const int m_i2c_addr = 0x45;
  const char* m_i2c_periph = "/dev/i2c-1";

  u_int16_t m_k[5];
  bool m_valid_data = false;

  double m_temperature;

};

inline double Temperature_TSYS01::get_temperature(){
  return m_temperature;
}

int16_t bin2decs(u_int16_t val, size_t nb_bit);

#endif // TEMPERATURE_H

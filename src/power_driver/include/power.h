#ifndef POWER_H
#define POWER_H

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

#define ADC_BATTERY_LEVEL_CONV 0.01509434

class Power
{
public:
  Power();
  ~Power();

  /**
   * @brief i2c_open
   * @return
   */
  int i2c_open();

  /**
   * @brief enable_led
   * @param val
   */
  void enable_led(bool val) const;

  void measure_battery();

  const float &get_level_battery(size_t id) const;

private:
  int m_file;
  const int m_i2c_addr = 0x39;
  const char* m_i2c_periph = "/dev/i2c-1";

  float m_level_battery[4] =  {0.0, 0.0, 0.0, 0.0};
};

inline const float& Power::get_level_battery(size_t id) const{
  return m_level_battery[id];
}

#endif // POWER_H

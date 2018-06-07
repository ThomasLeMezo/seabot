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

// 150/(150+330)
// 5V = 1024
#define ADC_BATTERY_LEVEL_CONV 0.015625

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
  void set_flash_led(const bool &val) const;

  /**
   * @brief set_flash_led_delay
   * @param dt in 0.1 s (20 = 2s)
   */
  void set_flash_led_delay(const unsigned char &dt) const;

  /**
   * @brief set_sleep_mode
   */
  void set_sleep_mode() const;

  /**
   * @brief stop_sleep_mode
   */
  void stop_sleep_mode() const;

  /**
   * @brief set_sleep_mode_countdown
   * @param hours
   * @param min
   * @param sec
   * @param sec_to_stop
   */
  void set_sleep_mode_countdown(const unsigned char &hours, const unsigned char &min, const unsigned char &sec, const unsigned char &sec_to_stop=60) const;

  /**
   * @brief measure_battery
   */
  void get_batteries();

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

#ifndef THRUSTER_H
#define THRUSTER_H

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

#define MOTOR_PWM_STOP 150
#define MAX_PWM 190
#define MIN_PWM 110

class Thruster
{
public:
  /**
   * @brief Thruster
   */
  Thruster();

  ~Thruster();

  /**
   * @brief i2c_open
   * @return
   */
  int i2c_open();

  /**
   * @brief write_cmd
   * @param left
   * @param right
   */
  void write_cmd(const uint8_t &left, const uint8_t &right) const;

  /**
   * @brief get_version
   */
  uint8_t& get_version();

private:
  int m_file;
  const int m_i2c_addr = 0x20;
  const char* m_i2c_periph = "/dev/i2c-1";

  uint8_t m_version=0;
};

#endif // THRUSTER_H

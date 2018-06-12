#ifndef THRUSTER_H
#define THRUSTER_H

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

#define MOTOR_PWM_STOP 150
#define MAX_PWM 190
#define MIN_PWM 110

class Thruster
{
public:
  Thruster();
  ~Thruster();

  int i2c_open();

  void write_cmd(const uint8_t &left, const uint8_t &right) const;

private:
  int m_file;
  const int m_i2c_addr = 0x20;
  const char* m_i2c_periph = "/dev/i2c-1";
};

#endif // THRUSTER_H

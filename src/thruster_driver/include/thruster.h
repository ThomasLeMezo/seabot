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

class Thruster
{
public:
  Thruster();
  ~Thruster();

  int i2c_open();

  void write_cmd(const unsigned short &left, const unsigned short &right) const;

private:
  int m_file;
  const int m_i2c_addr = 0x70;
  const char* m_i2c_periph = "/dev/i2c-1";
};


// Max values
//Stopped         1500 microseconds
//Max forward     1900 microseconds
//Max reverse     1100 microseconds
inline void Thruster::write_cmd(const unsigned short int &left, const unsigned short int &right) const{
  unsigned char buff[2];
  // Left Engine
  buff[0] = left >> 8;
  buff[1] = left & 0xFF;
  i2c_smbus_write_i2c_block_data(m_file, 0x01, 2, buff);

  // Right Engine
  buff[0] = right >> 8;
  buff[1] = right & 0xFF;
  i2c_smbus_write_i2c_block_data(m_file, 0x02, 2, buff);
}

#endif // THRUSTER_H

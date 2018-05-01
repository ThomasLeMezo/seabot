#include "thruster.h"

Thruster::Thruster(){

}

Thruster::~Thruster(){
  write_cmd(1500, 1500);
  close(m_file);
}

int Thruster::i2c_open(){
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





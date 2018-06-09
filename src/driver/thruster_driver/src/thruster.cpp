#include "thruster.h"

Thruster::Thruster(){

}

Thruster::~Thruster(){
  write_cmd(MOTOR_PWM_STOP, MOTOR_PWM_STOP);
  close(m_file);
}

int Thruster::i2c_open(){
  if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
    ROS_WARN("[Thruster_driver] Failed to open the I2C bus (%s)", m_i2c_periph);
    exit(1);
  }

  if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
    ROS_WARN("[Thruster_driver] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
    exit(1);
  }
  return 0;
}





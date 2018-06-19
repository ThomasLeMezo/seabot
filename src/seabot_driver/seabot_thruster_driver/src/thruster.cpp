#include "thruster.h"

Thruster::Thruster(){

}

Thruster::~Thruster(){
  write_cmd(MOTOR_PWM_STOP, MOTOR_PWM_STOP);
  close(m_file);
}

int Thruster::i2c_open(){
  m_file = open(m_i2c_periph,O_RDWR);
  if (m_file < 0) {
    ROS_WARN("[Piston_driver] Failed to open the I2C bus (%s) - %s", m_i2c_periph, strerror(m_file));
    exit(1);
  }

  int rslt = ioctl(m_file,I2C_SLAVE,m_i2c_addr);
  if (rslt < 0) {
    ROS_WARN("[Piston_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(rslt));
    exit(1);
  }
  usleep(100000);
  return 0;
}

// Max values
//Stopped         1500 microseconds
//Max forward     1900 microseconds
//Max reverse     1100 microseconds
void Thruster::write_cmd(const uint8_t &left, const uint8_t &right) const{
  if(i2c_smbus_write_word_data(m_file, 0x00, (left | right<<8))<0)
    ROS_WARN("[Thruster_driver] I2C Bus Failure - Write cmd");
}


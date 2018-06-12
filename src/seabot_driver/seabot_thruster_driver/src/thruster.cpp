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

// Max values
//Stopped         1500 microseconds
//Max forward     1900 microseconds
//Max reverse     1100 microseconds
void Thruster::write_cmd(const uint8_t &left, const uint8_t &right) const{
  uint8_t buff[2];
  buff[0] = left;
  buff[1] = right;
  if(i2c_smbus_write_i2c_block_data(m_file, 0x00, 2, buff)!=2)
    ROS_WARN("[Thruster_driver] I2C Bus Failure - Write cmd");
}





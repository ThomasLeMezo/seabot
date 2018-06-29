#include "piston.h"
#include <unistd.h>
#include <errno.h>
#include <climits>

Piston::Piston(){
}

Piston::~Piston(){
  close(m_file);
}

int Piston::i2c_open(){
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

void Piston::set_piston_reset() const{
  if(i2c_smbus_write_byte_data(m_file, 0x01, 0x01)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Piston Reset");
}

void Piston::set_led_enable(const bool &val) const{
  if(i2c_smbus_write_byte_data(m_file, 0x04, val?0x01:0x00)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - LED Enable");
}

void Piston::set_piston_enable(const bool &val) const{
  if(i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, val?0x05:0x06)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Enable piston");
}

void Piston::set_piston_speed(const uint16_t &speed_in, const uint16_t &speed_out) const{
  __u8 buff[4];
  buff[0] = speed_in;
  buff[1] = speed_in >> 8;
  buff[2] = speed_out;
  buff[3] = speed_out >> 8;
  if(i2c_smbus_write_i2c_block_data(m_file, 0x12, 4, buff)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Set Speed In/Out");
}

void Piston::set_piston_position(const uint16_t &position) const{
    ROS_INFO("Piston : %i", position);
  // S Addr Wr [A] Comm [A] DataLow [A] DataHigh [A] P
  if(i2c_smbus_write_word_data(m_file, 0x10, position)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Piston set position %s", strerror(errno));
}

void Piston::get_piston_all_data(){
  uint8_t buff[6];
  if(i2c_smbus_read_i2c_block_data(m_file, 0x00, 6,buff) != 6)
    ROS_WARN("[Piston_driver] I2C Bus Failure - Read piston data");

  uint16_t position = (buff[1] << 8 | buff[0]);
  if(position > 32763)
      m_position = -(65526-position)/4.0;
  else
      m_position = position/4.0;

//  m_position = (buff[1] << 8 | buff[0])/4.0;
  m_switch_out = buff[2] & 0b1;
  m_switch_in = (buff[2] >> 1) & 0b1;
  m_state = (buff[2] >> 2) & 0b11;
  m_motor_on = (buff[2] >> 4) & 0b1;
  m_enable_on = (buff[2] >> 5) & 0b1;
  m_position_set_point = (buff[4] << 8 | buff[3])/4.0;
  m_motor_speed = buff[5];
}




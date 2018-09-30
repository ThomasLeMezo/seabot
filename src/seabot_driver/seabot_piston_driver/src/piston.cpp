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

void Piston::set_piston_emergency() const{
  ROS_DEBUG("[Piston_driver] Start piston emergency");
  if(i2c_smbus_write_byte_data(m_file, 0xB0, 0x01)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Piston Reset");
  usleep(100);
}

void Piston::set_piston_reset() const{
  ROS_DEBUG("[Piston_driver] Start reseting piston");
  if(i2c_smbus_write_byte_data(m_file, 0x01, 0x01)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Piston Reset");
  usleep(100);
}

void Piston::set_led_enable(const bool &val) const{
  if(i2c_smbus_write_byte_data(m_file, 0x04, val?0x01:0x00)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - LED Enable");
  usleep(100);
}

void Piston::set_error_interval(const __u8 &val) const{
  if(i2c_smbus_write_byte_data(m_file, 0x05, val)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Error Interval");
  usleep(100);
}

void Piston::set_reached_switch_off(const bool &val) const{
  if(i2c_smbus_write_byte_data(m_file, 0x06, val?0x01:0x00)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Piston Switch off at reached");
  usleep(100);
}

void Piston::set_piston_speed(const __u8 &speed_in, const __u8 &speed_out) const{
  __u16 val = (speed_in | speed_out<<8);
  if(i2c_smbus_write_word_data(m_file, 0x12, val)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Set Speed In/Out");
  usleep(100);
}

void Piston::set_piston_speed_reset(const __u8 &speed) const{
  if(i2c_smbus_write_byte_data(m_file, 0x14, speed)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Set Speed Reset");
  usleep(100);
}

void Piston::set_piston_position(__u16 position) const{
  // S Addr Wr [A] Comm [A] DataLow [A] DataHigh [A] P

  if(i2c_smbus_write_word_data(m_file, 0x10, position)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Piston set position");
  usleep(100);
}

void Piston::set_time_shift_error(const __u8 &val) const{
  if(i2c_smbus_write_byte_data(m_file, 0x07, val)<0)
    ROS_WARN("[Piston_driver] I2C bus Failure - Time Shift Error");
  usleep(100);
}

void Piston::get_piston_all_data(){
  uint8_t buff[6];
  if(i2c_smbus_read_i2c_block_data(m_file, 0x00, 6,buff) != 6)
    ROS_WARN("[Piston_driver] I2C Bus Failure - Read piston data");

  uint16_t position = (buff[1] << 8 | buff[0]);
  if(position > 32768) // 2^16 / 2
    m_position = -(65536-position)/4.0; // 2^16-val
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
  usleep(100);
}

void Piston::get_piston_set_point(){
  m_position_set_point = i2c_smbus_read_word_data(m_file, 0x03)/4.0;
  usleep(100);
}

uint8_t& Piston::get_version(){
  m_version = i2c_smbus_read_byte_data(m_file, 0xC0);
  usleep(100);
  return m_version;
}

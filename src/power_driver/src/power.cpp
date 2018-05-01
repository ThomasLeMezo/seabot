#include "power.h"



Power::Power(){

}

Power::~Power(){
  close(m_file);
}

int Power::i2c_open(){
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

void Power::enable_led(bool val) const{
  unsigned char buff[2];
  // Left Engine
  buff[0] = 0x02;
  if(val)
    buff[1] = 0x01;
  else
    buff[1] = 0x00;
  i2c_smbus_write_i2c_block_data(m_file, 0x00, 2, buff);
}

void Power::measure_battery(){
  // i2cget -y 2 0x39 0x01 w
  m_level_battery[0] = i2c_smbus_read_word_data(m_file, 0x01) * ADC_BATTERY_LEVEL_CONV;
  m_level_battery[1] = i2c_smbus_read_word_data(m_file, 0x02) * ADC_BATTERY_LEVEL_CONV;
  m_level_battery[2] = i2c_smbus_read_word_data(m_file, 0x03) * ADC_BATTERY_LEVEL_CONV;
  m_level_battery[3] = i2c_smbus_read_word_data(m_file, 0x04) * ADC_BATTERY_LEVEL_CONV;
}





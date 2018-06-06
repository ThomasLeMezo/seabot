#include "piston.h"

Piston::Piston(){

}

Piston::~Piston(){
    close(m_file);
}

int Piston::i2c_open(){
    if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
        ROS_WARN("[Piston_driver] Failed to open the I2C bus (%s)", m_i2c_periph);
        exit(1);
    }

    if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
        ROS_WARN("[Piston_driver] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }
    return 0;
}

uint32_t Piston::set_piston_start() const{
    return i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, 0x01);
}

uint32_t Piston::set_piston_stop() const{
    return i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, 0x00);
}

uint32_t Piston::set_piston_reset() const{
    return i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, 0x02);
}

uint32_t Piston::set_piston_speed(const uint16_t &speed) const{
    __u8 buff[2];
    buff[0] = speed >> 8;
    buff[1] = speed & 0xFF;
    return i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_SPEED, 2, buff);
}

uint32_t Piston::set_piston_position(const uint16_t &position) const{
    __u8 buff[2];
    buff[0] = position >> 8;
    buff[1] = position & 0xFF;
    return i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_MOVE, 2, buff);
}

uint32_t Piston::set_piston_enable(const bool &val) const{
    return i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, val?0x05:0x06);
}

void Piston::update_piston_all_data(){
  uint8_t buff[6];
  if(i2c_smbus_read_i2c_block_data(m_file, 0x00, 6,buff) != 6){
    ROS_WARN("[Piston_driver] I2C Bus Failure");
  }

  m_position = buff[1] << 8 | buff[0];
  m_switch_out = buff[2] & 0b1;
  m_switch_in = (buff[2] >> 1) & 0b1;
  m_state = (buff[2] >> 2) & 0b11;
  m_system_on = (buff[2] >> 4) & 0b1;
  m_motor_on = (buff[2] >> 5) & 0b1;
  m_enable_on = (buff[2] >> 6) & 0b1;
  m_position_set_point = buff[4] << 8 | buff[3];
  m_motor_speed = buff[5];

}




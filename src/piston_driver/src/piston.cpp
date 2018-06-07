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

void Piston::set_piston_start() const{
    if(i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, 0x01)<0)
      ROS_WARN("[Piston_driver] I2C bus Failure - Piston Start");
}

void Piston::set_piston_stop() const{
    if(i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, 0x00)<0)
        ROS_WARN("[Piston_driver] I2C bus Failure - Piston Reset");
}

void Piston::set_piston_reset() const{
    if(i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, 0x02)<0)
      ROS_WARN("[Piston_driver] I2C bus Failure - Piston Reset");
}

void Piston::set_piston_speed(const uint16_t &speed_in, const uint16_t &speed_out) const{
    __u8 buff_in[2], buff_out[2];
    buff_in[0] = speed_in >> 8;
    buff_in[1] = speed_in;
    buff_out[0] = speed_out >> 8;
    buff_out[1] = speed_out;
    if(i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_SPEED_IN, 2, buff_in)!=2)
      ROS_WARN("[Piston_driver] I2C bus Failure - Set Speed In");
    if(i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_SPEED_IN, 2, buff_out)!=2)
      ROS_WARN("[Piston_driver] I2C bus Failure - Set Speed OUT");
}

void Piston::set_piston_position(const uint16_t &position) const{
    __u8 buff[2];
    buff[0] = position >> 8;
    buff[1] = position & 0xFF;
    if(i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_MOVE, 2, buff)<0)
      ROS_WARN("[Piston_driver] I2C bus Failure - Piston set position");
}

void Piston::set_piston_enable(const bool &val) const{
    if(i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, val?0x05:0x06)<0)
      ROS_WARN("[Piston_driver] I2C bus Failure - Enable piston");
}

void Piston::set_led_enable(const bool &val) const{
    if(i2c_smbus_write_byte_data(m_file, I2C_PISTON_CMD, val?0x07:0x08)<0)
      ROS_WARN("[Piston_driver] I2C bus Failure - LED Enable");
}

void Piston::get_piston_all_data(){
  uint8_t buff[6];
  if(i2c_smbus_read_i2c_block_data(m_file, 0x00, 6,buff) != 6)
    ROS_WARN("[Piston_driver] I2C Bus Failure");

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




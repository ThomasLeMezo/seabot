#include "piston.h"

Piston::Piston(){

}

Piston::~Piston(){
    close(m_file);
}

int Piston::i2c_open(){
    if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
        ROS_WARN("[Piston_driver] Failed to open the I2C bus");
        exit(1);
    }

    if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
        ROS_WARN("[Piston_driver] Failed to acquire bus access and/or talk to slave");
        exit(1);
    }
    return 0;
}

/*
* octet1 = 0xFE --> correspond à une consigne de position, suivit de la valeur de la position à atteindre
                    sur deux octets
* octet1 = 0xAB --> correspond à une consigne de vitesse, suivit de la valeur de la vitesse
                    sur deux octets
* octet1 = 0xEE --> correspond à une consigne de marche,arret,mise en butee (sur 1 octets)
*/

uint32_t Piston::set_piston_start() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x01;
    return i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

uint32_t Piston::set_piston_stop() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x00;
    return i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

uint32_t Piston::set_piston_reset() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x02;
    return i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
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
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = val?0x06:0x07;
    return i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

// 0x00: nb_pulse & 0xFF;
// 0x01: nb_pulse >> 8;
// 0x02: butee_out;
// 0x03: butee_in;
// 0x04: state;
// 0x05: system_on;
// 0x06: motor_on;
// 0x07: RC6_bit;
// 0x08: position_set_point & 0xFF;
// 0x09: position_set_point >> 8;
//default: 0x00;

//const uint16_t& Piston::get_piston_position(){
//    uint8_t buff[2];
//    i2c_smbus_read_i2c_block_data(m_file, 0x00, 2,buff);
//    m_position = (buff[1]<<8 | buff[0]);
//    return m_position;
//}

//const bool& Piston::get_piston_switch_out(){
//    m_switch_out = i2c_smbus_read_byte_data(m_file, 0x02);
//    return m_switch_out;
//}

//const bool& Piston::get_piston_switch_in(){
//    m_switch_in = i2c_smbus_read_byte_data(m_file, 0x03);
//    return m_switch_in;
//}

//const uint16_t& Piston::get_piston_state(){
//    m_state = i2c_smbus_read_byte_data(m_file, 0x04);
//    return m_state;
//}

//const bool &Piston::get_piston_system_on(){
//    m_system_on = i2c_smbus_read_byte_data(m_file, 0x05);
//    return m_system_on;
//}

//const bool &Piston::get_piston_motor_on(){
//    m_motor_on = i2c_smbus_read_byte_data(m_file, 0x06);
//    return m_motor_on;
//}

//const bool &Piston::get_piston_enable_on(){
//    m_enable_on = i2c_smbus_read_byte_data(m_file, 0x07);
//    return m_enable_on;
//}

//const uint16_t& Piston::get_piston_position_set_point(){
//    uint8_t buff[2];
//    i2c_smbus_read_i2c_block_data(m_file, 0x08, 2,buff);
//    m_position_set_point = buff[0] << 8 | buff[1];
//    return m_position_set_point;
//}

void Piston::update_piston_all_data(){
  uint8_t buff[10];
  if(i2c_smbus_read_i2c_block_data(m_file, 0x00, 7,buff) != 7){
    ROS_WARN("[Piston_driver] I2C Bus Failure");
  }

  m_position = buff[0] << 8 | buff[1];
  m_switch_out = buff[2] & 0b1;
  m_switch_in = (buff[2] >> 1) & 0b1;
  m_state = (buff[2] >> 2) & 0b11;
  m_system_on = (buff[2] >> 4) & 0b1;
  m_motor_on = (buff[2] >> 5) & 0b1;
  m_enable_on = (buff[2] >> 6) & 0b1;
  m_position_set_point = buff[3] << 8 | buff[4];
  m_motor_speed = buff[5] << 8 | buff[6];

}




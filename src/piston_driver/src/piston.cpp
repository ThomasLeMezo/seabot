#include "piston.h"

Piston::Piston(){

}

Piston::~Piston(){
    close(m_file);
}

int Piston::i2c_open(){
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

/*
* octet1 = 0xFE --> correspond à une consigne de position, suivit de la valeur de la position à atteindre
                    sur deux octets

* octet1 = 0xAB --> correspond à une consigne de vitesse, suivit de la valeur de la vitesse
                    sur deux octets

* octet1 = 0xEE --> correspond à une consigne de marche,arret,mise en butee (sur 1 octets)
* valeur 0xAA 0x00: arret
* valeur 0xAA 0x01: marche
* valeur 0xAA 0x02: mise en butee butee de sortie
* valeur 0xAA 0x03: mise en butee butee de rentree


* octet1 = 0x00 --> correspond à une demande, suivit de la valeur de la correspondance de la consigne
* valeur 0xAA 0x0A: valeur de la tension de batterie 1 sur 2 octets
* valeur 0xAA 0x0C: valeur de la tension de batterie 2 sur 2 octets
* valeur 0xAA 0x0E: valeur de la tension de batterie 3 sur 2 octets
* valeur 0xAA 0x10: valeur de la tension de batterie 4 sur 2 octets
* valeur 0xAA 0x04: valeur NB impulsions  sur 2 octets
* valeur 0xAA 0x06: valeur de la butée de sortie  sur 2 octets
* valeur 0xAA 0x08: valeur de la butée de rentrée sur 2 octets
*/

void Piston::set_piston_start() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x01;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

void Piston::set_piston_stop() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x00;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

void Piston::set_piston_full_exit() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x02;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

void Piston::set_piston_full_retract() const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = 0x03;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

void Piston::set_piston_speed(const uint16_t &speed) const{
    __u8 buff[2];
    buff[0] = speed >> 8;
    buff[1] = speed & 0xFF;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_SPEED, 2, buff);
}

void Piston::set_piston_position(const uint16_t &position) const{
    __u8 buff[2];
    buff[0] = position >> 8;
    buff[1] = position & 0xFF;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_MOVE, 2, buff);
}

void Piston::set_piston_enable(const bool &val) const{
    __u8 buff[2];
    buff[0] = I2C_PISTON_BLANK_VALUE;
    buff[1] = val?0x06:0x07;
    i2c_smbus_write_i2c_block_data(m_file, I2C_PISTON_CMD, 2, buff);
}

uint16_t Piston::get_piston_position(){
    uint8_t buff[2];
    i2c_smbus_read_i2c_block_data(m_file, 0x04, 2,buff);

    return (buff[0]<<8 | buff[1]);
}

uint16_t Piston::get_piston_switch_exit_position(){
    uint8_t buff[2];
    i2c_smbus_read_i2c_block_data(m_file, 0x06, 2,buff);

    return (buff[0]<<8 | buff[1]);
}

uint16_t Piston::get_piston_switch_retract_position(){
    uint8_t buff[2];
    i2c_smbus_read_i2c_block_data(m_file, 0x08, 2,buff);

    return (buff[0]<<8 | buff[1]);
}

uint16_t Piston::get_piston_state(){
    uint8_t buff[2];
    i2c_smbus_read_i2c_block_data(m_file, 0x012, 2,buff);

    return (buff[0]<<8 | buff[1]);
}






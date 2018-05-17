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


uint16_t Piston::get_piston_position(){
  __u8 reg_addr[3], reg_data[2];
  reg_addr[0] = I2C_PISTON_REQUEST;
  reg_addr[1] = I2C_PISTON_BLANK_VALUE;
  reg_addr[2] = 0x08;

  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
  messages[0].addr = m_i2c_addr;
  messages[0].flags = 0;
  messages[0].len = 3;
  messages[0].buf = (char*)&reg_addr;

  /* The data will get returned in this structure */
  messages[1].addr = m_i2c_addr;
  messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
  messages[1].len = 2;
  messages[1].buf = (char*)&reg_data;

  /* Send the request to the kernel and get the result back */
  packets.msgs = messages;
  packets.nmsgs = 2;
  ioctl(m_file, I2C_RDWR, &packets);

  return (reg_data[0]<<8 | reg_data[1]);
}

uint16_t Piston::get_piston_full_exit_position(){
  __u8 reg_addr[3], reg_data[2];
  reg_addr[0] = I2C_PISTON_REQUEST;
  reg_addr[1] = I2C_PISTON_BLANK_VALUE;
  reg_addr[2] = 0x06;

  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg messages[2];
  /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
  messages[0].addr = m_i2c_addr;
  messages[0].flags = 0;
  messages[0].len = 3;
  messages[0].buf = (char*)&reg_addr;

  /* The data will get returned in this structure */
  messages[1].addr = m_i2c_addr;
  messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
  messages[1].len = 2;
  messages[1].buf = (char*)&reg_data;

  /* Send the request to the kernel and get the result back */
  packets.msgs = messages;
  packets.nmsgs = 2;
  ioctl(m_file, I2C_RDWR, &packets);

  return (reg_data[0]<<8 | reg_data[1]);
}






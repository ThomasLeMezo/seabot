#ifndef PISTON_H
#define PISTON_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>

#include <ros/ros.h>

#define I2C_PISTON_SPEED_IN 0xAB
#define I2C_PISTON_SPEED_OUT 0xAC
#define I2C_PISTON_CMD 0x00
#define I2C_PISTON_BLANK_VALUE 0xAA

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

class Piston
{
public:
  /**
   * @brief Piston
   */
  Piston();

  /**
   * @brief Piston
   */
  ~Piston();

  /**
   * @brief i2c_open
   * @return
   */
  int i2c_open();

  /**
   * @brief set_piston_speed
   * @param speed_in
   * @param speed_out
   */
  //void set_piston_speed(const __u8 &speed_in, const __u8 &speed_out) const;
  void set_piston_speed_in(const __u8 &speed_in) const;
  void set_piston_speed_out(const __u8 &speed_out) const;
  

  /**
   * @brief set_piston_speed_reset
   * @param speed
   */
  void set_piston_speed_reset(const __u8 &speed) const;

  /**
   * @brief set_piston_position
   * @param position
   */
  void set_piston_position(__u16 position) const;

  /**
   * @brief set_led_enable
   * @param val
   */
  void set_led_enable(const bool &val) const;

  /**
   * @brief set_error_interval
   * @param val
   */
  void set_error_interval(const __u8 &val) const;

  /**
   * @brief set_time_shift_error
   * @param val
   */
  void set_time_shift_error(const __u8 &val) const;

  /**
   * @brief set_reached_switch_off
   * @param val
   */
  void set_reached_switch_off(const bool &val) const;

  /**
   * @brief set_piston_reset
   */
  void set_piston_reset() const;

  /**
   * @brief set_piston_emergency
   */
  void set_piston_emergency() const;

  /**
   * @brief update_piston_all_data
   */
  void get_piston_all_data();

  /**
   * @brief get_piston_set_point
   */
  void get_piston_set_point();

  /**
   * @brief get_version
   */
  uint8_t& get_version();

  /**
   * @brief write_cmd
   * @param left
   * @param right
   */
  void write_cmd(const unsigned short &left, const unsigned short &right) const;

public:
  float m_position = 0;
  bool m_switch_out = false;
  bool m_switch_in = false;
  bool m_switch_halfway = false; //modifications
  uint16_t m_state = 0;
  bool m_system_on = false;
  bool m_motor_on = true;
  bool m_enable_on = true;
  uint16_t m_position_set_point = 0;
  uint16_t m_motor_speed = 0;

  uint8_t m_version=0;



private:
  int m_file;
  const int m_i2c_addr = 0x38;
  const char* m_i2c_periph = "/dev/i2c-1";
};


#endif // PISTON_H

#ifndef PRESSURE_H
#define PRESSURE_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

class Pressure_89BSD
{
public:
  Pressure_89BSD();
  ~Pressure_89BSD();

  int i2c_open();
  int init_sensor();
  int get_value();

  int reset();

private:

  int m_file;
  const int m_i2c_addr = 0x77;
  const char* m_i2c_periph = "/dev/i2c-1";
  const float m_p_min = 0.0; // In Bar
  const float m_p_max = 6.0; // In Bar

  u_int16_t  m_prom[7];
  int16_t m_C0, m_C1, m_C2, m_C3, m_C4, m_C5, m_C6, m_A0, m_A1, m_A2;

};

int16_t bin2decs(u_int16_t val, size_t nb_bit);

#endif // PRESSURE_H

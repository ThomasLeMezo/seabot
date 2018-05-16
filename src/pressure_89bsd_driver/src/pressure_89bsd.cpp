#include "pressure_89bsd.h"

#define Q0 9
#define Q1 11
#define Q2 9
#define Q3 15
#define Q4 15
#define Q5 16
#define Q6 16

#define T_MIN -20.0
#define T_MAX 85.0
#define P_MAX 6.0
#define P_MIN 0.0

using namespace std;

int16_t bin2decs(u_int16_t val, size_t nb_bit){
    if((val & 0b1<<(nb_bit-1))==0)
        return val;
    else
        return (val-(1<<(nb_bit)));
}

Pressure_89BSD::Pressure_89BSD()
{
}

Pressure_89BSD::~Pressure_89BSD(){
  if(m_file !=0)
    close(m_file);
}

int Pressure_89BSD::reset(){
  int res = i2c_smbus_write_byte(m_file, CMD_RESET);
  ros::Duration(0.03).sleep(); // 28ms reload for the sensor (?)
//  usleep(30000);
  if (res < 0)
    ROS_WARN("[Pressure_89BSD] Error reseting sensor");
  else
    ROS_INFO("[Pressure_89BSD] Reset ok");
  return 0;
}

int Pressure_89BSD::i2c_open(){
  if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
    ROS_WARN("Failed to open the I2C bus (%s)", m_i2c_periph);
    return 1;
  }

  if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
    ROS_WARN("Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
    return 1;
  }
  return 0;
}

int Pressure_89BSD::init_sensor(){
  ROS_INFO("[Pressure_89BSD] Sensor initialization");
  reset();
  int return_val = 0;
  u_int16_t  prom[7];

  unsigned char buff[2] = {0, 0};
  for(int i=0; i<7; i++){
    __u8 add = CMD_PROM + (char) 2*(i+1);
    if (i2c_smbus_read_i2c_block_data(m_file, add, 2, buff)!=2){
        ROS_WARN("[Pressure_89BSD] Error Reading 0x%X", add);
        return_val = 1;
    }
    prom[i] = (buff[0] << 8) | buff[1] << 0;
  }
  if(return_val==0)
    ROS_INFO("[Pressure_89BSD] Sensor Read PROM OK");

  m_C0 = bin2decs(prom[0]>>2,14);
  m_C1 = bin2decs(((prom[0] & 0x3)<<12) | (prom[1] >> 4), 14);
  m_C2 = bin2decs(((prom[1] & 0xF)<<6) | (prom[2] >> 10), 10);
  m_C3 = bin2decs((prom[2] & 0x3FF), 10);
  m_C4 = bin2decs((prom[3] >> 6), 10);
  m_C5 = bin2decs(((prom[3] & 0x3F) << 4) | (prom[4]>>12), 10);
  m_C6 = bin2decs(((prom[4] >> 2) & 0x3FF), 10);
  m_A0 = bin2decs(((prom[4] & 0x3) << 8) | (prom[5]>>8), 10);
  m_A1 = bin2decs(((prom[5] & 0xFF) << 2) | (prom[6]>>14), 10);
  m_A2 = bin2decs(((prom[5] >> 3) & 0x3FF), 10);

  ROS_INFO("[Pressure_89BSD] C0 = %d", m_C0);
  ROS_INFO("[Pressure_89BSD] C1 = %d", m_C1);
  ROS_INFO("[Pressure_89BSD] C2 = %d", m_C2);
  ROS_INFO("[Pressure_89BSD] C3 = %d", m_C3);
  ROS_INFO("[Pressure_89BSD] C4 = %d", m_C4);
  ROS_INFO("[Pressure_89BSD] C5 = %d", m_C5);
  ROS_INFO("[Pressure_89BSD] C6 = %d", m_C6);

  ROS_INFO("[Pressure_89BSD] A0 = %d", m_A0);
  ROS_INFO("[Pressure_89BSD] A1 = %d", m_A1);
  ROS_INFO("[Pressure_89BSD] A2 = %d", m_A2);

  return return_val;
}

int Pressure_89BSD::measure(){
  get_D1();
  get_D2();

  double x = (double)m_D2/(double)(2<<24);
  m_temperature = m_A0/3.0+2.0*m_A1*x+2.0*m_A2*x*x;

  ROS_INFO("[Pressure_89BSD] D1 = %lu", m_D1);
  ROS_INFO("[Pressure_89BSD] D2 = %lu", m_D2);
  ROS_INFO("[Pressure_89BSD] x = %f", x);

  double top = m_D1 + m_C0*(double)(2<<Q0) + m_C3*(double)(2<<Q3)*x + m_C4*(double)(2<<Q4)*x*x;
  double bot = m_C1*(double)(2<<Q1) + m_C5*(double)(2<<Q5)*x + m_C6*(double)(2<<Q6)*x*x;
  double y = top/bot;
  double z = (2<<Q2)/(double)((2<<24));

  double p = (1.0-m_C2*z)*y+m_C2*z*y*y;

  m_pressure = ((p-0.1)/0.8*(P_MAX - P_MIN) + P_MIN)*1.0e5;
}

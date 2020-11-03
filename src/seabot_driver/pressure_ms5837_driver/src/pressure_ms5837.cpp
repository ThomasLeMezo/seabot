#include "pressure_ms5837.h"
#include <sys/ioctl.h>

#define T_MIN -20.0
#define T_MAX 85.0
#define T_REF 20.0
#define P_MAX 30.0
#define P_MIN 0.0

using namespace std;

int16_t bin2decs(u_int16_t val, size_t nb_bit){
    if((val & 0b1<<(nb_bit-1))==0)
        return val;
    else
        return (val-(1<<(nb_bit)));
}

Pressure_ms5837::Pressure_ms5837()
{
}

Pressure_ms5837::~Pressure_ms5837(){
    if(m_file !=0)
        close(m_file);
}

int Pressure_ms5837::reset(){
    int res = i2c_smbus_write_byte(m_file, CMD_RESET);
    ros::Duration(0.03).sleep(); // 28ms reload for the sensor (?)
    //  usleep(30000);
    if (res < 0)
        ROS_WARN("[Pressure_ms5837] Error reseting sensor");
    else
        ROS_DEBUG("[Pressure_ms5837] Reset ok");
    return 0;
}

int Pressure_ms5837::i2c_open(){
    if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
        ROS_WARN("[Pressure_ms5837] Failed to open the I2C bus (%s)", m_i2c_periph);
        exit(1);
    }

    if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
        ROS_WARN("[Pressure_ms5837] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }
    return 0;
}

int Pressure_ms5837::init_sensor(){
    ROS_DEBUG("[Pressure_ms5837] Sensor initialization");
    reset();
    int return_val = 0;
    u_int16_t  m_C[5];

    unsigned char buff[2] = {0, 0};
    for(int i=0; i<5; i++){
        __u8 add = CMD_PROM + (char) 2*(i+1);
        if (i2c_smbus_read_i2c_block_data(m_file, add, 2, buff)!=2){
            ROS_WARN("[Pressure_ms5837] Error Reading 0x%X", add);
            return_val = 1;
        }
        m_C[i] = (buff[0] << 8) | buff[1] << 0;
    }
    if(return_val==0)
        ROS_DEBUG("[Pressure_ms5837] Sensor Read PROM OK");

    ROS_DEBUG("[Pressure_ms5837] C1 = %d", m_C[0]);
    ROS_DEBUG("[Pressure_ms5837] C2 = %d", m_C[1]);
    ROS_DEBUG("[Pressure_ms5837] C3 = %d", m_C[2]);
    ROS_DEBUG("[Pressure_ms5837] C4 = %d", m_C[3]);
    ROS_DEBUG("[Pressure_ms5837] C5 = %d", m_C[4]);

    return return_val;
}

bool Pressure_ms5837::measure(){
    get_D1();
    get_D2();

    if(m_valid_data = true){

       int32_t dT = m_D2-(m_C[4]<<8);
       int32_t TEMP = 2000+((dT*m_C[5])>>23);

       int64_t OFF = (m_C[1]<<16)+((m_C[3]*dT)>>7);
       int64_t SENS = (m_C[0]<<15)+((m_C[2]*dT)>>8);
       int32_t P = (((m_D1*SENS)>>21)-OFF)>>13;

        m_temperature = TEMP/100.;
        m_pressure = P/100.;

        if(m_temperature < 0.0 || m_temperature > 50.0 || m_pressure<0.7 || m_pressure>6.5){
            m_valid_data = false;
            ROS_WARN("[Pressure_ms5837] Data out of range (p=%f t=%f)", m_pressure, m_temperature);
            if(m_temperature >0.0 && m_temperature < 50.0 && m_pressure > 0.7 && m_pressure<10.0) // detection of overpressure (>50m)
              return true;
            else
              return false;
        }
    }
    else
        return false;
    return true;
}

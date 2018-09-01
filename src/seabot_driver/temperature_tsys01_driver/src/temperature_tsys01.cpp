#include "temperature_tsys01.h"

using namespace std;

Temperature_TSYS01::Temperature_TSYS01(){
}

Temperature_TSYS01::~Temperature_TSYS01(){
    if(m_file !=0)
        close(m_file);
}

int Temperature_TSYS01::reset(){
    int res = i2c_smbus_write_byte(m_file, CMD_RESET);
    ros::Duration(0.03).sleep(); // 28ms reload for the sensor (?)
    //  usleep(30000);
    if (res < 0)
        ROS_WARN("[Temperature_TSYS01] Error reseting sensor");
    else
        ROS_DEBUG("[Temperature_TSYS01] Reset ok");
    return 0;
}

int Temperature_TSYS01::i2c_open(){
    if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
        ROS_WARN("[Temperature_TSYS01] Failed to open the I2C bus (%s)", m_i2c_periph);
        exit(1);
    }

    if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
        ROS_WARN("[Temperature_TSYS01] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }
    return 0;
}

int Temperature_TSYS01::init_sensor(){
    ROS_DEBUG("[Temperature_TSYS01] Sensor initialization");
    reset();
    int return_val = 0;
    u_int16_t  prom[5];

    unsigned char buff[2] = {0, 0};
    for(int i=0; i<5; i++){
        __u8 add = CMD_PROM + (char) 2*(i+1); // Start at 0xA2
        if (i2c_smbus_read_i2c_block_data(m_file, add, 2, buff)!=2){
            ROS_WARN("[Temperature_TSYS01] Error Reading 0x%X", add);
            return_val = 1;
        }
        m_k[4-i] = (buff[0] << 8) | buff[1] << 0;
    }
    if(return_val==0)
        ROS_DEBUG("[Temperature_TSYS01] Sensor Read PROM OK");

    ROS_DEBUG("[Temperature_TSYS01] k0 = %d", m_k[0]);
    ROS_DEBUG("[Temperature_TSYS01] k1 = %d", m_k[1]);
    ROS_DEBUG("[Temperature_TSYS01] k2 = %d", m_k[2]);
    ROS_DEBUG("[Temperature_TSYS01] k3 = %d", m_k[3]);
    ROS_DEBUG("[Temperature_TSYS01] k4 = %d", m_k[4]);

    return return_val;
}

bool Temperature_TSYS01::measure(){
    i2c_smbus_write_byte(m_file, CMD_ADC_CONV);
    ros::Duration(0.05).sleep();
    unsigned char buff[3] = {0, 0, 0};
    if (i2c_smbus_read_i2c_block_data(m_file, CMD_ADC_READ, 3, buff)!=3){
        ROS_WARN("[Temperature_TSYS01] Error Reading T");
        m_valid_data = false;
        return -1;
    }

    double adc24 = (buff[0] << 16) | (buff[1] << 8) | buff[2];
    if(adc24==0)
      m_valid_data = false;
    else
      m_valid_data = true;

    double adc16 = adc24/256.0;
    m_temperature = -2.0*m_k[4]*1e-21*pow(adc16, 4)
                    +4.0*m_k[3]*1e-16*pow(adc16, 3)
                    -2.0*m_k[2]*1e-11*pow(adc16, 2)
                    +1.0*m_k[1]*1e-6*adc16
                    -1.5*m_k[0]*1e-2;
    return m_valid_data;
}

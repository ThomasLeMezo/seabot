#include <iostream>
#include <unistd.h>

extern "C"{
  #include "bme280.h"
  #include "bme280_defs.h"
}

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/RelativeHumidity.h>

#include <pressure_bme280_driver/Bme280Data.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
extern "C" {
    #include <linux/i2c-dev.h>
}
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
 extern "C" {
    #include <i2c/smbus.h>
}
#endif

using namespace std;
double m_pressure = 0.0;
double m_temperature = 0.0;
double m_humidity = 0.0;

int file;
struct bme280_data comp_data;

void user_delay_ms(uint32_t period){
    usleep(period*1000);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
  return i2c_smbus_read_i2c_block_data(file, reg_addr, len, reg_data) != len;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len){
  return i2c_smbus_write_i2c_block_data(file, reg_addr, len, reg_data);
}

void print_calib_settings(struct bme280_dev &dev){
    ROS_DEBUG("[Pressure_BME280] dig_T1 = %i", dev.calib_data.dig_T1);
    ROS_DEBUG("[Pressure_BME280] dig_T2 = %i", dev.calib_data.dig_T2);
    ROS_DEBUG("[Pressure_BME280] dig_T3 = %i", dev.calib_data.dig_T3);

    ROS_DEBUG("[Pressure_BME280] dig_P1 = %i", dev.calib_data.dig_P1);
    ROS_DEBUG("[Pressure_BME280] dig_P2 = %i", dev.calib_data.dig_P2);
    ROS_DEBUG("[Pressure_BME280] dig_P3 = %i", dev.calib_data.dig_P3);
    ROS_DEBUG("[Pressure_BME280] dig_P4 = %i", dev.calib_data.dig_P4);
    ROS_DEBUG("[Pressure_BME280] dig_P5 = %i", dev.calib_data.dig_P5);
    ROS_DEBUG("[Pressure_BME280] dig_P6 = %i", dev.calib_data.dig_P6);
    ROS_DEBUG("[Pressure_BME280] dig_P7 = %i", dev.calib_data.dig_P7);
    ROS_DEBUG("[Pressure_BME280] dig_P8 = %i", dev.calib_data.dig_P8);
    ROS_DEBUG("[Pressure_BME280] dig_P9 = %i", dev.calib_data.dig_P9);

    ROS_DEBUG("[Pressure_BME280] dig_H1 = %i", dev.calib_data.dig_H1);
    ROS_DEBUG("[Pressure_BME280] dig_H2 = %i", dev.calib_data.dig_H2);
    ROS_DEBUG("[Pressure_BME280] dig_H3 = %i", dev.calib_data.dig_H3);
    ROS_DEBUG("[Pressure_BME280] dig_H4 = %i", dev.calib_data.dig_H4);
    ROS_DEBUG("[Pressure_BME280] dig_H5 = %i", dev.calib_data.dig_H5);
    ROS_DEBUG("[Pressure_BME280] dig_H6 = %i", dev.calib_data.dig_H6);
}

void print_settings(struct bme280_dev &dev){
    ROS_DEBUG("[Pressure_BME280] dev_id = %i", dev.dev_id);
    ROS_DEBUG("[Pressure_BME280] chip_id = %i", dev.chip_id);
    ROS_DEBUG("[Pressure_BME280] settings.filter = %i", dev.settings.filter);
    ROS_DEBUG("[Pressure_BME280] settings.osr_h = %i", dev.settings.osr_h);
    ROS_DEBUG("[Pressure_BME280] settings.osr_p = %i", dev.settings.osr_p);
    ROS_DEBUG("[Pressure_BME280] settings.osr_t = %i", dev.settings.osr_t);
    ROS_DEBUG("[Pressure_BME280] settings.standby_time = %i", dev.settings.standby_time);
}

void print_sensor_mode(struct bme280_dev &dev){
    uint8_t sensor_mode;
    bme280_get_sensor_mode(&sensor_mode, &dev);
    ROS_DEBUG("[Pressure_BME280] Sensor Mode = %i", sensor_mode);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pressure_bme280");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    double frequency = n_private.param<double>("frequency", 2.0);
    bool primary_i2c_address = n_private.param<bool>("primary_i2c_address", true);
    const char* m_i2c_periph = "/dev/i2c-1";

    // Publishers
    ros::Publisher pub = n.advertise<pressure_bme280_driver::Bme280Data>("sensor_internal", 1);

    // Sensor initialization
    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    if(primary_i2c_address)
      dev.dev_id = BME280_I2C_ADDR_PRIM;
    else
      dev.dev_id = BME280_I2C_ADDR_SEC;

    dev.intf = BME280_I2C_INTF;

    if ((file = open(m_i2c_periph,O_RDWR)) < 0) {
        ROS_WARN("[Pressure_BME280] Failed to open the I2C bus (%s)", m_i2c_periph);
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,dev.dev_id) < 0) {
        ROS_WARN("[Pressure_BME280] Failed to acquire bus access and/or talk to slave (0x%X)", I2C_SLAVE);
        exit(1);
    }

    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev); // Get Calib data
    print_calib_settings(dev);

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_2X; // 16X
    dev.settings.osr_t = BME280_OVERSAMPLING_2X; // 2X
    dev.settings.filter = BME280_FILTER_COEFF_2; // 16
    dev.settings.standby_time = BME280_STANDBY_TIME_125_MS;

    uint8_t settings_sel;
    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    print_settings(dev);

    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    print_sensor_mode(dev);

    ros::Duration(1.5).sleep(); // Sleep to activate Normal Mode

    // Loop with sensor reading
    pressure_bme280_driver::Bme280Data msg;

    ROS_INFO("[Pressure_BME280] Start Ok");
    ros::Rate loop_rate(frequency);
    while (ros::ok()){
        ros::spinOnce();

        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        m_pressure = comp_data.pressure/100.0;
        m_humidity = comp_data.humidity;
        m_temperature = comp_data.temperature;

        msg.temperature = m_temperature;
        msg.pressure = m_pressure;
        msg.humidity = m_humidity;
        pub.publish(msg);

        loop_rate.sleep();
    }

    return 0;
}

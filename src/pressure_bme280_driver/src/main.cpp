#include <iostream>
#include <unistd.h>
#include "bme280.h"
#include "bme280_defs.h"

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/RelativeHumidity.h>

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
#include <linux/i2c-dev.h>

using namespace std;

int file;

void user_delay_ms(uint32_t period){
    usleep(period*1000);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
         * In order to read a register, we first do a "dummy write" by writing
         * 0 bytes to the register we want to read from.  This is similar to
         * the packet in set_i2c_register, except it's 1 byte rather than 2.
         */
    messages[0].addr = dev_id;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = (char*)&reg_addr;

    /* The data will get returned in this structure */
    messages[1].addr = dev_id;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len = len;
    messages[1].buf = (char*)reg_data;

    /* Send the request to the kernel and get the result back */
    packets.msgs = messages;
    packets.nmsgs = 2;
    if (ioctl(file, I2C_RDWR, &packets) < 0) {
#ifdef DEBUG_MODE
        printf("Unable to send/receive data\n");
#endif
        return 1;
    }

    return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    uint8_t buff[len + 1];
    buff[0] = reg_addr;
    for (uint16_t i = 0; i < len; i++)
        buff[i + 1] = reg_data[i];

    messages[0].addr = dev_id;
    messages[0].flags = 0;
    messages[0].len = len + 1;
    messages[0].buf = (char*)buff;

    /* Transfer the i2c packets to the kernel and verify it worked */
    packets.msgs = messages;
    packets.nmsgs = 1;
    if (ioctl(file, I2C_RDWR, &packets) < 0) {
#ifdef DEBUG_MODE
        printf("Unable to send data\n");
#endif
        ROS_WARN("UNABLE TO SEND DATA");
        return 1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pressure_89bsd");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    double frequency = n_private.param<double>("frequency", 2.0);

    // Publishers
    ros::Publisher temperature_pub = n.advertise<sensor_msgs::Temperature>("temperature_int", 1);
    ros::Publisher pressure_pub = n.advertise<sensor_msgs::FluidPressure>("pressure_int", 1);
    ros::Publisher humidity_pub = n.advertise<sensor_msgs::RelativeHumidity>("humidity_int", 1);

    // Sensor initialization
    struct bme280_dev dev;
    int8_t rslt = BME280_OK;

    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;

    if ((file = open("/dev/i2c-1",O_RDWR)) < 0) {
        ROS_WARN("Failed to open the I2C bus");
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,dev.dev_id) < 0) {
        ROS_WARN("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev); // Get Calib data

    ROS_INFO("[Pressure BME280] dig_T1 = %i", dev.calib_data.dig_T1);
    ROS_INFO("[Pressure BME280] dig_T2 = %i", dev.calib_data.dig_T2);
    ROS_INFO("[Pressure BME280] dig_T3 = %i", dev.calib_data.dig_T3);

    ROS_INFO("[Pressure BME280] dig_P1 = %i", dev.calib_data.dig_P1);
    ROS_INFO("[Pressure BME280] dig_P2 = %i", dev.calib_data.dig_P2);
    ROS_INFO("[Pressure BME280] dig_P3 = %i", dev.calib_data.dig_P3);
    ROS_INFO("[Pressure BME280] dig_P4 = %i", dev.calib_data.dig_P4);
    ROS_INFO("[Pressure BME280] dig_P5 = %i", dev.calib_data.dig_P5);
    ROS_INFO("[Pressure BME280] dig_P6 = %i", dev.calib_data.dig_P6);
    ROS_INFO("[Pressure BME280] dig_P7 = %i", dev.calib_data.dig_P7);
    ROS_INFO("[Pressure BME280] dig_P8 = %i", dev.calib_data.dig_P8);
    ROS_INFO("[Pressure BME280] dig_P9 = %i", dev.calib_data.dig_P9);

    ROS_INFO("[Pressure BME280] dig_H1 = %i", dev.calib_data.dig_H1);
    ROS_INFO("[Pressure BME280] dig_H2 = %i", dev.calib_data.dig_H2);
    ROS_INFO("[Pressure BME280] dig_H3 = %i", dev.calib_data.dig_H3);
    ROS_INFO("[Pressure BME280] dig_H4 = %i", dev.calib_data.dig_H4);
    ROS_INFO("[Pressure BME280] dig_H5 = %i", dev.calib_data.dig_H5);
    ROS_INFO("[Pressure BME280] dig_H6 = %i", dev.calib_data.dig_H6);

    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X; // 16X
    dev.settings.osr_t = BME280_OVERSAMPLING_2X; // 2X
    dev.settings.filter = BME280_FILTER_COEFF_16; // 16
    dev.settings.standby_time = BME280_STANDBY_TIME_125_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    settings_sel |= BME280_OSR_HUM_SEL;
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;

    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    rslt = bme280_set_sensor_settings(settings_sel, &dev);

    uint8_t sensor_mode;
    int8_t result = bme280_get_sensor_mode(&sensor_mode, &dev);
    ROS_INFO("[Pressure BME280] Sensor Mode = %ui | reading result = %i", sensor_mode, result);

    dev.delay_ms(70);

    //  - int32_t for temperature with the units 100 * °C
    //  - uint32_t for humidity with the units 1024 * % relative humidity
    //  - uint32_t for pressure
    //       If macro "BME280_64BIT_ENABLE" is enabled, which it is by default, the unit is 100 * Pascal
    //       If this macro is disabled, Then the unit is in Pascal

    // Loop with sensor reading
    sensor_msgs::Temperature temperature_msg;
    sensor_msgs::FluidPressure pressure_msg;
    sensor_msgs::RelativeHumidity humidity_msg;

    ROS_INFO("[Pressure BME280] Start Reading data");

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {

        //      rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        //	  dev.delay_ms(40);
        /* Wait for the measurement to complete */
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

        temperature_msg.temperature = comp_data.temperature;
        pressure_msg.fluid_pressure =  comp_data.pressure;
        humidity_msg.relative_humidity = comp_data.humidity;

        temperature_msg.header.stamp = ros::Time::now();
        pressure_msg.header.stamp = temperature_msg.header.stamp;
        humidity_msg.header.stamp = temperature_msg.header.stamp;

        temperature_pub.publish(temperature_msg);
        pressure_pub.publish(pressure_msg);
        humidity_pub.publish(humidity_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

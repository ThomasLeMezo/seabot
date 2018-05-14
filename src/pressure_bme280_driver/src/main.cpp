#include <iostream>
#include <unistd.h>
#include "bme280.h"
#include "bme280_defs.h"

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/RelativeHumidity.h>

#include <linux/i2c-dev.h>
#include <fcntl.h>

using namespace std;

int file;

void user_delay_ms(uint32_t period){
  usleep(period*1000);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
  if (i2c_smbus_read_i2c_block_data(file, reg_addr, len, reg_data)!=len)
    rslt = 1;
  return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
  if (i2c_smbus_write_block_data(file, reg_addr, len, reg_data)<0)
    rslt = 1;
  return rslt;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pressure_89bsd");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 5.0);

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

  rslt = bme280_init(&dev);

  uint8_t settings_sel;
  struct bme280_data comp_data;

  /* Recommended mode of operation: Indoor navigation */
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL
      | BME280_OSR_TEMP_SEL
      | BME280_OSR_HUM_SEL
      | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings(settings_sel, &dev);

  //  - int32_t for temperature with the units 100 * °C
  //  - uint32_t for humidity with the units 1024 * % relative humidity
  //  - uint32_t for pressure
  //       If macro "BME280_64BIT_ENABLE" is enabled, which it is by default, the unit is 100 * Pascal
  //       If this macro is disabled, Then the unit is in Pascal

  // Loop with sensor reading
  sensor_msgs::Temperature temperature_msg;
  sensor_msgs::FluidPressure pressure_msg;
  sensor_msgs::RelativeHumidity humidity_msg;

  ros::Rate loop_rate(frequency);
  while (ros::ok())
  {
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    /* Wait for the measurement to complete */
    dev.delay_ms(40);
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    temperature_msg.temperature = comp_data.temperature/100.0;
    pressure_msg.fluid_pressure =  comp_data.pressure;
    humidity_msg.relative_humidity = comp_data.humidity/1024.0;

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

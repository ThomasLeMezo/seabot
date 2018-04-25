#include <iostream>
#include "pressure_89bsd.h"
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>

using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pressure_89bsd");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 5.0);

  // Publishers
  ros::Publisher temperature_pub = n.advertise<sensor_msgs::Temperature>("temperature_ext", 1);
  ros::Publisher pressure_pub = n.advertise<sensor_msgs::FluidPressure>("pressure_ext", 1);

  // Sensor initialization
  Pressure_89BSD p1;
  p1.i2c_open();
  p1.init_sensor();

  // Loop with sensor reading
  sensor_msgs::Temperature temperature_msg;
  sensor_msgs::FluidPressure pressure_msg;

  ros::Rate loop_rate(frequency);
  while (ros::ok())
  {
    p1.measure();
    temperature_msg.temperature = p1.get_temperature();
    temperature_msg.header.stamp = ros::Time::now();
    pressure_msg.fluid_pressure = p1.get_pression();
    pressure_msg.header.stamp = temperature_msg.header.stamp;

    temperature_pub.publish(temperature_msg);
    pressure_pub.publish(pressure_msg);

    loop_rate.sleep();
  }

  return 0;
}

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

    Pressure_89BSD p1;
    p1.i2c_open();
    p1.init_sensor();
    p1.get_value();

    return 0;
}

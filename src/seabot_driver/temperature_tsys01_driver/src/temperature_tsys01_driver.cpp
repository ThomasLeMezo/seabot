#include <iostream>
#include "temperature_tsys01.h"
#include <unistd.h>

#include <ros/ros.h>

#include <temperature_tsys01_driver/Temperature.h>

using namespace std;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "temperature_tsys01");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    double frequency = n_private.param<double>("frequency", 5.0);

    // Publishers
    ros::Publisher pub = n.advertise<temperature_tsys01_driver::Temperature>("sensor_temperature", 1);

    // Sensor initialization
    Temperature_TSYS01 p1;
    p1.i2c_open();
    p1.init_sensor();

    // Loop with sensor reading
    temperature_tsys01_driver::Temperature msg;

    ros::Rate loop_rate(frequency);
    while (ros::ok()){
        ros::spinOnce();
        if(p1.measure() == true){
            msg.temperature = p1.get_temperature();
            pub.publish(msg);
        }
        loop_rate.sleep();
    }
    return 0;
}

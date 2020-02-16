#include <iostream>
#include "pressure_89bsd.h"
#include <unistd.h>

#include <ros/ros.h>

#include <pressure_89bsd_driver/PressureBsdData.h>

using namespace std;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "pressure_89bsd");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    double frequency = n_private.param<double>("frequency", 5.0);

    // Publishers
    ros::Publisher pub = n.advertise<pressure_89bsd_driver::PressureBsdData>("sensor_external", 1);

    // Sensor initialization
    Pressure_89BSD p1;
    p1.i2c_open();
    p1.init_sensor();

    // Loop with sensor reading
    pressure_89bsd_driver::PressureBsdData msg;

    ROS_INFO("[Pressure_89BSD] Start Ok");
    ros::Rate loop_rate(frequency);
    while (ros::ok()){
        ros::spinOnce();
        msg.header.stamp = ros::Time::now();
        if(p1.measure() == true){
            msg.temperature = p1.get_temperature();
            msg.pressure = p1.get_pression();
            pub.publish(msg);
        }
        loop_rate.sleep();
    }
    return 0;
}

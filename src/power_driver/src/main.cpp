#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "power.h"
#include "power_driver/Battery.h"

#include <std_srvs/SetBool.h>

using namespace std;

Power p;

bool led_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  p.enable_led(req.data);
  res.success = true;
  return true;
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "thrusters");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 0.2);

  // Sensor initialization
  p.i2c_open();

  // Publisher
  ros::Publisher battery_pub = n.advertise<power_driver::Battery>("battery", 1);
  power_driver::Battery battery_msg;

  // Service (ON/OFF)
  ros::ServiceServer service = n.advertiseService("led_enable", led_enable);

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    p.measure_battery();

    battery_msg.battery1 = p.get_level_battery(0);
    battery_msg.battery2 = p.get_level_battery(1);
    battery_msg.battery3 = p.get_level_battery(2);
    battery_msg.battery4 = p.get_level_battery(3);
    battery_msg.header.stamp = ros::Time::now();
    battery_pub.publish(battery_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

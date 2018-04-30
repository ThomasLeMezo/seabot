#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "neom8l.h"

using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gnss_neom8l");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 5.0);

  // Publishers
  ros::Publisher navSatFix_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 1);

  // Sensor init
  NeoM8L sensor;

  // Loop with sensor reading
  sensor_msgs::NavSatFix navSatFix_msg;

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    // ToDo
    sensor.read_data();

    if(navSatFix_msg.latitude != sensor.get_nmea_info().lat || navSatFix_msg.longitude != sensor.get_nmea_info().lon){
      navSatFix_msg.altitude = sensor.get_nmea_info().elv;
      navSatFix_msg.latitude = sensor.get_nmea_info().lat;
      navSatFix_msg.longitude = sensor.get_nmea_info().lon;
      navSatFix_msg.status.status = sensor.get_nmea_info().sig;
      navSatFix_msg.header.stamp = ros::Time::now();

      navSatFix_pub.publish(navSatFix_msg);
    }

    loop_rate.sleep();
  }

  return 0;
}

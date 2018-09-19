#include <ros/ros.h>
#include "gpsdclient.h"

using namespace gpsd_client;
using namespace sensor_msgs;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");

  GPSDClient client;

  if (!client.start())
    return -1;

  ROS_INFO("[GPSD_Client] Start Ok");
//  ros::Rate loop_rate(1.0);
  while(ros::ok()) {
//    ros::spinOnce();
    client.step();
//    loop_rate.sleep();
  }

  client.stop();
}

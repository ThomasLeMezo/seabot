#include <ros/ros.h>
#include "gpsdclient.h"

using namespace gpsd_client;
using namespace sensor_msgs;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");

  GPSDClient client;

  if (!client.start())
    return -1;


  while(ros::ok()) {
    ros::spinOnce();
    client.step();
  }

  client.stop();
}

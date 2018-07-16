#include <ros/ros.h>
#include <cmath>
#include "iridium.h"

using namespace std;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "iridium_node");
  ros::NodeHandle n;

//  piston_position = rospy.Publisher('/regulation/depth_set_point', DepthPose, queue_size=1)

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);

  ros::Rate loop_rate(frequency);

  while (ros::ok()){
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <seabot_fusion/GnssPose.h>
#include <cmath>

#include <proj_api.h>

using namespace std;
double latitude, longitude, altitude;
bool new_data = false;

void navSatFix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  latitude =  msg->latitude;
  longitude =  msg->longitude;
  altitude =  msg->altitude;
  new_data = true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lambert_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double offset_east = n_private.param<double>("offset_east", 0.0);
  double offset_north = n_private.param<double>("offset_north", 0.0);
  double frequency = n_private.param<double>("frequency", 1.0);

  // Init proj
  projPJ pj_lambert, pj_latlong;
  if (!(pj_lambert = pj_init_plus("+init=epsg:2154"))){
    ROS_WARN("[Lambert_node] Error Lambert \n");
    exit(1);
  }

  if (!(pj_latlong = pj_init_plus("+init=epsg:4326")))    {
    ROS_WARN("[Lambert_node] Error LatLong \n");
    exit(1);
  }

  // Publishers
  ros::Subscriber navSatFix_sub = n.subscribe("/driver/fix", 1, navSatFix_callback);
  ros::Publisher pose_pub = n.advertise<seabot_fusion::GnssPose>("pose", 1);

  seabot_fusion::GnssPose msg_pose;

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(new_data){
      double east = longitude*M_PI/180.0; // Longitude
      double north = latitude*M_PI/180.0; // Latitude
      pj_transform(pj_latlong, pj_lambert, 1, 1, &east, &north, nullptr);
      msg_pose.east = east + offset_east;
      msg_pose.north = north + offset_north;
      new_data = false;

      pose_pub.publish(msg_pose);
    }

    loop_rate.sleep();
  }

  pj_free(pj_latlong);
  pj_free(pj_lambert);
  return 0;
}


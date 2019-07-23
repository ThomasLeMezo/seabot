#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <gpsd_client/GPSFix.h>
#include <seabot_fusion/GnssPose.h>
#include <cmath>

#include <proj_api.h>

using namespace std;
double latitude, longitude;
bool new_data = false;
bool data_valid = false;

void navFix_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  latitude =  msg->latitude;
  longitude =  msg->longitude;
  data_valid = (msg->status>=msg->STATUS_MODE_2D);
  new_data = true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lambert_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 1.0);

  // Init proj
  projPJ pj_lambert, pj_latlong;
  if (!(pj_lambert = pj_init_plus("+init=epsg:2154"))){
    ROS_WARN("[Lambert_node] Error Lambert \n");
    exit(1);
  }

  if (!(pj_latlong = pj_init_plus("+init=epsg:4326"))){
    ROS_WARN("[Lambert_node] Error LatLong \n");
    exit(1);
  }

  // Topics
  ros::Subscriber navFix_sub = n.subscribe("/driver/fix", 1, navFix_callback);
  ros::Publisher pose_pub = n.advertise<seabot_fusion::GnssPose>("pose", 1);

  seabot_fusion::GnssPose msg_pose;

  ROS_INFO("[FUSION lambert] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(new_data){
      if(longitude != 0. && latitude != 0. && data_valid){
        double east = longitude*M_PI/180.0; // Longitude
        double north = latitude*M_PI/180.0; // Latitude
        pj_transform(pj_latlong, pj_lambert, 1, 1, &east, &north, nullptr);
        msg_pose.east = east;
        msg_pose.north = north;

        pose_pub.publish(msg_pose);
      }
      new_data = false;
    }

    loop_rate.sleep();
  }

  pj_free(pj_latlong);
  pj_free(pj_lambert);
  return 0;
}


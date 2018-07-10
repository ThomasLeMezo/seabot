#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/GnssPose.h>
#include <seabot_mission/Waypoint.h>
#include <seabot_thruster_driver/Velocity.h>
#include <geometry_msgs/Vector3.h>
#include <gpsd_client/GPSFix.h>
#include <gpsd_client/GPSStatus.h>
#include <seabot_fusion/DepthPose.h>
#include <cmath>

using namespace std;
double east_set_point = 0.0;
double north_set_point = 0.0;
double east = 0.0;
double north = 0.0;
double yaw_imu = 0.0;
bool is_surface = false;
double speed = 0.0;
double yaw_gnss = 0.0;
double depth = 0.0;

bool mission_enable = false;

ros::WallTime last_received_set_point;
ros::WallTime last_received_pose;

void pose_callback(const seabot_fusion::GnssPose::ConstPtr& msg){
  east = msg->east;
  north = msg->north;
  last_received_pose = ros::WallTime::now();
}

void set_point_callback(const seabot_mission::Waypoint::ConstPtr& msg){
  east_set_point = msg->east;
  north_set_point = msg->north;
  mission_enable = msg->mission_enable;
  if(msg->depth == 0)
    is_surface = true;
  else
    is_surface = false;
  last_received_set_point = ros::WallTime::now();
}

void imu_callback(const geometry_msgs::Vector3::ConstPtr& msg){
  yaw_imu = msg->z; // Check unity ?
}

void gnss_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  speed = msg->speed;
  yaw_gnss = msg->track * M_PI/180.0; // Degree from north
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "waypoint_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 10.0);
  const double delta_valid_time = n_private.param<double>("delta_valid_time", 3.0);
  const double coeff_P = n_private.param<double>("coeff_P", 1.0);
  const double hysteresis_circle_in = n_private.param<double>("hysteresis_circle_in", 10.0);
  const double hysteresis_circle_out = n_private.param<double>("hysteresis_circle_out", 30.0);
  const double linear_speed = n_private.param<double>("linear_speed", 1.0);
  const double depth_limit_switch_off = n_private.param<double>("depth_limit_switch_off", 0.5);

  // Subscriber
  ros::Subscriber pose_sub = n.subscribe("/fusion/pose", 1, pose_callback);
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber mission_sub = n.subscribe("/mission/waypoint", 1, set_point_callback);
  ros::Subscriber imu_sub = n.subscribe("/driver/euler", 1, imu_callback);
  ros::Subscriber gnss_sub = n.subscribe("/driver/fix_extended", 1, gnss_callback);

  // Publisher
  ros::Publisher engine_pub = n.advertise<seabot_thruster_driver::Velocity>("/driver/thruster/cmd_engine", 1);

  seabot_thruster_driver::Velocity engine_msg;

  ros::Rate loop_rate(frequency);
  ros::WallTime t;

  bool hysteresis_inside = false;

  // Main regulation loop
  while (ros::ok()){
    ros::spinOnce();

    t = ros::WallTime::now();

    // ToDo : Add repulsive field if near coastline !!
    double yaw_set_point = atan2(north_set_point-north, east_set_point-east);
    double yaw_error = 2*atan(tan((yaw_imu-yaw_set_point)/2.0));

    double distance_error = sqrt(pow(north_set_point-north, 2)+pow(east_set_point-east, 2));
    bool enable_regulation = true;

    if(distance_error > hysteresis_circle_out){
      hysteresis_inside = false;
    }
    else if(distance_error > hysteresis_circle_in && hysteresis_inside == false){
      enable_regulation = true;
    }
    else{
      hysteresis_inside = true;
      enable_regulation = false;
    }

    if((last_received_set_point-t).toSec()<delta_valid_time && (last_received_pose-t).toSec()<delta_valid_time)
      enable_regulation = false;

    // Limitation of regulation
    if(!mission_enable || depth>depth_limit_switch_off)
      enable_regulation = false;

    if(is_surface && enable_regulation){
      engine_msg.linear = linear_speed;
      engine_msg.angular = coeff_P*yaw_error;
    }
    else{
      engine_msg.linear = 0.0;
      engine_msg.angular = 0.0;
    }
    engine_pub.publish(engine_msg);
  }
  loop_rate.sleep();

  return 0;
}


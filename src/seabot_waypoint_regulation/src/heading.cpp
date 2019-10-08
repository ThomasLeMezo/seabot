#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/GnssPose.h>
#include <seabot_mission/Waypoint.h>
#include <seabot_thruster_driver/Velocity.h>
#include <geometry_msgs/Vector3.h>
#include <gpsd_client/GPSFix.h>
#include <seabot_fusion/DepthPose.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <seabot_waypoint_regulation/HeadingDebug.h>

using namespace std;

double yaw_set_point = 0.0;
double yaw_imu = 0.0;
double angular_velocity = 0.0;
bool is_surface = false;
double speed = 0.0;
double yaw_gnss = 0.0;
double depth = 0.0;
bool depth_only = false;

bool mission_enable = false;

ros::WallTime last_received_set_point;
ros::WallTime last_received_pose;

void set_point_callback(const std_msgs::Float64::ConstPtr& msg){
  yaw_set_point = msg->data * M_PI/180.0;;
  last_received_set_point = ros::WallTime::now();
}

void euler_callback(const geometry_msgs::Vector3::ConstPtr& msg){
  yaw_imu = msg->z; // Check unity ?
  last_received_pose = ros::WallTime::now();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  angular_velocity = msg->angular_velocity.z;
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
  const double delta_valid_time = n_private.param<double>("delta_valid_time", 1.0);
  const double coeff_P = n_private.param<double>("coeff_P", 1.0);
  const double coeff_D = n_private.param<double>("coeff_D", 0.2);
  const double linear_speed = n_private.param<double>("linear_speed", 1.0);
  const double depth_limit_switch_off = n_private.param<double>("depth_limit_switch_off", 0.5);
  const double max_angular_velocity = n_private.param<double>("max_angular_velocity", 1.0);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber euler_sub = n.subscribe("/driver/euler", 1, euler_callback);
  ros::Subscriber imu_sub = n.subscribe("/driver/imu", 1, imu_callback);
  ros::Subscriber heading_sub = n.subscribe("heading_set_point", 1, set_point_callback);

  // Publisher
  ros::Publisher engine_pub = n.advertise<seabot_thruster_driver::Velocity>("/driver/thruster/cmd_engine", 1);
  ros::Publisher debug_pub = n.advertise<seabot_waypoint_regulation::HeadingDebug>("debug_heading", 1);

  seabot_thruster_driver::Velocity engine_msg;

  ros::Rate loop_rate(frequency);
  ros::WallTime t;
  seabot_waypoint_regulation::HeadingDebug msg_debug;

  ROS_INFO("[Heading] Start Ok");
  // Main regulation loop
  while (ros::ok()){
    ros::spinOnce();

    t = ros::WallTime::now();

    double yaw_error = 2*atan(tan((yaw_imu-yaw_set_point)/2.0));

    if((last_received_set_point-t).toSec()<delta_valid_time
       && depth<depth_limit_switch_off
       && (last_received_pose-t).toSec()<delta_valid_time){
      engine_msg.linear = linear_speed;
      engine_msg.angular = coeff_P*yaw_error + coeff_D*angular_velocity;

      msg_debug.error = yaw_error;
      msg_debug.p_var = coeff_P*yaw_error;
      msg_debug.d_var = coeff_D*angular_velocity;
      msg_debug.command = engine_msg.angular;
      msg_debug.set_point = yaw_set_point;

      // Limit max angular speed
      if(abs(engine_msg.angular)>max_angular_velocity)
        engine_msg.angular = copysign(max_angular_velocity, engine_msg.angular);

      msg_debug.command_limit = engine_msg.angular;
      debug_pub.publish(msg_debug);
    }
    else{
      engine_msg.linear = 0.0;
      engine_msg.angular = 0.0;
    }
    engine_pub.publish(engine_msg);

    loop_rate.sleep();
  }


  return 0;
}


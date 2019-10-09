#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/GnssPose.h>
#include <seabot_mission/Waypoint.h>
#include <seabot_thruster_driver/Velocity.h>
#include <geometry_msgs/Vector3.h>
#include <gpsd_client/GPSFix.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_waypoint_regulation/WaypointDebug.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

using namespace std;
double east_set_point = 0.0;
double north_set_point = 0.0;
double east = 0.0;
double north = 0.0;
double yaw_imu = 0.0;
bool is_surface = false;
double depth = 0.0;
bool depth_only = false;

double angular_velocity = 0.0;

double speed = 0.0;
double yaw_gnss = 0.0;

bool mission_enable = false;

ros::WallTime last_received_set_point;
ros::WallTime last_received_pose;
ros::WallTime last_received_heading;

void pose_callback(const seabot_fusion::GnssPose::ConstPtr& msg){
  east = msg->east;
  north = msg->north;
  last_received_pose = ros::WallTime::now();
}

void euler_callback(const geometry_msgs::Vector3::ConstPtr& msg){
  yaw_imu = msg->z; // Check unity ?
  last_received_heading = ros::WallTime::now();
}

void set_point_callback(const seabot_mission::Waypoint::ConstPtr& msg){
  east_set_point = msg->east;
  north_set_point = msg->north;
  mission_enable = msg->mission_enable;
  depth_only = msg->depth_only;
  if(msg->depth == 0)
    is_surface = true;
  else
    is_surface = false;
  last_received_set_point = ros::WallTime::now();
}

void gnss_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  speed = msg->speed;
  yaw_gnss = msg->track * M_PI/180.0; // Degree from north
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  angular_velocity = msg->angular_velocity.z;
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "waypoint_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 10.0);
  const double delta_valid_time = n_private.param<double>("delta_valid_time", 3.0);

  const double hysteresis_circle_in = n_private.param<double>("hysteresis_circle_in", 10.0);
  const double hysteresis_circle_out = n_private.param<double>("hysteresis_circle_out", 30.0);

  const double coeff_P = n_private.param<double>("coeff_P", 1.0);
  const double coeff_D = n_private.param<double>("coeff_D", 0.2);
  const double linear_speed = n_private.param<double>("linear_speed", 1.0);
  const double depth_limit_switch_off = n_private.param<double>("depth_limit_switch_off", 0.5);
  const double max_angular_velocity = n_private.param<double>("max_angular_velocity", 1.0);

  // Subscriber
  ros::Subscriber pose_sub = n.subscribe("/fusion/pose", 1, pose_callback);
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber mission_sub = n.subscribe("/mission/set_point", 1, set_point_callback);
  ros::Subscriber euler_sub = n.subscribe("/driver/euler", 1, euler_callback);
  ros::Subscriber gnss_sub = n.subscribe("/driver/fix_extended", 1, gnss_callback);
  ros::Subscriber imu_sub = n.subscribe("/driver/imu", 1, imu_callback);

  // Publisher
  ros::Publisher engine_pub = n.advertise<seabot_thruster_driver::Velocity>("/driver/thruster/cmd_engine", 1);
  ros::Publisher debug_pub = n.advertise<seabot_waypoint_regulation::WaypointDebug>("waypoint_debug", 1);

  seabot_thruster_driver::Velocity engine_msg;
  seabot_waypoint_regulation::WaypointDebug debug_msg;

  ros::Rate loop_rate(frequency);
  ros::WallTime t;
  ros::WallTime t_last_debug;

  bool hysteresis_inside = false;
  bool first_data_received = false;

  ROS_INFO("[Waypoint] Start Ok");
  // Main regulation loop
  while (ros::ok()){
    ros::spinOnce();

    t = ros::WallTime::now();

    // ToDo : Add repulsive field if near coastline (?)
    double yaw_set_point = -(atan2(north_set_point-north, east_set_point-east)-M_PI_2); // 0 deg yaw at in (x,y) != mag heading
    double yaw_error = -2*atan(tan((yaw_set_point-yaw_imu)/2.0)); // indirect frame => (-)

    double distance_error = sqrt(pow(north_set_point-north, 2)+pow(east_set_point-east, 2));

    // Hysteresis
    bool enable_regulation = true;
    if(distance_error > hysteresis_circle_out){
      hysteresis_inside = false;
      enable_regulation = true;
    }
    else if(distance_error > hysteresis_circle_in && hysteresis_inside == false){
      enable_regulation = true;
    }
    else{
      hysteresis_inside = true;
      enable_regulation = false;
    }

    // Valid Time
    bool valid_time = true;
    if(!((t-last_received_set_point).toSec()<delta_valid_time
       && (t-last_received_pose).toSec()<delta_valid_time
       && (t-last_received_heading).toSec()<delta_valid_time)){
      enable_regulation = false;
      valid_time = false;
    }
    else{
      first_data_received = true;
    }

    // Depth switch off & enable mission
    if(!mission_enable || depth>depth_limit_switch_off)
      enable_regulation = false;

    if(is_surface && enable_regulation && !depth_only){
      engine_msg.linear = linear_speed;

      if(abs(yaw_error)<M_PI/2.)
        engine_msg.angular = coeff_P*yaw_error + coeff_D*angular_velocity;
      else
        engine_msg.angular = -max_angular_velocity;
    }
    else{
      engine_msg.linear = 0.0;
      engine_msg.angular = 0.0;
    }
    // Limit max angular speed
    debug_msg.angular = engine_msg.angular;
    if(abs(engine_msg.angular)>max_angular_velocity)
      engine_msg.angular = copysign(max_angular_velocity, engine_msg.angular);
    debug_msg.angular_limit = engine_msg.angular;

    engine_pub.publish(engine_msg);

    if((t-t_last_debug).toSec()>1.0){
      if(!first_data_received){
        debug_msg.distance_error = 0.0;
        debug_msg.yaw_error = 0.0;
        debug_msg.yaw_set_point = 0.0;
      }
      else{
        debug_msg.distance_error = distance_error;
        debug_msg.yaw_error = yaw_error;
        debug_msg.yaw_set_point = yaw_set_point;
      }
      debug_msg.enable_regulation = enable_regulation;
      debug_msg.hysteresis_inside = hysteresis_inside;
      debug_msg.valid_time = valid_time;
      debug_pub.publish(debug_msg);
      t_last_debug = t;
    }

    loop_rate.sleep();
  }


  return 0;
}


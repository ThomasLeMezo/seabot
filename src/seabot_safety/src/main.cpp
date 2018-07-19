#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>
#include <seabot_depth_regulation/RegulationDebug.h>
#include <seabot_mission/Waypoint.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <pressure_bme280_driver/Bme280Data.h>

#include <cmath>

using namespace std;

double depth = 0;
double velocity = 0;
double piston_position = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;
double pressure_limit = 6.2;
int pressure_limit_count = 5*3;
int pressure_limit_count_reset = 5*3; // 5Hz * 3s of data
bool pressure_limit_reached = true;

bool flash_is_enable = false;

ros::ServiceClient service_zero_depth;
ros::ServiceClient service_flash_enable;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position = msg->position;
  piston_switch_in = msg->switch_in;
  piston_switch_out = msg->switch_out;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
  velocity= msg->velocity;
  t = ros::Time::now();
}

void pressure_callback(const pressure_bme280_driver::Bme280Data::ConstPtr& msg){
  if(msg->pressure > pressure_limit){
    if(pressure_limit_count!=0)
      pressure_limit_count--;
    else
      pressure_limit_reached=true;
  }
  else
    pressure_limit_count = pressure_limit_count_reset;
}

void call_zero_depth(){
  std_srvs::Trigger srv;
  if (!service_zero_depth.call(srv)){
    ROS_ERROR("[DepthRegulation] Failed to call zero depth");
  }
}

void call_flash_enable(const bool &val){
  if(val != flash_is_enable){
    std_srvs::SetBool srv;
    srv.request.data = val;
    if (!service_flash_enable.call(srv)){
      ROS_ERROR("[DepthRegulation] Failed to call flash enable");
    }
    else
      flash_is_enable = val;
  }
}

bool reset_limit_depth(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  pressure_limit_reached = false;
  res.success = true;
  return true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "safety_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);

  const double max_depth_reset_zero = n_private.param<double>("max_depth_reset_zero", 1.0);
  const double max_speed_reset_zero = n_private.param<double>("max_speed_reset_zero", 0.1);

  const double limit_depth_flash_enable = n_private.param<double>("limit_depth_flash_enable", 0.5);

  pressure_limit = n_private.param<double>("pressure_limit", 6.2);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);

  // Service
  ROS_INFO("[Safety] Wait for zero depth service from fusion");
  ros::service::waitForService("/fusion/zero_depth");
  service_zero_depth = n.serviceClient<std_srvs::Trigger>("/fusion/zero_depth");
  call_zero_depth();
  ROS_INFO("[Safety] Reset zero");

  ROS_INFO("[Safety] Wait for flash service from power_driver");
  ros::service::waitForService("/driver/power/flash_led");
  service_flash_enable = n.serviceClient<std_srvs::SetBool>("/driver/power/flash_led");

  // Server
  ros::ServiceServer server_reset_limit_depth = n.advertiseService("reset_limit_depth", reset_limit_depth);

  ros::Rate loop_rate(frequency);

  // Main regulation loop
  while (ros::ok()){
    ros::spinOnce();

    // Analyze zero depth
    if(piston_position == 0 && depth < max_depth_reset_zero && abs(velocity) < max_speed_reset_zero)
      call_zero_depth();

    if(depth < limit_depth_flash_enable)
      call_flash_enable(true);
    else
      call_flash_enable(false);
    loop_rate.sleep();
  }

  return 0;
}


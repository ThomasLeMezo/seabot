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
#include <std_srvs/Empty.h>
#include <pressure_bme280_driver/Bme280Data.h>
#include <seabot_power_driver/Battery.h>

#include <cmath>

using namespace std;

double depth = 0;
double velocity = 0;
double piston_position = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;

// Pressure limit
double pressure_limit = 6.2;
bool is_pressure_limit_reached = false;
ros::WallTime time_pressure_limit_reached;

// Flash
bool flash_is_enable = false;

// Batteries
double battery_limit = 10.0;
bool battery_limit_reached = false;

ros::ServiceClient service_zero_depth;
ros::ServiceClient service_flash_enable;
ros::ServiceClient service_emergency;
ros::ServiceClient service_sleep;

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
  if(msg->battery1 < battery_limit
     && msg->battery2 < battery_limit
     && msg->battery3 < battery_limit
     && msg->battery4 < battery_limit){
    battery_limit_reached = true;
  }
  else{
    battery_limit_reached = false;
  }
}

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position = msg->position;
  piston_switch_in = msg->switch_in;
  piston_switch_out = msg->switch_out;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
  velocity= msg->velocity;
}

void pressure_callback(const pressure_bme280_driver::Bme280Data::ConstPtr& msg){
  if(msg->pressure > pressure_limit){
    if(!is_pressure_limit_reached){
      time_pressure_limit_reached = ros::WallTime::now();
      is_pressure_limit_reached = true;
    }
  }
  else
    is_pressure_limit_reached = false;
}

void call_zero_depth(){
  std_srvs::Trigger srv;
  if (!service_zero_depth.call(srv)){
    ROS_ERROR("[Safety] Failed to call zero depth");
  }
}

void call_emergency_depth(const bool &val){
  std_srvs::SetBool srv;
  if (!service_emergency.call(srv)){
    ROS_ERROR("[Safety] Failed to call emergency");
  }
}

void call_sleep(){
  std_srvs::Empty srv;
  if (!service_sleep.call(srv)){
    ROS_ERROR("[Safety] Failed to call sleep");
  }
}

void call_flash_enable(const bool &val){
  if(val != flash_is_enable){
    std_srvs::SetBool srv;
    srv.request.data = val;
    if (!service_flash_enable.call(srv)){
      ROS_ERROR("[Safety] Failed to call flash enable");
    }
    else
      flash_is_enable = val;
  }
}

bool reset_limit_depth(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  call_emergency(false);
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
  const double time_before_pressure_emergency = n_private.param<double>("time_before_pressure_emergency", 3.0);

  battery_limit = n_private.param<double>("battery_limit", 10.0);

  pressure_limit = n_private.param<double>("pressure_limit", 6.2);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber batteries_sub = n.subscribe("/fusion/batteries", 1, batteries_callback);

  // Service
  ROS_INFO("[Safety] Wait for zero depth service from fusion");
  ros::service::waitForService("/fusion/zero_depth");
  service_zero_depth = n.serviceClient<std_srvs::Trigger>("/fusion/zero_depth");
  call_zero_depth();
  ROS_INFO("[Safety] Reset zero");

  ROS_INFO("[Safety] Wait for flash service from power_driver");
  ros::service::waitForService("/driver/power/flash_led");
  service_flash_enable = n.serviceClient<std_srvs::SetBool>("/driver/power/flash_led");

  ROS_INFO("[Safety] Wait for emergency service from depth regulation");
  ros::service::waitForService("/regulation/emergency");
  service_emergency = n.serviceClient<std_srvs::SetBool>("/regulation/emergency");

  ROS_INFO("[Safety] Wait for sleep service from power_driver");
  ros::service::waitForService("/driver/power/sleep_mode");
  service_sleep = n.serviceClient<std_srvs::Empty>("/driver/power/sleep_mode");

  // Server
  ros::ServiceServer server_reset_limit_depth = n.advertiseService("reset_limit_depth", reset_limit_depth);

  ros::Rate loop_rate(frequency);

  // Main regulation loop
  ROS_INFO("[Safety] Start safety loop");
  while (ros::ok()){
    ros::spinOnce();

    // Analyze zero depth
    if(piston_position == 0 && depth < max_depth_reset_zero && abs(velocity) < max_speed_reset_zero)
      call_zero_depth();

    // Flash at surface
    if(depth < limit_depth_flash_enable)
      call_flash_enable(true);
    else
      call_flash_enable(false);

    // Depth limit
    if(is_pressure_limit_reached && (ros::WallTime::now()-time_pressure_limit_reached).toSec()>time_before_pressure_emergency){
      ROS_WARN("[Safety] Limit depth detected");
      call_emergency_depth(true);
    }

    // Batteries
    if(battery_limit_reached){
      ROS_WARN("[Safety] Batteries limit detected");
      call_emergency_depth(true);
      // ToDo : shutdown ? => launch with iridium sleep ?
    }

    // Internal pressure
    // ToDo

    loop_rate.sleep();
  }

  return 0;
}


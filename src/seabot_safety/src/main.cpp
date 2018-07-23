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
#include <seabot_fusion/InternalPose.h>
#include <pressure_89bsd_driver/PressureBsdData.h>
#include <seabot_power_driver/Battery.h>
#include <seabot_safety/SafetyLog.h>

#include <cmath>

using namespace std;

// Depth
ros::WallTime time_depth;
double depth = 0;
double velocity = 0;

// Piston state
ros::WallTime time_piston_state;
double piston_position = 0;
int piston_state = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;

// Pressure limit
double pressure_limit = 6.2;
bool is_pressure_limit_reached = false;
ros::WallTime time_external_sensor;
ros::WallTime time_pressure_limit_reached;

// Flash
bool flash_is_enable = false;

// Batteries
ros::WallTime time_batteries;
double battery_limit = 10.0;
bool battery_limit_reached = false;

// Emergency mode
bool is_emergency_depth = false;

// Internal Pressure/Temp
ros::WallTime time_internal_sensor;
double internal_pressure = 0.0;
double internal_temperature = 0.0;

ros::ServiceClient service_zero_depth;
ros::ServiceClient service_flash_enable;
ros::ServiceClient service_emergency;
ros::ServiceClient service_sleep;

/// ****************** CALLBACK ****************** ///

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
  time_batteries = ros::WallTime::now();
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
  time_piston_state = ros::WallTime::now();
  piston_position = msg->position;
  piston_switch_in = msg->switch_in;
  piston_switch_out = msg->switch_out;
  piston_state = msg->state;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  time_depth = ros::WallTime::now();
  depth = msg->depth;
  velocity= msg->velocity;
}

void pressure_callback(const pressure_89bsd_driver::PressureBsdData::ConstPtr& msg){
  time_external_sensor = ros::WallTime::now();
  if(msg->pressure > pressure_limit){
    if(!is_pressure_limit_reached){
      time_pressure_limit_reached = ros::WallTime::now();
      is_pressure_limit_reached = true;
    }
  }
  else
    is_pressure_limit_reached = false;
}

void internal_sensor_callback(const seabot_fusion::InternalPose::ConstPtr& msg){
  time_internal_sensor = ros::WallTime::now();
  internal_pressure = msg->pressure*1e2;
  internal_temperature = msg->temperature + 273.15;
}

/// ****************** SERVICES ****************** ///

void call_zero_depth(){
  std_srvs::Trigger srv;
  if (!service_zero_depth.call(srv)){
    ROS_ERROR("[Safety] Failed to call zero depth");
  }
}

void call_emergency_depth(const bool &val){
  if(val != is_emergency_depth){
    std_srvs::SetBool srv;
    if (!service_emergency.call(srv)){
      ROS_ERROR("[Safety] Failed to call emergency");
    }
    else{
      is_emergency_depth = val;
    }
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
  call_emergency_depth(false);
  res.success = true;
  return true;
}

/// ****************** MAIN ****************** ///

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
  const bool enable_flash = n_private.param<bool>("enable_flash", true);

  const double d_batteries_ref = n_private.param<double>("time_delay_batteries_msg", 30.0);
  const double d_internal_sensor_ref = n_private.param<double>("time_delay_internal_sensor_msg", 2.0);
  const double d_external_sensor_ref = n_private.param<double>("time_delay_external_sensor_msg", 2.0);
  const double d_depth_ref = n_private.param<double>("time_delay_depth_msg", 2.0);
  const double d_piston_state_ref = n_private.param<double>("time_delay_piston_state_msg", 2.0);

  battery_limit = n_private.param<double>("battery_limit", 10.0);

  pressure_limit = n_private.param<double>("pressure_limit", 6.2);

  double tick_to_volume = (1.75e-3/48.0)*(pow((0.05/2.0),2))*M_PI;
  const double delta_volume_allowed = n_private.param<double>("delta_volume_allowed", 0.001);
  const double volume_ref = n_private.param<double>("volume_ref", 6.0);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber internal_sensor_sub = n.subscribe("/driver/internal_sensor", 1, internal_sensor_callback);
  ros::Subscriber batteries_sub = n.subscribe("/fusion/batteries", 1, batteries_callback);

  // Publisher
  ros::Publisher safety_pub = n.advertise<seabot_safety::SafetyLog>("safety", 1);

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

  // Compute the reference value for internal pressure and temperature
  double p_t_ratio_ref = 0.0;
  double piston_position_ref = 0.0;
  unsigned int cpt = 0;
  unsigned int nb_sample = 5;
  ros::WallDuration dt(1.0);
  while(cpt<nb_sample){
    ros::spinOnce();
    ros::WallDuration d_internal_sensor = ros::WallTime::now()-time_internal_sensor;
    ros::WallDuration d_piston_state = ros::WallTime::now()-time_piston_state;
    if(d_internal_sensor.toSec()<1.0 && d_piston_state.toSec()<1.0 && internal_temperature!=0.0){
      p_t_ratio_ref += internal_pressure/internal_temperature;
      piston_position_ref += piston_position;
      cpt++;
    }
    else
      ROS_WARN("[Safety] Not receiving internal sensor data or piston state data");
    dt.sleep();
  }
  p_t_ratio_ref /= (double)nb_sample;
  piston_position_ref /= (double) nb_sample;

  // Main regulation loop
  ROS_INFO("[Safety] Start safety loop");
  while (ros::ok()){
    ros::spinOnce();
    bool enable_emergency_depth = false;
    seabot_safety::SafetyLog safety_msg;

    // Sensor published data
    ros::WallTime t_ref = ros::WallTime::now();
    double d_batteries = (t_ref-time_batteries).toSec();
    double d_internal_sensor = (t_ref-time_internal_sensor).toSec();
    double d_external_sensor = (t_ref-time_external_sensor).toSec();
    double d_depth = (t_ref-time_depth).toSec();
    double d_piston_state = (t_ref-time_piston_state).toSec();

    if(d_batteries < d_batteries_ref
       || d_internal_sensor < d_internal_sensor_ref
       || d_external_sensor < d_external_sensor_ref
       || d_depth < d_depth_ref
       || d_piston_state < d_piston_state_ref){
      ROS_WARN("[Safety] No data published by sensors");
      enable_emergency_depth = true;
      safety_msg.published_frequency = true;
    }
    else
      safety_msg.published_frequency = false;

    // Analyze zero depth
    if(piston_position == 0 && depth < max_depth_reset_zero && abs(velocity) < max_speed_reset_zero)
      call_zero_depth();

    // Flash at surface
    if(enable_flash){
      if(depth < limit_depth_flash_enable)
        call_flash_enable(true);
      else
        call_flash_enable(false);
    }

    // Depth limit
    if(is_pressure_limit_reached && (ros::WallTime::now()-time_pressure_limit_reached).toSec()>time_before_pressure_emergency){
      ROS_WARN("[Safety] Limit depth detected");
      enable_emergency_depth = true;
      safety_msg.depth_limit = true;
    }
    else
      safety_msg.depth_limit = false;

    // Batteries
    if(battery_limit_reached){
      ROS_WARN("[Safety] Batteries limit detected");
      enable_emergency_depth = true;
      // ToDo : shutdown ? => launch with iridium sleep ?
      safety_msg.batteries_limit = true;
    }
    else
      safety_msg.batteries_limit = false;

    // Internal sensor
    safety_msg.depressurization = false;
    if(internal_temperature!=0.0){
      double p_t_ratio = internal_pressure/internal_temperature;
      double piston = piston_position-piston_position_ref;
      if(abs(piston)<620.0){
        if(abs(p_t_ratio-p_t_ratio_ref)>1.5){
          ROS_WARN("[Safety] Sealing issue detected");
          enable_emergency_depth = true;
          safety_msg.depressurization = true;
        }
      }
      else{
        if(p_t_ratio != p_t_ratio_ref){
          double volume = piston*tick_to_volume*p_t_ratio/(p_t_ratio-p_t_ratio_ref);
          if(abs(volume-volume_ref)>delta_volume_allowed){
            ROS_WARN("[Safety] Sealing issue detected");
            enable_emergency_depth = true;
            safety_msg.depressurization = true;
          }
        }
        else{
          ROS_WARN("[Safety] Sealing issue detected");
          enable_emergency_depth = true;
          safety_msg.depressurization = true;
        }
      }
    }
    else{
      ROS_WARN("[Safety] Internal sensor send wrong data or is disconnected");
      enable_emergency_depth = true;
    }

    if(enable_emergency_depth)
      call_emergency_depth(true);

    safety_pub.publish(safety_msg);
    loop_rate.sleep();
  }

  return 0;
}

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
#include <seabot_safety/SafetyDebug.h>

#include <cmath>

using namespace std;

// Depth
ros::WallTime time_depth;
double depth = 0;
double velocity = 0;

// Piston state
ros::WallTime time_piston_state;
double piston_position = -100;
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
double internal_humidity = 0.0;

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
  internal_humidity = msg->humidity;
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
    srv.request.data = val;
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
    if (service_flash_enable.call(srv))
      flash_is_enable = val;
    else
      ROS_ERROR("[Safety] Failed to call flash enable");
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
  const double delta_volume_allowed = n_private.param<double>("delta_volume_allowed", 0.002);
  const double delta_ref_allowed = n_private.param<double>("delta_ref_allowed", 3.0);
  const double volume_ref = n_private.param<double>("volume_ref", 0.006);
  const double pressure_internal_max = 1e2*n_private.param<double>("pressure_internal_max", 850.0); // in Pa
  const double transition_tick_law = n_private.param<double>("transition_tick_law", 1000.0);

  const double humidity_limit = n_private.param<double>("humidity_limit", 75.0);

  const bool enable_safety_battery = n_private.param<bool>("safety_battery", true);
  const bool enable_safety_pressure_limit = n_private.param<bool>("safety_pressure_limit", true);
  const bool enable_safety_depressure = n_private.param<bool>("safety_depressure", true);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber internal_sensor_sub = n.subscribe("/fusion/sensor_internal", 1, internal_sensor_callback);
  ros::Subscriber batteries_sub = n.subscribe("/fusion/battery", 1, batteries_callback);
  ros::Subscriber external_sensor_sub = n.subscribe("/driver/sensor_external", 1, pressure_callback);

  // Publisher
  ros::Publisher safety_pub = n.advertise<seabot_safety::SafetyLog>("safety", 1);
  ros::Publisher safety_debug_pub = n.advertise<seabot_safety::SafetyDebug>("debug", 1);

  // Service
  ROS_DEBUG("[Safety] Wait for zero depth service from fusion");
  ros::service::waitForService("/fusion/zero_depth");
  service_zero_depth = n.serviceClient<std_srvs::Trigger>("/fusion/zero_depth");
  call_zero_depth();
  ROS_DEBUG("[Safety] Reset zero");

  ROS_DEBUG("[Safety] Wait for flash service from power_driver");
  ros::service::waitForService("/driver/power/flash");
  service_flash_enable = n.serviceClient<std_srvs::SetBool>("/driver/power/flash");

  ROS_DEBUG("[Safety] Wait for emergency service from depth regulation");
  ros::service::waitForService("/regulation/emergency");
  service_emergency = n.serviceClient<std_srvs::SetBool>("/regulation/emergency");

  ROS_DEBUG("[Safety] Wait for sleep service from power_driver");
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

  /// ******************************************************************************
  /// **************************** Initialization **********************************
  /// ******************************************************************************

  // Call piston to move at 0
  call_emergency_depth(true);

  // Set Ref Pressure & Temperature
  ros::WallDuration dt(1.0);
  while(cpt<nb_sample){
    ros::spinOnce();
    ros::WallDuration d_internal_sensor = ros::WallTime::now()-time_internal_sensor;
    ros::WallDuration d_piston_state = ros::WallTime::now()-time_piston_state;
    if(d_internal_sensor.toSec()<1.0 &&
       d_piston_state.toSec()<1.0 &&
       internal_temperature!=0.0 &&
       piston_switch_out){
      p_t_ratio_ref += internal_pressure/internal_temperature;
      piston_position_ref += piston_position;
      cpt++;
    }
    else
      ROS_WARN("[Safety] Not receiving internal sensor data or piston state data yet (%f, %f)", d_internal_sensor.toSec(), d_piston_state.toSec());
    dt.sleep();
  }
  p_t_ratio_ref /= (double) cpt;
  piston_position_ref /= (double) cpt;

  // Clearance of regulation
  call_emergency_depth(false);

  /// ******************************************************************************
  /// **************************** Main regulation loop ****************************
  /// ******************************************************************************

  ROS_INFO("[Safety] Start Ok");
  while (ros::ok()){
    ros::spinOnce();
    bool enable_emergency_depth = false;
    seabot_safety::SafetyLog safety_msg;
    seabot_safety::SafetyDebug safety_debug_msg;

    ///*******************************************************
    ///**************** Sensor published data ****************

    ros::WallTime t_ref = ros::WallTime::now();
    double d_batteries = (t_ref-time_batteries).toSec();
    double d_internal_sensor = (t_ref-time_internal_sensor).toSec();
    double d_external_sensor = (t_ref-time_external_sensor).toSec();
    double d_depth = (t_ref-time_depth).toSec();
    double d_piston_state = (t_ref-time_piston_state).toSec();

    if(d_batteries > d_batteries_ref
       || d_internal_sensor > d_internal_sensor_ref
       || d_external_sensor > d_external_sensor_ref
       || d_depth > d_depth_ref
       || d_piston_state > d_piston_state_ref){
      ROS_WARN("[Safety] No data published by sensors (%f, %f, %f, %f, %f)", d_batteries, d_internal_sensor, d_external_sensor, d_depth, d_piston_state);
      enable_emergency_depth = true;
      safety_msg.published_frequency = true;
    }
    else
      safety_msg.published_frequency = false;

    ///*******************************************************
    ///**************** Analyze zero depth *******************
    if(piston_position == 0 && depth < max_depth_reset_zero && abs(velocity) < max_speed_reset_zero)
      call_zero_depth();

    ///*******************************************************
    ///**************** Flash at surface ********************
    if(depth < limit_depth_flash_enable){
      call_flash_enable(true);
      safety_debug_msg.flash = true;
    }
    else{
      call_flash_enable(false);
      safety_debug_msg.flash = false;
    }

    ///*******************************************************
    ///**************** Depth limit **************************
    if(is_pressure_limit_reached && (ros::WallTime::now()-time_pressure_limit_reached).toSec()>time_before_pressure_emergency){
      ROS_WARN("[Safety] Limit depth detected");
      enable_emergency_depth = true;
      safety_msg.depth_limit = true;
    }
    else
      safety_msg.depth_limit = false;

    ///*******************************************************
    ///**************** Batteries ****************************
    if(battery_limit_reached && enable_safety_battery){
      ROS_WARN("[Safety] Batteries limit detected");
      enable_emergency_depth = true;
      // ToDo : shutdown ? => launch with iridium sleep ?
      safety_msg.batteries_limit = true;
    }
    else
      safety_msg.batteries_limit = false;

    ///*******************************************************
    ///**************** Internal sensor **********************
    safety_msg.depressurization = false;
    int warning_number = 0;
    double p_t_ratio;
    if(internal_temperature!=0.0){
      p_t_ratio = internal_pressure/internal_temperature;
      double piston = piston_position-piston_position_ref;

      /// ********* PV=nRT law ********* ///
      // Case piston near zero
      if(abs(piston)<transition_tick_law){

        double delta = p_t_ratio-p_t_ratio_ref;
        if(abs(delta)>delta_ref_allowed){
          safety_msg.depressurization = true;
          warning_number = 1;
        }

        safety_debug_msg.ratio_p_t = p_t_ratio;
        safety_debug_msg.ratio_delta = delta;
      }
      // General case
      else{
        if(p_t_ratio != p_t_ratio_ref){
          double volume = piston*tick_to_volume*p_t_ratio/(p_t_ratio-p_t_ratio_ref);
          double volume_delta = volume-volume_ref;
          if(abs(volume_delta)>delta_volume_allowed){
            safety_msg.depressurization = true;
            warning_number = 2;
          }

          safety_debug_msg.volume = volume;
          safety_debug_msg.volume_delta = volume_delta;
        }
        else{
          safety_msg.depressurization = true;
          warning_number = 3;
        }
      }

      // Limit max
      if(internal_pressure>pressure_internal_max){
        safety_msg.depressurization = true;
        warning_number = 4;
      }
    }
    else{
      ROS_WARN("[Safety] Internal sensor send wrong data or is disconnected");
      safety_msg.depressurization = true;
      warning_number = 5;
    }

    if(internal_humidity>humidity_limit){
      safety_msg.depressurization = true;
      warning_number = 6;
    }

    if(piston_switch_in && piston_switch_out){
      safety_msg.depressurization = true;
      warning_number = 7;
    }

    if(safety_msg.depressurization == true){
      ROS_WARN("[Safety] Sealing issue detected %i (p=%f, t=%f, h=%f)", warning_number, internal_pressure, internal_temperature, internal_humidity);
      ROS_WARN("[Safety] Ratio (%f / %f)", p_t_ratio, p_t_ratio_ref);
    }

    ///*******************************************************
    ///**************** Summary ******************************
    if((enable_safety_battery && safety_msg.batteries_limit) || (enable_safety_depressure && safety_msg.depressurization) || (enable_safety_pressure_limit && safety_msg.depth_limit) || safety_msg.published_frequency)
      enable_emergency_depth = true;

    if(enable_emergency_depth)
      call_emergency_depth(true);

    safety_pub.publish(safety_msg);
    safety_debug_pub.publish(safety_debug_msg);
    loop_rate.sleep();
  }

  return 0;
}

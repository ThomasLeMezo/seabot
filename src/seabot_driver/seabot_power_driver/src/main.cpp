#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "power.h"
#include "seabot_power_driver/Battery.h"

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "seabot_power_driver/FlashSpeed.h"
#include "seabot_power_driver/SleepModeParam.h"
#include "seabot_power_driver/FlashCounter.h"

#define THRESHOLD_LIPO_3S_MAX 10.4
#define THRESHOLD_LIPO_3S_FIRST 9.9

using namespace std;

Power p;
double flash_sec_left = 0.0;
bool flash_is_enable = false;
bool flash_counter_enable = false;
bool flash_manual_enable = false;

bool flash_enable(std_srvs::SetBool::Request  &req,
                  std_srvs::SetBool::Response &res){
  p.set_flash_enable(req.data);
  flash_manual_enable = req.data;
  flash_is_enable = req.data;
  res.success = true;
  return true;
}

bool flash_counter(seabot_power_driver::FlashCounter::Request  &req,
                   seabot_power_driver::FlashCounter::Response &res){
  flash_sec_left = req.counter;
  flash_counter_enable = true;
  return true;
}

bool flash_speed(seabot_power_driver::FlashSpeed::Request  &req,
                 seabot_power_driver::FlashSpeed::Response &res){
  p.set_flash_delay(req.period);
  return true;
}

bool sleep_mode(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &res){
  p.set_sleep_mode();
  return true;
}

bool sleep_mode_param(seabot_power_driver::SleepModeParam::Request  &req,
                      seabot_power_driver::SleepModeParam::Response &res){
  p.set_sleep_mode_countdown(req.hours, req.min, req.sec, req.sec_to_sleep);
  return true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "power_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 0.2);

  // Sensor initialization
  p.i2c_open();

  // Publisher
  ros::Publisher battery_pub = n.advertise<seabot_power_driver::Battery>("battery", 1);
  seabot_power_driver::Battery battery_msg;

  // Service (ON/OFF)
  ros::ServiceServer service_flash = n.advertiseService("flash", flash_enable);
  ros::ServiceServer service_flash_speed = n.advertiseService("flash_speed", flash_speed);
  ros::ServiceServer service_flash_counter = n.advertiseService("flash_counter", flash_counter);
  ros::ServiceServer service_sleep_mode = n.advertiseService("sleep_mode", sleep_mode);
  ros::ServiceServer service_sleep_mode_param = n.advertiseService("sleep_mode_param", sleep_mode_param);

  if(p.get_version()!=0x02){
    ROS_WARN("[POWER] Wrong PIC code version");
  }

  p.set_sleep_mode_countdown(1, 0, 0, 250);
  p.set_flash_delay(20);
  ROS_DEBUG("[POWER] Set sleep mode to 1 hour");

  // ToDo : Add serices to turn off/on Iridium, GPS
  // Add turn off screen by default in linux launch script

  ROS_INFO("[POWER] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();
    p.get_batteries();

    battery_msg.battery1 = p.get_level_battery(0);
    battery_msg.battery2 = p.get_level_battery(1);
    battery_msg.battery3 = p.get_level_battery(2);
    battery_msg.battery4 = p.get_level_battery(3);
    battery_pub.publish(battery_msg);

    if(flash_sec_left>0){
      if(!flash_is_enable){
        p.set_flash_enable(true);
        flash_is_enable = true;
      }
      flash_sec_left -= 1./frequency;
    }
    else{
      if(flash_counter_enable){
        flash_counter_enable = false;

        if(!flash_manual_enable){
          flash_is_enable = false;
          p.set_flash_enable(false);
        }
      }
    }

    loop_rate.sleep();
  }

  return 0;
}

#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "power.h"
#include "power_driver/Battery.h"

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "power_driver/FlashSpeed.h"
#include "power_driver/SleepModeParam.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define THRESHOLD_LIPO_3S_MAX 10.4
#define THRESHOLD_LIPO_3S_FIRST 9.9

using namespace std;

Power p;

bool flash_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  p.set_flash_led(req.data);
  res.success = true;
  return true;
}

bool flash_period(power_driver::FlashSpeed::Request  &req,
         power_driver::FlashSpeed::Response &res){
  p.set_flash_led_delay(req.period);
  return true;
}

bool sleep_mode(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res){
  p.set_sleep_mode();
  return true;
}

bool sleep_mode_param(power_driver::SleepModeParam::Request  &req,
         power_driver::SleepModeParam::Response &res){
  p.set_sleep_mode_countdown(req.hours, req.min, req.sec, req.sec_to_sleep);
  return true;
}

void power_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat, size_t id){
  float val = p.get_level_battery(id);

  if(val < 0){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "I2C Failure %f", val);
  }
  else if(val<THRESHOLD_LIPO_3S_MAX){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Very Low Battery Level %f", val);
  }
  else if(val<THRESHOLD_LIPO_3S_FIRST){
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Low Battery Level %f", val);
  }
  else{
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Battery Level OK %f", val);
  }

  // add and addf are used to append key-value pairs.
  stat.add("Battery Level", val);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "power_node");
  ros::NodeHandle n;

  // Diagnostics
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  double min_freq = 0.15;
  double max_freq = 5;
  diagnostic_updater::HeaderlessTopicDiagnostic battery_pub_freq("battery", updater,
      diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
  diagnostic_updater::FunctionDiagnosticTask b0("Battery 1", boost::bind(&power_diagnostic, _1, 0));
  diagnostic_updater::FunctionDiagnosticTask b1("Battery 2", boost::bind(&power_diagnostic, _1, 1));
  diagnostic_updater::FunctionDiagnosticTask b2("Battery 3", boost::bind(&power_diagnostic, _1, 2));
  diagnostic_updater::FunctionDiagnosticTask b3("Battery 4", boost::bind(&power_diagnostic, _1, 3));
  diagnostic_updater::CompositeDiagnosticTask b_global("Batteries check");
  b_global.addTask(&b0);
  b_global.addTask(&b1);
  b_global.addTask(&b2);
  b_global.addTask(&b3);
  updater.add(b_global);

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 0.2);

  // Sensor initialization
  p.i2c_open();

  // Publisher
  ros::Publisher battery_pub = n.advertise<power_driver::Battery>("battery", 1);
  power_driver::Battery battery_msg;

  // Service (ON/OFF)
  ros::ServiceServer service_flash = n.advertiseService("flash_led", flash_enable);
  ros::ServiceServer service_flash_period = n.advertiseService("flash_led_period", flash_period);
  ros::ServiceServer service_sleep_mode = n.advertiseService("sleep_mode", sleep_mode);
  ros::ServiceServer service_sleep_mode_param = n.advertiseService("sleep_mode_param", sleep_mode_param);

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    p.get_batteries();

    battery_msg.battery1 = p.get_level_battery(0);
    battery_msg.battery2 = p.get_level_battery(1);
    battery_msg.battery3 = p.get_level_battery(2);
    battery_msg.battery4 = p.get_level_battery(3);
    battery_msg.header.stamp = ros::Time::now();
    battery_pub.publish(battery_msg);

    battery_pub_freq.tick();
    updater.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

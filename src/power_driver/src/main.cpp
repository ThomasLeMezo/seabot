#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "power.h"
#include "power_driver/Battery.h"

#include <std_srvs/SetBool.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define THRESHOLD_LIPO_3S_MAX 12.0
#define THRESHOLD_LIPO_3S_FIRST 12.2

using namespace std;

Power p;

bool led_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  p.enable_led(req.data);
  res.success = true;
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

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "thrusters");
  ros::NodeHandle n;

  // Diagnostics
  diagnostic_updater::Updater updater;
  updater.setHardwareID("none");
  double min_freq = 0.5;
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
  n.advertiseService("led_enable", led_enable);

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    p.measure_battery();

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

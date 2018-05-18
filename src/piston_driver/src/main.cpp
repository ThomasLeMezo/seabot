#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt16.h>

#include "piston.h"
#include "piston_driver/PosePiston.h"
#include "piston_driver/PistonSpeed.h"

using namespace std;

Piston p;
bool state_start = false;
uint16_t cmd_position_piston = 0;
uint16_t new_cmd_position_piston = 0;

bool piston_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  if(req.data == true)
    p.set_piston_enable(true);
  else
    p.set_piston_enable(false);

  res.success = true;
  return true;
}

bool piston_start(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  if(req.data == true){
    p.set_piston_start();
    state_start = true;
  }
  else{
    p.set_piston_stop();
    state_start = false;
  }

  res.success = true;
  return true;
}

bool piston_speed(piston_driver::PistonSpeed::Request  &req,
         piston_driver::PistonSpeed::Response &res){
    p.set_piston_speed(req.speed);
  return true;
}

void position_callback(const std_msgs::UInt16::ConstPtr& msg){
  new_cmd_position_piston = msg->data;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "piston");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 5.0);

  // Service (ON/OFF)
  ros::ServiceServer service_enable = n.advertiseService("piston_enable", piston_enable);
  ros::ServiceServer service_start = n.advertiseService("piston_start", piston_start);
  ros::ServiceServer service_speed = n.advertiseService("piston_speed", piston_speed);

  // Subscriber
  ros::Subscriber position_sub = n.subscribe("cmd_position_piston", 1, position_callback);

  // Publisher
  ros::Publisher position_pub = n.advertise<piston_driver::PosePiston>("position_piston", 1);
  piston_driver::PosePiston position_msg;

  // Sensor initialization
  p.i2c_open();

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(state_start){
      position_msg.position = p.get_piston_position();
      position_msg.header.stamp = ros::Time::now();
      position_pub.publish(position_msg);

      if(cmd_position_piston != new_cmd_position_piston){
        cmd_position_piston = new_cmd_position_piston;
        p.set_piston_position(cmd_position_piston);
      }
    }

    loop_rate.sleep();
  }

  return 0;
}

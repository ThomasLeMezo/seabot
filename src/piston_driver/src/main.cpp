#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt16.h>

#include "piston.h"
#include "piston_driver/PosePiston.h"

using namespace std;

Piston p;
bool enable_motor = false;
uint16_t cmd_position_piston = 0;
uint16_t new_cmd_position_piston = 0;

bool piston_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  if(req.data = true){
    p.set_piston_start();
    enable_motor = true;
  }
  else{
    p.set_piston_stop();
    enable_motor = false;
  }

  res.success = true;
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
  ros::ServiceServer service = n.advertiseService("piston_enable", piston_enable);

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

    if(cmd_position_piston != new_cmd_position_piston){
      cmd_position_piston = new_cmd_position_piston;
      p.set_piston_position(cmd_position_piston);
    }

    if(enable_motor){
      position_msg.position = p.get_piston_position();
      position_msg.header.stamp = ros::Time::now();
      position_pub.publish(position_msg);
    }

    loop_rate.sleep();
  }

  return 0;
}

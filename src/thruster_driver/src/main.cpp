#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "thruster.h"
#include "thruster_driver/Velocity.h"
#include "thruster_driver/Engine.h"

using namespace std;
float linear_velocity = 0.0;
float angular_velocity = 0.0;
bool state_idle = false;
float K = 100;

void velocity_callback(const thruster_driver::Velocity::ConstPtr& msg){
  linear_velocity = msg->linear;
  angular_velocity = msg->angular;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "thrusters");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 20.0);

  // Subscriber
  ros::Subscriber velocity_sub = n.subscribe("cmd_engine", 1, velocity_callback);

  // Publisher
  ros::Publisher cmd_pub = n.advertise<thruster_driver::Engine>("engine", 1);
  thruster_driver::Engine cmd_msg;

  // Sensor initialization
  Thruster t;
  t.i2c_open();

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(!state_idle){

      float u_left = linear_velocity + angular_velocity;
      float u_right = linear_velocity - angular_velocity;
      float n = max((float)1.0, max(u_left, u_right));
      u_left /=n;
      u_right /=n;

      unsigned short int cmd_left = u_left*K;
      unsigned short int cmd_right = u_right*K;

      t.write_cmd(cmd_left, cmd_right);

      // Publish cmd send for loggin
      cmd_msg.left = cmd_left;
      cmd_msg.right = cmd_right;
      cmd_msg.header.stamp = ros::Time::now();
      cmd_pub.publish(cmd_msg);
    }

    loop_rate.sleep();
  }

  return 0;
}

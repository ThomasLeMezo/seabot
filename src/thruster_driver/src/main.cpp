#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "thruster.h"
#include "thruster_driver/Engine.h"
#include "thruster_driver/Velocity.h"

#include "std_srvs/SetBool.h"

using namespace std;
float linear_velocity = 0.0;
float angular_velocity = 0.0;
bool state_idle = true;

unsigned short int max_cmd = 190;
unsigned short int min_cmd = 110;

Thruster t;

void velocity_callback(const thruster_driver::Velocity::ConstPtr& msg){
  linear_velocity = msg->linear;
  angular_velocity = msg->angular;
}

bool engine_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  state_idle = req.data;
  res.success = true;
  t.write_cmd(MOTOR_PWM_STOP, MOTOR_PWM_STOP);

  return true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "thruster_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 10.0);
  float coeff_cmd_to_pwm = n_private.param<float>("coeff_cmd_to_pwm", 15);

  // Subscriber
  ros::Subscriber velocity_sub = n.subscribe("cmd_engine", 1, velocity_callback);

  // Publisher
  ros::Publisher cmd_pub = n.advertise<thruster_driver::Engine>("engine", 1);
  thruster_driver::Engine cmd_msg;

  // Service (ON/OFF)
  ros::ServiceServer service = n.advertiseService("engine_enable", engine_enable);

  // Sensor initialization
  t.i2c_open();

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(!state_idle){
      float u_left = linear_velocity + angular_velocity;
      float u_right = linear_velocity - angular_velocity;

      unsigned short int cmd_left = u_left*coeff_cmd_to_pwm + MOTOR_PWM_STOP;
      unsigned short int cmd_right = u_right*coeff_cmd_to_pwm + MOTOR_PWM_STOP;

      if(cmd_left>max_cmd)
        cmd_left = max_cmd;
      if(cmd_left<min_cmd)
        cmd_left = min_cmd;
      if(cmd_right>max_cmd)
        cmd_right = max_cmd;
      if(cmd_right<min_cmd)
        cmd_right = min_cmd;

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

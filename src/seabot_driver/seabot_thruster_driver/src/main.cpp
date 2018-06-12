#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "thruster.h"
#include "seabot_thruster_driver/Engine.h"
#include "seabot_thruster_driver/Velocity.h"

#include "std_srvs/SetBool.h"

using namespace std;
float linear_velocity = 0.0;
float angular_velocity = 0.0;
bool state_enable = true;

Thruster t;

void velocity_callback(const seabot_thruster_driver::Velocity::ConstPtr& msg){
  linear_velocity = msg->linear;
  angular_velocity = msg->angular;
}

bool engine_enable(std_srvs::SetBool::Request  &req,
         std_srvs::SetBool::Response &res){
  state_enable = req.data;
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
  float coeff_cmd_to_pwm = n_private.param<float>("coeff_cmd_to_pwm", 15.0);

  // Subscriber
  ros::Subscriber velocity_sub = n.subscribe("cmd_engine", 1, velocity_callback);

  // Publisher
  ros::Publisher cmd_pub = n.advertise<seabot_thruster_driver::Engine>("engine", 1);
  seabot_thruster_driver::Engine cmd_msg;

  // Service (ON/OFF)
  ros::ServiceServer service = n.advertiseService("engine_enable", engine_enable);

  // Sensor initialization
  t.i2c_open();

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(state_enable){
      float u_left = linear_velocity + angular_velocity;
      float u_right = linear_velocity - angular_velocity;

      uint8_t cmd_left = round(u_left*coeff_cmd_to_pwm + MOTOR_PWM_STOP);
      uint8_t cmd_right = round(u_right*coeff_cmd_to_pwm + MOTOR_PWM_STOP);

      if(cmd_left>MAX_PWM)
        cmd_left = MAX_PWM;
      if(cmd_left<MIN_PWM)
        cmd_left = MIN_PWM;

      if(cmd_right>MAX_PWM)
        cmd_right = MAX_PWM;
      if(cmd_right<MIN_PWM)
        cmd_right = MIN_PWM;

      t.write_cmd(cmd_left, cmd_right);

      // Publish cmd send for loggin
      cmd_msg.left = (float)cmd_left;
      cmd_msg.right = cmd_right;
      cmd_pub.publish(cmd_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

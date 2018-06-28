#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include "piston.h"
#include "seabot_piston_driver/PistonSpeed.h"
#include "seabot_piston_driver/PistonState.h"
#include "seabot_piston_driver/PistonPosition.h"
#include "seabot_fusion/DepthPose.h"

using namespace std;

Piston p;
bool state_start = true;
uint16_t cmd_position_piston = 0;
uint16_t new_cmd_position_piston = 0;
bool state_emergency = false;
double depth = 0;
bool adaptative_speed = false;
int speed_in_last = 50;
int speed_out_last = 50;

bool piston_enable(std_srvs::SetBool::Request  &req,
                   std_srvs::SetBool::Response &res){
  if(req.data == true)
    p.set_piston_enable(true);
  else
    p.set_piston_enable(false);

  res.success = true;
  return true;
}

bool piston_reset(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &res){
  p.set_piston_reset();
  return true;
}

bool piston_speed(seabot_piston_driver::PistonSpeed::Request  &req,
                  seabot_piston_driver::PistonSpeed::Response &res){
  p.set_piston_speed(req.speed_in, req.speed_out);
  return true;
}

void position_callback(const seabot_piston_driver::PistonPosition::ConstPtr& msg){
  if(state_start && state_emergency==false)
    p.set_piston_position(msg->position);
}

bool piston_emergency(std_srvs::SetBool::Request  &req,
                      std_srvs::SetBool::Response &res){
  if(req.data == true){
    new_cmd_position_piston = 0;
    p.set_piston_position(0);
    state_emergency = true;
  }
  else{
    state_emergency = false;
  }

  res.success = true;
  return true;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
    depth = msg->depth;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "piston");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 5.0);
  adaptative_speed = n_private.param<bool>("adaptative_speed", false);
  double adaptative_coeff_slope_in = n_private.param<double>("adaptative_coeff_slope_in", 1.0/20.0);
  double adaptative_coeff_offset_in = n_private.param<double>("adaptative_coeff_offset_in", 50.0);
  double adaptative_coeff_slope_out = n_private.param<double>("adaptative_coeff_slope_out", 1.0/20.0);
  double adaptative_coeff_offset_out = n_private.param<double>("adaptative_coeff_offset_out", 50.0);

  const double max_speed = 126.0;
  const double min_speed = 30.0;
  const double nb_step = 5.0;

  // Service (ON/OFF)
  ros::ServiceServer service_speed = n.advertiseService("speed", piston_speed);

  ros::ServiceServer service_reset = n.advertiseService("reset", piston_reset);
  ros::ServiceServer service_enable = n.advertiseService("enable", piston_enable);
  ros::ServiceServer service_emergency = n.advertiseService("emergency", piston_emergency);

  // Publisher
  ros::Publisher state_pub = n.advertise<seabot_piston_driver::PistonState>("state", 1);
  seabot_piston_driver::PistonState state_msg;

  // Subscriber
  ros::Subscriber piston_position_sub = n.subscribe("position", 1, position_callback);
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);

  // Sensor initialization
  p.i2c_open();
  sleep(1); // 1s sleep (wait until i2c open)

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    // Piston state
    p.get_piston_all_data();
    state_msg.position = p.m_position;
    state_msg.switch_out = p.m_switch_out;
    state_msg.switch_in = p.m_switch_in;
    state_msg.state = p.m_state;
    state_msg.motor_on = p.m_motor_on;
    state_msg.enable_on = p.m_enable_on;
    state_msg.position_set_point = p.m_position_set_point;
    state_msg.motor_speed = p.m_motor_speed;
    state_pub.publish(state_msg);

    // Analyze depth
    if(adaptative_speed){
        int speed_in = (max_speed/nb_step)*round(max(min(depth*adaptative_coeff_slope_in+adaptative_coeff_offset_in, max_speed), min_speed)*(nb_step/max_speed));
        int speed_out = (max_speed/nb_step)*round(max(min(depth*adaptative_coeff_slope_out+adaptative_coeff_offset_out, max_speed), min_speed)*(nb_step/max_speed));

        // Hysteresis
        if(abs(speed_in-speed_in_last)>(max_speed/nb_step)/2.0
                || abs(speed_out-speed_out_last)>(max_speed/nb_step)/2.0){
            p.set_piston_speed((uint16_t) speed_in, (uint16_t) speed_out);
            speed_in_last = speed_in;
            speed_out_last = speed_out;
        }
    }

    loop_rate.sleep();
  }

  return 0;
}

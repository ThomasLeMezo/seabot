#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "thruster.h"
#include "seabot_thruster_driver/Engine.h"
#include "seabot_thruster_driver/Velocity.h"
#include "geometry_msgs/Twist.h"

#include "std_srvs/SetBool.h"

using namespace std;
float linear_velocity = 0.0;
float angular_velocity = 0.0;
bool state_enable = true;

float coeff_cmd_to_pwm = 9.0;

Thruster t;
ros::Time time_last_cmd;
bool stop_sent = false;
bool send_cmd = true;

float manual_linear_velocity = 0.0;
float manual_angular_velocity = 0.0;
ros::Time manual_time_last_cmd;

void velocity_callback(const seabot_thruster_driver::Velocity::ConstPtr& msg){
  linear_velocity = msg->linear;
  angular_velocity = msg->angular;
  time_last_cmd = ros::Time::now();
}

void manual_velocity_callback(const geometry_msgs::Twist::ConstPtr& msg){
  manual_linear_velocity = msg->linear.x;
  manual_angular_velocity = msg->angular.z;
  manual_time_last_cmd = ros::Time::now();
}

bool engine_enable(std_srvs::SetBool::Request  &req,
                   std_srvs::SetBool::Response &res){
  state_enable = req.data;
  res.success = true;
  t.write_cmd(MOTOR_PWM_STOP, MOTOR_PWM_STOP);
  stop_sent = true;

  return true;
}

uint8_t convert_u(const double &u){
  uint8_t cmd = round(u*coeff_cmd_to_pwm + MOTOR_PWM_STOP);

  if(cmd>MAX_PWM)
    cmd = MAX_PWM;
  if(cmd<MIN_PWM)
    cmd = MIN_PWM;

  return cmd;
}

inline uint8_t invert_cmd(const uint8_t &cmd){
  int16_t tmp = cmd - MOTOR_PWM_STOP;
  tmp = -tmp;
  return (uint8_t)(MOTOR_PWM_STOP + tmp);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "thruster_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 10.0);
  coeff_cmd_to_pwm = n_private.param<float>("coeff_cmd_to_pwm", 9.0);
  const double delay_stop = n_private.param<float>("delay_stop", 0.5);
  const bool backward_engine = n_private.param<bool>("backward_engine", false);

  const bool invert_left = n_private.param<bool>("invert_left", false);
  const bool invert_right = n_private.param<bool>("invert_right", false);

  const double max_angular_velocity = n_private.param<float>("max_angular_velocity", 1.0);
  const double max_linear_velocity = n_private.param<float>("max_linear_velocity", 1.0);

  const double max_engine_change = n_private.param<float>("max_engine_change", 1.0);


  // Subscriber
  ros::Subscriber velocity_sub = n.subscribe("cmd_engine", 1, velocity_callback);
  ros::Subscriber manual_velocity_sub = n.subscribe("/cmd_vel", 1, manual_velocity_callback);

  // Publisher
  ros::Publisher cmd_pub = n.advertise<seabot_thruster_driver::Engine>("engine", 1);
  seabot_thruster_driver::Engine cmd_msg;

  // Service (ON/OFF)
  ros::ServiceServer service = n.advertiseService("engine_enable", engine_enable);

  // Sensor initialization
  t.i2c_open();

  if(t.get_version()!=0x02){
    ROS_WARN("[Thruster] Wrong PIC code version");
  }

  time_last_cmd = ros::Time::now();
  manual_time_last_cmd = ros::Time::now();
  ros::Duration(delay_stop*1.1).sleep();

  uint8_t cmd_left_last = MOTOR_PWM_STOP, cmd_right_last = MOTOR_PWM_STOP;
  double dt;
  ros::Time last_time = ros::Time::now();

  ROS_INFO("[Thruster] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    dt = (ros::Time::now()-last_time).toSec();
    last_time = ros::Time::now();
    int max_engine_change_dt = max((int)round(max_engine_change*dt), 1);

    if(state_enable){
      uint8_t cmd_left, cmd_right;

      if((ros::Time::now() - manual_time_last_cmd).toSec() < delay_stop){
        double left = manual_linear_velocity + manual_angular_velocity;
        double right = manual_linear_velocity - manual_angular_velocity;
        if(backward_engine==false){
          left = max(0., left);
          right = max(0., right);
        }
        cmd_left = convert_u(left);
        cmd_right = convert_u(right);
      }
      else if((ros::Time::now() - time_last_cmd).toSec() < delay_stop){
        if(abs(linear_velocity)>max_linear_velocity)
          linear_velocity = std::copysign(max_linear_velocity, linear_velocity);
        if(abs(angular_velocity)>max_angular_velocity)
          angular_velocity = std::copysign(max_angular_velocity, angular_velocity);

        double left = linear_velocity + angular_velocity;
        double right = linear_velocity - angular_velocity;
        if(backward_engine==false){
          left = max(0., left);
          right = max(0., right);
        }
        cmd_left = convert_u(left);
        cmd_right = convert_u(right);
      }
      else{
        cmd_right = MOTOR_PWM_STOP;
        cmd_left = MOTOR_PWM_STOP;
      }

      if(cmd_right != MOTOR_PWM_STOP || cmd_left != MOTOR_PWM_STOP){
        send_cmd = true;
        stop_sent = false;
      }
      else{
        if(!stop_sent){
          send_cmd = true;
          stop_sent = true;
        }
        else
          send_cmd = false;
      }

      if(send_cmd){
        if(invert_left)
          cmd_left = invert_cmd(cmd_left);
        if(invert_right)
          cmd_right = invert_cmd(cmd_right);

        int cmd_change_left = cmd_left-cmd_left_last;
        uint8_t cmd_left_tmp = cmd_left_last + copysign(min(abs(cmd_change_left), max_engine_change_dt), cmd_change_left);
        cmd_left_last = cmd_left_tmp;

        int cmd_change_right = cmd_right-cmd_right_last;
        uint8_t cmd_right_tmp = cmd_right_last + copysign(min(abs(cmd_change_right), max_engine_change_dt), cmd_change_right);
        cmd_right_last = cmd_right_tmp;

        t.write_cmd(cmd_left_tmp, cmd_right);

        // Publish cmd send for loggin
        cmd_msg.left = (float)cmd_left;
        cmd_msg.right = cmd_right;
        cmd_pub.publish(cmd_msg);
      }
    }

    loop_rate.sleep();
  }

  return 0;
}

#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include "piston.h"
#include "seabot_piston_driver/PistonSpeed.h"
#include "seabot_piston_driver/PistonState.h"
#include "seabot_piston_driver/PistonPosition.h"
#include "seabot_piston_driver/PistonSpeedDebug.h"
#include "seabot_piston_driver/PistonVelocity.h"
#include "seabot_piston_driver/PistonErrorInterval.h"
#include "seabot_piston_driver/PistonDistanceTravelled.h"
#include "seabot_fusion/DepthPose.h"

using namespace std;

Piston p;
uint16_t cmd_position_piston = 0;
uint16_t new_cmd_position_piston = 0;
bool state_emergency = false;
double depth = 0;
int speed_in_last = 50;
int speed_out_last = 50;
__u16 piston_set_point = 0;
size_t cpt_error_zero = 0;

bool fast_move = false;

double distance_travelled = 0;
double last_piston_position_hf = 0.0;

ros::WallTime time_velocity_issue_detected;
bool velocity_issue_detected=false;

bool is_sealing_issue = false;
ros::Time time_sealing_issue;

size_t speed_index = 0;

int speed_in_min = 15;
int speed_out_min = 15;

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

bool piston_error_interval(seabot_piston_driver::PistonErrorInterval::Request  &req,
                           seabot_piston_driver::PistonErrorInterval::Response &res){
  p.set_error_interval(req.error_interval);
  return true;
}

void position_callback(const seabot_piston_driver::PistonPosition::ConstPtr& msg){
  piston_set_point = msg->position;
}

bool piston_emergency(std_srvs::SetBool::Request  &req,
                      std_srvs::SetBool::Response &res){
  if(req.data == true){
    new_cmd_position_piston = 0;
    p.set_piston_speed(45, 45);
    p.set_piston_position(0);
    speed_in_last = 45;
    speed_out_last = 45;
    state_emergency = true;
    ROS_INFO("[Piston_driver] Emergency ON");
  }
  else{
    state_emergency = false;
    ROS_INFO("[Piston_driver] Emergency OFF");
    p.set_piston_speed(speed_in_min, speed_out_min); // ?
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
  const double frequency = n_private.param<double>("frequency", 5.0);

  const double speed_in_slope = n_private.param<double>("speed_in_slope", 0.2);
  const double speed_out_slope = n_private.param<double>("speed_out_slope", 0.2);

  const double speed_depth_layer = n_private.param<double>("speed_depth_layer", 5.0);
  const double depth_max = n_private.param<double>("depth_max", 50.0);
  const size_t speed_max = (size_t) n_private.param<int>("speed_max", 50);
  const size_t speed_step_increase = (size_t) n_private.param<int>("speed_step_increase", 5);

  speed_in_min = n_private.param<int>("speed_in_min", 15);
  speed_out_min = n_private.param<int>("speed_out_min", 15);

  const int speed_reset = n_private.param<int>("speed_reset", 30);

  const double distance_fast_move = n_private.param<double>("distance_fast_move", 100);
  const double speed_fast_move_factor = n_private.param<double>("speed_fast_move_factor", 2.0);

  const bool reached_switch_off = n_private.param<bool>("reached_switch_off", true);
  const int error_interval = n_private.param<int>("error_interval", 6);

  const double tick_max = n_private.param<double>("tick_max", 2500);

  // Service (ON/OFF)
  ros::ServiceServer service_speed = n.advertiseService("speed", piston_speed);
  ros::ServiceServer service_reset = n.advertiseService("reset", piston_reset);
  ros::ServiceServer service_emergency = n.advertiseService("emergency", piston_emergency);
  ros::ServiceServer service_error_interval = n.advertiseService("error_interval", piston_error_interval);

  // Publisher
  ros::Publisher state_pub = n.advertise<seabot_piston_driver::PistonState>("state", 1);
  ros::Publisher speed_pub = n.advertise<seabot_piston_driver::PistonSpeedDebug>("speed", 1);
  ros::Publisher velocity_pub = n.advertise<seabot_piston_driver::PistonVelocity>("velocity", 1);
  ros::Publisher distance_travelled_pub = n.advertise<seabot_piston_driver::PistonDistanceTravelled>("distance_travelled", 1);
  seabot_piston_driver::PistonState state_msg;
  seabot_piston_driver::PistonSpeedDebug speed_msg;
  seabot_piston_driver::PistonVelocity velocity_msg;
  seabot_piston_driver::PistonDistanceTravelled distance_travelled_msg;

  // Subscriber
  ros::Subscriber piston_position_sub = n.subscribe("position", 1, position_callback);
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);

  ros::Time t_last_velocity, t_last_set_point;
  double velocity = 0.0;
  double position_last = 0.0;

  // Sensor initialization
  p.i2c_open();
  sleep(1); // 1s sleep (wait until i2c open)

  if(p.get_version()!=0x06){
    ROS_WARN("[Piston_driver] Wrong PIC code version");
  }

  p.set_piston_reset();
  // Wait reset_out
  while(p.m_state!=1){
    p.get_piston_all_data();
    sleep(1);
  }
  p.set_error_interval(error_interval);
  p.set_reached_switch_off(reached_switch_off);
  p.set_piston_speed(speed_in_min, speed_out_min);
  p.set_piston_speed_reset(speed_reset);

  double layer_number = 0; // number of the layer where the float is (depth / speed_layer)
  double layer_number_last = 0;
  std::vector<uint8_t> speed_table_out, speed_table_in;

  for(size_t i=0; i<ceil(depth_max/speed_depth_layer); i++){
    speed_table_out.push_back(speed_out_slope*i*speed_depth_layer + speed_out_min);
    speed_table_in.push_back(speed_in_slope*i*speed_depth_layer + speed_in_min);
  }

  bool new_speed = true;


  ROS_INFO("[Piston_driver] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    ros::Time t = ros::Time::now();

    /// ********************************************
    /// ******** Send Command to the piston ********
    /// ********************************************
    if(state_emergency){
      if(p.m_position_set_point!=0){
        p.set_piston_position(0);
        if(p.m_position<10)
          p.set_piston_speed(50, 50);
      }
    }
    else{
      // Piston set point
      if((piston_set_point != p.m_position_set_point) || (t-t_last_set_point).toSec()>30.0){
        t_last_set_point = t;
        p.set_piston_position(piston_set_point);
      }

      // Fast speed if position is far from position
      if(abs(piston_set_point - p.m_position)>distance_fast_move){
        if(!fast_move){
          new_speed = true;
          fast_move = true;
        }
      }
      else{
        if(fast_move){
          new_speed = true;
          fast_move = false;
        }
      }

      // Compute new speed index (20% hysteresis) from depth
      layer_number = depth/speed_depth_layer;
      if(abs(layer_number_last-layer_number)>1.2 && !fast_move){
        layer_number_last = min((size_t)ceil(abs(layer_number)), (size_t)(speed_table_in.size()-1));
        new_speed = true;
      }

      if(new_speed){
        size_t speed_in, speed_out;
        if(fast_move){
          speed_in = min((size_t)floor(speed_table_in[layer_number_last]*speed_fast_move_factor), speed_max);
          speed_out = min((size_t)floor(speed_table_out[layer_number_last]*speed_fast_move_factor), speed_max);
        }
        else{
          speed_in = speed_table_in[layer_number_last];
          speed_out = speed_table_out[layer_number_last];
        }
        p.set_piston_speed(speed_in,speed_out);
        speed_msg.speed_in = speed_in;
        speed_msg.speed_out = speed_out;

        // Log
        speed_pub.publish(speed_msg);
      }
    }

    //
    /// ********************************************
    /// ********      Get Piston state      ********
    /// ********************************************
    p.get_piston_all_data();
    state_msg.position = p.m_position;
    if(state_msg.position > tick_max) // Filter
      state_msg.position = tick_max;
    state_msg.switch_out = p.m_switch_out;
    state_msg.switch_in = p.m_switch_in;
    state_msg.state = p.m_state;
    state_msg.motor_on = p.m_motor_on;
    state_msg.enable_on = p.m_enable_on;
    state_msg.position_set_point = p.m_position_set_point;
    state_msg.motor_speed = p.m_motor_speed;
    state_pub.publish(state_msg);

    // Piston Velocity publisher
    double delta_t = (t - t_last_velocity).toSec();
    if(delta_t > 1.0){
      velocity = (p.m_position - position_last)/delta_t;
      t_last_velocity = t;
      position_last = p.m_position;
      velocity_msg.velocity =velocity;
      velocity_pub.publish(velocity_msg);
    }

    // Distance travelled
    distance_travelled += abs(p.m_position-last_piston_position_hf);
    last_piston_position_hf = p.m_position;
    distance_travelled_msg.distance = round(distance_travelled);
    distance_travelled_pub.publish(distance_travelled_msg);

    // Analyze speed issue (update speed_table)
    if(p.m_motor_speed!=50 && abs(velocity)==0.0 && p.m_state==1){
      if(!velocity_issue_detected){
        time_velocity_issue_detected = ros::WallTime::now();
        velocity_issue_detected = true;
      }
      else{
        if((ros::WallTime::now()-time_velocity_issue_detected).toSec()>3.0){
          for(size_t i=layer_number_last; i<speed_table_in.size(); i++){
            if(p.m_position_set_point > p.m_position){ // Sens of movement
              speed_table_in[i] = min(speed_max, speed_table_in[i]+speed_step_increase);
            }
            else{
              speed_table_out[i] = min(speed_max, speed_table_out[i]+speed_step_increase);
            }
          }
          velocity_issue_detected = false;
          new_speed = true;
          ROS_WARN("[Piston_driver] Velocity Issue detected - speed was increased of %zu for layer %f", speed_step_increase, layer_number_last);
        }
      }
    }
    else
      velocity_issue_detected=false;

    if(p.m_switch_in && p.m_switch_out){
      if(!is_sealing_issue){
        ROS_WARN("[Piston_driver] Sealing issue detected");
        time_sealing_issue = ros::Time::now();
        is_sealing_issue = true;
      }
      else{
        if((ros::Time::now()-time_sealing_issue).toSec()>15.0){
          ROS_WARN("[Piston_driver] Sealing issue detected - Start emergency procedure");
          p.set_piston_emergency();
          sleep(30);
          p.set_piston_reset();
        }
      }
    }
    else{
      is_sealing_issue = false;
    }

    loop_rate.sleep();
  }

  return 0;
}


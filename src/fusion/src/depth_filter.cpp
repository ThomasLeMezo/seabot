#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <pressure_89bsd_driver/PressureBsdData.h>
#include <fusion/DepthPose.h>

#include <algorithm>    // std::sort
#include <deque>

using namespace std;

double zero_depth = 1.0;
double depth = 0;
deque<pair<double, ros::Time>> depth_memory;
double velocity = 0;
double pressure_to_depth = 10.0;
size_t filter_size = 3;

deque<double> pressure_deque;
ros::Time time_pressure;

bool handle_zero_depth(std_srvs::Trigger::Request  &req,
                       std_srvs::Trigger::Response &res){
    if(!pressure_deque.empty()){
        zero_depth = pressure_deque[pressure_deque.size()-1];
        res.success = true;
    }
    else
        res.success = false;
    return true;
}

void pressure_callback(const pressure_89bsd_driver::PressureBsdData::ConstPtr& msg){
  pressure_deque.push_back(msg->pressure);
  if(pressure_deque.size()>filter_size)
    pressure_deque.pop_front();
  time_pressure = msg->header.stamp;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "piston");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 5.0);
  pressure_to_depth = n_private.param<double>("pressure_to_depth", 10.0);
  filter_size = (size_t) n_private.param<int>("filter_size", 3);
  size_t velocity_delta_size = (size_t) n_private.param<int>("velocity_delta_size", 5);

  // Service
  ros::ServiceServer service_reset_zero_depth = n.advertiseService("zero_depth", handle_zero_depth);

  // Subscriber
  ros::Subscriber pressure_sub = n.subscribe("/driver/sensor_external", 1, pressure_callback);

  // Publisher
  ros::Publisher depth_pub = n.advertise<fusion::DepthPose>("depth", 1);

  fusion::DepthPose msg;
  time_pressure = ros::Time::now();

  ros::Rate loop_rate(frequency);
  while (ros::ok()){

    if(!pressure_deque.empty()){
      deque<double> pressure_deque_tmp(pressure_deque);
      size_t n = pressure_deque_tmp.size() / 2;
      nth_element(pressure_deque_tmp.begin(), pressure_deque_tmp.begin()+n, pressure_deque_tmp.end());
      depth = (pressure_deque_tmp[n] - zero_depth) * pressure_to_depth;
      if(!depth_memory.empty())
        velocity = (depth-depth_memory[0].first)/(time_pressure-depth_memory[0].second).toSec();

      depth_memory.push_back(std::pair<double,ros::Time>(depth, time_pressure));
      if(depth_memory.size()>velocity_delta_size)
        depth_memory.pop_front();

      if((ros::Time::now()-time_pressure).toSec() < 1.0){
        msg.depth = depth;
        msg.velocity = velocity;
        depth_pub.publish(msg);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

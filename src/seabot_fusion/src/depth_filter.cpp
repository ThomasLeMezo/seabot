#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pressure_89bsd_driver/PressureBsdData.h>
#include <seabot_fusion/DepthPose.h>

#include <algorithm>    // std::sort
#include <deque>

using namespace std;

double zero_depth = 0.0;
double depth = 0;
deque<pair<double, ros::Time>> depth_memory;
double pressure_to_depth = 10.0;
int filter_median_size = 5;
int filter_mean_width = 3;

deque<double> pressure_deque;
ros::Time time_pressure;

bool zero_depth_valid = false;
bool new_data = false;

bool handle_zero_depth(std_srvs::Trigger::Request  &req,
                       std_srvs::Trigger::Response &res){
  if(!pressure_deque.empty()){
    for(double &p:pressure_deque)
      zero_depth +=p;
    zero_depth /= pressure_deque.size();

    res.success = true;
    zero_depth_valid = true;
  }
  else
    res.success = false;
  return true;
}

void pressure_callback(const pressure_89bsd_driver::PressureBsdData::ConstPtr& msg){
  pressure_deque.push_front(msg->pressure);
  if(pressure_deque.size()>filter_median_size)
    pressure_deque.pop_back();
  time_pressure = msg->header.stamp;
  new_data = true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "depth_filter_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 5.0);

  const double rho = n_private.param<double>("rho", 1020.0);
  const double g = n_private.param<double>("g", 9.81);
  const double g_rho = g*rho/1e5; // To be homogeneous to Bar

  filter_median_size = n_private.param<int>("filter_median_size", 10);
  filter_mean_width = n_private.param<int>("filter_mean_width", 3);

  const size_t filter_mean_width_velocity = (size_t) n_private.param<int>("filter_mean_width_velocity", 5);
  const size_t velocity_delta_size = (size_t) n_private.param<int>("velocity_delta_size", 5);
  const double velocity_limit = n_private.param<double>("velocity_limit", 0.5);

  // Service
  ros::ServiceServer service_reset_zero_depth = n.advertiseService("zero_depth", handle_zero_depth);

  // Subscriber
  ros::Subscriber pressure_sub = n.subscribe("/driver/sensor_external", 10, pressure_callback);

  // Publisher
  ros::Publisher depth_pub = n.advertise<seabot_fusion::DepthPose>("depth", 1);

  // Loop variables
  seabot_fusion::DepthPose msg;
  time_pressure = ros::Time::now();
  double velocity = 0.0;

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();
    if(!pressure_deque.empty() && new_data){
      /// ************** Compute depth ************** //
      /// MEDIAN + MEAN FILTER
      deque<double> pressure_deque_tmp(pressure_deque); // Make a copy
      sort(pressure_deque_tmp.begin(), pressure_deque_tmp.end()); // Sort to take median

      // Compute the mean with value centered to the median
      int n_mid = round(pressure_deque_tmp.size()/2.0);
      double pressure_mean = 0.0;
      int k=0;
      for(size_t i = max(n_mid-filter_mean_width, 0); i<min(n_mid+filter_mean_width, (int)pressure_deque_tmp.size()); i++){
        pressure_mean += pressure_deque_tmp[i];
        k++;
      }
      pressure_mean /= max(k, 1);
      depth = (pressure_mean - zero_depth) / (g_rho); // Take the median

      /// ************** Compute velocity ************** //
      // Save depth
      depth_memory.push_front(std::pair<double,ros::Time>(depth, time_pressure));
      if(depth_memory.size()>(velocity_delta_size+filter_mean_width_velocity))
        depth_memory.pop_back();

      if(depth_memory.size()==(velocity_delta_size+filter_mean_width_velocity)){
        double tmp_velocity = 0.0;
        for(size_t i=0; i<filter_mean_width_velocity; i++){
          tmp_velocity += (depth_memory[i].first-depth_memory[velocity_delta_size+i].first)/(depth_memory[i].second-depth_memory[velocity_delta_size+i].second).toSec();
        }
        velocity = tmp_velocity/filter_mean_width_velocity;

        if(abs(velocity)>velocity_limit)
          velocity = std::copysign(velocity_limit, velocity);
      }

      if(zero_depth_valid){
        msg.depth = depth;
        msg.velocity = velocity;
        depth_pub.publish(msg);
      }
      new_data = false;
    }

    loop_rate.sleep();
  }

  return 0;
}

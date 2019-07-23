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
double depth = 0.0;
deque<pair<double, ros::Time>> depth_memory;
double pressure_to_depth = 10.0;
int filter_window_size = 5;
int filter_mean_half_width = 3;

deque<double> pressure_deque;
ros::Time time_pressure;

bool new_data = false;

bool handle_zero_depth(std_srvs::Trigger::Request  &req,
                       std_srvs::Trigger::Response &res){
  if(!pressure_deque.empty()){
    zero_depth = 0.0;
    for(double &p:pressure_deque)
      zero_depth +=p;
    zero_depth /= pressure_deque.size();
    ROS_DEBUG("[Fusion_Depth] Zero_depth = %f", zero_depth);

    res.success = true;
  }
  else
    res.success = false;
  return true;
}

void pressure_callback(const pressure_89bsd_driver::PressureBsdData::ConstPtr& msg){
  pressure_deque.push_front(msg->pressure);
  if(pressure_deque.size()>filter_window_size)
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
  zero_depth = n_private.param<double>("zero_depth_pressure", 1.024);

  filter_window_size = n_private.param<int>("filter_window_size", 10);
  filter_mean_half_width = n_private.param<int>("filter_mean_half_width", 3);

  const size_t filter_velocity_window_size = (size_t) n_private.param<int>("filter_velocity_window_size", 6);
  const size_t velocity_dt_sample = (size_t) n_private.param<int>("velocity_dt_sample", 5);
  const size_t filter_velocity_mean_half_width = n_private.param<int>("filter_velocity_mean_half_width", 2);
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

  ROS_INFO("[FUSION depth] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();
    if(!pressure_deque.empty() && new_data){
      /// ************** Compute depth ************** //
      /// MEDIAN + MEAN FILTER
      deque<double> pressure_deque_tmp(pressure_deque); // Make a copy
      sort(pressure_deque_tmp.begin(), pressure_deque_tmp.end()); // Sort to take median

      // Compute the mean with value centered to the median
      const double n_mid = pressure_deque_tmp.size()/2.0;
      double pressure_mean = 0.0;
      int k=0;
      for(size_t i = max((size_t)round(n_mid-filter_mean_half_width), (size_t)0); i<min((size_t)round(n_mid+filter_mean_half_width), pressure_deque_tmp.size()); i++){
        pressure_mean += pressure_deque_tmp[i];
        k++;
      }
      pressure_mean /= max(k, 1);
      depth = (pressure_mean - zero_depth) / (g_rho); // Take the median

      /// ************** Compute velocity ************** //
      // Save depth
      depth_memory.push_front(std::pair<double,ros::Time>(depth, time_pressure));
      if(depth_memory.size()>(velocity_dt_sample+filter_velocity_window_size))
        depth_memory.pop_back();

      if(depth_memory.size()==(velocity_dt_sample+filter_velocity_window_size)){
        vector<double> velocity_memory;
        for(size_t i=0; i<filter_velocity_window_size; i++){
          // Delta_depth / Delta_dt
          double dt = (depth_memory[i].second-depth_memory[velocity_dt_sample+i].second).toSec();
          if(dt!=0)
            velocity_memory.push_back((depth_memory[i].first-depth_memory[velocity_dt_sample+i].first)/dt);
        }
        sort(velocity_memory.begin(), velocity_memory.end());
        const double n_mid_velocity = velocity_memory.size()/2.0;
        double velocity_mean = 0.0;
        int k_v=0;
        for(size_t i = max((size_t)round(n_mid_velocity-filter_velocity_mean_half_width), (size_t)0); i<min((size_t)round(n_mid_velocity+filter_velocity_mean_half_width), velocity_memory.size()); i++){
          velocity_mean += velocity_memory[i];
          k_v++;
        }
        velocity = velocity_mean / max(k, 1);;

        if(abs(velocity)>velocity_limit)
          velocity = std::copysign(velocity_limit, velocity);
      }

      msg.depth = depth;
      msg.velocity = velocity;
      msg.zero_depth_pressure = zero_depth;
      depth_pub.publish(msg);

      new_data = false;
    }

    loop_rate.sleep();
  }

  return 0;
}

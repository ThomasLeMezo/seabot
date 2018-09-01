#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pressure_89bsd_driver/PressureBsdData.h>
#include <seabot_fusion/TemperaturePose.h>

#include <algorithm>    // std::sort
#include <deque>

using namespace std;

deque<pair<double, ros::Time>> temperature_memory;
int filter_median_size = 5;

deque<double> temperature_deque;
ros::Time time_temperature;

bool new_data = false;

void temperature_callback(const pressure_89bsd_driver::PressureBsdData::ConstPtr& msg){
  temperature_deque.push_front(msg->temperature);
  if(temperature_deque.size()>filter_median_size)
    temperature_deque.pop_back();
  time_temperature = msg->header.stamp;
  new_data = true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "temperature_filter_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 5.0);

  filter_median_size = n_private.param<int>("filter_median_size", 10);
  const int filter_mean_width = n_private.param<int>("filter_mean_width", 3);

  const size_t filter_mean_width_velocity = (size_t) n_private.param<int>("filter_mean_width_velocity", 5);
  const size_t velocity_delta_size = (size_t) n_private.param<int>("velocity_delta_size", 5);
  const double velocity_limit = n_private.param<double>("velocity_limit", 2.0);

  // Subscriber
  ros::Subscriber temperature_sub = n.subscribe("/driver/sensor_external", 10, temperature_callback);

  // Publisher
  ros::Publisher temperature_pub = n.advertise<seabot_fusion::TemperaturePose>("temperature", 1);

  // Loop variables
  seabot_fusion::TemperaturePose msg;
  time_temperature = ros::Time::now();
  double temperature = 0;
  double velocity = 0.0;

  ROS_INFO("[FUSION temperature] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();
    if(!temperature_deque.empty() && new_data){
      /// ************** Compute temperature ************** //
      /// MEDIAN + MEAN FILTER
      deque<double> temperature_deque_tmp(temperature_deque); // Make a copy
      sort(temperature_deque_tmp.begin(), temperature_deque_tmp.end()); // Sort to take median

      // Compute the mean with value centered to the median
      int n_mid = round(temperature_deque_tmp.size()/2.0);
      double temperature_mean = 0.0;
      int k=0;
      for(size_t i = max(n_mid-filter_mean_width, 0); i<min(n_mid+filter_mean_width, (int)temperature_deque_tmp.size()); i++){
        temperature_mean += temperature_deque_tmp[i];
        k++;
      }
      temperature_mean /= max(k, 1);
      temperature = temperature_mean;

      /// ************** Compute velocity ************** //
      // Save temperature
      temperature_memory.push_front(std::pair<double,ros::Time>(temperature, time_temperature));
      if(temperature_memory.size()>(velocity_delta_size+filter_mean_width_velocity))
        temperature_memory.pop_back();

      if(temperature_memory.size()==(velocity_delta_size+filter_mean_width_velocity)){
        double tmp_velocity = 0.0;
        for(size_t i=0; i<filter_mean_width_velocity; i++){
          tmp_velocity += (temperature_memory[i].first-temperature_memory[velocity_delta_size+i].first)/(temperature_memory[i].second-temperature_memory[velocity_delta_size+i].second).toSec();
        }
        velocity = tmp_velocity/filter_mean_width_velocity;

        if(abs(velocity)>velocity_limit)
          velocity = std::copysign(velocity_limit, velocity);
      }

      msg.temperature = temperature;
      msg.velocity = velocity;
      temperature_pub.publish(msg);

      new_data = false;
    }

    loop_rate.sleep();
  }

  return 0;
}

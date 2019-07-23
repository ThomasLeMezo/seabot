#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <pressure_bme280_driver/Bme280Data.h>
#include <seabot_fusion/InternalPose.h>

#include <algorithm>    // std::sort
#include <deque>

using namespace std;

deque<double> pressure_memory;
deque<double> temperature_memory;
deque<double> humidity_memory;

int filter_median_size = 5;
int filter_mean_width = 3;

bool new_data = false;

void sensor_callback(const pressure_bme280_driver::Bme280Data::ConstPtr& msg){
  pressure_memory.push_front(msg->pressure);
  temperature_memory.push_front(msg->temperature);
  humidity_memory.push_front(msg->humidity);

  if(pressure_memory.size()>filter_median_size)
    pressure_memory.pop_back(); 
  if(temperature_memory.size()>filter_median_size)
    temperature_memory.pop_back();
  if(humidity_memory.size()>filter_median_size)
    humidity_memory.pop_back();
  new_data = true;
}

double compute_filter(const deque<double> &queue){
  deque<double> data_memory_tmp(queue); // Make a copy
  sort(data_memory_tmp.begin(), data_memory_tmp.end()); // Sort to take median

  // Compute the mean with value centered to the median
  int n_mid = round(data_memory_tmp.size()/2.0);
  double data_mean = 0.0;
  int k=0;
  for(size_t i = max(n_mid-filter_mean_width, 0); i<min(n_mid+filter_mean_width, (int)data_memory_tmp.size()); i++){
    data_mean += data_memory_tmp[i];
    k++;
  }
  data_mean /= max(k, 1);
  return data_mean;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "internal_sensor_filter_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 5.0);

  filter_median_size = n_private.param<int>("filter_median_size", 10);
  filter_mean_width = n_private.param<int>("filter_mean_width", 3);

  // Subscriber
  ros::Subscriber internal_sensor_sub = n.subscribe("/driver/sensor_internal", 10, sensor_callback);

  // Publisher
  ros::Publisher fusion_pub = n.advertise<seabot_fusion::InternalPose>("sensor_internal", 1);

  // Loop variables
  seabot_fusion::InternalPose msg;

  ROS_INFO("[FUSION internal sensor] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(new_data){
      if(!pressure_memory.empty()){
        msg.pressure = compute_filter(pressure_memory);
      }

      if(!temperature_memory.empty()){
        msg.temperature = compute_filter(temperature_memory);
      }

      if(!humidity_memory.empty()){
        msg.humidity = compute_filter(humidity_memory);
      }

      fusion_pub.publish(msg);
    }
    loop_rate.sleep();
  }

  return 0;
}

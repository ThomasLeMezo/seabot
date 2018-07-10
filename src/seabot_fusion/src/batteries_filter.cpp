#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_power_driver/Battery.h>

#include <algorithm>    // std::sort
#include <deque>

using namespace std;

array<deque<double>, 4> batteries_memory;

int filter_median_size = 5;
int filter_mean_width = 3;

bool zero_depth_valid = false;

void batteries_callback(const seabot_power_driver::Battery::ConstPtr& msg){
    batteries_memory[0].push_front(msg->battery1);
    batteries_memory[1].push_front(msg->battery2);
    batteries_memory[2].push_front(msg->battery3);
    batteries_memory[3].push_front(msg->battery4);
    if(batteries_memory[0].size()>filter_median_size){
        for(size_t i=0; i<4; i++)
            batteries_memory[i].pop_back();
    }
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "batteries_filter_node");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    const double frequency = n_private.param<double>("frequency", 5.0);

    filter_median_size = n_private.param<int>("filter_median_size", 10);
    filter_mean_width = n_private.param<int>("filter_mean_width", 3);

    // Subscriber
    ros::Subscriber batteries_sub = n.subscribe("/driver/power/battery", 10, batteries_callback);

    // Publisher
    ros::Publisher batteries_pub = n.advertise<seabot_power_driver::Battery>("battery", 1);

    // Loop variables
    seabot_power_driver::Battery msg;

    ros::Rate loop_rate(frequency);
    while (ros::ok()){
        ros::spinOnce();
        if(!batteries_memory.empty()){
            /// ************** Compute depth ************** //
            /// MEDIAN + MEAN FILTER

            array<double, 4> battery_level;
            for(size_t i=0; i<4; i++){
                deque<double> batteries_memory_tmp(batteries_memory[i]); // Make a copy
                sort(batteries_memory_tmp.begin(), batteries_memory_tmp.end()); // Sort to take median

                // Compute the mean with value centered to the median
                int n_mid = round(batteries_memory_tmp.size()/2.0);
                double battery_mean = 0.0;
                int k=0;
                for(size_t i = max(n_mid-filter_mean_width, 0); i<min(n_mid+filter_mean_width, (int)batteries_memory_tmp.size()); i++){
                    battery_mean += batteries_memory_tmp[i];
                    k++;
                }
                battery_mean /= max(k, 1);
                battery_level[i] = battery_mean;
            }

            msg.battery1 = battery_level[0];
            msg.battery2 = battery_level[1];
            msg.battery3 = battery_level[2];
            msg.battery4 = battery_level[3];
        }
        loop_rate.sleep();
    }

    return 0;
}

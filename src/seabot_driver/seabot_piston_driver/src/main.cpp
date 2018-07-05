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
__u16 piston_set_point = 0;
size_t cpt_error_zero = 0;

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
        p.set_piston_speed(125, 125);
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

    const double max_speed = 100.0;
    const double min_speed = 10.0;
    const double nb_step = 20.0;

    // Service (ON/OFF)
    ros::ServiceServer service_speed = n.advertiseService("speed", piston_speed);
    ros::ServiceServer service_reset = n.advertiseService("reset", piston_reset);
    ros::ServiceServer service_emergency = n.advertiseService("emergency", piston_emergency);
    ros::ServiceServer service_error_interval = n.advertiseService("error_interval", piston_error_interval);

    // Publisher
    ros::Publisher state_pub = n.advertise<seabot_piston_driver::PistonState>("state", 1);
    ros::Publisher speed_pub = n.advertise<seabot_piston_driver::PistonSpeedDebug>("speed", 1);
    ros::Publisher velocity_pub = n.advertise<seabot_piston_driver::PistonVelocity>("velocity", 1);
    seabot_piston_driver::PistonState state_msg;
    seabot_piston_driver::PistonSpeedDebug speed_msg;
    seabot_piston_driver::PistonVelocity velocity_msg;

    // Subscriber
    ros::Subscriber piston_position_sub = n.subscribe("position", 1, position_callback);
    ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);

    ros::Time t_last_velocity, t_last_set_point;
    double velocity = 0.0;
    double position_last = 0.0;

    // Sensor initialization
    p.i2c_open();
    sleep(1); // 1s sleep (wait until i2c open)

    // Wait reset_out
    while(p.m_state==1){
        p.get_piston_all_data();
        sleep(1);
    }
    p.set_error_interval(2);
    p.set_reached_enable(true);
    p.set_piston_speed(10, 10);

    ros::Rate loop_rate(frequency);
    while (ros::ok()){
        ros::spinOnce();

        ros::Time t = ros::Time::now();

        // Piston set point
        if(state_start && state_emergency==false){
            if((piston_set_point != p.m_position_set_point) || (t-t_last_set_point).toSec()>30.0){
                t_last_set_point = t;
                p.set_piston_position(piston_set_point);
            }
        }

        if(state_emergency){
            if(p.m_position_set_point!=0){
                p.set_piston_position(0);
                if(p.m_position<10)
                    p.set_piston_speed(50, 50);
            }
        }

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

        // Piston Velocity publisher
        double delta_t = (t - t_last_velocity).toSec();
        if(delta_t > 1.0){
            velocity = (p.m_position - position_last)/delta_t;
            t_last_velocity = t;
            position_last = p.m_position;
            velocity_msg.velocity =velocity;
            velocity_pub.publish(velocity_msg);
        }

        // Analyze depth to change motor speed
        if(adaptative_speed){
            int speed_in = (max_speed/nb_step)*round(max(min(depth*adaptative_coeff_slope_in+adaptative_coeff_offset_in, max_speed), min_speed)*(nb_step/max_speed));
            int speed_out = (max_speed/nb_step)*round(max(min(depth*adaptative_coeff_slope_out+adaptative_coeff_offset_out, max_speed), min_speed)*(nb_step/max_speed));

            // Hysteresis
            if(abs(speed_in-speed_in_last)>(max_speed/nb_step)/2.0
                    || abs(speed_out-speed_out_last)>(max_speed/nb_step)/2.0){
                p.set_piston_speed((uint16_t) speed_in, (uint16_t) speed_out);
                speed_in_last = speed_in;
                speed_out_last = speed_out;
                speed_msg.speed_in = speed_in;
                speed_msg.speed_out = speed_out;
                speed_pub.publish(speed_msg);
            }
        }

        loop_rate.sleep();
    }

    return 0;
}


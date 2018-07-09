#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>
#include <seabot_depth_regulation/RegulationDebug.h>
#include <seabot_depth_regulation/DepthPoint.h>
#include <seabot_mission/Waypoint.h>

#include <cmath>

using namespace std;

double depth = 0;
double velocity = 0;
double piston_position = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;

double depth_set_point = 0.0;
ros::Time t;
ros::Time t_old;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
    piston_position = msg->position;
    piston_switch_in = msg->switch_in;
    piston_switch_out = msg->switch_out;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
    depth = msg->depth;
    velocity= msg->velocity;
    t = ros::Time::now();
}

void depth_set_point_callback(const seabot_mission::Waypoint::ConstPtr& msg){
    depth_set_point = msg->depth;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sliding_node");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    const double frequency = n_private.param<double>("frequency", 1.0);
    const int offset_piston = n_private.param<int>("offset_piston", 700);
    int offset = offset_piston;

    const double g = n_private.param<double>("g", 9.81);
    const double rho_eau = n_private.param<double>("rho_eau", 1000.0);
    const double C_f = n_private.param<double>("C_f", 0.08);
    const double tick_to_volume = n_private.param<double>("tick_to_volume", 1.431715402026599e-07);
    const double hysteresis_piston = n_private.param<double>("hysteresis_piston", 0.6);

    const double K_factor = n_private.param<double>("K_factor", 1.0);
    const double K_velocity = n_private.param<double>("K_velocity", 300.0);
    const double K_acc = n_private.param<double>("K_acc", 100.0);

    const double piston_position_min = n_private.param<double>("piston_position_min", 0.0);
    const double piston_position_max = n_private.param<double>("piston_position_max", 1280.0);

    const double set_point_following = n_private.param<double>("set_point_following", 10.0);

    const double compression_factor = tick_to_volume * n_private.param<double>("compression_tick_factor", 10.0);

    // Subscriber
    ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
    ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
    ros::Subscriber depth_set_point_sub = n.subscribe("depth_set_point", 1, depth_set_point_callback);

    // Publisher
    ros::Publisher position_pub = n.advertise<seabot_piston_driver::PistonPosition>("/driver/piston/position", 1);
    ros::Publisher debug_pub = n.advertise<seabot_depth_regulation::RegulationDebug>("debug", 1);

    seabot_piston_driver::PistonPosition position_msg;
    seabot_depth_regulation::RegulationDebug debug_msg;

    double piston_set_point = 0.0;
    double piston_set_point_offset = piston_set_point + offset;
    t = ros::Time::now();
    t_old = ros::Time::now() - ros::Duration(1);
    ros::Rate loop_rate(frequency);

    // Main regulation loop
    while (ros::ok()){
        ros::spinOnce();

        double dt = (t-t_old).toSec();
        t_old = t;
        if(depth_set_point>0.2){
            offset = offset_piston - depth*compression_factor;

//            double V_piston = -(piston_position-offset) * tick_to_volume; // Inverted bc if position increase, volume decrease
//            double a = K_acc*(-g*(V_piston-depth*compression_factor*tick_to_volume)*rho_eau -0.5*C_f*velocity*abs(velocity)*rho_eau);
            double v = K_velocity*velocity;
            double e = depth_set_point-depth;
            double u = K_factor*dt*(- v + e); // (-a -v +e)

            // Antiwindup following (?)
            if(abs(piston_set_point+offset-piston_position)<set_point_following)
                piston_set_point+=u;

            // Antiwindup for switch
            if((piston_switch_out && (piston_set_point+offset)<piston_position) // To zero
                    || piston_switch_in && (piston_set_point+offset)>piston_position){ // To max set point
              piston_set_point = piston_position - offset;
              ROS_WARN("[Regulation] Antiwindup set");
            }

            // Min/Max antiwindup
            if((piston_set_point + offset) < piston_position_min)
                piston_set_point = piston_position_min - offset;
            if((piston_set_point + offset) > piston_position_max)
                piston_set_point = piston_position_max - offset;

            // Hysteresis effect to limit move of the motor
            if(abs(piston_set_point_offset - (piston_set_point + offset))>hysteresis_piston)
                piston_set_point_offset = round(piston_set_point + offset);

            // Position set point
            position_msg.position = piston_set_point_offset;

            // Debug msg
            debug_msg.acceleration = 0.0;
            debug_msg.velocity = v;
            debug_msg.depth_error = e;
            debug_msg.u = u;
            debug_msg.piston_set_point = piston_set_point;
            debug_msg.piston_set_point_offset = position_msg.position;
            debug_pub.publish(debug_msg);
        }
        else{
            // Position set point
            position_msg.position = 0;
        }
        position_pub.publish(position_msg);
        loop_rate.sleep();
    }

    return 0;
}


#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>
#include <seabot_depth_regulation/RegulationDebug.h>

#include <cmath>

using namespace std;

double depth = 0;
double velocity = 0;
double piston_position = 0;

double depth_set_point = 0.0;
ros::Time t;
ros::Time t_old;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
    piston_position = msg->position;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
    depth = msg->depth;
    velocity= msg->velocity;
    t = ros::Time::now();
}

void depth_set_point_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
    depth_set_point = msg->depth;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "sliding_node");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    const double frequency = n_private.param<double>("frequency", 1.0);
    const int offset = n_private.param<int>("offset", 700);

    const double g = n_private.param<double>("g", 9.81);
    const double rho_eau = n_private.param<double>("rho_eau", 1000.0);
    const double m = n_private.param<double>("m", 6.855);
    const double C_f = n_private.param<double>("C_f", 0.08);
    const double tick_to_volume = n_private.param<double>("tick_to_volume", 1.431715402026599e-07);
    const int max_delta_tick = n_private.param<int>("max_delta_tick", 200);
    const double hysteresis_piston = n_private.param<double>("hysteresis_piston", 0.63);

    const double K_factor = n_private.param<double>("K_factor", 0.1);
    const double K_velocity = n_private.param<double>("K_velocity", 150.0);

    const double rho_eau_m = rho_eau/m;

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
    while (ros::ok()){
        ros::spinOnce();

        double dt = (t-t_old).toSec();
        t_old = t;
        if(depth_set_point>0.2){
            double V_piston = -(piston_position-offset) * tick_to_volume; // Inverted bc if position increase, volume decrease
            double a = -g*V_piston*rho_eau_m -0.5*C_f*velocity*abs(velocity)*rho_eau_m;
            double v = K_velocity*velocity;
            double e = depth-depth_set_point;
            double u = -K_factor*dt*(a + v + e);
            piston_set_point+=u;

            if(abs(piston_set_point)>max_delta_tick)
                piston_set_point=copysign(max_delta_tick, piston_set_point);

            if(abs(piston_set_point_offset - (piston_set_point + offset))>hysteresis_piston)
                piston_set_point_offset = round(piston_set_point + offset);

            position_msg.position = piston_set_point_offset;

            debug_msg.acceleration = a;
            debug_msg.velocity = v;
            debug_msg.depth_error = e;
            debug_msg.u = u;
            debug_msg.piston_set_point = piston_set_point;
            debug_msg.piston_set_point_offset = position_msg.position;
            debug_pub.publish(debug_msg);
        }
        else{
            position_msg.position = 0;
        }
        position_pub.publish(position_msg);
        loop_rate.sleep();
    }

    return 0;
}


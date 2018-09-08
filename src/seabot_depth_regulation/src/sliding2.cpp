#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>
#include <seabot_depth_regulation/RegulationDebug2.h>
#include <seabot_mission/Waypoint.h>
#include <std_srvs/SetBool.h>

#include <cmath>

using namespace std;

double depth = 0;
double velocity = 0;
double piston_position = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;

bool is_surface = true;

double depth_set_point = 0.0;
ros::WallTime t_old;

double vector_field_velocity = 0.05;
double vector_field_approach_threshold = 2.0;

bool emergency = true; // Wait safety clearance on startup

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position = msg->position;
  piston_switch_in = msg->switch_in;
  piston_switch_out = msg->switch_out;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
  velocity = msg->velocity;
}

void depth_set_point_callback(const seabot_mission::Waypoint::ConstPtr& msg){
  if(msg->mission_enable)
    depth_set_point = msg->depth;
  else
    depth_set_point = 0.0;
}

bool emergency_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  emergency = req.data;
  res.success = true;
  return true;
}

double vector_field(const double &depth, const double &set_point){
  double e = set_point-depth;
  if(abs(e)>vector_field_approach_threshold)
    return copysign(vector_field_velocity, e);
  else
    return atan(tan(1)*e/vector_field_approach_threshold)*vector_field_velocity;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "sliding_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);

  const double hysteresis_piston = n_private.param<double>("hysteresis_piston", 0.6);
  const double set_point_following = n_private.param<double>("set_point_following", 10.0);

  const double K_factor = n_private.param<double>("K_factor", 200.0);
  double K_velocity = n_private.param<double>("K_velocity", 1.0);

  vector_field_velocity = n_private.param<double>("vector_field_velocity", 0.02);
  vector_field_approach_threshold = n_private.param<double>("vector_field_approach_threshold", 2.0);

  const double equilibrium_piston = n_private.param<double>("equilibrium_piston", 0.0);
  const double compressibility_coeff = n_private.param<double>("compressibility_coeff", 10.0);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber depth_set_point_sub = n.subscribe("/mission/set_point", 1, depth_set_point_callback);

  // Publisher
  ros::Publisher position_pub = n.advertise<seabot_piston_driver::PistonPosition>("/driver/piston/position", 1);
  ros::Publisher debug_pub = n.advertise<seabot_depth_regulation::RegulationDebug2>("debug", 1);

  seabot_piston_driver::PistonPosition position_msg;
  seabot_depth_regulation::RegulationDebug2 debug_msg;

  // Server
  ros::ServiceServer server_emergency = n.advertiseService("emergency", emergency_service);

  /// *************** Variable Description ***************//
  /// piston_position : Real position of the piston send by the microcontroller
  /// piston_set_point : sum of "u" (without equilibrium & compressibility)
  /// piston_set_point_offset : value send to the microcontroller

  // piston_set_point :
  double piston_set_point = 0.0;
  double piston_set_point_offset = piston_set_point + equilibrium_piston;
  position_msg.position = 0.0;
  t_old = ros::WallTime::now() - ros::WallDuration(1);
  ros::Rate loop_rate(frequency);

  // Main regulation loop
  ROS_INFO("[DepthRegulation2] Start Ok");
  while (ros::ok()){
    ros::spinOnce();

    ros::WallTime t = ros::WallTime::now();
    double dt = (t-t_old).toSec();
    t_old = t;

    if(depth_set_point>0.2 && !emergency){

      /// *********** Compressibility (Linear model) + Equilibrium *********** ///
      double offset_piston = equilibrium_piston + depth*compressibility_coeff;

      /// ********************** Command law ****************** ///
      double v_target = vector_field(depth, depth_set_point);
      double v = K_velocity*(v_target-velocity);
      double u = K_factor*dt*(v);

      double piston_set_point_new = piston_set_point + u;
      double piston_set_point_offset_new = piston_set_point_new + offset_piston;

      /// ********************** Antiwindup ****************** ///
      bool antiwindup = false;
      // Antiwindup following (?) : do not increase cmd if piston cannot follow
      // Avoid add too much energy (and so oscillations) to the command if motor is too slow
      double e_old = abs(piston_set_point_offset-piston_position);
      double e_new = abs(piston_set_point_offset_new-piston_position);
      if(e_old>set_point_following && e_new>e_old && !is_surface)
        antiwindup = true;

      // Antiwindup for switch
      if((piston_switch_out && piston_set_point_offset_new<piston_position) // To zero
         || (piston_switch_in && piston_set_point_offset_new>piston_position)) // To max set point
        antiwindup = true;

      /// ********************** Write command ****************** ///
      if(!antiwindup){
        //  Hysteresis to limit motor movement
        if(abs(piston_set_point_offset_new - piston_position)>hysteresis_piston)
          position_msg.position = round(piston_set_point_offset_new);

        piston_set_point = piston_set_point_new;
        piston_set_point_offset = piston_set_point_offset_new;
      }

      // Debug msg
      debug_msg.velocity_error = v;
      debug_msg.depth_error = (depth_set_point-depth);
      debug_msg.vector_field_target = v_target;

      debug_msg.u = u;
      debug_msg.piston_set_point = piston_set_point;
      debug_msg.piston_set_point_offset = piston_set_point_offset;
      debug_msg.antiwindup = antiwindup;
      debug_pub.publish(debug_msg);

      is_surface = false;
    }
    else{
      // Position set point
      position_msg.position = 0;
      is_surface = true;
    }
    position_pub.publish(position_msg);

    loop_rate.sleep();
  }

  return 0;
}


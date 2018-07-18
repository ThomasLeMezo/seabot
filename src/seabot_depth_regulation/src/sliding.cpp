#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>
#include <seabot_depth_regulation/RegulationDebug.h>
#include <seabot_mission/Waypoint.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <pressure_bme280_driver/Bme280Data.h>

#include <cmath>

using namespace std;

double depth = 0;
double velocity = 0;
double piston_position = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;
double pressure_limit = 6.2;
int pressure_limit_count = 5*3;
int pressure_limit_count_reset = 5*3; // 5Hz * 3s of data
bool pressure_limit_reached = true;

bool flash_is_enable = false;

double depth_set_point = 0.0;
ros::Time t;
ros::Time t_old;

ros::ServiceClient service_zero_depth;

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
  if(msg->mission_enable)
    depth_set_point = msg->depth;
  else
    depth_set_point = 0.0;
}

void pressure_callback(const pressure_bme280_driver::Bme280Data::ConstPtr& msg){
  if(msg->pressure > pressure_limit){
    if(pressure_limit_count!=0)
      pressure_limit_count--;
    else
      pressure_limit_reached=true;
  }
  else
    pressure_limit_count = pressure_limit_count_reset;
}

void call_zero_depth(){
  std_srvs::Trigger srv;
  if (!service_zero_depth.call(srv)){
    ROS_ERROR("[DepthRegulation] Failed to call zero depth");
  }
}

void call_flash_enable(const bool &val){
  if(val != flash_is_enable){
    std_srvs::SetBool srv;
    srv.request.data = val;
    if (!service_flash_enable.call(srv)){
      ROS_ERROR("[DepthRegulation] Failed to call flash enable");
    }
    else
      flash_is_enable = val;
  }
}

bool reset_limit_depth(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  pressure_limit_reached = false;
  res.success = true;
  return true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "sliding_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 1.0);

  //  const double g = n_private.param<double>("g", 9.81);
  //  const double rho_eau = n_private.param<double>("rho_eau", 1000.0);
  //  const double C_f = n_private.param<double>("C_f", 0.08);

  //  const double piston_diameter = n_private.param<double>("piston_diameter", 50.0e-3);
  //  const double tick_per_rotation = n_private.param<double>("tick_per_rotation", 48);
  //  const double screw_thread = n_private.param<double>("screw_thread", 1.75e-3);
  //  const double tick_to_volume = (screw_thread/tick_per_rotation)*pow(piston_diameter/2.0, 2)*M_PI;
  //  ROS_INFO("[Sliding_Node] tick_to_volume = %.10e", tick_to_volume);

  const double hysteresis_piston = n_private.param<double>("hysteresis_piston", 0.6);
  const double set_point_following = n_private.param<double>("set_point_following", 10.0);

  const double K_factor = n_private.param<double>("K_factor", 1.0);
  double K_velocity = n_private.param<double>("K_velocity", 300.0);
  //    const double K_acc = n_private.param<double>("K_acc", 100.0);

  const double cf_x0 = n_private.param<double>("offset_piston", 700);
  const double cf_x2 = n_private.param<double>("cf_x2", 0.0);
  const double cf_x1 = n_private.param<double>("cf_x1", 0.0);
  const double max_depth_compression_validity = n_private.param<double>("max_depth_compression_validity", 17.0);

  const double max_depth_reset_zero = n_private.param<double>("max_depth_reset_zero", 1.0);
  const double max_speed_reset_zero = n_private.param<double>("max_speed_reset_zero", 0.1);

  const double limit_depth_flash_enable = n_private.param<double>("limit_depth_flash_enable", 0.5);

  pressure_limit = n_private.param<double>("pressure_limit", 6.2);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber depth_set_point_sub = n.subscribe("/mission/set_point", 1, depth_set_point_callback);

  // Publisher
  ros::Publisher position_pub = n.advertise<seabot_piston_driver::PistonPosition>("/driver/piston/position", 1);
  ros::Publisher debug_pub = n.advertise<seabot_depth_regulation::RegulationDebug>("debug", 1);

  seabot_piston_driver::PistonPosition position_msg;
  seabot_depth_regulation::RegulationDebug debug_msg;

  // Service
  ROS_INFO("[DepthRegulation] Wait for zero depth service from fusion");
  ros::service::waitForService("/fusion/zero_depth");
  service_zero_depth = n.serviceClient<std_srvs::Trigger>("/fusion/zero_depth");
  call_zero_depth();
  ROS_INFO("[DepthRegulation] Reset zero");

  ROS_INFO("[DepthRegulation] Wait for flash service from power_driver");
  ros::service::waitForService("/driver/power/flash_led");
  service_flash_enable = n.serviceClient<std_srvs::SetBool>("/driver/power/flash_led");

  // Server
  ros::ServiceServer server_reset_limit_depth = n.advertiseService("reset_limit_depth", reset_limit_depth);

  double piston_set_point = 0.0;
  double offset_piston = cf_x0;
  double piston_set_point_offset = piston_set_point + offset_piston;
  t = ros::Time::now();
  t_old = ros::Time::now() - ros::Duration(1);
  ros::Rate loop_rate(frequency);

  // Main regulation loop
  ROS_INFO("[DepthRegulation] Start regulation");
  while (ros::ok()){
    ros::spinOnce();

    double dt = (t-t_old).toSec();
    t_old = t;
    if(depth_set_point>0.2 && !pressure_limit_reached){
      // Polynomial model
      double depth_validity = min(depth, max_depth_compression_validity);
      offset_piston = cf_x2*pow(depth_validity, 2) + cf_x1*depth_validity + cf_x0;

      //            double V_piston = -(piston_position-offset) * tick_to_volume; // Inverted bc if position increase, volume decrease
      //            double a = K_acc*(-g*(V_piston-depth*compression_factor*tick_to_volume)*rho_eau -0.5*C_f*velocity*abs(velocity)*rho_eau);
      double v = K_velocity*velocity;
      double e = depth_set_point-depth;
      double u = K_factor*dt*(- v + e); // (-a -v +e)

      // Antiwindup following (?) : do not increase cmd if piston cannot follow
      // Avoid add too much energy (and so oscillations) to the command if motor is too slow
      if(abs(piston_set_point+offset_piston-piston_position)<set_point_following)
        piston_set_point+=u;

      // Antiwindup for switch
      if((piston_switch_out && (piston_set_point+offset_piston)<piston_position) // To zero
         || piston_switch_in && (piston_set_point+offset_piston)>piston_position){ // To max set point
        piston_set_point = piston_position - offset_piston;
        ROS_WARN("[Regulation] Antiwindup set");
      }

      // Hysteresis effect to limit move of the motor
      if(abs(piston_set_point_offset - (piston_set_point + offset_piston))>hysteresis_piston)
        piston_set_point_offset = round(piston_set_point + offset_piston);

      // Position set point
      position_msg.position = piston_set_point_offset;

      // Debug msg
      debug_msg.acceleration = 0.0;
      debug_msg.velocity = v;
      debug_msg.depth_error = e;
      debug_msg.u = u;
      debug_msg.piston_set_point = piston_set_point;
      debug_msg.piston_set_point_offset = piston_set_point_offset;
      debug_pub.publish(debug_msg);
    }
    else{
      // Position set point
      position_msg.position = 0;

      // Analyze zero depth
      if(piston_position == 0 && depth < max_depth_reset_zero && abs(velocity) < max_speed_reset_zero)
        call_zero_depth();
    }
    position_pub.publish(position_msg);

    if(depth < limit_depth_flash_enable)
      call_flash_enable(true);
    else
      call_flash_enable(false);
    loop_rate.sleep();
  }

  return 0;
}


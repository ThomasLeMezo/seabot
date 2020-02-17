#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>
#include <seabot_depth_controller/RegulationDebug.h>
#include <seabot_mission/Waypoint.h>
#include <std_msgs/Float64.h>

#include <std_srvs/SetBool.h>
#include <seabot_fusion/Kalman.h>
#include <eigen3/Eigen/Dense>

#include <cmath>

using namespace std;
using namespace Eigen;

double piston_position = 0;
bool piston_switch_in = false;
bool piston_switch_out = false;
size_t piston_state = 0;

double depth_set_point = 0.0;
double limit_velocity = 0.0;
double approach_velocity = 1.0;
ros::WallTime t_old;
ros::Time time_last_state;
ros::Time time_depth_data;

bool emergency = true; // Wait safety clearance on startup

double s = -1.0;
double coeff_A = 0.;
double coeff_B = 0.;
double tick_to_volume = 0.;

enum STATE_MACHINE {STATE_SURFACE, STATE_SINK, STATE_REGULATION, STATE_STATIONARY, STATE_EMERGENCY, STATE_PISTON_ISSUE, STATE_HOLD_DEPTH};
STATE_MACHINE regulation_state = STATE_SURFACE;

seabot_depth_controller::RegulationDebug debug_msg;

int cpt_divider_frequency;

#define NB_STATES 6
// [Velocity; Depth; Volume; Offset, chi, chi2]
Matrix<double, NB_STATES, 1> x = Matrix<double, NB_STATES, 1>::Zero();

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position = msg->position;
  piston_switch_in = msg->switch_in;
  piston_switch_out = msg->switch_out;
  piston_state = msg->state;
}

void kalman_callback(const seabot_fusion::Kalman::ConstPtr& msg){
  x(0) = msg->velocity;
  x(1) = msg->depth;
  x(3) = msg->offset;
  x(4) = msg->chi;
  x(5) = msg->chi2;
  time_last_state = ros::Time::now();
  time_depth_data = msg->stamp;
}

void depth_set_point_callback(const seabot_mission::Waypoint::ConstPtr& msg){
  if(msg->mission_enable)
    depth_set_point = msg->depth;
  else
    depth_set_point = 0.0;
  limit_velocity = msg->limit_velocity;
  approach_velocity = msg->approach_velocity;
  cpt_divider_frequency = 1; // Trigger new control
}

bool emergency_service(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  emergency = req.data;
  res.success = true;
  return true;
}

double compute_u(const Matrix<double, NB_STATES, 1> &x, double set_point, double limit_velocity, double approach_velocity=1.0){
  const double x1 = x(0);
  const double x2 = x(1);
  const double x3 = x(2);
  const double x4 = x(3);
  const double x5 = x(4);
  const double x6 = x(5);
  const double A = coeff_A;
  const double B = coeff_B;
  const double beta = limit_velocity/M_PI_2;
  const double alpha = approach_velocity;
  const double gamma = beta/alpha;

  double e = (set_point-x2)/alpha;
  double y = x1-gamma*atan(e);
  double dx1 = -A*(x3+x4-(x5*x2+x6*pow(x2,2)))-B*abs(x1)*x1;
  double D = 1+pow(e,2);
  double dy = dx1 + gamma*x1/D;

  debug_msg.y = y;
  debug_msg.dy = dy;

  return (-2.*s*dy+pow(s,2)*y+ gamma*(dx1*D+2.*e*pow(x1,2)/pow(alpha,2))/(pow(D,2))-2.*B*abs(x1)*dx1)/A+x1*(x5+2.*x6*x2);
}

bool sortCommandAbs (double i,double j) { return (abs(i)<abs(j)); }

double optimize_u(std::array<double, 4> &u_tab){
  sort(u_tab.begin(), u_tab.end());
  if(u_tab[0]<0.0 && u_tab[u_tab.size()-1]>0.0) // Case one positive, one negative => do not move
    return 0.0;
  else{ // Else choose the command that minimizes u
    sort(u_tab.begin(), u_tab.end(), sortCommandAbs);
    return u_tab[0];
  }
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "state_feedback");
  ros::NodeHandle n;

  /// ************************* Parameters *************************
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 25.0);
  // Frequency divider
  const int divider_frequency = n_private.param<int>("divider_frequency", 5);

  const double delta_velocity_lb = n_private.param<double>("delta_velocity_lb", 0.0);
  const double delta_velocity_ub = n_private.param<double>("delta_velocity_ub", 0.0);

  const double delta_position_lb = n_private.param<double>("delta_position_lb", 0.0);
  const double delta_position_ub = n_private.param<double>("delta_position_ub", 0.0);

  // Physical characteristics
  const double rho = n_private.param<double>("rho", 1025.0);
  const double g = n_private.param<double>("g", 9.81);
  const double m = n_private.param<double>("m", 8.800);
  const double diam_collerette = n_private.param<double>("diam_collerette", 0.24);
  const double screw_thread = n_private.param<double>("screw_thread", 1.75e-3);
  const double tick_per_turn = n_private.param<double>("tick_per_turn", 48);
  const double piston_diameter = n_private.param<double>("piston_diameter", 0.05);
  const double piston_max_value = n_private.param<double>("piston_max_value", 2400);
  const double Cf = M_PI*pow(diam_collerette/2.0, 2);
  tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
  coeff_A = g*rho/m;
  coeff_B = 0.5*rho*Cf/m;

  // Compute regulation constant
  s = n_private.param<double>("root_regulation", -1.0);
  double limit_depth_regulation = n_private.param<double>("limit_depth_controller", 0.5);
  double speed_volume_sink = n_private.param<double>("speed_volume_sink", 2.0);

  const double hysteresis_piston = n_private.param<double>("hysteresis_piston", 0.6);
  const double piston_max_velocity = n_private.param<double>("piston_speed_max_tick", 30)*tick_to_volume; // in m3/sec

  // Hold depth parameters
  const bool hold_depth_enable = n_private.param<bool>("hold_depth", false);
  const double hold_depth_value_enter = n_private.param<double>("hold_depth_value_enter", 0.05);
  const double hold_depth_value_exit = n_private.param<double>("hold_depth_value_exit", 0.1);

  /// ************************* ROS Communication *************************
  // Subscriber
  ros::Subscriber kalman_sub = n.subscribe("/fusion/kalman", 1, kalman_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber depth_set_point_sub = n.subscribe("/mission/set_point", 1, depth_set_point_callback);

  // Publisher
  ros::Publisher position_pub = n.advertise<seabot_piston_driver::PistonPosition>("/driver/piston/position", 1);
  ros::Publisher debug_pub = n.advertise<seabot_depth_controller::RegulationDebug>("debug", 1);

  seabot_piston_driver::PistonPosition position_msg;

  // Server
  ros::ServiceServer server_emergency = n.advertiseService("emergency", emergency_service);

  /// ************************* Loop *************************
  // Variables
  position_msg.position = 0.0;
  t_old = ros::WallTime::now() - ros::WallDuration(1);
  ros::Rate loop_rate(frequency);

  double piston_position_old = 0.;
  double u = 0.; // in m3/s
  double piston_set_point=0.;
  size_t piston_set_point_msg = 0;

  cpt_divider_frequency = divider_frequency; // To reduce delay
  double control_loop_frequency = frequency/(double)divider_frequency;

  // Main regulation loop
  ROS_INFO("[DepthRegulation_Feedback] Start Ok");
  ROS_INFO("[DepthRegulation_Feedback] root_regulation = %f", s);
  while (ros::ok()){
    ros::spinOnce();

    cpt_divider_frequency--;
    if(cpt_divider_frequency<=0){ // New control is triggered by message reception (see depth set point callback)
      cpt_divider_frequency=divider_frequency;

      if(emergency)
        regulation_state = STATE_EMERGENCY;
      if(piston_state!=1)
        regulation_state = STATE_PISTON_ISSUE;

      switch(regulation_state){
      case STATE_SURFACE:
        if(depth_set_point>=limit_depth_regulation)
          regulation_state = STATE_SINK;
        u = 0;
        piston_set_point = 0;

        break;
      case STATE_SINK:

        if(depth_set_point<limit_depth_regulation)
          regulation_state = STATE_SURFACE;
        else if(x(1)<limit_depth_regulation){
          u = -speed_volume_sink*tick_to_volume;
          double ref_eq = x(3)/tick_to_volume;

          if(piston_position <ref_eq*0.99) // coefficient to validate reaching piston_ref_eq
            piston_set_point = ref_eq;
          else{
            piston_set_point = ref_eq - u/(tick_to_volume*control_loop_frequency);
          }
        }
        else
          regulation_state = STATE_REGULATION;

        break;

      case STATE_REGULATION:

        if(depth_set_point<limit_depth_regulation)
          regulation_state = STATE_SURFACE;
        else if(x(1)>=limit_depth_regulation){
          if((ros::Time::now()-time_last_state).toSec()<1.0){

            x(2) = -piston_position*tick_to_volume;

            // Compute several commands according to velocity acceptable bounds
            array<double, 4> u_tab;
            u_tab[0] = compute_u(x, depth_set_point, limit_velocity+delta_velocity_lb, approach_velocity);
            u_tab[1] = compute_u(x, depth_set_point, limit_velocity+delta_velocity_ub, approach_velocity);
            u_tab[2] = compute_u(x, depth_set_point+delta_position_lb, limit_velocity, approach_velocity);
            u_tab[3] = compute_u(x, depth_set_point+delta_position_ub, limit_velocity, approach_velocity);

            // Find best command
            u=optimize_u(u_tab);

            // Mechanical limits (in = v_min, out = v_max)
            if((piston_switch_in && u<0) || (piston_switch_out && u>0))
              u = 0.0;
          }
          else{
            // Did not received state => resurfacing
            u=speed_volume_sink*tick_to_volume;
            ROS_INFO("[DepthRegulation] Timing issue");
          }
        }
        else
          regulation_state = STATE_SINK;

        // Limitation of u according to engine capabilities
        if(abs(u)>piston_max_velocity){
          u=copysign(piston_max_velocity, u);
        }

        // Check next formula in the case where piston_set_point-piston_position > piston_max_velocity/frequency
        piston_set_point -= u/(tick_to_volume*control_loop_frequency);
        // Previous form do not allow movement under 1 tick
        // piston_set_point = piston_position - u/(tick_to_volume*control_loop_frequency);

        if(hold_depth_enable && abs(depth_set_point-x(1))<hold_depth_value_enter)
          regulation_state = STATE_HOLD_DEPTH;

        break;

      case STATE_HOLD_DEPTH:
        u=0.0;
        if(abs(depth_set_point-x(1))>=hold_depth_value_exit)
          regulation_state = STATE_REGULATION;
        break;

      case STATE_EMERGENCY:
        u = 0.0;
        piston_set_point = 0;

        if(!emergency)
          regulation_state = STATE_SURFACE;
        break;

      case STATE_PISTON_ISSUE:
        u = 0.0;
        if(piston_state==1)
          regulation_state = STATE_SURFACE;
        break;

      default:
        break;
      }

      /// ********************** Write command ****************** ///
      //  Hysteresis to limit motor movement
      if(abs(piston_position_old - piston_set_point)>hysteresis_piston){
        if(piston_set_point>piston_max_value)
          piston_set_point = piston_max_value;
        if(piston_set_point<0)
          piston_set_point = 0;

        piston_set_point_msg = round(piston_set_point);
        piston_position_old = piston_set_point;
      }

      // Debug msg (publish only if change)
      if(debug_msg.u != u || debug_msg.piston_set_point != piston_set_point || debug_msg.mode != regulation_state){
        debug_msg.u = u;
        debug_msg.piston_set_point = piston_set_point;
        debug_msg.mode = regulation_state;
        debug_pub.publish(debug_msg);
      }

      if(position_msg.position != piston_set_point_msg){
        position_msg.position = piston_set_point_msg;
        position_msg.stamp = time_depth_data;
        position_pub.publish(position_msg);
      }
    }
    loop_rate.sleep();
  }

  return 0;
}


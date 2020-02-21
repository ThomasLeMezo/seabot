#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pressure_89bsd_driver/PressureBsdData.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonVelocity.h>
#include <seabot_depth_controller/RegulationDebug.h>
#include <seabot_fusion/Kalman.h>
#include <std_msgs/Float64.h>
#include <seabot_mission/Waypoint.h>

#include <algorithm>    // std::sort
#include <deque>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define NB_MESURES 1
#define NB_STATES 4
#define NB_COMMAND 1

double depth = 0.0;
double velocity_fusion = 0.0;
double piston_position = 0.0;
double piston_set_point = 0.0;
double piston_command_u = 0.0;
double depth_set_point = 0.0;

double coeff_A = 0.0;
double coeff_B = 0.0;
double tick_to_volume = 0.0;

double g_rho_bar = 0.0;

ros::Time time_last_depth;
bool depth_valid = false;
//double zero_depth_pressure = 1.024;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position = msg->position;
  piston_set_point = msg->position_set_point;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
  velocity_fusion = msg->velocity;
  depth_valid = true;
  time_last_depth = ros::Time::now();
}

void depth_set_point_callback(const seabot_mission::Waypoint::ConstPtr& msg){
  if(msg->mission_enable)
    depth_set_point = msg->depth;
  else
    depth_set_point = 0.0;
}


Matrix<double,NB_STATES, 1> f(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_COMMAND, 1> &u){
  Matrix<double,NB_STATES, 1> y = Matrix<double,NB_STATES, 1>::Zero();
  y(0) = -coeff_A*(u(0)+x(2)-x(3)*x(1))-coeff_B*copysign(x(0)*x(0), x(0));
  y(1) = x(0);
  y(2) = 0;
  y(3) = 0;
  return y;
}

void kalman_predict(const Matrix<double,NB_STATES, 1> &xup,
                    const Matrix<double,NB_STATES, NB_STATES> &Gup,
                    const Matrix<double,NB_COMMAND, 1> &u,
                    const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                    const Matrix<double,NB_STATES, NB_STATES> &Ak,
                    const double &dt,
                    Matrix<double,NB_STATES, 1> &xnew,
                    Matrix<double,NB_STATES, NB_STATES> &gamma){
  gamma = Ak*Gup*Ak.transpose()+gamma_alpha; // Covariation estimatation
  xnew = xup + f(xup, u)*dt;  // New state estimation
}

void kalman_correc(const Matrix<double,NB_STATES, 1> &x0,
                   const Matrix<double,NB_STATES,NB_STATES> &gamma_0,
                   const Matrix<double,NB_MESURES, 1> &y,
                   const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
                   const Matrix<double,NB_MESURES, NB_STATES> &C,
                   Matrix<double,NB_STATES, 1> &xup,
                   Matrix<double,NB_STATES,NB_STATES> &Gup){
  Matrix<double,NB_MESURES,NB_MESURES> S = C * gamma_0 * C.transpose() + gamma_beta;
  Matrix<double,NB_STATES, NB_MESURES> K = gamma_0 * C.transpose() * S.inverse();
  Matrix<double,NB_MESURES, 1> ytilde = y - C*x0;
  Gup = (Matrix<double,NB_STATES,NB_STATES>::Identity()-K*C)*gamma_0;
  xup = x0 + K*ytilde;
}

void kalman(Matrix<double,NB_STATES, 1> &x,
            Matrix<double,NB_STATES,NB_STATES> &gamma,
            const Matrix<double,NB_COMMAND, 1> &u,
            const Matrix<double,NB_MESURES, 1> &y,
            const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
            const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
            const Matrix<double,NB_STATES, NB_STATES> &Ak,
            const Matrix<double,NB_MESURES, NB_STATES> &C,
            const double &dt
            ){
  Matrix<double, NB_STATES, 1> xup;
  Matrix<double, NB_STATES, NB_STATES> Gup;
  kalman_correc(x, gamma, y, gamma_beta, C, xup, Gup);
  kalman_predict(xup, Gup, u, gamma_alpha, Ak, dt, x, gamma);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "kalman");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 5.0);

  const double rho = n.param<double>("/rho", 1025.0);
  const double g = n.param<double>("/g", 9.81);
  const double m = n.param<double>("/m", 9.045);
  const double diam_collerette = n.param<double>("/diam_collerette", 0.24);
  const double screw_thread = n.param<double>("/screw_thread", 1.75e-3);
  const double tick_per_turn = n.param<double>("/tick_per_turn", 48);
  const double piston_diameter = n.param<double>("/piston_diameter", 0.05);
  tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;

  const double piston_ref_eq = n.param<double>("/piston_ref_eq", 2100);
  const double limit_offset = n.param<double>("limit_offset", 2400)*tick_to_volume;
  const double limit_chi = n.param<double>("limit_chi", 100)*tick_to_volume;

  const double limit_min_depth = n.param<double>("limit_min_depth", 0.5);

  const double gamma_alpha_velocity = n_private.param<double>("gamma_alpha_velocity", 1e-4);
  const double gamma_alpha_depth = n_private.param<double>("gamma_alpha_depth", 1e-5);
  const double gamma_alpha_offset = n_private.param<double>("gamma_alpha_offset", 1e-8);
  const double gamma_alpha_chi = n_private.param<double>("gamma_alpha_chi", 1e-8);

  g_rho_bar = g*rho/1e5;

  coeff_A = g*rho/m;
  const double Cf = M_PI*pow(diam_collerette/2.0, 2);
  coeff_B = 0.5*rho*Cf/m;


  //  ROS_INFO("tick_to_volume = %.10e", tick_to_volume);
  ROS_INFO("Coeff_A %.10e", coeff_A);
  ROS_INFO("Coeff_B %.10e", coeff_B);
  ROS_INFO("tick_to_volume %.10e", tick_to_volume);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber depth_set_point_sub = n.subscribe("/mission/set_point", 1, depth_set_point_callback);

  // Publisher
  ros::Publisher kalman_pub = n.advertise<seabot_fusion::Kalman>("kalman", 1);
  seabot_fusion::Kalman msg;

  // Loop variables
  // Line, Row
  Matrix<double, NB_STATES, NB_STATES> gamma_alpha = Matrix<double, NB_STATES, NB_STATES>::Zero();
  Matrix<double, NB_MESURES, NB_MESURES> gamma_beta = Matrix<double, NB_MESURES, NB_MESURES>::Zero();
  Matrix<double, NB_STATES, NB_STATES> Ak = Matrix<double, NB_STATES, NB_STATES>::Zero();
  Matrix<double, NB_MESURES, NB_STATES> Ck = Matrix<double, NB_MESURES, NB_STATES>::Zero();

  Matrix<double, NB_STATES, 1> xhat = Matrix<double, NB_STATES, 1>::Zero();
  Matrix<double,NB_STATES,NB_STATES> gamma = Matrix<double,NB_STATES,NB_STATES>::Zero();

  gamma(0,0) = pow(1e-1, 2); // velocity
  gamma(1,1) = pow(1e-3, 2); // Depth
  gamma(2,2) = pow(limit_offset, 2); // Error offset;
  gamma(3,3) = pow(limit_chi,2); // Compressibility

  gamma_alpha(0,0) = pow(gamma_alpha_velocity, 2); // velocity (1e-4)
  gamma_alpha(1,1) = pow(gamma_alpha_depth, 2); // Depth (1e-5)
  gamma_alpha(2,2) = pow(gamma_alpha_offset, 2); // Error offset (1e-7)
  gamma_alpha(3,3) = pow(gamma_alpha_chi, 2); // Compressibility

  gamma_beta(0, 0) = pow(1e-4, 2); // Depth

  Ak(0, 0) = 0.0;
  Ak(0, 2) = -coeff_A;
  Ak(1, 0) = 1.;

  Ck(0, 1) = 1.;

  xhat(0) = 0.0;
  xhat(1) = 0.0;
  xhat(2) = -limit_offset/2.0;
  xhat(3) = 0.0;

  Matrix<double,NB_MESURES, 1> measure = Matrix<double,NB_MESURES, 1>::Zero();
  Matrix<double,NB_COMMAND, 1> command = Matrix<double,NB_COMMAND, 1>::Zero();

  const double dt = 1.0/frequency;
//  ros::Time t_last;

  ROS_INFO("[Kalman depth] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    bool update = false;

    /// Allow an update of the Kalman filter if:
    /// * Min depth
    /// * No surface set point
    /// * Received a new depth measure
    if(depth>limit_min_depth && depth_set_point!=0.0 && depth_valid){
//      dt = (time_last_depth-t_last).toSec();
//      t_last = time_last_depth;
//      if(dt>10./frequency) // Case where kalman is re-enabled
//        dt=1./frequency;

      Ak(0,0) = -2.*coeff_B*abs(xhat(0));
      Ak(0,1) = xhat(3)*coeff_A;
      Ak(0,3) = xhat(1)*coeff_A;
      Matrix<double, NB_STATES, NB_STATES> Ak_tmp = Ak*dt + Matrix<double, NB_STATES, NB_STATES>::Identity();
      measure(0) = depth;
      command(0) = (piston_ref_eq - piston_position)*tick_to_volume; // u

      kalman(xhat,gamma,command,measure,gamma_alpha,gamma_beta,Ak_tmp,Ck, dt);
      depth_valid = false;
      update = true;
      msg.valid = true;

      // Case Divergence of Kalman filter
      if((xhat(2)-xhat(3)*xhat(1))>limit_offset){
//        xhat(2) = min(max(xhat(2), -limit_offset), limit_offset);
//        xhat(3) = min(max(xhat(3), -limit_chi), limit_chi);
        gamma(2,2) = pow(limit_offset, 2); // Error offset;
        gamma(3,3) = pow(limit_chi,2); // Compressibility
        msg.valid = false;
      }

    }
    else if(depth<=limit_min_depth){
      xhat(0) = velocity_fusion;
      xhat(1) = depth;
      xhat(2) = xhat(2);
      xhat(3) = xhat(3);
      update = true;
      msg.valid = false;
    }

    if(update){
      msg.velocity = xhat(0);
      msg.depth = xhat(1);
      msg.offset = xhat(2);
      msg.chi = xhat(3);

      msg.variance[0] = gamma(0,0);
      msg.variance[1] = gamma(1,1);
      msg.variance[2] = gamma(2,2);
      msg.variance[3] = gamma(3,3);

      kalman_pub.publish(msg);
    }

    loop_rate.sleep();
  }

  return 0;
}

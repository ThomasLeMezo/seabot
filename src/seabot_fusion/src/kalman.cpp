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
#include <seabot_piston_driver/RegulationLoopDuration.h>
#include <seabot_safety/SafetyLog.h>

#include <algorithm>    // std::sort
#include <deque>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define NB_MESURES 1
#define NB_STATES 5
#define NB_COMMAND 1

double depth = 0.0;
double velocity_fusion = 0.0;
double piston_position = 0.0;
double piston_position_last = 0.0;

double piston_set_point = 0.0;

double coeff_A = 0.0;
double coeff_B = 0.0;
double tick_to_volume = 0.0;

double g_rho_bar = 0.0;

ros::Time time_piston_data, time_piston_data_last;
ros::Time time_depth_data, time_depth_data_last;

bool new_depth_data = false;
bool new_piston_data = false;

double forecast_dt_regulation = 0.0;
double forecast_filter = 0.7;

bool seafloor_landing = false;
bool seafloor_detected = false;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position_last = piston_position;
  time_piston_data_last = time_piston_data;

  time_piston_data = msg->stamp;
  piston_position = msg->position;
  piston_set_point = msg->position_set_point;
  new_piston_data = true;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  time_depth_data_last = time_depth_data;

  depth = msg->depth;
  velocity_fusion = msg->velocity;
  time_depth_data = msg->stamp;
  new_depth_data = true;
}

void regulation_loop_callback(const seabot_piston_driver::RegulationLoopDuration::ConstPtr& msg){
  forecast_dt_regulation = (1.-forecast_filter)* forecast_dt_regulation + forecast_filter*(msg->dt.toSec());
}

void mission_callback(const seabot_mission::Waypoint::ConstPtr& msg){
  seafloor_landing = msg->seafloor_landing;
}

void safety_callback(const seabot_safety::SafetyLog::ConstPtr& msg){
  seafloor_detected = msg->seafloor;
}

Matrix<double,NB_STATES, 1> f(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_COMMAND, 1> &u){
  Matrix<double,NB_STATES, 1> y = Matrix<double,NB_STATES, 1>::Zero();
  y(0) = -coeff_A*(u(0)+x(2)-(x(3)*x(1)+x(4)*x(1)))-coeff_B*copysign(x(0)*x(0), x(0));
  y(1) = x(0);
  y(2) = 0;
  y(3) = 0;
  return y;
}

void kalman_predict(Matrix<double,NB_STATES, 1> &x,
                    Matrix<double,NB_STATES, NB_STATES> &gamma,
                    const Matrix<double,NB_COMMAND, 1> &u,
                    const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                    const Matrix<double,NB_STATES, NB_STATES> &Ak,
                    const double &dt){
  Matrix<double, NB_STATES, NB_STATES> Ak_tmp = Ak*dt + Matrix<double, NB_STATES, NB_STATES>::Identity();
  gamma = Ak_tmp*gamma*Ak_tmp.transpose()+gamma_alpha*dt; // Variance estimatation
  x += f(x, u)*dt;  // New State estimation
}

void kalman_correc(Matrix<double,NB_STATES, 1> x,
                   Matrix<double,NB_STATES,NB_STATES> &gamma,
                   const Matrix<double,NB_MESURES, 1> &y,
                   const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
                   const Matrix<double,NB_MESURES, NB_STATES> &Ck){
  Matrix<double,NB_MESURES,NB_MESURES> S = Ck * gamma * Ck.transpose() + gamma_beta;
  Matrix<double,NB_STATES, NB_MESURES> K = gamma * Ck.transpose() * S.inverse();
  Matrix<double,NB_MESURES, 1> ytilde = y - Ck*x;

  gamma *= (Matrix<double,NB_STATES,NB_STATES>::Identity()-K*Ck);
  x += K*ytilde;
}

void kalman(Matrix<double,NB_STATES, 1> &x,
            Matrix<double,NB_STATES,NB_STATES> &gamma,
            const Matrix<double,NB_COMMAND, 1> &u,
            const Matrix<double,NB_MESURES, 1> &y,
            const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
            const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
            const Matrix<double,NB_STATES, NB_STATES> &Ak,
            const Matrix<double,NB_MESURES, NB_STATES> &Ck,
            const double &dt
            ){
  kalman_correc(x, gamma, y, gamma_beta, Ck);
  kalman_predict(x, gamma, u, gamma_alpha, Ak, dt);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "kalman");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 5.0);

  const double rho = n_private.param<double>("rho", 1025.0);
  const double g = n_private.param<double>("g", 9.81);
  const double m = n_private.param<double>("m", 9.045);
  const double diam_collerette = n_private.param<double>("diam_collerette", 0.24);
  const double screw_thread = n_private.param<double>("screw_thread", 1.75e-3);
  const double tick_per_turn = n_private.param<double>("tick_per_turn", 48);
  const double piston_diameter = n_private.param<double>("piston_diameter", 0.05);
  const double piston_ref_eq = n_private.param<double>("piston_ref_eq", 2100);
  tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;

  const double limit_offset = n_private.param<double>("limit_offset", 2400)*tick_to_volume;
  const double limit_chi = n_private.param<double>("limit_chi", 100)*tick_to_volume;
  const double limit_chi2 = limit_chi;

  const double limit_min_depth = n_private.param<double>("limit_min_depth", 0.5);

  const double gamma_alpha_velocity = n_private.param<double>("gamma_alpha_velocity", 1e-4);
  const double gamma_alpha_depth = n_private.param<double>("gamma_alpha_depth", 1e-5);
  const double gamma_alpha_offset = n_private.param<double>("gamma_alpha_offset", 1e-8);
  const double gamma_alpha_chi = n_private.param<double>("gamma_alpha_chi", 1e-8);
  const double gamma_alpha_chi2 = n_private.param<double>("gamma_alpha_chi2", 1e-8);

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
  ros::Subscriber regulation_loop_sub = n.subscribe("/driver/piston/regulation_loop", 1, regulation_loop_callback);
  ros::Subscriber safety_sub = n.subscribe("/safety/safety", 1, safety_callback);
  ros::Subscriber mission_sub = n.subscribe("/mission/set_point", 1, mission_callback);

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
  Matrix<double,NB_STATES, 1> x_forcast(xhat);
  Matrix<double,NB_STATES, NB_STATES> gamma_forcast(gamma);

  gamma(0,0) = pow(1e-1, 2); // velocity
  gamma(1,1) = pow(1e-3, 2); // Depth
  gamma(2,2) = pow(limit_offset/2.0, 2); // Error offset;
  gamma(3,3) = pow(limit_chi,2); // Compressibility
  gamma(4,4) = pow(limit_chi2,2); // Compressibility 2

  gamma_alpha(0,0) = pow(gamma_alpha_velocity, 2); // Velocity
  gamma_alpha(1,1) = pow(gamma_alpha_depth, 2); // Depth
  gamma_alpha(2,2) = pow(gamma_alpha_offset, 2); // Offset
  gamma_alpha(3,3) = pow(gamma_alpha_chi, 2); // Compressibility
  gamma_alpha(4,4) = pow(gamma_alpha_chi2, 2); // Compressibility 2

  gamma_beta(0, 0) = pow(1e-4, 2); // Depth

  Ak(0, 0) = 0.0;
  Ak(0, 2) = -coeff_A;
  Ak(1, 0) = 1.;

  Ck(0, 1) = 1.;

  xhat(0) = 0.0; // dz
  xhat(1) = 0.0; // z
  xhat(2) = piston_ref_eq*tick_to_volume; // Vp
  xhat(3) = 0.0; // chi
  xhat(4) = 0.0; // chi2

  Matrix<double,NB_MESURES, 1> y = Matrix<double,NB_MESURES, 1>::Zero();
  Matrix<double,NB_COMMAND, 1> u = Matrix<double,NB_COMMAND, 1>::Zero();

  ROS_INFO("[Kalman depth] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(new_piston_data || new_depth_data){
      /// Do not enable Kalman filter if
      /// * depth limit condition
      /// * or robot is on seafloor
      if(depth>limit_min_depth || (seafloor_detected && seafloor_landing)){

        Ak(0,0) = -2.*coeff_B*abs(xhat(0));
        Ak(0,1) = (xhat(3)+2.*xhat(4)*xhat(1))*coeff_A;
        Ak(0,3) = xhat(1)*coeff_A;
        Ak(0,4) = pow(xhat(1),2)*coeff_A;

        ros::Duration dt;
        if(new_piston_data){
          u(0) = -piston_position_last*tick_to_volume; // u
          dt = time_piston_data - time_piston_data_last;
        }
        else{
          u(0) = -piston_position*tick_to_volume; // u
          dt = time_depth_data - time_piston_data_last;
          time_piston_data_last = time_depth_data;
        }
        // ToDo : case where we have both new_piston_data & new_depth_data : dt should be handle more accuratly

        kalman_predict(xhat, gamma, u, gamma_alpha, Ak, dt.toSec());

        if(new_depth_data){
          y(0) = depth;
          kalman_correc(xhat, gamma, y, gamma_beta, Ck);

          // Forecast
          x_forcast = xhat;
          gamma_forcast = gamma;
          if(forecast_dt_regulation != 0.0){
            u(0) = -piston_position*tick_to_volume;
            kalman_predict(x_forcast, gamma_forcast, u, gamma_alpha, Ak, forecast_dt_regulation);
          }

          msg.valid = true;
          msg.forecast_dt_regulation = forecast_dt_regulation;

          // Case Divergence of Kalman filter
          double equilibrium_volume = xhat(2)-(xhat(3)*xhat(1)+xhat(4)*pow(xhat(1),2));
          if(equilibrium_volume>limit_offset || equilibrium_volume<0){
            gamma(2,2) = pow(limit_offset, 2); // Error offset;
            gamma(3,3) = pow(limit_chi,2); // Compressibility
            msg.valid = false;
          }
        }
      }
      else{
        if(new_depth_data){
          xhat(0) = velocity_fusion;
          xhat(1) = depth;
          msg.valid = false;
        }
      }

      if(new_depth_data){
        msg.velocity = x_forcast(0);
        msg.depth = x_forcast(1);
        msg.offset = x_forcast(2);
        msg.chi = x_forcast(3);
        msg.chi2 = x_forcast(4);
        msg.stamp = time_piston_data;

        msg.variance[0] = gamma_forcast(0,0);
        msg.variance[1] = gamma_forcast(1,1);
        msg.variance[2] = gamma_forcast(2,2);
        msg.variance[3] = gamma_forcast(3,3);
        msg.variance[4] = gamma_forcast(4,4);

        kalman_pub.publish(msg);
      }

      new_piston_data = false;
      new_depth_data = false;
    }

    loop_rate.sleep();
  }

  return 0;
}

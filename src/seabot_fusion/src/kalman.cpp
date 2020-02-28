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
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace std;
using namespace Eigen;

#define NB_MESURES 1
#define NB_STATES 6
#define NB_COMMAND 1

float depth = 0.0;
float velocity_fusion = 0.0;
float piston_position = 0.0;
float piston_position_last = 0.0;
double piston_volume_eq = 0.0;

double piston_set_point = 0.0;

double coeff_A = 0.0;
double coeff_B = 0.0;
double tick_to_volume = 0.0;

double g_rho_bar = 0.0;

ros::Time time_piston_data, time_depth_data, time_last_predict;

bool new_depth_data = false;
bool new_piston_data = false;

double forecast_dt_regulation = 0.0;
double forecast_filter = 0.7;

bool seafloor_detected = false;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position_last = piston_position;

  time_piston_data = msg->stamp;
  piston_position = msg->position;
  piston_set_point = msg->position_set_point;
  new_piston_data = true;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
  velocity_fusion = msg->velocity;
  time_depth_data = msg->stamp;
  new_depth_data = true;
}

void regulation_loop_callback(const seabot_piston_driver::RegulationLoopDuration::ConstPtr& msg){
  forecast_dt_regulation = (1.-forecast_filter)* forecast_dt_regulation + forecast_filter*(msg->dt.toSec());
}

void safety_callback(const seabot_safety::SafetyLog::ConstPtr& msg){
  seafloor_detected = msg->seafloor;
}

Matrix<double,NB_STATES, 1> f(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_COMMAND, 1> &u){
  Matrix<double,NB_STATES, 1> dx = Matrix<double,NB_STATES, 1>::Zero();
  dx(0) = -coeff_A*(u(0)+x(2)-(x(3)*x(1)+x(4)*pow(x(1),2)))-coeff_B*x(5)*copysign(x(0)*x(0), x(0));
  dx(1) = x(0);
  dx(2) = 0.0;
  dx(3) = 0.0;
  dx(4) = 0.0;
  dx(5) = 0.0;
  return dx;
}

void kalman_predict(Matrix<double,NB_STATES, 1> &x,
                    Matrix<double,NB_STATES, NB_STATES> &gamma,
                    const Matrix<double,NB_COMMAND, 1> &u,
                    const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                    const double &dt){
  if(dt <= 0.0 || dt >= 1.0){
//    ROS_INFO("[Kalman] dt issue %f", dt);
    return;
  }

  Matrix<double, NB_STATES, NB_STATES> Ak_tmp = Matrix<double, NB_STATES, NB_STATES>::Identity();
  Matrix<double,NB_STATES, NB_STATES> Ak = Matrix<double, NB_STATES, NB_STATES>::Zero();
  Ak(0,0) = -2.*coeff_B*abs(x(0))*x(5);
  Ak(0,1) = coeff_A*(x(3)+2.*x(4)*x(1));
  Ak(0,2) = -coeff_A;
  Ak(0,3) = x(1)*coeff_A;
  Ak(0,4) = pow(x(1),2)*coeff_A;
  Ak(0,5) = -coeff_B*abs(x(0))*x(0);
  Ak(1, 0) = 1.;
  Ak_tmp += Ak*dt;

  gamma = Ak_tmp*gamma*Ak_tmp.transpose()+gamma_alpha*sqrt(dt); // Variance estimatation
  x += f(x, u)*dt;  // New State estimation
}

void kalman_correc(Matrix<double,NB_STATES, 1> &x,
                   Matrix<double,NB_STATES,NB_STATES> &gamma,
                   const Matrix<double,NB_MESURES, 1> &y,
                   const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
                   const Matrix<double,NB_MESURES, NB_STATES> &Ck){
  const Matrix<double,NB_MESURES,NB_MESURES> S = Ck * gamma * Ck.transpose() + gamma_beta;
  const Matrix<double,NB_STATES, NB_MESURES> K = gamma * Ck.transpose() * S.inverse();
  const Matrix<double,NB_MESURES, 1> ztilde = y - Ck*x;

  const Matrix<double,NB_STATES,NB_STATES> Id = Matrix<double,NB_STATES,NB_STATES>::Identity();
  const Matrix<double,NB_STATES,NB_STATES> tmp = Id - K*Ck;

//    gamma = ((tmp*gamma)*(gamma.transpose())*tmp.transpose()).sqrt();
  gamma = tmp*gamma;
  x += K*ztilde;
}

void kalman(Matrix<double,NB_STATES, 1> &x,
            Matrix<double,NB_STATES,NB_STATES> &gamma,
            const Matrix<double,NB_COMMAND, 1> &u,
            const Matrix<double,NB_MESURES, 1> &y,
            const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
            const Matrix<double,NB_MESURES,NB_MESURES> &gamma_beta,
            const Matrix<double,NB_MESURES, NB_STATES> &Ck,
            const double &dt
            ){
  kalman_correc(x, gamma, y, gamma_beta, Ck);
  kalman_predict(x, gamma, u, gamma_alpha, dt);
}

void init_gamma(Matrix<double,NB_STATES,NB_STATES> &gamma, const double &gamma_init_velocity, const double &gamma_init_depth,
                const double &gamma_init_offset, const double &gamma_init_chi, const double &gamma_init_chi2, const double &gamma_init_cz){
  gamma = Matrix<double,NB_STATES,NB_STATES>::Zero();
  gamma(0,0) = pow(gamma_init_velocity, 2); // velocity
  gamma(1,1) = pow(gamma_init_depth, 2); // Depth
  gamma(2,2) = pow(gamma_init_offset, 2); // Error offset;
  gamma(3,3) = pow(gamma_init_chi,2); // Compressibility
  gamma(4,4) = pow(gamma_init_chi2,2); // Compressibility 2
  gamma(5,5) = pow(gamma_init_cz,2); // Cz
}

void init_xhat(Matrix<double, NB_STATES, 1> &xhat ){
  xhat(0) = velocity_fusion;
  xhat(1) = depth;
  xhat(2) = piston_volume_eq; // Vp
  xhat(3) = 15.0*tick_to_volume; // chi
  xhat(4) = 0.*tick_to_volume; // chi2
  xhat(5) = 1.0; // Cz
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "kalman");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 25.0);

  const double rho = n_private.param<double>("rho", 1020.0);
  const double g = n_private.param<double>("g", 9.81);
  const double m = n_private.param<double>("m", 18.0);
  const double diam_collerette = n_private.param<double>("diam_collerette", 0.24);
  const double screw_thread = n_private.param<double>("screw_thread", 1.75e-3);
  const double tick_per_turn = n_private.param<double>("tick_per_turn", 48);
  const double piston_diameter = n_private.param<double>("piston_diameter", 0.05);
  tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
  piston_volume_eq = n_private.param<double>("piston_ref_eq", 2100.0)*tick_to_volume;

  const double piston_ticks_max_value = n_private.param<double>("piston_max_value", 2400.0);
  const double piston_volume_max = piston_ticks_max_value*tick_to_volume;

  const double enable_depth = n_private.param<double>("enable_depth", 0.5);

  const double gamma_alpha_velocity = n_private.param<double>("gamma_alpha_velocity", 1e-3); // 1e-5
  const double gamma_alpha_depth = n_private.param<double>("gamma_alpha_depth", 1e-5); // 1e-5
  const double gamma_alpha_offset = n_private.param<double>("gamma_alpha_offset", 5e-2)*tick_to_volume; // 2e-5
  const double gamma_alpha_chi = n_private.param<double>("gamma_alpha_chi", 1e-3)*tick_to_volume; // 2e-8
  const double gamma_alpha_chi2 = n_private.param<double>("gamma_alpha_chi2", 1e-3)*tick_to_volume; // 2e-8
  const double gamma_alpha_cz = n_private.param<double>("gamma_alpha_cz", 1e-3);

  const double gamma_init_velocity = n_private.param<double>("gamma_init_velocity", 1e-1);
  const double gamma_init_depth = n_private.param<double>("gamma_init_depth", 1.0e-2);
  const double gamma_init_offset = n_private.param<double>("gamma_init_offset", piston_ticks_max_value)*tick_to_volume; // 1e-2
  const double gamma_init_chi = n_private.param<double>("gamma_init_chi", 30.0)*tick_to_volume; // 20
  const double gamma_init_chi2 = n_private.param<double>("gamma_init_chi2", 30.0)*tick_to_volume; // 1e-1
  const double gamma_init_cz = n_private.param<double>("gamma_init_cz", 0.1);

  const double gamma_beta_depth = n_private.param<double>("gamma_beta_depth", 1.0e-3); // 5e-4

  g_rho_bar = g*rho/1e5;

  coeff_A = g*rho/m;
  const double Cf = M_PI*pow(diam_collerette/2.0, 2);
  coeff_B = 0.5*rho*Cf/m;

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 10, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 10, piston_callback);
  ros::Subscriber regulation_loop_sub = n.subscribe("/driver/piston/regulation_loop", 10, regulation_loop_callback);
  ros::Subscriber safety_sub = n.subscribe("/safety/safety", 10, safety_callback);

  // Publisher
  ros::Publisher kalman_pub = n.advertise<seabot_fusion::Kalman>("kalman", 1);
  seabot_fusion::Kalman msg;

  // Loop variables
  // Line, Row
  Matrix<double, NB_STATES, NB_STATES> gamma_alpha = Matrix<double, NB_STATES, NB_STATES>::Zero();
  Matrix<double, NB_MESURES, NB_MESURES> gamma_beta = Matrix<double, NB_MESURES, NB_MESURES>::Zero();
  Matrix<double, NB_MESURES, NB_STATES> Ck = Matrix<double, NB_MESURES, NB_STATES>::Zero();

  Matrix<double, NB_STATES, 1> xhat = Matrix<double, NB_STATES, 1>::Zero();
  Matrix<double,NB_STATES,NB_STATES> gamma = Matrix<double,NB_STATES,NB_STATES>::Zero();
  Matrix<double,NB_STATES, 1> x_forcast(xhat);
  Matrix<double,NB_STATES, NB_STATES> gamma_forcast = Matrix<double,NB_STATES,NB_STATES>::Zero();

  init_gamma(gamma, gamma_init_velocity, gamma_init_depth, gamma_init_offset, gamma_init_chi, gamma_init_chi2, gamma_init_cz);

  gamma_alpha(0,0) = pow(gamma_alpha_velocity, 2); // Velocity
  gamma_alpha(1,1) = pow(gamma_alpha_depth, 2); // Depth
  gamma_alpha(2,2) = pow(gamma_alpha_offset, 2); // Offset
  gamma_alpha(3,3) = pow(gamma_alpha_chi, 2); // Compressibility
  gamma_alpha(4,4) = pow(gamma_alpha_chi2, 2); // Compressibility 2
  gamma_alpha(5,5) = pow(gamma_alpha_cz, 2); // cz

  gamma_beta(0, 0) = pow(gamma_beta_depth, 2); // Depth

  Ck(0, 1) = 1.;

  init_xhat(xhat);

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
      if(depth>enable_depth && !seafloor_detected){

        double dt;
        y(0) = depth;

        if(new_depth_data && new_piston_data){
          if(time_piston_data>time_depth_data){
            if(time_last_predict.toSec()<=0)
              time_last_predict = time_depth_data;

            u(0) = -piston_position_last*tick_to_volume; // u
            dt = (time_depth_data - time_last_predict).toSec();
            kalman_predict(xhat, gamma, u, gamma_alpha, dt); // predict up to new depth

            kalman_correc(xhat, gamma, y, gamma_beta, Ck);

            u(0) = -piston_position*tick_to_volume; // u
            dt = (time_piston_data - time_depth_data).toSec();
            kalman_predict(xhat, gamma, u, gamma_alpha, dt);
            time_last_predict = time_piston_data;
          }
          else{
            if(time_last_predict.toSec()<=0)
              time_last_predict = time_piston_data;

            u(0) = -piston_position_last*tick_to_volume; // u
            dt = (time_piston_data - time_last_predict).toSec();
            kalman_predict(xhat, gamma, u, gamma_alpha, dt);

            u(0) = -piston_position*tick_to_volume; // u
            dt = (time_depth_data-time_piston_data).toSec();

            kalman_predict(xhat, gamma, u, gamma_alpha, dt);
            time_last_predict = time_depth_data;

            kalman_correc(xhat, gamma, y, gamma_beta, Ck);
          }
        }
        else if(new_depth_data){
          if(time_last_predict.toSec()<=0)
            time_last_predict = time_depth_data;

          u(0) = -piston_position*tick_to_volume; // u
          dt = (time_depth_data - time_last_predict).toSec();
          kalman_predict(xhat, gamma, u, gamma_alpha, dt);
          kalman_correc(xhat, gamma, y, gamma_beta, Ck);
          time_last_predict = time_depth_data;
        }
        else if(new_piston_data){
          if(time_last_predict.toSec()<=0)
            time_last_predict = time_piston_data;
          u(0) = -piston_position_last*tick_to_volume; // u
          dt = (time_piston_data - time_last_predict).toSec();
          kalman_predict(xhat, gamma, u, gamma_alpha, dt);
          time_last_predict = time_piston_data;
        }

        if(new_depth_data){
          // Forecast
          x_forcast = xhat;
          gamma_forcast = gamma;
          if(forecast_dt_regulation != 0.0){
            u(0) = -piston_position*tick_to_volume;
            kalman_predict(x_forcast, gamma_forcast, u, gamma_alpha, forecast_dt_regulation);
          }

          msg.valid = true;
          msg.forecast_dt_regulation = forecast_dt_regulation;

          // Case Divergence of Kalman filter
//          double equilibrium_volume = xhat(2)-(xhat(3)*xhat(1)+xhat(4)*pow(xhat(1),2));
//          if(abs(equilibrium_volume)>piston_volume_max || abs(gamma(0,0))>100.0*gamma_init_velocity){
//            init_gamma(gamma, gamma_init_velocity, gamma_init_depth, gamma_init_offset, gamma_init_chi, gamma_init_chi2, gamma_init_cz);
//            msg.valid = false;
//            ROS_INFO("[Kalman] Divergence of Kalman filter");
//          }

          if(!xhat.allFinite()){
            init_gamma(gamma, gamma_init_velocity, gamma_init_depth, gamma_init_offset, gamma_init_chi, gamma_init_chi2, gamma_init_cz);
            init_xhat(xhat);
          }
        }
      }
      else{
        if(new_depth_data){
          xhat(0) = velocity_fusion;
          xhat(1) = depth;
          x_forcast = xhat;
          gamma_forcast = gamma;
          msg.valid = false;
        }
      }

      if(new_depth_data){
        msg.velocity = x_forcast(0);
        msg.depth = x_forcast(1);
        msg.offset = x_forcast(2);
        msg.chi = x_forcast(3);
        msg.offset_total = x_forcast(2)+x_forcast(3)*x_forcast(1);
        msg.chi2 = x_forcast(4);
        msg.cz = x_forcast(5);
        msg.offset_total = x_forcast(2)+x_forcast(3)*x_forcast(1) + x_forcast(4)*pow(x_forcast(1),2);
        msg.stamp = time_depth_data;

        msg.variance[0] = gamma_forcast(0,0);
        msg.variance[1] = gamma_forcast(1,1);
        msg.variance[2] = gamma_forcast(2,2);
        msg.variance[3] = gamma_forcast(3,3);
        msg.variance[4] = gamma_forcast(4,4);
        msg.variance[5] = gamma_forcast(5,5);

        kalman_pub.publish(msg);
      }

      new_piston_data = false;
      new_depth_data = false;
    }

    loop_rate.sleep();
  }

  return 0;
}

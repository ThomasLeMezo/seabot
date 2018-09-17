#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pressure_89bsd_driver/PressureBsdData.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_depth_regulation/RegulationDebug2.h>
#include <seabot_fusion/Kalman.h>

#include <algorithm>    // std::sort
#include <deque>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define NB_MESURES 2
#define NB_STATES 4

double depth = 0.0;
double piston_position = 0.0;
double piston_variation = 0.0;

double coeff_A = 0.0;
double coeff_B = 0.0;
double tick_to_volume = 0.0;
double coeff_compressibility = 0.0;

ros::Time time_last_depth;
bool depth_valid = false;

void piston_callback(const seabot_piston_driver::PistonState::ConstPtr& msg){
  piston_position = msg->position;
}

void depth_callback(const seabot_fusion::DepthPose::ConstPtr& msg){
  depth = msg->depth;
  depth_valid = true;
  time_last_depth = ros::Time::now();
}

void regulation_callback(const seabot_depth_regulation::RegulationDebug2::ConstPtr& msg){
  if(!msg->antiwindup)
    piston_variation = msg->u;
  else
    piston_variation = 0.0;
}

Matrix<double,NB_STATES, 1> f(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_STATES, 1> &u){
  Matrix<double,NB_STATES, 1> y;
  y(0) = -coeff_A*(x(2)+x(3)-coeff_compressibility*x(1))-coeff_B*copysign(x(0)*x(0), x(0));
  y(1) = x(0);
  y(2) = u(2);
  y(3) = 0.0;
//  ROS_INFO("y(0) = %f, y(1) = %f, y(2) = %.4e, y(3) = %.4e", y(0), y(1), y(2), y(3));
//  ROS_INFO("x(0) = %f, x(1) = %f, x(2) = %.4e, x(3) = %.4e", x(0), x(1), x(2), x(3));
  return y;
}

void kalman_predict(const Matrix<double,NB_STATES, 1> &xup,
                    const Matrix<double,NB_STATES, NB_STATES> &Gup,
                    const Matrix<double,NB_STATES, 1> &u,
                    const Matrix<double,NB_STATES, NB_STATES> &gamma_alpha,
                    const Matrix<double,NB_STATES, NB_STATES> &Ak,
                    const double &dt,
                    Matrix<double,NB_STATES, 1> &xnew,
                    Matrix<double,NB_STATES, NB_STATES> &gamma){
  gamma = Ak*Gup*Ak.transpose()+gamma_alpha;
  xnew = xup + f(xup, u)*dt;
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
            const Matrix<double,NB_STATES, 1> &u,
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

void euler(Matrix<double,NB_STATES, 1> &x, const Matrix<double,NB_STATES, 1> &u, double dt){
  Matrix<double,NB_STATES, 1> y;
  y(0) = -coeff_A*(x(2)+x(3)-coeff_compressibility*x(1))-coeff_B*copysign(x(0)*x(0), x(0));
  y(1) = x(0);
  y(2) = u(2);
  y(3) = 0.0;
  x += y*dt;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "kalman");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 5.0);

  const double rho = n_private.param<double>("rho", 1025.0);
  const double g = n_private.param<double>("g", 9.81);
  const double m = n_private.param<double>("m", 8.800);
  const double diam_collerette = n_private.param<double>("diam_collerette", 0.24);
  const double compressibility_tick = n_private.param<double>("compressibility_tick", 20.0);
  const double screw_thread = n_private.param<double>("screw_thread", 1.75e-3);
  const double tick_per_turn = n_private.param<double>("tick_per_turn", 48);
  const double piston_diameter = n_private.param<double>("piston_diameter", 0.05);
  const double piston_ref_eq = n_private.param<double>("piston_ref_eq", 2100);

  coeff_A = g*rho/m;
  const double Cf = M_PI*pow(diam_collerette/2.0, 2);
  coeff_B = 0.5*rho*Cf/m;
  tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
  coeff_compressibility = compressibility_tick*tick_to_volume*0.;

  //  ROS_INFO("tick_to_volume = %.10e", tick_to_volume);
  ROS_INFO("Coeff_A %.10e", coeff_A);
  ROS_INFO("Coeff_B %.10e", coeff_B);
  ROS_INFO("tick_to_volume %.10e", tick_to_volume);
  ROS_INFO("coeff_compressibility %.10e", coeff_compressibility);

  // Subscriber
  ros::Subscriber depth_sub = n.subscribe("/fusion/depth", 1, depth_callback);
  ros::Subscriber state_sub = n.subscribe("/driver/piston/state", 1, piston_callback);
  ros::Subscriber regulator_sub = n.subscribe("/regulation/debug", 1, regulation_callback);

  // Publisher
  ros::Publisher kalman_pub = n.advertise<seabot_fusion::Kalman>("kalman", 1);
  seabot_fusion::Kalman msg;

  // Loop variables
  // Line, Row
  Matrix<double, NB_STATES, NB_STATES> gamma_alpha;
  Matrix<double, NB_MESURES, NB_MESURES> gamma_beta;
  Matrix<double, NB_STATES, NB_STATES> Ak;
  Matrix<double, NB_MESURES, NB_STATES> Ck;

  Matrix<double, NB_STATES, 1> xhat;
  Matrix<double,NB_STATES,NB_STATES> gamma;

  gamma(0,0) = pow(1.0, 2); // velocity
  gamma(1,1) = pow(25.0, 2); // Depth
  gamma(2,2) = pow(1e-3, 2); // Piston Volume (m3)
  gamma(3,3) = pow(1e-3, 2); // Error offset;

  gamma_alpha(0,0) = pow(1e-2, 2); // velocity
  gamma_alpha(1,1) = pow(1e-3, 2); // Depth
  gamma_alpha(2,2) = pow(1e-5, 2); // Piston Volume (m3)
  gamma_alpha(3,3) = pow(1e-8, 2); // Error offset;

  gamma_beta(0, 0) = pow(1e-3, 2); // Depth
  gamma_beta(1, 1) = pow(3e-10, 2); // Piston Volume

  Ak(0, 0) = -2.*coeff_B*abs(xhat(0));
  Ak(0, 1) = coeff_A*coeff_compressibility; // To Check
  Ak(0, 2) = -coeff_A;
  Ak(0, 3) = -coeff_A;
  Ak(1, 0) = 1.;

  Ck(0, 1) = 1.;
  Ck(1, 2) = 1.;

  xhat(0) = 0.0;
  xhat(1) = 0.0;
  xhat(2) = 0.0;
  xhat(3) = 20*tick_to_volume;

  Matrix<double,NB_MESURES, 1> measure;
  Matrix<double,NB_STATES, 1> command;

  double dt = 1./frequency;
  ros::Time t_last, t;
  t_last = ros::Time::now();

  Matrix<double,NB_STATES, 1> x;
  x(1) = 10.0;
  x(0) = 0.0;
  Matrix<double,NB_STATES, 1> u;

  ROS_INFO("[FUSION depth] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
//    ros::spinOnce();

    /// ******************** Test Euler ********************
//    u(2) = sin(t.toSec()/100.)*100.*tick_to_volume;
//    u(2) = 1.0*tick_to_volume*sin(t.toSec()/100.);
//    u(2) = 1.0*tick_to_volume;
//    euler(x, u, dt);
//    Ak(0,0) = -2.*coeff_B*abs(x(0));
//    measure(0) = x(1);
//    measure(1) = x(2)+20*tick_to_volume;
//    command(2) = u(2);

//    kalman(xhat,gamma,command,measure,gamma_alpha,gamma_beta,Ak,Ck, dt);
//    ROS_INFO("Veq = %.5e, Veq_hat = %.5e, gamma(3,3)=%5.e", x(2), xhat(2)+xhat(3), gamma(3,3));
    /// ******************** Test Euler ********************

    t = ros::Time::now();
    dt = (t-t_last).toSec();
    t_last = t;

    if(depth>0.5 && (t-time_last_depth).toSec()<0.1 && depth_valid){
      Ak(0,0) = -2.*coeff_B*abs(xhat(0));
      measure(0) = depth;
      measure(1) = (piston_ref_eq - piston_position)*tick_to_volume;
      command(2) = -piston_variation*tick_to_volume;

      kalman(xhat,gamma,command,measure,gamma_alpha,gamma_beta,Ak,Ck, dt);
      depth_valid = false;
    }
//    else{
//      xhat(0) = 0.0;
//      xhat(1) = depth;
//      xhat(2) = (piston_ref_eq - piston_position)*tick_to_volume;
//      xhat(3) = xhat(3);
//      ROS_INFO("No Message or depth<=0.5");
//    }

    msg.velocity = xhat(0);
    msg.depth = xhat(1);
    msg.volume = xhat(2);
    msg.offset = xhat(3);

    msg.covariance[0] = gamma(0,0);
//    msg.covariance[1] = gamma(1,0);
//    msg.covariance[2] = gamma(2,0);
//    msg.covariance[3] = gamma(3,0);
//    msg.covariance[4] = gamma(0,1);
    msg.covariance[5] = gamma(1,1);
//    msg.covariance[6] = gamma(2,1);
//    msg.covariance[7] = gamma(3,1);
//    msg.covariance[8] = gamma(0,2);
//    msg.covariance[9] = gamma(1,2);
    msg.covariance[10] = gamma(2,2);
//    msg.covariance[11] = gamma(3,2);
//    msg.covariance[12] = gamma(0,3);
//    msg.covariance[13] = gamma(1,3);
//    msg.covariance[14] = gamma(2,3);
    msg.covariance[15] = gamma(3,3);

    kalman_pub.publish(msg);

    loop_rate.sleep();
  }

  return 0;
}

#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <pressure_89bsd_driver/PressureBsdData.h>
#include <seabot_piston_driver/PistonState.h>
#include <seabot_piston_driver/PistonPosition.h>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

#define NB_MESURES 2
#define NB_STATES 4

double coeff_A = 0.0;
double coeff_B = 0.0;
double coeff_compressibility = 0.0;
double piston_set_point = 0.0;

inline Matrix<double,NB_STATES, 1> f(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_STATES, 1> &u){
  Matrix<double,NB_STATES, 1> y;
  y(0) = -coeff_A*(x(2)+x(3)-coeff_compressibility*x(1))-coeff_B*copysign(x(0)*x(0), x(0));
  y(1) = x(0);
  y(2) = u(2);
  y(3) = 0.0;
  return y;
}

inline void euler(Matrix<double,NB_STATES, 1> &x, const Matrix<double,NB_STATES, 1> &u, double dt){
  x += f(x, u)*dt;
}

void position_callback(const seabot_piston_driver::PistonPosition::ConstPtr& msg){
    piston_set_point = msg->position;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "simulator");
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

  const double piston_offset = n_private.param<double>("piston_offset", 31.5);

  coeff_A = g*rho/m;
  const double Cf = M_PI*pow(diam_collerette/2.0, 2);
  coeff_B = 0.5*rho*Cf/m;
  const double tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
  coeff_compressibility = compressibility_tick*tick_to_volume*0.;

  //  ROS_INFO("tick_to_volume = %.10e", tick_to_volume);
  ROS_INFO("Coeff_A %.10e", coeff_A);
  ROS_INFO("Coeff_B %.10e", coeff_B);
  ROS_INFO("tick_to_volume %.10e", tick_to_volume);
  ROS_INFO("coeff_compressibility %.10e", coeff_compressibility);

  // Subscriber
  ros::Subscriber piston_position_sub = n.subscribe("/driver/piston/position", 1, position_callback);

  // Publisher
  ros::Publisher pressure_pub = n.advertise<pressure_89bsd_driver::PressureBsdData>("/driver/sensor_external", 1);
  ros::Publisher state_pub = n.advertise<seabot_piston_driver::PistonState>("/driver/piston/state", 1);

  seabot_piston_driver::PistonState state_msg;
  pressure_89bsd_driver::PressureBsdData pressure_msg;

  // Loop variables
  double dt = 1./frequency;
  ros::Time t_last, t;
  t_last = ros::Time::now();

  Matrix<double,NB_STATES, 1> x, u;
  x(1) = 0.0;
  x(0) = 0.0;
  u(2) = 0.0;
  piston_set_point = piston_ref_eq;

  ros::Rate loop_rate(frequency);
  while (ros::ok()){
//    ros::spinOnce();

    u(2) = x(3)-(piston_ref_eq + piston_offset - piston_set_point)*tick_to_volume;
    euler(x, u, dt);

    pressure_msg.pressure = x(1)*rho*g/1e5;
    pressure_msg.temperature = 20.0;

    pressure_pub.publish(pressure_msg);

    state_msg.position = (x(2)/tick_to_volume+piston_ref_eq+piston_offset);

    state_pub.publish(state_msg);

    loop_rate.sleep();
  }

  return 0;
}

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "vibes.h"

using namespace std;
using namespace Eigen;

#define NB_MESURES 2
#define NB_STATES 4

const double rho =  1025.0;
const double g =  9.81;
const double m =  8.800;
const double diam_collerette =  0.24;
const double compressibility_tick =  20.0;
const double screw_thread =  1.75e-3;
const double tick_per_turn =  48;
const double piston_diameter =  0.05;
const double piston_ref_eq =  2100;

double depth = 0.0;
double piston_position = 0.0;
double piston_variation = 0.0;

const double coeff_A = g*rho/m;
const double Cf = M_PI*pow(diam_collerette/2., 2);
const double coeff_B = 0.5*rho*Cf/m;
const double tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
const double coeff_compressibility = compressibility_tick*tick_to_volume;


Matrix<double,NB_STATES, 1> f(const Matrix<double,NB_STATES,1> &x, const Matrix<double,NB_STATES, 1> &u){
  Matrix<double,NB_STATES, 1> y = Matrix<double,NB_STATES, 1>::Zero();
  y(0) = -coeff_A*(x(2)+x(3)-coeff_compressibility*x(1))-coeff_B*copysign(x(0)*x(0), x(0));
  y(1) = x(0);
  y(2) = u(2);
  y(3) = 0.0;
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
  x += f(x, u)*dt;
}

void draw_result(const string &name, const vector<double> &x, const vector<double> &y,
                 const double &width, const double &height, const double &x_pos=0., const double &y_pos=0.){
  vibes::newFigure(name);
  vibes::setFigureProperties(vibesParams("figure", name,
                          "x", x_pos,
                          "y", y_pos,
                          "width", width,
                          "height", height));
  vibes::drawLine(x, y, "b");
  vibes::axisAuto();
}

void draw_result(const string &name, const vector<double> &x, const vector<double> &y, const vector<double> &y2,
                 const double &width, const double &height, const double &x_pos=0., const double &y_pos=0.){
  vibes::newFigure(name);
  vibes::setFigureProperties(vibesParams("figure", name,
                          "x", x_pos,
                          "y", y_pos,
                          "width", width,
                          "height", height));
  vibes::drawLine(x, y, "b");
  vibes::drawLine(x, y2, "r");
  vibes::axisAuto();
}

double compute_u(const Matrix<double, NB_STATES, 1> &x, double set_point){
  double beta = -0.03*M_PI/2.0;
  double l = 10.;

  double e=set_point-x(1);
  double v_eq = x(2)+x(3);
  double dx1 = -coeff_A*(v_eq-coeff_compressibility*x(1))-coeff_B*abs(x(0))*x(0);
  double y = x(0) + beta * atan(e);
  double e2_1 = 1.+pow(e,2);
  double dy = dx1-beta*x(0)/e2_1;

  return (1./coeff_A)*(coeff_A*coeff_compressibility*x(0)-2.*coeff_B*dx1*abs(x(0)) -beta*(2.*pow(x(0), 2)*e + dx1*e2_1)/pow(e2_1,2) +l*dy+y);
}

int main(int argc, char *argv[]){

  double frequency = 30.;
  double dt = 1./frequency;

  // Loop variables
  // Line, Row
  Matrix<double, NB_STATES, NB_STATES> gamma_alpha = Matrix<double, NB_STATES, NB_STATES>::Zero();
  Matrix<double, NB_MESURES, NB_MESURES> gamma_beta = Matrix<double, NB_MESURES, NB_MESURES>::Zero();
  Matrix<double, NB_STATES, NB_STATES> Ak = Matrix<double, NB_STATES, NB_STATES>::Zero();
  Matrix<double, NB_MESURES, NB_STATES> Ck = Matrix<double, NB_MESURES, NB_STATES>::Zero();

  Matrix<double, NB_STATES, 1> xhat = Matrix<double, NB_STATES, 1>::Zero();
  Matrix<double,NB_STATES,NB_STATES> gamma = Matrix<double,NB_STATES,NB_STATES>::Zero();

  gamma(0,0) = pow(1.0, 2); // velocity
  gamma(1,1) = pow(25.0, 2); // Depth
  gamma(2,2) = pow(1e-3, 2); // Piston Volume (m3)
  gamma(3,3) = pow(1e-3, 2); // Error offset;

  gamma_alpha(0,0) = pow(1e-2, 2); // velocity
  gamma_alpha(1,1) = pow(1e-3, 2); // Depth
  gamma_alpha(2,2) = pow(1e-5, 2); // Piston Volume (m3)
  gamma_alpha(3,3) = pow(1e-8, 2); // Error offset (m3);

  gamma_beta(0, 0) = pow(1e-3, 2); // Depth (m)
  gamma_beta(1, 1) = pow(tick_to_volume/10., 2); // Piston Volume (m3)

  Ak(0, 0) = 0.0;
  Ak(0, 1) = coeff_A*coeff_compressibility; // To Check
  Ak(0, 2) = -coeff_A;
  Ak(0, 3) = -coeff_A;
  Ak(1, 0) = 1.;

  Ck(0, 1) = 1.;
  Ck(1, 2) = 1.;

  xhat(0) = 0.0;
  xhat(1) = 0.0;
  xhat(2) = 0.0;
  xhat(3) = 20.0*tick_to_volume;

  Matrix<double,NB_MESURES, 1> measure = Matrix<double,NB_MESURES, 1>::Zero();
  Matrix<double,NB_STATES, 1> command = Matrix<double,NB_STATES, 1>::Zero();

  Matrix<double,NB_STATES, 1> x = Matrix<double,NB_STATES, 1>::Zero();
  x(1) = 0.0;
  x(0) = 0.0;
  x(2) = 0.0;
  Matrix<double,NB_STATES, 1> u = Matrix<double,NB_STATES, 1>::Zero();

  vibes::beginDrawing();
  array<vector<double>, 4> x_list, xhat_list, cov_list;
  vector<double> time;

  double energy;
  vector<double> energy_list, command_list;

  double set_point = 5.;

  for(double t=0.0; t<200.; t+=dt){

    if(t>100)
      set_point = 3.;

//    u(2) = 1.0e-6*sin(t/30.0);
    u(2) = compute_u(xhat, set_point);

    euler(x, u, dt);

    Ak(0,0) = -2.*coeff_B*abs(x(0));
    measure(0) = x(1); // Position
    measure(1) = x(2)+100.*tick_to_volume; // Volume
    command(2) = u(2);

    Matrix<double, NB_STATES, NB_STATES> Ak_tmp = Ak*dt+Matrix<double, NB_STATES, NB_STATES>::Identity();
    kalman(xhat,gamma,command,measure,gamma_alpha,gamma_beta,Ak_tmp,Ck, dt);

    x_list[0].push_back(x(0));
    x_list[1].push_back(x(1));
    x_list[2].push_back(x(2));
    x_list[3].push_back(x(3));
    xhat_list[0].push_back(xhat(0));
    xhat_list[1].push_back(xhat(1));
    xhat_list[2].push_back(xhat(2)+xhat(3));
    xhat_list[3].push_back(xhat(3));
    cov_list[0].push_back(gamma(0, 0));
    cov_list[1].push_back(gamma(1, 1));
    cov_list[2].push_back(gamma(2, 2));
    cov_list[3].push_back(gamma(3, 3));
    time.push_back(t);

    energy+= coeff_B*pow(x(0), 4);
    energy_list.push_back(energy);
    command_list.push_back(u(2));
  }


  double width = 400;
  double height = 300;
  draw_result("Position", time, x_list[1], xhat_list[1], width, height, 0, 0);
  draw_result("Velocity", time, x_list[0], xhat_list[0], width, height, width+100, 0);
  draw_result("Command", time, command_list, width, height, 2*width+150, 0);

  draw_result("Volume", time, x_list[2], xhat_list[2], width, height, 0, height+100);
  draw_result("Offset", time, xhat_list[3], width, height, width+100, height+100);
  draw_result("Energy", time, energy_list, width, height, 2*width+150, height+100);


//  draw_result("Cov V", time, cov_list[1], width, height, 2*width+100, height+100);

  cout << gamma << endl;
  cout << "offset = " << xhat_list[3][xhat_list[3].size()-1] << endl;
  cout << "x = " << x << endl;
  cout << "xhat = " << xhat << endl;
//  cout << xhat_list[0][0] << " " << xhat_list[0][xhat_list[0].size()-1] << endl;
//  cout << x_list[0][0] << " " << x_list[0][x_list[0].size()-1] << endl;

  return 0;
}

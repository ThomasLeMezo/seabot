#include "ceres/ceres.h"
#include "glog/logging.h"
#include <vector>
#include <cmath>

#include <iostream>   // std::cout
#include <string>     // std::string, std::stod

#include <string>
#include <fstream>
#include <streambuf>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

#include <boost/tokenizer.hpp>

using namespace std;

vector<double> data_x1;
vector<double> data_x1dot;
vector<double> data_x2;

double m = 6.331585468998954;
double g = 9.81;
double rho_e = 1025.0;
double C = 0.009503317777109124;
double V = 0.006177156555120931;
double tick_to_volume = 1.431715402026599e-07;

struct FlotteurResidual {
    FlotteurResidual(double x1dot, double x1, double x2)
        : x1dot_(x1dot), x1_(x1), x2_(x2) {}
    template <typename T> bool operator()(const T* const beta,
                                          T* residual) const {
        residual[0] = g-(g*(V+x2_+beta[0])-0.5*C*x1_*abs(x1_))*rho_e/m-x1dot_;
        return true;
    }
private:
    const double x1dot_;
    const double x1_;
    const double x2_;
};

void read_file(){
    std::string::size_type sz;     // alias of size_t
    typedef boost::tokenizer< boost::escaped_list_separator<char> , std::string::const_iterator, std::string> Tokenizer;
    boost::escaped_list_separator<char> seps('\\', ',', '\"');;

    std::ifstream t("/home/lemezoth/workspaceFlotteur/tools/data_dx.txt");
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    Tokenizer tok(str, seps);
    for (auto i : tok){
        if(i.length()!=0)
            data_x1dot.push_back(stod(i,&sz));
    }
    t.close();
    cout << "Read data_dx (" << data_x1dot.size() << ")" << endl;

    std::ifstream t1("/home/lemezoth/workspaceFlotteur/tools/data_x1.txt");
    std::string str1((std::istreambuf_iterator<char>(t1)),
                     std::istreambuf_iterator<char>());
    Tokenizer tok1(str1, seps);
    for (auto i : tok1){
        if(i.length()!=0)
            data_x1.push_back(stod(i,&sz));
    }
    t1.close();
    cout << "Read data_x1 (" << data_x1.size() << ")" << endl;

    std::ifstream t2("/home/lemezoth/workspaceFlotteur/tools/data_x2.txt");
    std::string str2((std::istreambuf_iterator<char>(t2)),
                     std::istreambuf_iterator<char>());
    Tokenizer tok2(str2, seps);
    for (auto i : tok2){
        if(i.length()!=0)
            data_x2.push_back(stod(i,&sz));
    }
    t2.close();
    cout << "Read data_x2 (" << data_x2.size() << ")" << endl;
}

void compute_beta(char** argv){
    google::InitGoogleLogging(argv[0]);
    double beta = 0.0;

    Problem problem;
    for (int i = 0; i < data_x1dot.size(); ++i) {
        problem.AddResidualBlock(
                    new AutoDiffCostFunction<FlotteurResidual, 1, 1>(
                        new FlotteurResidual(data_x1dot[i], data_x1[i], data_x2[i])),
                    NULL,
                    &beta);
    }
    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    //  options.max_solver_time_in_seconds = 1.0;
      options.num_threads = 2;
    options.initial_trust_region_radius = tick_to_volume*100.0;
//    options.max_trust_region_radius = tick_to_volume*1000.0;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial beta: " << 0.0 << std::endl;
    std::cout << "Final   beta: " << beta << std::endl;
    std::cout << "Tick = " << beta/tick_to_volume << std::endl;
    cout << "max region = " << options.max_trust_region_radius << endl;
}

int main(int argc, char** argv) {
    read_file();
    compute_beta(argv);
    cout << "Number data = " << data_x1.size() << endl;
}

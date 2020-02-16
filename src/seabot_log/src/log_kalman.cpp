#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_mission/Waypoint.h>
#include <seabot_fusion/Kalman_old.h>
#include <iostream>
#include <fstream>

#include "ipegenerator/figure.h"
#include "ibex_IntervalVector.h"

#include "cnpy.h"

using namespace std;

int main(int argc, char *argv[]){

  std::vector<std::string> topics;
  topics.push_back(std::string("/fusion/depth"));
  topics.push_back(std::string("/fusion/kalman"));

  rosbag::Bag bag;
  bag.open("/home/lemezoth/workspaceFlotteur/tools/rosbag/bag/cfm/2019-04-11-10-05-20_0.bag", rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<double> cov_offset_vec, cov_chi_vec, cov_time;

  double startTime = view.getBeginTime().toSec();
  cout << startTime << endl;
  int k=0;

  // ************************ GET DATA ************************
  for(rosbag::MessageInstance const m: view){
    if(m.getTopic() == "/fusion/kalman"){
      seabot_fusion::Kalman_old::ConstPtr s = m.instantiate<seabot_fusion::Kalman_old>();
      if (s != nullptr and k==0)
      {
        double t = (m.getTime().toSec()-startTime);
        double cov_offset = (s->covariance[2]);
        double cov_chi = (s->covariance[3]);
        cov_offset_vec.push_back(cov_offset);
        cov_chi_vec.push_back(cov_chi);
        cov_time.push_back(t);
      }
      k = (k+1)%20;
    }
  }
  cout << "Data Loaded " << cov_offset_vec.size() << endl;
  bag.close();

  // ************************ Write data ************************

  cnpy::npy_save("/home/lemezoth/workspaceFlotteur/tools/rosbag/bag/cfm/cov_offset.npy", cov_offset_vec);
  cnpy::npy_save("/home/lemezoth/workspaceFlotteur/tools/rosbag/bag/cfm/cov_chi.npy", cov_chi_vec);
  cnpy::npy_save("/home/lemezoth/workspaceFlotteur/tools/rosbag/bag/cfm/cov_time.npy", cov_time);


  return 0;
}

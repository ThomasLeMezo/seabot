#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <seabot_fusion/DepthPose.h>
#include <iostream>
#include <fstream>

#include "cnpy.h"

using namespace std;

int main(int argc, char *argv[]){

  std::vector<std::string> topics;
  topics.push_back(std::string("/fusion/depth"));

  rosbag::Bag bag;
  bag.open("/home/lemezoth/workspaceFlotteur/tools/rosbag/bag/guerledan/2019-10-10-15-06-02_0.bag", rosbag::bagmode::Read);

  // rosbag::View(bag)

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<std::vector<float>> depth_vec;
  std::vector<double> depth;
  double startTime = view.getBeginTime().toSec();
  cout << startTime << endl;

  cout << "Start Loop" << endl;
  for(rosbag::MessageInstance const m: view){
    if(m.getTopic() == "/fusion/depth"){
      seabot_fusion::DepthPose::ConstPtr s = m.instantiate<seabot_fusion::DepthPose>();
      if (s != nullptr)
      depth_vec.push_back(std::vector<float>{(float)(m.getTime().toSec()-startTime), s->depth});
      depth.push_back(s->depth);
    }
  }
  cout << "Data Loaded" << endl;

  cnpy::npy_save("/home/lemezoth/workspaceFlotteur/tools/rosbag/bag/guerledan/test.pyc", depth_vec.data(), {depth_vec[0].size(), depth_vec.size()});
  bag.close();
  return 0;
}

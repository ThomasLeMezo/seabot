#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_mission/Waypoint.h>
#include <iostream>
#include <fstream>

#include "ipegenerator/figure.h"
#include "ibex_IntervalVector.h"

using namespace std;

int main(int argc, char *argv[]){

  std::vector<std::string> topics;
  topics.push_back(std::string("/fusion/depth"));
  topics.push_back(std::string("/mission/set_point"));

  rosbag::Bag bag;
  bag.open("/home/lemezoth/Videos/thesis/flotteur/2019-11-15-12-23-39_0.bag", rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<double> depth, velocity, time_depth;
  std::vector<double> depth_set_point, time_depth_set_point;
  double startTime = view.getBeginTime().toSec();
  cout << startTime << endl;

  cout << "Start Loop" << endl;
  ibex::IntervalVector space(2);
  space[0] = ibex::Interval(-20, 80.0);
  space[1] = ibex::Interval(-0.4,0.4);

  // ************************ GET DATA ************************
  for(rosbag::MessageInstance const m: view){
    if(m.getTopic() == "/fusion/depth"){
      seabot_fusion::DepthPose::ConstPtr s = m.instantiate<seabot_fusion::DepthPose>();
      if (s != nullptr)
      {
        double t = (m.getTime().toSec()-startTime);
        time_depth.push_back(t);
        depth.push_back(s->depth);
        velocity.push_back(s->velocity);
      }
    }
    if(m.getTopic() == "/mission/set_point"){
      seabot_mission::Waypoint::ConstPtr s = m.instantiate<seabot_mission::Waypoint>();
      if (s != nullptr)
      {
        double t = (m.getTime().toSec()-startTime);
        double d = s->depth;
        time_depth_set_point.push_back(t);
        depth_set_point.push_back(d);
      }
    }
  }
  cout << "Data Loaded" << endl;
  bag.close();

  // ************************ Write data ************************

  cout << "Load file" << endl;
  ipegenerator::Figure fig("/home/lemezoth/Desktop/piston_kernel.ipe", space, 112,63);
//  fig.set_thickness_pen_factor(1.0);

  cout << "Draw set point" << endl;
  for(size_t i=0; i<depth_set_point.size(); i++){
    fig.set_color_stroke("green");
    fig.draw_symbol(depth_set_point[i], 0.0, "disk(sx)", 1.0);
  }

  cout << "Draw curve" << endl;
  fig.set_color_stroke("black");
  fig.draw_curve(depth, velocity);

  cout << "Save file" << endl;
  fig.save_ipe("/home/lemezoth/Desktop/piston_kernel_traj.ipe");

  return 0;
}

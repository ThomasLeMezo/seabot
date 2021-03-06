#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <seabot_fusion/DepthPose.h>
#include <seabot_mission/Waypoint.h>
#include <iostream>
#include <fstream>

#include "ipegenerator/figure.h"
#include "ibex_IntervalVector.h"

using namespace std;

std::vector<double> depth, velocity, time_depth;
std::vector<double> depth_set_point, time_depth_set_point;
ibex::IntervalVector frame_data = ibex::IntervalVector(2, ibex::Interval::EMPTY_SET);

void get_data()
{
  std::vector<std::string> topics;
  topics.push_back(std::string("/fusion/depth"));
  topics.push_back(std::string("/mission/set_point"));

  rosbag::Bag bag;
  bag.open("/home/lemezoth/Videos/thesis/flotteur/2019-11-15-12-23-39_0.bag", rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  double startTime = view.getBeginTime().toSec();
  cout << startTime << endl;

  cout << "Start Loop" << endl;
  frame_data = ibex::IntervalVector(2, ibex::Interval::EMPTY_SET);
  for(rosbag::MessageInstance const m: view){
    if(m.getTopic() == "/fusion/depth"){
      seabot_fusion::DepthPose::ConstPtr s = m.instantiate<seabot_fusion::DepthPose>();
      if (s != nullptr)
      {
        double t = (m.getTime().toSec()-startTime);
        double d = s->depth;
        time_depth.push_back(t);
        depth.push_back(d);
        velocity.push_back(s->velocity);
        frame_data[0] |= ibex::Interval(t);
        frame_data[1] |= ibex::Interval(d);
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
}

void example1()
{
  cout << "frame_data = " << frame_data << endl;
  frame_data[0] += ibex::Interval(0, 30);
  frame_data[1] = ibex::Interval(-0.1, 1.6);

  for(size_t step = 2; step<time_depth.size(); step+=5){
    vector<double>::const_iterator time_depth_first = time_depth.begin();
    vector<double>::const_iterator time_depth_last = time_depth.begin() + step;
    vector<double> time_depth_sub(time_depth_first, time_depth_last);
    vector<double>::const_iterator depth_first = depth.begin();
    vector<double>::const_iterator depth_last = depth.begin() + step;
    vector<double> depth_sub(depth_first, depth_last);

    ipegenerator::Figure fig(frame_data, 200, 50);

    fig.set_graduation_parameters(15.0, 240.0, 0.0, 0.25);
    fig.draw_axis("t \\text{ (in s)}", "z \\text{ (in m)}");
    fig.set_color_fill("lightgray");
    fig.set_color_type(ipegenerator::STROKE_AND_FILL);
    fig.draw_circle_radius_final(time_depth[step], depth[step], 5.0);
    fig.set_color_fill("green");
    fig.set_color_type(ipegenerator::STROKE_ONLY);
    fig.draw_curve(time_depth_set_point, depth_set_point);
    fig.set_color_fill("red");
    fig.draw_curve(time_depth_sub, depth_sub);


    std::stringstream filename;
    filename << "/home/lemezoth/Videos/thesis/traj/traj";
    filename << std::setfill('0') << std::setw(5) << step << ".ipe";
    cout << filename.str() << '\r' << endl;
    fig.save_ipe(filename.str());
  }
}

void example2()
{
  cout << "frame_data = " << frame_data << endl;
  frame_data[0] = ibex::Interval(-0.3, 1.5);
  frame_data[1] = ibex::Interval(-0.15, 0.05);

  ipegenerator::Figure fig(frame_data, 50, 50);
  fig.set_graduation_parameters(0.0, 0.5, -0.1, 0.05);
  fig.set_number_digits_axis_x(1);
  fig.set_number_digits_axis_y(2);

  fig.draw_curve(depth, velocity);
  for(size_t i=0; i<time_depth_set_point.size(); ++i)
  {
    fig.draw_symbol(depth_set_point[i], 0.0, "disk(sx)");
  }
  fig.draw_axis("z","\\dot{z}");

  std::stringstream filename;
  filename << "/home/lemezoth/Videos/thesis/traj/depth_velocity.ipe";
  fig.save_ipe(filename.str());
}

int main(int argc, char *argv[]){
  get_data();

  //  example1();
  example2();
  return 0;
}

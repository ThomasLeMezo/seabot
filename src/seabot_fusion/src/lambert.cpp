#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <gpsd_client/GPSFix.h>
#include <seabot_fusion/GnssPose.h>
#include <cmath>

#include <proj_api.h>

using namespace std;
double latitude, longitude, track;
bool new_data = false;
bool data_valid = false;
size_t nb_sample_mean,nb_sample_between_heading;

void navFix_callback(const gpsd_client::GPSFix::ConstPtr& msg){
  latitude =  msg->latitude;
  longitude =  msg->longitude;
  track = msg->track;
  data_valid = (msg->status>=msg->STATUS_MODE_2D);
  new_data = true;
}

void get_mean_pose(vector<double> &east_mem, vector<double> &north_mem, double &east_mean, double &north_mean, double &heading_mean){
  double east_mean_begin = 0;
  double east_mean_end = 0;
  double north_mean_begin = 0;
  double north_mean_end = 0;
  heading_mean = 0;

  for(size_t i=east_mem.size()-1; i>max((size_t)0,east_mem.size()-1-nb_sample_mean); --i){
    east_mean_end += east_mem[i];
    north_mean_end += north_mem[i];
  }
  east_mean_end /= nb_sample_mean;
  north_mean_end /= nb_sample_mean;

  for(size_t i=0; i<min(east_mem.size(),nb_sample_mean); ++i){
    east_mean_begin += east_mem[i];
    north_mean_begin += north_mem[i];
  }
  east_mean_begin /= nb_sample_mean;
  north_mean_begin /= nb_sample_mean;

  east_mean=east_mean_end;
  north_mean=north_mean_end;

  double heading_rad = atan2(east_mean_end-east_mean_begin, north_mean_end-north_mean_begin);
  heading_mean = (heading_rad > 0 ? heading_rad : (2*M_PI + heading_rad)) * 360. / (2.*M_PI);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lambert_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 1.0);
  nb_sample_mean = (size_t) n_private.param<int>("nb_sample_mean", 5);
  nb_sample_between_heading = (size_t) n_private.param<int>("nb_sample_between_heading", 150);
  const size_t nb_sample = nb_sample_mean+nb_sample_between_heading;

  // Init proj
  projPJ pj_lambert, pj_latlong;
  if (!(pj_lambert = pj_init_plus("+init=epsg:2154"))){
    ROS_WARN("[Lambert_node] Error Lambert \n");
    exit(1);
  }

  if (!(pj_latlong = pj_init_plus("+init=epsg:4326"))){
    ROS_WARN("[Lambert_node] Error LatLong \n");
    exit(1);
  }

  // Topics
  ros::Subscriber navFix_sub = n.subscribe("/driver/fix", 1, navFix_callback);
  ros::Publisher pose_pub = n.advertise<seabot_fusion::GnssPose>("pose", 1);
  ros::Publisher pose_mean_pub = n.advertise<seabot_fusion::GnssPose>("pose_mean", 1);

  seabot_fusion::GnssPose msg_pose;
  seabot_fusion::GnssPose msg_pose_mean;

  vector<double> east_mem, north_mem;

  ROS_INFO("[FUSION lambert] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(new_data){
      if(longitude != 0. && latitude != 0. && data_valid){
        double east = longitude*M_PI/180.0; // Longitude
        double north = latitude*M_PI/180.0; // Latitude
        pj_transform(pj_latlong, pj_lambert, 1, 1, &east, &north, nullptr);
        msg_pose.east = east;
        msg_pose.north = north;
        msg_pose.heading = track;

        pose_pub.publish(msg_pose);

        if(data_valid=false){
          east_mem.clear();
          north_mem.clear();
        }
        else{
          east_mem.push_back(east);
          north_mem.push_back(north);
          if(east_mem.size()>nb_sample){
            east_mem.erase(east_mem.begin());
            north_mem.erase(north_mem.begin());
          }
        }

        double east_mean, north_mean, heading_mean;
        get_mean_pose(east_mem, north_mem, east_mean, north_mean, heading_mean);
        msg_pose_mean.east = east_mean;
        msg_pose_mean.north = north_mean;
        msg_pose_mean.heading = heading_mean;
      }
      new_data = false;
    }

    loop_rate.sleep();
  }

  pj_free(pj_latlong);
  pj_free(pj_lambert);
  return 0;
}



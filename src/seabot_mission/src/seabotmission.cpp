#include "seabotmission.h"
#include <fstream>
#include "boost/filesystem.hpp"
#include "boost/regex.hpp"
#include <iostream>

using namespace std;
using namespace boost;

SeabotMission::SeabotMission(std::string folder_path){
    m_folder_path = folder_path;
}

void SeabotMission::compute_command(double &north, double &east, double &depth, double &ratio){
    // Test if last waypoint
    if(m_current_waypoint < m_waypoints.size()-1){
        // Test if next time
        ros::WallTime t_now = ros::WallTime::now();

        if(t_now < m_waypoints[m_current_waypoint].time_start){
            m_mission_enable = false;
            depth = 0.0;
            north = m_waypoints[m_current_waypoint].north;
            east = m_waypoints[m_current_waypoint].east;
            ratio = 0;
        }
        else{
            m_mission_enable = true;

            depth = m_waypoints[m_current_waypoint].depth;

            ros::WallTime t1 = m_waypoints[m_current_waypoint].time_start;
            ros::WallTime t2 = m_waypoints[m_current_waypoint+1].time_start;
            ratio = (t_now-t1).toSec()/(t2-t1).toSec(); // Should be between [0, 1]

            double d_north = m_waypoints[m_current_waypoint+1].north - m_waypoints[m_current_waypoint].north;
            double d_east = m_waypoints[m_current_waypoint+1].east - m_waypoints[m_current_waypoint].east;

            north = m_waypoints[m_current_waypoint].north + d_north*ratio;
            east = m_waypoints[m_current_waypoint].east + d_east*ratio;

            if(t_now>=m_waypoints[m_current_waypoint+1].time_start)
                m_current_waypoint++;
        }
    }
    else{
        depth = 0.0;
        north = m_waypoints[m_current_waypoint].north;
        east = m_waypoints[m_current_waypoint].east;
        ratio = 1;
        m_mission_enable = false;
    }
}

// Sort filename
// https://stackoverflow.com/questions/18723984/sorting-files-with-boost-filesystem
boost::regex re("(\\d+)");
boost::match_results<std::string::const_iterator> what1,what2;
template <typename T>
T st2num ( const std::string &Text ){
    std::stringstream ss(Text);
    T result;
    return ss >> result ? result : 0;
}
struct number_mission_sort{
    bool operator ()(const std::string & a,const std::string & b)
    {
        boost::regex_search(a.cbegin(), a.cend(), what1, re, boost::match_default);
        boost::regex_search(b.cbegin(), b.cend(), what2, re, boost::match_default);
        return st2num<int>(what1[1]) < st2num<int>(what2[1]);
    }
};

bool SeabotMission::load_mission(){
    ROS_INFO("File = %s", m_file_name.c_str());

    // Load waypoints
    std::vector<Waypoint> m_waypoints_load;


    // Offset (time, north, east)

}

bool SeabotMission::update_mission(){
  if(m_update_mission){
    if (!filesystem::exists(m_folder_path)){
        ROS_INFO("Path doesn't exist %s", m_folder_path.c_str());
        return false;
    }
    filesystem::directory_iterator b(m_folder_path), e;
    vector<filesystem::path> paths(b, e);

    if(!paths.empty()){
        vector<std::string> filenames;
        for(filesystem::path p:paths){
            if(p.extension() == ".mission")
                filenames.push_back(p.filename().string());
        }

        if(!filenames.empty()){
            std::sort(filenames.begin(), filenames.end(), number_mission_sort());
            ROS_INFO("Find : %s", filenames.back().c_str());

            if(filenames.back() != m_file_name){
                ROS_INFO("NEW mission file found");
                m_file_name = paths[0].parent_path().string() + "/"+ filenames.back();
                load_mission();
                return true;
            }
            else
                return false;
        }
        else
            ROS_INFO("No mission file in the directory");
    }
    else
        ROS_INFO("No file in the directory");
    return false;
  }
  else
    return false;
}

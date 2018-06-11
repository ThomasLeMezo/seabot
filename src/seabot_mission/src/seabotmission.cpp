#include "seabotmission.h"
#include <bitset>
#include <fstream>

using namespace std;

//Waypoint::Waypoint(unsigned int time, int east, int north, unsigned int depth){
//  m_time = time;
//  m_east = east;
//  m_north = north;
//  m_depth = depth;
//}

//Waypoint::Waypoint(std::ifstream &binFile){
//  deserialize(binFile);
//}

//void Waypoint::serialize(std::ofstream &binFile) const{
////  std::bitset<sizeof(T) * CHAR_BIT> bs(byte);

//  bitset<18> bit_time(m_time);
//  bitset<9> bit_depth(m_depth);
//  bitset<17> bit_east(m_east);
//  bitset<17> bit_north(m_north);

//  binFile.write((const char*)&bit_time, sizeof(bitset<18>));
//  binFile.write((const char*)&bit_depth, sizeof(bitset<9>));
//  binFile.write((const char*)&bit_east, sizeof(bitset<17>));
//  binFile.write((const char*)&bit_north, sizeof(bitset<17>));
//}

//void Waypoint::deserialize(std::ifstream &binFile){

//}


SeabotMission::SeabotMission()
{

}

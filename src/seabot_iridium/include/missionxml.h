#ifndef MISSIONXML_H
#define MISSIONXML_H

#include "logData.h"
#include <boost/property_tree/ptree.hpp>

class MissionXML
{
public:

  /**
   * @brief MissionXML
   * @param log
   */
  MissionXML(LogData &log);

  /**
   * @brief write
   * @param filename
   */
  void write(const std::string &filename) const;

private:
  LogData m_log;
//  boost::property_tree::ptree m_tree;
};

#endif // MISSIONXML_H

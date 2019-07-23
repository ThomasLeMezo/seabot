#ifndef MISSIONXML_H
#define MISSIONXML_H

#include "logData.h"

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
};

#endif // MISSIONXML_H

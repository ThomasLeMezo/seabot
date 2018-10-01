#ifndef MISSIONXML_H
#define MISSIONXML_H

#include "logtdt.h"

class MissionXML
{
public:

  /**
   * @brief MissionXML
   * @param log
   */
  MissionXML(LogTDT &log);

  /**
   * @brief write
   * @param filename
   */
  void write(const std::string &filename) const;

private:
  LogTDT m_log;
};

#endif // MISSIONXML_H

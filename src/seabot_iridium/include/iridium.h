#ifndef IRIDIUM_H
#define IRIDIUM_H

#include <string>
#include <vector>

class Iridium
{
public:
    /**
   * @brief Iridium
   */
    Iridium();

    /**
     * @brief send_and_receive_data
     * @param files
     * @param files_count
     */
    bool send_and_receive_data(std::vector<std::string> &files);

private:
    uint64_t m_imei = 300234065392110;
    std::string m_path_received = "/home/pi/";
};

#endif // IRIDIUM_H

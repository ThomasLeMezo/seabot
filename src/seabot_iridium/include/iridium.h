#ifndef IRIDIUM_H
#define IRIDIUM_H

#include <string>
#include <vector>
#include <boost/multiprecision/cpp_int.hpp>

using boost::multiprecision::cpp_int;

extern "C"{
    #include "tis.h"
}

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
    bool send_and_receive_data();

    /**
     * @brief add_log_TDT1
     * @return
     */
    bool add_log_TDT1();

    /**
     * @brief iridium_power
     * @param enable
     * @return
     */
    bool iridium_power(const bool &enable);

    /**
     * @brief enable_com
     * @param val
     */
    void enable_com(bool val);

    /**
     * @brief uart_init
     * @param fd
     * @return
     */
    int32_t uart_init();

    /**
     * @brief get_new_tdt_file
     * @return
     */
    const std::string get_new_tdt_file();

    /**
     * @brief add_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @param min
     * @param max
     */
    int add_data(cpp_int &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double&value_max);

    /**
     * @brief add_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    int add_data(cpp_int &bits, const int &nb_bit, const int &start_bit, const unsigned int &value);

public:
    double m_east = 2097151; // 2^21-1
    double m_north = 8097151; // 2^21-1 + 6e6
    double m_gnss_speed = 0.0;
    double m_gnss_heading = 0.0;
    double m_batteries[4] = {0.0, 0.0, 0.0, 0.0};

    double m_internal_pressure = 680.0;
    double m_internal_temperature = 5.0;

    unsigned int m_seabot_state = 0;
    size_t  m_current_waypoint = 0; // 0 to 255 max

private:
    uint64_t m_imei = 300234065392110;
    std::string m_path_received = "iridium/received";
    std::string m_path_received_full = "";
    std::string m_path_send = "iridium/send";
    unsigned int m_gpio_power = 5;

    int m_transmission_number_attempt = 10;
    int m_transmission_sleep_time = 30;

    std::vector<std::string> m_files_to_send;

    TIS_properties m_tis;
    int m_uart_fd = 0;

    bool m_enable_iridium = false;

    bool m_demo_mode = false;
};

int32_t uart_init(int &fd);
int32_t uart_send_data(void *serial_struct, uint8_t *data, int32_t count);
int32_t uart_receive_data(void *serial_struct, uint8_t *data, int32_t count);
int32_t uart_wait_data(void *serial_struct, uint32_t timeout);
int32_t uart_flush_TX(void *serial_struct);
int32_t uart_flush_RX(void *serial_struct);
int32_t uart_release(void *serial_struct);

inline void Iridium::enable_com(bool val){
    m_enable_iridium = val;
}

#endif // IRIDIUM_H

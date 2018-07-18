#ifndef IRIDIUM_H
#define IRIDIUM_H

#include <string>
#include <vector>

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
     * @brief add_new_log_file
     * @return
     */
    bool add_new_log_file();

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

public:
    double m_east = 0.0;
    double m_north = 0.0;

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

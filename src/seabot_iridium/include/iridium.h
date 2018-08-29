#ifndef IRIDIUM_H
#define IRIDIUM_H

#include <string>
#include <vector>
#include <boost/multiprecision/cpp_int.hpp>

using boost::multiprecision::cpp_int;

#define NB_BITS 112
typedef boost::multiprecision::number<boost::multiprecision::cpp_int_backend<NB_BITS, NB_BITS, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked, void> > uint_log1_t;

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
     * @brief serialize_log_TDT1
     * @return
     */
    bool serialize_log_TDT1();

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
     * @brief serialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @param min
     * @param max
     */
    int serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const double &value, const double &value_min, const double&value_max);

    /**
     * @brief deserialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @param value_min
     * @param value_max
     * @return
     */
    int deserialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, double &value, const double &value_min, const double &value_max);

    /**
     * @brief serialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    int serialize_data(uint_log1_t &bits, const int &nb_bit, const int &start_bit, const unsigned int &value);

    /**
     * @brief deserialize_data
     * @param bits
     * @param nb_bit
     * @param start_bit
     * @param value
     * @return
     */
    int deserialize_data(const uint_log1_t &bits, const int &nb_bit, const int &start_bit, unsigned int &value);

    /**
     * @brief is_demo_mode
     * @return
     */
    bool is_demo_mode() const;

    /**
     * @brief deserialize_log_TDT1
     * @param file_name
     */
    bool deserialize_log_TDT1(const std::string &file_name);

public:
    double m_east = 42.0; // 2^21-1
    double m_north = 6000042.0; // 2^21-1 + 6e6
    double m_gnss_speed = 4.2;
    double m_gnss_heading = 242.0;
    double m_batteries[4] = {10.0, 10.0, 10.0, 10.0};

    double m_internal_pressure = 742.0;
    double m_internal_temperature = 42.0;
    double m_internal_humidity = 0.0;

    unsigned int m_seabot_state = 0;
    unsigned int  m_current_waypoint = 42; // 0 to 255 max

    bool m_iridium_power_state = false;

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

inline bool Iridium::is_demo_mode()const{
  return m_demo_mode;
}

#endif // IRIDIUM_H

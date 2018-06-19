
#define SIZE_RX_BUFFER 8
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

/**
 * @brief initialisation de l'I2C en mode esclave
 */
void init_i2c();

/**
 * @brief interrupt_low
 */
void interrupt_low();
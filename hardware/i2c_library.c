#include "i2c_library.c"

/**
 * @brief initialisation de l'I2C en mode esclave
 */
void init_i2c(){

  // **** IO I2C **** //
  TRISB4_bit = 1; // RB4 en entrée
  TRISB6_bit = 1; // RB6 en entrée

  // **** Interruptions **** //
  PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
  PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave

  PIR2.BCLIE = 1;
  PIR2.BCLIF = 0;

  // **** ADDRESS **** //
  SSPADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
  SSPMSK = 0xFF; // A zero (‘0’) bit in the SSPMSK register has the effect of making 
                 // the corresponding bit in the SSPSR register a “don’t care”

  // **** SSPSTAT **** //
  SSPSTAT.SMP = 1; // Slew Rate Control bit
  // 1 = Slew rate control disabled for standard Speed mode (100 kHz and 1 MHz)
  // 0 = Slew rate control enabled for High-Speed mode (400 kHz)
  SSPSTAT.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)

  // **** SSPCON2 **** //
  SSPCON2 = 0x00;
  SSPCON2.GCEN = 0; // General Call Enable bit (0 = disabled)
  SSPCON2.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)

  // **** SSPCON1 **** //
  SSPCON1.WCOL = 0; // Write Collision Detect bit
  SSPCON1.SSPOV = 0; // Receive Overflow Indicator bit
  SSPCON1.CKP = 1; // SCK Release Control bit (1=Release clock)
  SSPCON1.SSPM = 0b1110; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled

  // (START the I2C Module)
  SSPCON1.SSPEN = 1; // Synchronous Serial Port Enable bit 
}

/**
 * @brief interrupt_low
 */
void interrupt_low(){
  if (PIR1.SSPIF){  // I2C Interrupt

      if(SSPCON1.SSPOV || SSPCON1.WCOL){
          SSPCON1.SSPOV = 0;
          SSPCON1.WCOL = 0;
          tmp_rx = SSPBUF;
      }            

      //****** receiving data from master ****** //
      // 0 = Write (master -> slave - reception)
      if (SSPSTAT.R_W == 0){
        if(SSPSTAT.P == 0){ 
          if (SSPSTAT.D_A == 0){ // Address
            nb_rx_octet = 0;
            tmp_rx = SSPBUF;
          }
          else{ // Data
            if(nb_rx_octet < SIZE_RX_BUFFER){
              rxbuffer_tab[nb_rx_octet] = SSPBUF;
              nb_rx_octet++;
            }
            else{
              tmp_rx = SSPBUF;
            }
          }
        }
        else{
          if(nb_rx_octet>1)
            i2c_read_data_from_buffer();
        }
      }
      //******  transmitting data to master ****** //
      // 1 = Read (slave -> master - transmission)
      else{
          if(SSPSTAT.D_A == 0){
            nb_tx_octet = 0;
            tmp_rx = SSPBUF;
          }

          // In both D_A case (transmit data after receive add)
          i2c_write_data_to_buffer(nb_tx_octet);
          nb_tx_octet++;
      }

    SSPCON1.CKP = 1;
    PIR1.SSPIF = 0; // reset SSP interrupt flag
  }
}

// D/A   P    S  R/W  UA  BF  State
//  X    0    1   0   0   0   Start
//  0    0    1   0   0   1   Rcvd write address
//  0    0    1   1   0   0   Rcvd read address (ready for first byte)
//  1    Y    1   0   0   1   Rcvd data from master
//  1    0    1   1   0   0   Ready for data to send to master
//  1    Z    1   0   0   0   Data byte sent, master responded with NAK
//  1    1    0   0   0   0   Stop
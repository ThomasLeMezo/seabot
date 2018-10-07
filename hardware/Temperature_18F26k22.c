/*  PIC18F14K22  mikroC PRO for PIC v6.4
Oscillateur interne 16MHZ (attention l'edit project ne marche pas, paramétrer 
l'oscillateur en dur)


Hardware:
  18F26K22  SOIC28
  pin 1     MCR/
  pin 2     RA0 --> LED
  pin 3     RA1
  pin 4     RA2
  pin 5     RA3
  pin 6     RA4/T0CKI
  pin 7     RA5
  pin 8     VSS alim 0V
  pin 9     OSC1
  pin 10    OSC2/RA6
  pin 11    RC0
  pin 12    RC1
  pin 13    RC2
  pin 14    RC3 ---> SCL I2C vers bus i2c

  pin 15    RC4 ---> SDA I2C vers bus i2c
  pin 16    RC5
  pin 17    RC6 ---> TX
  pin 18    RC7
  pin 19    VSS alim 0V
  pin 20    VDD Alim +5V
  pin 21    RB0/INT
  pin 22    RB1 ---> SCL2 I2C2 vers capteur TSY01
  pin 23    RB2 ---> SDA2 I2C2 vers capteur TSY01
  pin 24    RB3
  pin 25    RB4
  pin 26    RB5/PGM
  pin 27    RB6/PGC
  pin 28    RB7/PGD

*/


//sbit LED at LATA.B0; // sortie LED
sbit LED at RA0_bit; // sortie LED
#define CODE_VERSION 0x02

// I2C
#define TSYS01_ADDR 0xEE
#define SIZE_RX_BUFFER 8
const unsigned short ADDRESS_I2C = 0x45; // linux I2C Adresse
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned int coefficient_tab[8];   // tableau de coefficients de calibration

unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;
unsigned short k = 0;

void init_i2c();

unsigned short octet1;
unsigned short octet2;
unsigned short octet3;

unsigned short reset = 0;
unsigned short conversion = 0;

// State Machine
enum power_state {IDLE,RESET_TSYS01,CONVERSION_READ};
unsigned short state = IDLE;


/**
 * @brief i2c_read_data_from_buffer
 */
void i2c_read_data_from_buffer(){
    unsigned short i = 0;
    unsigned short nb_data = nb_rx_octet;

    for(i=0; i<nb_data; i++){
        switch(rxbuffer_tab[0]+i){
            case 0x1E:
                reset = 1;
                break;
            case 0x48:
                conversion = 1;
                octet1 = 0x00;
                octet2 = 0x00;
                octet3 = 0x00;
                break;
            default:
                break;
        }
    }
}

/**
 * @brief i2c_write_data_to_buffer
 * @param nb_tx_octet
 */
void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
  switch(rxbuffer_tab[0]+nb_tx_octet){
  case 0x00:
    SSP2BUF = octet1;
    break;
  case 0x01:
    SSP2BUF = octet2;
    break;
  case 0x02:
    SSP2BUF = octet3;
    break;
  case 0xA2:
    SSP2BUF = coefficient_tab[1] >> 8; //K4
    break;
  case 0xA3:
    SSP2BUF = coefficient_tab[1]; //K4
    break;
  case 0xA4:
    SSP2BUF = coefficient_tab[2] >> 8; //K3
    break;
  case 0xA5:
    SSP2BUF = coefficient_tab[2]; //K3
    break;
  case 0xA6:
    SSP2BUF = coefficient_tab[3] >> 8; //K2
    break;
  case 0xA7:
    SSP2BUF = coefficient_tab[3]; //K2
    break;
  case 0xA8:
    SSP2BUF = coefficient_tab[4] >> 8; //K1
    break;
  case 0xA9:
    SSP2BUF = coefficient_tab[4]; //K1
    break;
  case 0xAA:
    SSP2BUF = coefficient_tab[5] >> 8; //K0
    break;
  case 0xAB:
    SSP2BUF = coefficient_tab[5]; //K0
    break;
  case 0xC0:
    SSP2BUF = CODE_VERSION;
    break;
  case 0xC1:
    SSP2BUF = state;
    break;
  default:
    SSP2BUF = 0x00;
    break;
  }
}


/**
 * @brief init_io
 * Initialisation des entrées sorties du PIC
 */
void init_io(){
  ANSELA = 0x00;
  ANSELB = 0x00;
  ANSELC = 0x00;


  CM1CON0 = 0x00; // Not using the comparators
  CM2CON0 = 0x00;

  TRISA = 0xFF;

  TRISA0_bit = 0; // RA0 en sortie

  INTCON2.RBPU = 1; // PORTA and PORTB Pull-up disable bit
}


/**
 * @brief reset_TSYS01_sequence
 * Reset du circuit TSYS01
 */
void reset_TSYS01_sequence(){
  I2C1_Start();
  I2C1_Wr(TSYS01_ADDR);
  I2C1_Wr(0x1E);
  I2C1_Stop();
  
  delay_ms(10);
  reset = 0;
}


/**
 * @brief prom_read_TSYS01_sequence
 * Lecture de la mémoire du capteur, lecture des adresses 1 à 5, pour récupérer
 * les coefficients k0,k1,k2,k3,k4.
 * 0xA2 --> K4
 * 0XA4 --> k3
 * 0XA6 --> K2
 * 0XA8 --> K1
 * 0XAA --> K0
 *
 */
void prom_read_TSYS01_sequence(){
   unsigned short j = 0;

// Read calibration values
  for(j = 0 ; j < 8 ; j++ ){
         I2C1_Start();
         I2C1_Wr(TSYS01_ADDR);
         I2C1_Wr(0xA0+j*2);
         I2C1_Stop();

         I2C1_Start();
         I2C1_Wr(TSYS01_ADDR+1);
         octet1 = I2C1_Rd(1);
         octet2 = I2C1_Rd(0);
         I2C1_Stop();

         coefficient_tab[j] = (octet1 << 8) | octet2;
   }
}


void test_prom_read_sequence(){
  coefficient_tab[0] = 0;
  coefficient_tab[1] = 28446;  //0xA2 K4 valeur --> 6F1E
  coefficient_tab[2] = 24926;  //0XA4 k3 valeur --> 615E
  coefficient_tab[3] = 36016;  //0XA6 K2 valeur --> 8CB0
  coefficient_tab[4] = 32791;  //0XA8 K1 valeur --> 8017
  coefficient_tab[5] = 40781;  //0XAA K0 valeur --> 9F4D
  coefficient_tab[6] = 0;
  coefficient_tab[7] = 0;
}

/**
 * @brief read_result_TSYS01_sequence
 * Lecture du résultat de la conversion
 */
void read_TSYS01_sequence(){
  I2C1_Start();
  I2C1_Wr(TSYS01_ADDR);
  I2C1_Wr(0x48);   // Start ADC Temperature Conversion --> 0x48
  I2C1_Stop();

  delay_ms(20);

  I2C1_Start();
  I2C1_Wr(TSYS01_ADDR);  // Read ADC Temperature Result --> 0x00
  I2C1_Wr(0x00);
  I2C1_Stop();
  
  I2C1_Start();
  I2C1_Wr(TSYS01_ADDR+1);
  octet1 = I2C1_Rd(1);
  octet2 = I2C1_Rd(1);
  octet3 = I2C1_Rd(0);
  I2C1_Stop();
  
  conversion = 0;
}


void uart_prom_sequence(){
  UART1_Write(255);
  for(k=1; k<6; k++){
  UART1_Write(coefficient_tab[k]);
  UART1_Write(coefficient_tab[k] >> 8);
  delay_ms(10);
  }
}


void uart_read_sequence(){
  UART1_Write(255);
  UART1_Write(octet1);
  UART1_Write(octet2);
  UART1_Write(octet3);
}

/**
 * @brief main
 */
void main(){
  // Oscillateur interne de 16Mhz
  OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block

  // UART1_Init(9600);
  init_io(); // Initialisation des I/O

  I2C1_Init(100000);// initialize I2C communication bus I2C N°1
  delay_ms(10);
  
  reset_TSYS01_sequence();
  prom_read_TSYS01_sequence();

  init_i2c(); // Initialisation de l'I2C en esclave bus I2C N°2
  
  LED = 0; // sortie LED
  RCON.IPEN = 1;  //Enable priority levels on interrupts
  IPR3.SSP2IP = 0; //Master Synchronous Serial Port Interrupt Priority bit (low priority = 0) bus I2C N°2
  INTCON.GIEH = 1; //enable all high-priority interrupts
  INTCON.GIEL = 1; //enable all low-priority interrupts

  INTCON.GIE = 1; // Global Interrupt Enable bit
  INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

  while(1){

    //UART1_Write(state);
    
    switch (state){
      case IDLE: // Idle state
        LED = 0;
        if(reset == 1)
          state = RESET_TSYS01;
        else if(conversion == 1)
          state = CONVERSION_READ;

        break;

      case RESET_TSYS01:
        LED = 1;
        reset_TSYS01_sequence();
        prom_read_TSYS01_sequence();
        state = IDLE;
        break;

      case CONVERSION_READ:
        LED = 1;
        read_TSYS01_sequence();
        state = IDLE;
        break;
      default:
        break;
    }

    // I2C
    if(nb_rx_octet>0 && SSP2STAT.P == 1){
        i2c_read_data_from_buffer();
        nb_rx_octet = 0;
    }
  }
}


/**
 * @brief initialisation de l'I2C en mode esclave
 */
void init_i2c(){

  // **** IO I2C **** //
  TRISB1_bit = 1; // RB4 en entrée
  TRISB2_bit = 1; // RB6 en entrée

  // **** Interruptions **** //
  PIE3.SSP2IE = 1; // Synchronous Serial Port Interrupt Enable bit
  PIR3.SSP2IF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave

  PIE3.BCL2IE = 1;
  PIR3.BCL2IF = 0;

  // **** ADDRESS **** //
  SSP2ADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
  SSP2MSK = 0xFF; // A zero (0) bit in the SSPMSK register has the effect of making
                 // the corresponding bit in the SSPSR register a dont care

  // **** SSPSTAT **** //
  SSP2STAT.SMP = 1; // Slew Rate Control bit
  // 1 = Slew rate control disabled for standard Speed mode (100 kHz and 1 MHz)
  // 0 = Slew rate control enabled for High-Speed mode (400 kHz)
  SSP2STAT.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)

  // **** SSPCON2 **** //
  SSP2CON2 = 0x00;
  SSP2CON2.GCEN = 0; // General Call Enable bit (0 = disabled)
  SSP2CON2.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)

  // **** SSPCON1 **** //
  SSP2CON1.WCOL = 0; // Write Collision Detect bit
  SSP2CON1.SSPOV = 0; // Receive Overflow Indicator bit
  SSP2CON1.CKP = 1; // SCK Release Control bit (1=Release clock)
  SSP2CON1.SSPM3 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled (1-> with S/P, 0 -> without)
  SSP2CON1.SSPM2 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSP2CON1.SSPM1 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSP2CON1.SSPM0 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled

  // (START the I2C Module)
  SSP2CON1.SSPEN = 1; // Synchronous Serial Port Enable bit
}

/**
 * @brief interrupt
 * Fonction de gestion des interruptions:
 * interruption sur TIMER3 interruptions tous les 100ms pour visu STATE0(tous les 500ms)
 * interruption sur le bus I2C
 */
void interrupt(){

}

/**
 * @brief interrupt_low
 */
void interrupt_low(){
  if (PIR3.SSP2IF){  // I2C Interrupt

      if(SSP2CON1.SSPOV || SSP2CON1.WCOL){
          SSP2CON1.SSPOV = 0;
          SSP2CON1.WCOL = 0;
          tmp_rx = SSP2BUF;
      }

      //****** receiving data from master ****** //
      // 0 = Write (master -> slave - reception)
      if (SSP2STAT.R_W == 0){
        if(SSP2STAT.P == 0){
          if (SSP2STAT.D_A == 0){ // Address
            nb_rx_octet = 0;
            tmp_rx = SSP2BUF;
          }
          else{ // Data
            if(nb_rx_octet < SIZE_RX_BUFFER){
              rxbuffer_tab[nb_rx_octet] = SSP2BUF;
              nb_rx_octet++;
            }
            else{
              tmp_rx = SSP2BUF;
            }
          }
        }

        // if(nb_rx_octet>1){
        //   Delay_us(30); // Wait P signal ?
        //   if(SSP2STAT.P == 1){
        //     i2c_read_data_from_buffer();
        //     nb_rx_octet = 0;
        //   }
        // }
      }
      //******  transmitting data to master ****** //
      // 1 = Read (slave -> master - transmission)
      else{
          if(SSP2STAT.D_A == 0){
            nb_tx_octet = 0;
            tmp_rx = SSP2BUF;
          }

          // In both D_A case (transmit data after receive add)
          i2c_write_data_to_buffer(nb_tx_octet);
          nb_tx_octet++;
      }

    SSP2CON1.CKP = 1;
    PIR3.SSP2IF = 0; // reset SSP interrupt flag
  }
}
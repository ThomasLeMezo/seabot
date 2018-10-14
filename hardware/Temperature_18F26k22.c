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

// CONFIG2H
// #pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
// #pragma config WDTPS = 256    // Watchdog Timer Postscale Select bits (1:256)

//sbit LED at LATA.B0; // sortie LED
sbit LED at RA0_bit; // sortie LED
sbit LED2 at RA2_bit; // sortie LED
sbit LED3 at RA3_bit; // sortie LED
#define CODE_VERSION 0x03

// I2C Master
void init_i2c_master();
void i2c_master_write_byte();
void i2c_master_read_data();
#define TSYS01_ADDR           0x77  
#define TSYS01_RESET          0x1E
#define TSYS01_ADC_READ       0x00
#define TSYS01_ADC_TEMP_CONV  0x48
#define TSYS01_PROM_READ      0XA0
unsigned char mssp_interrupt_received = 0;
#define SIZE_RX_BUFFER 3
unsigned char i2c_master_rxbuffer_tab[SIZE_MASTER_RX_BUFFER];
unsigned char i2c_master_adc_tab[3];

unsigned char tsys01_prom[10];   // tableau de coefficients de calibration

// I2C Slave
void init_i2c_slave();
#define SIZE_RX_BUFFER 8
const unsigned char ADDRESS_I2C = 0x45; // linux I2C Adresse
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];

unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;
unsigned short k = 0;

unsigned short reset = 0;
unsigned short conversion = 0;

// State Machine
enum power_state {IDLE,RESET_TSYS01,CONVERSION_READ, INIT};
unsigned short state = INIT;
unsigned short cpt = 0;


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
    SSP2BUF = i2c_master_adc_tab[0];
    break;
  case 0x01:
    SSP2BUF = i2c_master_adc_tab[1];
    break;
  case 0x02:
    SSP2BUF = i2c_master_adc_tab[2];
    break;
  case 0xA2:
    SSP2BUF = tsys01_prom[0];
    break;
  case 0xA3:
    SSP2BUF = tsys01_prom[1];
    break;
  case 0xA4:
    SSP2BUF = tsys01_prom[2];
    break;
  case 0xA5:
    SSP2BUF = tsys01_prom[3];
    break;
  case 0xA6:
    SSP2BUF = tsys01_prom[4];
    break;
  case 0xA7:
    SSP2BUF = tsys01_prom[5];
    break;
  case 0xA8:
    SSP2BUF = tsys01_prom[6];
    break;
  case 0xA9:
    SSP2BUF = tsys01_prom[7];
    break;
  case 0xAA:
    SSP2BUF = tsys01_prom[8];
    break;
  case 0xAB:
    SSP2BUF = tsys01_prom[9];
    break;
  case 0xC0:
    SSP2BUF = CODE_VERSION;
    break;
  case 0xC1:
    SSP2BUF = state;
    break;
  case 0xC2:
    SSP2BUF = cpt;
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
  TRISA2_bit = 0; // RA2 en sortie
  TRISA3_bit = 0; // RA3 en sortie

  INTCON2.RBPU = 1; // PORTA and PORTB Pull-up disable bit
}


/**
 * @brief reset_TSYS01_sequence
 * Reset du circuit TSYS01
 */
void reset_TSYS01_sequence(){
  i2c_master_write_byte(TSYS01_RESET);
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
  for(j = 0 ; j < 5 ; j++){
        i2c_master_read_data(TSYS01_PROM_READ+(j+1)*2);
        tsys01_prom[j*2] = i2c_master_rxbuffer_tab[0];
        tsys01_prom[j*2+1] = i2c_master_rxbuffer_tab[1];
        delay_us(100);
   }
}

/**
 * @brief read_result_TSYS01_sequence
 * Lecture du résultat de la conversion
 */
void read_TSYS01_sequence(){
  i2c_master_write_byte(TSYS01_ADC_TEMP_CONV);

  delay_ms(20);

  i2c_master_read_data(TSYS01_ADC_READ);
  i2c_master_adc_tab[0] = i2c_master_adc_tab[0];
  i2c_master_adc_tab[1] = i2c_master_adc_tab[1];
  i2c_master_adc_tab[2] = i2c_master_adc_tab[2];
  
  conversion = 0;
  cpt++;
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

void wait_MSSP(){
  while(mssp_interrupt_received==0);
  mssp_interrupt_received = 0;
}

void i2c_fail(){
   int i;
   SSP1CON2bits.PEN = 1; //Send Stop Condition
   wait_MSSP(); //Wait to complete

   //Signal error
   while(1){
       LATCbits.LC2 = 1;
       for(i=0;i<50;i++) __delay_ms(10);
       LATCbits.LC2 = 0;
       for(i=0;i<50;i++) __delay_ms(10);
   }
}

void i2c_master_write_byte(char cmd){
  SSPCON2.SEN = 1; //Send Start Condition
  wait_MSSP(); //Wait to complete

  SSP1BUF = (TSYS01_ADDR << 1); //Send Add (Write)
  wait_MSSP();
  if(SSP1CON2.ACKSTAT == 1) //If no ACK is received
    i2c_fail();

  SSP1BUF = cmd; //Send data to slave
  wait_MSSP();
  if(SSP1CON2.ACKSTAT == 1) //If no ACK is received
    i2c_fail();

  SSPCON2.PEN = 1; //Send Stop Condition
  wait_MSSP(); //Wait to complete
}

void i2c_master_read_data(char cmd, unsigned char nb_bytes){
  unsigned char i = 0;
  i2c_master_write_byte(cmd);

  SSPCON2.SEN = 1; //Send Start Condition
  wait_MSSP(); //Wait to complete

  Send_I2C_Byte((TSYS01_ADDR << 1) | 0b1); //Send Control Write Byte
  wait_MSSP();
  if(SSP1CON2.ACKSTAT == 1) //If no ACK is received
    i2c_fail();

  for(i=0; i<nb_bytes; i++){
    SSPCON2.RCEN = 1; // Configure master to receive bytes
    wait_MSSP(); // Wait data to be received
    i2c_master_rxbuffer_tab[i] = SSP1BUF; // Read data
    SSP1CON2.ACKEN = 1; // Acknowledge read
    wait_MSSP();
  }

  SSPCON2.PEN = 1; //Send Stop Condition
  wait_MSSP(); //Wait to complete
}

/**
 * @brief main
 */
void main(){

  /** Edit config (Project > Edit Project)
  *   -> Oscillator Selection : Internal oscillator block 16 MHz
  *   -> 4xPLL : Diseabled
  *   -> Watchdog Timer : WDT is controlled by SWDTEN bit of the WDTCON register
  *   -> Watchdog Time Postscale : 1:256 (32768/31000 = environ 1Hz)
  *   -> MCLR : disabled (external reset)
  */
  
  // Oscillateur interne de 16Mhz
  OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block
  
  asm CLRWDT;// Watchdog
  RCON2bits.SWDTEN=1; //armement du watchdog (?)

  init_io(); // Initialisation des I/O
  init_i2c_slave(); // Initialisation de l'I2C en esclave bus I2C N?2
  init_i2c_master();
  // I2C1_Init(100000);// initialize I2C communication bus I2C N?1

  LED = 1; // sortie LED
  delay_ms(250);

  RCON.IPEN = 1;  //Enable priority levels on interrupts
  IPR3.SSP2IP = 0; //Master Synchronous Serial Port Interrupt Priority bit (low priority = 0) bus I2C N?2
  INTCON.GIEH = 1; //enable all high-priority interrupts
  INTCON.GIEL = 1; //enable all low-priority interrupts

  INTCON.GIE = 1; // Global Interrupt Enable bit
  INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

  reset_TSYS01_sequence();
  prom_read_TSYS01_sequence();
  
  delay_ms(250);
  LED = 0;

  while(1){

    asm CLRWDT; // Watchdog

    switch (state){
      case INIT:
        state = IDLE;
        break;
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
 * @brief init master I2C
 */
void init_i2c_master(){
  TRISRC3_bit = 1; // RC3 en entrée
  TRISRC4_bit = 1; // RC4 en entrée

  //Configure MSSP mode for Master Mode
  SSP1CON1bits.SSPM = 0b1000; //I2C Master Mode
  SSP1CON1bits.SSPEN = 1; //Enable MSSP
  SSP1STATbits.SMP = 1; //Disable slew rate

  //Configure baud rate (32MHz => 0x4F, 16MHz => 0x27)
   SSPADD = 0x27;
}

/**
 * @brief initialisation de l'I2C en mode esclave
 */
void init_i2c_slave(){

  // **** IO I2C **** //
  TRISB1_bit = 1; // RB1 en entrée
  TRISB2_bit = 1; // RB2 en entrée

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

  if(PIR1.SSP1IF){
    PIR3.SSP2IF = 0;
    mssp_interrupt_received = 1;

    if(SSP1CON1.SSPOV || SSP1CON1.WCOL){
          SSP1CON1.SSPOV = 0;
          SSP1CON1.WCOL = 0;
          tmp_rx = SSP1BUF;
      }
  }
}
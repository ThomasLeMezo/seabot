/*  PIC18F14K22  mikroC PRO for PIC v6.4
Oscillateur externe quartz 16MHZ, PLL * 4, MCU clock frequency 64MHZ


Hardware:
  18F14K22  DIP20,SOIC
  
  pin 1     VDD Alim +5V
  pin 2     OSC1/RA5 ---> quartz 16MHZ
  pin 3     OSC2/RA4 ---> quartz 16MHZ
  pin 4     RA3/MCR
  pin 5     RC5
  pin 6     RC4
  pin 7     RC3
  pin 8     RC6
  pin 9     RC7
  pin 10    RB7
  pin 11    RB6 ---> SCL I2C clock input
  pin 12    RB5
  pin 13    RB4 ---> SDA I2C data output
  pin 14    RC2 ---> sortie commande moteur 3
  pin 15    RC1 ---> sortie commande moteur 2
  pin 16    RC0 ---> sortie commande moteur 1
  pin 17    RA2/INT2/T0CKI ---> sortie LED
  pin 18    RA1/INT1
  pin 19    RA0/INT0
  pin 20    VSS Alim 0V


Programme de commande pour commander 3 moteurs avec les module BESC30-R3 (BlueRobotics)
http://docs.bluerobotics.com/bescr3/

Pulse Width Signal
Signal Voltage  3.3-5 volts
Max Update Rate 400 Hz
Stopped         1500 microseconds
Max forward     1900 microseconds
Max reverse     1100 microseconds
Signal Deadband    +/- 25 microseconds (centered around 1500 microseconds)

1500us moteur stoppé
<1500us moteur en arrière (vitesse max en arrière 1100us)
>1500us moteur en avant (vitesse max en avant 1900us)

on prendra une fréquence de 50Hz, donc T = 20ms

Utilisation de quatre Timer:

TIMER0: Géneration de la période de 20ms
TIMER1: Géneration d'une temporisation variable par pas de 10us

1500us --> 150 pas de 10us --> moteur stoppé
1100us --> 110 pas de 10us --> moteur vitesse max en arrière
1900us --> 190 pas de 10us --> moteur vitesse max en avant

*/

#define CODE_VERSION 0x03

// I2C
const unsigned short ADDRESS_I2C = 0x20;
#define SIZE_RX_BUFFER 8
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

void init_i2c();

unsigned short is_init = 1;

sbit MOT1 at LATC.B0; // sorties de commande moteur 1
sbit MOT2 at LATC.B1; // sorties de commande moteur 2
sbit MOT3 at LATC.B2; // sorties de commande moteur 3

sbit LED at LATA.B2; // sortie LED

// Motor
#define MOTOR_CMD_STOP 150
#define PWM_PERIOD 2000
unsigned short cmd_motor[3] = {MOTOR_CMD_STOP, MOTOR_CMD_STOP, MOTOR_CMD_STOP};


unsigned char cpt_motor_1 = 0;
unsigned char cpt_motor_2 = 0;
unsigned char cpt_motor_3 = 0;
unsigned int cpt_global = PWM_PERIOD;

// Watchdog
unsigned short watchdog_restart = 3;
#define WATCHDOG_RESTART_DEFAULT 3 // 3 s

/**
 * @brief i2c_read_data_from_buffer
 */
void i2c_read_data_from_buffer(){
  unsigned short k=0;
  unsigned short nb_motor=0;

  for(k=1; k<nb_rx_octet; k++){
    nb_motor = rxbuffer_tab[0] + k - 1;

    if(nb_motor<3){
      if(rxbuffer_tab[k]>=110 && rxbuffer_tab[k]<=190)
        cmd_motor[nb_motor] = rxbuffer_tab[k];
      else
        cmd_motor[nb_motor] = MOTOR_CMD_STOP;
    }
  }
  watchdog_restart = WATCHDOG_RESTART_DEFAULT;
}

/**
 * @brief i2c_write_data_to_buffer
 * @param nb_tx_octet
 */
void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
  switch(rxbuffer_tab[0]+nb_tx_octet){
    case 0xC0:
      SSPBUF = CODE_VERSION;
      break;
    case 0xC1:
      SSPBUF = is_init;
      break;
    default:
      SSPBUF = 0x00;
      break;
  }
}

/**
 * @brief init_timer0
 * Fonction d'initialisation du TIMER0
 * Prescaler 1:128; TMR0 Preload = 3036; Actual Interrupt Time : 1 s
 */
void init_timer0(){
  T0CON = 0x85; // TIMER0 ON (1 s)
  TMR0H = 0x0B;
  TMR0L = 0xDC;
  TMR0IE_bit = 0;
}

/**
 * @brief init_timer1
 * Fonction d'initialisation du TIMER1
 * Prescaler 1:1; TMR1 Preload = 65136; Actual Interrupt Time : 10 us
 */
void init_timer1(){
  T1CON = 0x01;
  TMR1IF_bit = 0;
  TMR1H = 255;
  TMR1L = 136;
  TMR1IE_bit = 0;
  INTCON = 0xC0;
}


/**
 * @brief init_io
 * Initialisation des entrées sorties du PIC
 */
void init_io(){
  ANSEL = 0xFF;

  CM1CON0 = 0x00; // Not using the comparators
  CM2CON0 = 0x00; //

  PORTA = 0;
  LATA = 0;

  TRISA = 0xFF;

  TRISA2_bit = 0; // RA2 en sortie

  INTCON.RABIE = 0;
  INTCON2.RABPU = 1; // PORTA and PORTB Pull-up disable bit

  TRISB5_bit = 0; // RB5 en sortie
  TRISB7_bit = 0; // RB7 en sortie

  TRISC = 0xFF;
  TRISC0_bit = 0; // RC0 en sortie
  TRISC1_bit = 0; // RC1 en sortie
  TRISC2_bit = 0; // RC2 en sortie
}

/**
 * @brief main
 */
void main(){
  /** Edit config (Project > Edit Project)
  *   -> Oscillator Selection : HS Oscillator 64 MHz
  *   -> 4xPLL : Enable
  *   -> Watchdog Timer : WDT is controlled by SWDTEN bit of the WDTCON register
  *   -> Watchdog Time Postscale : 1:256 (32768/31000 = environ 1Hz)
  *   -> MCLR : disabled (external reset)
  */

  asm CLRWDT;// Watchdog
  SWDTEN_bit = 1; //armement du watchdog

  init_io(); // Initialisation des I/O
  init_i2c(); // Initialisation de l'I2C en esclave
  init_timer0(); // Initi TIMER0 toutes les 1s
  init_timer1(); // Initialisation du TIMER1 toutes les 1us

  LATC = 0;

  TMR0IE_bit = 1; //Enable TIMER0
  TMR0ON_bit = 1; // Start TIMER0
  
  TMR1IE_bit = 1;
  TMR1ON_bit = 1; // Start TIMER1

  INTCON3.INT1IP = 1; //INT1 External Interrupt Priority bit, INT0 always a high
  //priority interrupt source

  IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit, low priority
  RCON.IPEN = 1;  //Enable priority levels on interrupts
  INTCON.GIEH = 1; //enable all high-priority interrupts
  INTCON.GIEL = 1; //enable all low-priority interrupts

  INTCON.GIE = 1; // Global Interrupt Enable bit
  INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

  LED = 1;
  delay_ms(250);

  is_init = 0;

  while(1){
    asm CLRWDT;

    if(cmd_motor[0] != MOTOR_CMD_STOP || cmd_motor[1] != MOTOR_CMD_STOP || cmd_motor[2] != MOTOR_CMD_STOP)
      LED = 1;
    else
      LED = 0;

    if(nb_rx_octet>1 && SSPSTAT.P == 1){
        i2c_read_data_from_buffer();
        nb_rx_octet = 0;
    }
  }
}

/**
 * @brief interrupt
 */
void interrupt(){
  /// ************************************************** //
  /// ********************** TIMERS  ******************* //

  // Interruption TIMER1 toutes les 10us
  if (TMR1IF_bit){

    if(cpt_global==0){
      MOT1 = 1;
      MOT2 = 1;
      MOT3 = 1;
      cpt_global = PWM_PERIOD;

      cpt_motor_1 = cmd_motor[0];
      cpt_motor_2 = cmd_motor[1];
      cpt_motor_3 = cmd_motor[2];
    }
    else{
      cpt_global--;

      // MOT 1
      if(cpt_motor_1==0)
        MOT1 = 0;
      else
        cpt_motor_1--;

      // MOT 2
      if(cpt_motor_2==0)
        MOT2 = 0;
      else
        cpt_motor_2--;

      // MOT 3
      if(cpt_motor_3==0)
        MOT3 = 0;
      else
        cpt_motor_3--;
    }

    TMR1H = 255;
    TMR1L = 136;
    TMR1IF_bit = 0;
  }

  else if (TMR0IF_bit){
    // Watchdog
    if(watchdog_restart>0)
      watchdog_restart--;  
    else{
      cmd_motor[0] = MOTOR_CMD_STOP;
      cmd_motor[1] = MOTOR_CMD_STOP;
      cmd_motor[2] = MOTOR_CMD_STOP;
      watchdog_restart = WATCHDOG_RESTART_DEFAULT;
    }

    TMR0H = 0x0B;
    TMR0L = 0xDC;
    TMR0IF_bit = 0;
  }
}

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
  SSPCON1.SSPM3 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSPCON1.SSPM2 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSPCON1.SSPM1 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSPCON1.SSPM0 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled

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
          delay_us(20);
          nb_tx_octet++;
      }

    SSPCON1.CKP = 1;
    PIR1.SSPIF = 0; // reset SSP interrupt flag
  }
}
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
TIMER1: Géneration d'une temporisation variable par pas de 10us, sortie MOT1 active
TIMER2: Géneration d'une temporisation variable par pas de 10us, sortie MOT2 active
TIMER3: Géneration d'une temporisation variable par pas de 10us, sortie MOT3 active

1500us --> 150 pas de 10us --> moteur stoppé
1100us --> 110 pas de 10us --> moteur vitesse max en arrière
1900us --> 190 pas de 10us --> moteur vitesse max en avant

*/


sbit MOT1 at LATC.B0; // sorties de commande moteur 1
sbit MOT2 at LATC.B1; // sorties de commande moteur 2
sbit MOT3 at LATC.B2; // sorties de commande moteur 3

sbit LED at LATA.B2; // sortie LED

// Motor
unsigned char cmd_motor_1 = 150;
unsigned char cmd_motor_2 = 150;
unsigned char cmd_motor_3 = 150;
unsigned short int cmd_global = 2000;

unsigned char cpt_motor_1 = 0;
unsigned char cpt_motor_2 = 0;
unsigned char cpt_motor_3 = 0;
unsigned char cpt_global = 2000;

// I2C
#define ADDRESS_I2C 0x70;
#define SIZE_RX_BUFFER 3
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

/**
 * @brief i2c_read_data_from_buffer
 */
void i2c_read_data_from_buffer(){
  switch(rxbuffer_tab[0]){
  case 0x00:  // motor 1
    if(rxbuffer_tab[1]>=110 && rxbuffer_tab[1]<=190)
      cmd_motor_1 = rxbuffer_tab[1];
    else
      cmd_motor_1 = 150;
    break;
  case 0x01:  // led power
    if(rxbuffer_tab[1]>=110 && rxbuffer_tab[1]<=190)
      cmd_motor_1 = rxbuffer_tab[1];
    else
      cmd_motor_1 = 150;
    break;
  case 0x02:
    if(rxbuffer_tab[1]>=110 && rxbuffer_tab[1]<=190)
      cmd_motor_1 = rxbuffer_tab[1];
    else
      cmd_motor_1 = 150;
    break;
  default:
    break;
  }
}

/**
 * @brief i2c_write_data_to_buffer
 * @param nb_tx_octet
 */
void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
  switch(rxbuffer_tab[0]+nb_tx_octet){
  case 0x00:
    SSPBUF = 0x00;
    break;
  case 0x01:
    SSPBUF = 0x00;
    break;
  default:
    SSPBUF = 0x00;
    break;
  }
}

/**
 * @brief initialisation de l'I2C en mode esclave
 */
void init_i2c(){
  SSPADD = ADDRESS_I2C; // Address Register, Get address (7bit). Lsb is read/write flag
  SSPCON1 = 0x3E; // SYNC SERIAL PORT CONTROL REGISTER
  SSPCON1.SSPEN = 1;
  // bit 3-0 SSPM3:SSPM0: I2C Firmware Controlled Master mode,
  // 7-bit address with START and STOP bit interrupts enabled
  // bit 4 CKP: 1 = Enable clock
  // bit 5 SSPEN: Enables the serial port and configures the SDA and SCL
  // pins as the source of the serial port pins

  SSPCON2 = 0x00;
  SSPSTAT=0x00;
  SSPSTAT.SMP = 1; // 1 = Slew rate control disabled for standard speed mode (100 kHz and 1 MHz)
  SSPSTAT.CKE = 1; // 1 = Input levels conform to SMBus spec

  PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
  PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
  // a transmission/reception has taken place.
  PIR2.BCLIF = 0;
}

/**
 * @brief init_timer0
 * Fonction d'initialisation du TIMER0
 * 2 s
 */
void init_timer0(){
  T0CON = 0x86; // TIMER0 ON (2 s)
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
  T1CON = 0x00;
  TMR1IF_bit = 0;
  TMR1H = 0xFF;
  TMR1L = 0x80;
  TMR1IE_bit = 0;
  INTCON = 0xC0;
}

/**
 * @brief init_timer2
 * Fonction d'initialisation du TIMER2
 * Prescaler 1:1; Postscaler 1:2; TMR2 Preload = 199; Actual Interrupt Time : 10 us
 */
//void init_timer2(){
//  T2CON = 0x08;
//  TMR2IE_bit = 0;
//  PR2 = 79;
//  INTCON = 0xC0;
//}

/**
 * @brief init_timer3
 * Fonction d'initialisation du TIMER3
 * Prescaler 1:1; TMR1 Preload = 65136; Actual Interrupt Time : 10 US
 */
//void init_timer3(){
//  T3CON = 0x00;
//  TMR3IF_bit = 0;
//  TMR3H = 0xFF;
//  TMR3L = 0x84;
//  TMR3IE_bit = 0;
//  INTCON = 0xC0;
//}

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

  TRISB4_bit = 1; // RB4 en entrée
  TRISB6_bit = 1; // RB6 en entrée
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

  init_io(); // Initialisation des I/O
  init_i2c(); // Initialisation de l'I2C en esclave
  init_timer0(); // Initialisation du TIMER0 toutes les 20 ms
  init_timer1(); // Initialisation du TIMER1 toutes les 1us
  //init_timer2(); // Initialisation du TIMER2 toutes les 1us
  //init_timer3(); // Initialisation du TIMER3 toutes les 1us

  //UART1_Init(115200);
  LATC = 0;

  for (int i=0; i< 90; i++){  // Tempo de 45 secondes pour faire monter Linux sur la carte PCDUINO
    LED =~ LED;
    delay_ms(500);
  }

  TMR0IE_bit = 1;
  TMR1IE_bit = 1;
  //TMR2IE_bit = 1;
  //TMR3IE_bit = 1;

  TMR0ON_bit = 1; // Start TIMER0
  TMR1ON_bit = 1; // Start TIMER1

  PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
  INTCON.GIE = 1; // Global Interrupt Enable bit
  INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

  while(1){
    if(cmd_motor_1 != 150 || cmd_motor_2 != 150 || cmd_motor_3 != 150)
      LED = 1;
    else
      LED = 0;

    delay_ms(250);
  }
}

/**
 * @brief interrupt
 * Fonction de gestion des interruptions:
 * interruption sur TIMER3 interruptions tous les 100ms pour visu STATE0(tous les 500ms)
 * interruption sur le bus I2C
 */
void interrupt(){

  /// ************************************************** //
  /// ********************** I2C  ********************** //

  if (PIR1.SSPIF){  // I2C Interrupt

    if (SSPSTAT.R_W == 1){   //******  transmit data to master ****** //
      i2c_write_data_to_buffer(nb_tx_octet);
      nb_tx_octet++;
      delay_us(10);
      SSPCON1.CKP = 1;
    }
    else{ //****** recieve data from master ****** //
      if (SSPSTAT.BF == 1){ // Buffer is Full (transmit in progress)
        if (SSPSTAT.D_A == 1){ //1 = Indicates that the last byte received or transmitted was data
          if(nb_rx_octet < SIZE_RX_BUFFER)
            rxbuffer_tab[nb_rx_octet] = SSPBUF;
          nb_rx_octet++;
        }
        else{
          nb_tx_octet = 0;
          nb_rx_octet = 0;
        }
      }
      else{ // At the end of the communication
        i2c_read_data_from_buffer();
      }
      tmp_rx = SSPBUF;
    }

    PIR1.SSPIF = 0; // reset SSP interrupt flag
  }
  if (PIR2.BCLIF)
    PIR2.BCLIF = 0;

  /// ************************************************** //
  /// ********************** TIMERS  ******************* //

  // Interruption TIMER1 toutes les 10us
  if (TMR1IE_bit && TMR1IF_bit){

    if(cpt_global==0){
      MOT1 = 1;
      MOT2 = 1;
      MOT3 = 1;
      cpt_global = cmd_global;

      cpt_motor_1 = cmd_motor_1;
      cpt_motor_2 = cmd_motor_2;
      cpt_motor_3 = cmd_motor_3;
    }
    else{
      cpt_global--;
    }

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

    TMR1H = 0xFF; //(necessary ?)
    TMR1L = 0x80;
    TMR1IF_bit = 0;
  }

}

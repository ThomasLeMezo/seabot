/*  PIC18F14K22  mikroC PRO for PIC v6.4
Oscillateur interne 16MHZ (attention l'edit project ne marche pas, paramétrer 
l'oscillateur en dur)


Hardware:
  18F14K22  DIP20,SOIC
  
  pin 1     VDD Alim +5V
  pin 2     OSC1/RA5 ---> sortie commande de l'alimentation générale
  pin 3     OSC2/RA4 ---> sortie commande LED
  pin 4     RA3/MCR
  pin 5     RC5
  pin 6     RC4
  pin 7     RC3 ---> mesure VBAT4
  pin 8     RC6
  pin 9     RC7
  pin 10    RB7
  pin 11    RB6 ---> SCL I2C clock input
  pin 12    RB5
  pin 13    RB4 ---> SDA I2C data output
  pin 14    RC2 ---> mesure VBAT3
  pin 15    RC1 ---> mesure VBAT2
  pin 16    RC0 ---> mesure VBAT1
  pin 17    RA2/INT2/T0CKI ---> entrée ILS
  pin 18    RA1/INT1
  pin 19    RA0/INT0 ---> sortie LED
  pin 20    VSS Alim 0V


Programme pour commander l'alimentation générale d'un système (via un MOSFET de
puissance IPB80P03P4L) à partir d'un ILS connecté sur l'entrée INT2, le programme
contrôle la tension d'alimentation de 4 batteries, la valeur est retournée sur 
ordre via l'I2C.
La sortie RA4 commande trois LED de repérage via le circuit ZXLD1350.

14/03/18/ Implantation et essai du programme, seuil des batteries à corriger

*/
#define CODE_VERSION 0x04

// I2C
const unsigned short ADDRESS_I2C = 0x39; // Linux Version
#define SIZE_RX_BUFFER 8
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

void init_i2c();

unsigned short is_init = 1;

sbit BAT1 at PORTC.B0; // entrée de controle de tension BAT1
sbit BAT2 at PORTC.B1; // entrée de controle de tension BAT2
sbit BAT3 at PORTC.B2; // entrée de controle de tension BAT2
sbit BAT4 at PORTC.B3; // entrée de controle de tension BAT2
sbit ILS at PORTA.B2; //  entrée de l'ILS

sbit LED1 at LATA.B1; // sortie LED1
sbit LED at LATA.B0; // sortie LED
sbit LED_PUISSANCE at LATA.B4; // sortie LED de puissance
sbit ALIM at LATA.B5; // sortie MOSFET de puissance, commande de l'alimentation

// ILS
#define ILS_CPT_TIME 4
unsigned short ils_cpt = 4;
unsigned short ils_removed = 1;

// State Machine
enum power_state {IDLE,POWER_ON,WAIT_TO_SLEEP, SLEEP};
unsigned short state = IDLE;
// unsigned int cpt_wait = 0;
// #define WAIT_LOOP 10000

// Batteries
#define WARNING_LOW_VOLTAGE 665 // 0.015625 (quantum) / 10.4 (min tension)
unsigned int battery_voltage[4];
unsigned short battery_global_default = 0;

// Flasher LED
unsigned short start_led_puissance = 0;
unsigned short led_puissance_delay = 20; // 100ms * val (default = 2s)
unsigned short cpt_led_puissance = 20;

// Led
unsigned short led_delay = 100;
unsigned short cpt_led = 100;
unsigned short set_led_on = 0;

// Sleep mode
unsigned char time_to_start[3] = {0, 0, 5}; // hour, min, sec
unsigned char time_to_stop = 60; // in sec (max 255 sec)

unsigned char default_time_to_start[3] = {0, 0, 5}; // hour, min, sec
unsigned char default_time_to_stop = 60;

unsigned short start_time_to_stop = 0;
unsigned short start_time_to_power_on = 0;
unsigned short start_time_to_start = 0;

unsigned short k = 0;

// Watchdog
unsigned short watchdog_restart = 60; // in min
unsigned short watchdog_restart_default = 60;
unsigned short watchdog_cpt_sec = 59;
unsigned short watchdog_cpt_default = 59;

/**
 * @brief i2c_read_data_from_buffer
 */
void i2c_read_data_from_buffer(){
  unsigned short i = 0;

  for(i=0; i<(nb_rx_octet-1); i++){
    switch(rxbuffer_tab[0]+i){
    case 0x00:  // alimentation
      switch(rxbuffer_tab[i+1]){
        case 0x01:
          state = POWER_ON;
          break;
        case 0x02:
          time_to_stop = default_time_to_stop;  // Go to Sleep mode
          start_time_to_stop = 1;
          state = WAIT_TO_SLEEP;
          break;
        default:
          break;
      }
      break;
    case 0x01:  // led power
      start_led_puissance = (rxbuffer_tab[i+1]!=0x00);
      break;
    case 0x02:
      led_puissance_delay = rxbuffer_tab[i+1];
      break;
    case 0x03:
      default_time_to_start[0] = rxbuffer_tab[i+1]; // hours
      break;
    case 0x04:
      default_time_to_start[1] = rxbuffer_tab[i+1]; // min
      break;
    case 0x05:
      default_time_to_start[2] = rxbuffer_tab[i+1]; // sec
      break;
    case 0x06:
      default_time_to_stop = rxbuffer_tab[i+1]; // sec
      break;
    case 0x07:
      watchdog_restart_default = rxbuffer_tab[i+1];
      watchdog_restart = watchdog_restart_default;
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
    SSPBUF = battery_voltage[0];
    break;
  case 0x01:
    SSPBUF = battery_voltage[0] >> 8;
    break;
  case 0x02:
    SSPBUF = battery_voltage[1];
    break;
  case 0x03:
    SSPBUF = battery_voltage[1] >> 8;
    break;
  case 0x04:
    SSPBUF = battery_voltage[2];
    break;
  case 0x05:
    SSPBUF = battery_voltage[2] >> 8;
    break;
  case 0x06:
    SSPBUF = battery_voltage[3];
    break;
  case 0x07:
    SSPBUF = battery_voltage[3] >> 8;
    break;
  case 0x08:
    SSPBUF = start_led_puissance;
    break;
  case 0x09:
    SSPBUF = state;
    break;
  case 0x10:
    SSPBUF = watchdog_restart_default;
    break;
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
  watchdog_restart = watchdog_restart_default;
}

/**
 * @brief read batteries voltage
 */
void read_batteries_voltage(){
  battery_voltage[0] = ADC_Get_Sample(4);   // Get 10-bit results of AD conversion AN4 batterie 1
  battery_voltage[1] = ADC_Get_Sample(5);   // Get 10-bit results of AD conversion AN5 batterie 2
  battery_voltage[2] = ADC_Get_Sample(6);   // Get 10-bit results of AD conversion AN6 batterie 3
  battery_voltage[3] = ADC_Get_Sample(7);   // Get 10-bit results of AD conversion AN7 batterie 4
}

/**
 * @brief analyze batteries voltage
 * 3 cellules lithium --> 4.1v/éléments --> en fin de charge
 * 9v totalement déchargé
 */
void analyze_batteries_voltage(){
     unsigned short l = 0;
  battery_global_default = 0;

  for(l=0; l<4; l++){
    if(battery_voltage[l] < WARNING_LOW_VOLTAGE)
      battery_global_default = 1;
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
 * @brief init_timer3
 * Fonction d'initialisation du TIMER3
 * Prescaler 1:8; TMR1 Preload = 15536; Actual Interrupt Time : 100 ms
 */
void init_timer3(){
  T3CON = 0x30;
  TMR3IF_bit = 0;
  TMR3H = 0x3C;
  TMR3L = 0xB0;
  TMR3IE_bit = 0;
}

/**
 * @brief init_io
 * Initialisation des entrées sorties du PIC
 */
void init_io(){
  ANSEL = 0xF0;  // Set RC0,RC1,RC2,RC3 to analog (AN4,AN5,AN6,AN7)

  CM1CON0 = 0x00; // Not using the comparators
  CM2CON0 = 0x00;

  // NVCFG = 00; PVCFG = 00;

  TRISA = 0xFF;
  //TRISA1_bit = 1; // RA1 en entrée
  TRISA0_bit = 0; // RA0 en sortie
  TRISA2_bit = 1; // RA2 en entrée
  TRISA4_bit = 0; // RA4 en sortie
  TRISA5_bit = 0; // RA5 en sortie

  TRISA1_bit = 0; // RA1 en sortie

  INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit
  WPUA.WPUA2 = 1; // Pull-up enabled sur RA2

  TRISB5_bit = 1; // RB5 en entrée
  TRISB7_bit = 0; // RB6 en sortie

  TRISC = 0xFF;
  TRISC0_bit = 1; // RC0 en entree voie AN4
  TRISC1_bit = 1; // RC1 en entree voie AN5
  TRISC2_bit = 1; // RC2 en entree voie AN6
  TRISC3_bit = 1; // RC3 en entree voie AN7
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
  SWDTEN_bit = 1; //armement du watchdog

  init_io(); // Initialisation des I/O
  init_i2c(); // Initialisation de l'I2C en esclave
  init_timer0(); // Initialisation du TIMER0 toutes les 1 secondes
  init_timer3(); // Initialisation du TIMER3 toutes les 100ms

  ADC_Init();

  LED = 0; // sortie LED
  LED1 = 0;
  LED_PUISSANCE = 0; // sortie LED de puissance
  ALIM = 0; // sortie MOSFET de puissance, commande de l'alimentation
  battery_global_default = 0;

  UART1_Init(115200);

  delay_ms(250);

  INTCON3.INT1IP = 0; //INT1 External Interrupt Priority bit, INT0 always a high
  //priority interrupt source

  RCON.IPEN = 1;  //Enable priority levels on interrupts
  IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit (low priority = 0)
  INTCON.GIEH = 1; //enable all high-priority interrupts
  INTCON.GIEL = 1; //enable all low-priority interrupts

  INTCON.GIE = 1; // Global Interrupt Enable bit
  INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

  TMR0IE_bit = 1;  //Enable TIMER0
  TMR0ON_bit = 1; // Start TIMER1
  
  TMR3IE_bit = 1;  //Enable TIMER3
  TMR3ON_bit = 1; // Start TIMER3

  is_init = 0;

  while(1){
    asm CLRWDT;

    read_batteries_voltage();
    analyze_batteries_voltage();
    
    //UART1_Write(state);
    
    switch (state){
    case IDLE: // Idle state
      ALIM = 0;
      led_delay = 50;
      start_led_puissance = 0;

      if(ILS==0){ // Magnet detected
        ils_cpt--;
        set_led_on = 1;
      }
      else{
        ils_cpt = ILS_CPT_TIME;
        set_led_on = 0;
        ils_removed = 1;
      }

      if(ils_removed == 1 && ils_cpt == 0){
        ils_cpt = ILS_CPT_TIME;
        state = POWER_ON;
        ils_removed = 0;
        set_led_on = 0;
      }
      break;

    case POWER_ON:
      ALIM = 1;
      if(battery_global_default == 1)
        led_delay = 5; // 0.5 sec
      else
        led_delay = 20; // 2 sec

      if(ILS==0){ // Magnet detected
        ils_cpt--;
        set_led_on = 1;
      }
      else{
        ils_cpt = ILS_CPT_TIME;
        set_led_on = 0;
        ils_removed = 1;
      }

      if(ils_removed == 1 && ils_cpt == 0){
        ils_cpt = ILS_CPT_TIME;
        state = IDLE;
        ils_removed = 0;
        set_led_on = 0;
      }

      break;

    case WAIT_TO_SLEEP:

      ALIM = 1;
      led_delay = 1;
      if(time_to_stop==0){
        for(k=0; k<3; k++)
          time_to_start[k] = default_time_to_start[k];
        state = SLEEP;
        time_to_stop = default_time_to_stop;
      }
      break;

    case SLEEP:
      ALIM = 0;
      led_delay = 200; // 20 sec
      if(time_to_start[0] == 0 && time_to_start[1] == 0 && time_to_start[2] == 0){
        state = POWER_ON;
      }
      break;

    default:
      state = POWER_ON;
      break;
    }
    delay_ms(500);
    // for(cpt_wait=0; cpt_wait<WAIT_LOOP; cpt_wait++){
    //   delay_us(50);
    //   // I2C
    //     if(nb_rx_octet>1 && SSPSTAT.P == 1){
    //         i2c_read_data_from_buffer();
    //         nb_rx_octet = 0;
    //     }
    // }
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
  /// ********************** TIMERS  ******************* //

  // Interruption du TIMER0 (1 s) (cpt to start/stop + Watchdog)
  if (TMR0IF_bit){

    // To Do
    if(state == SLEEP){
      if(time_to_start[2]>0){
        time_to_start[2]--;
      }
      else{
        if(time_to_start[1]>0){
          time_to_start[1]--;
          time_to_start[2]=59;
        }
        else{
          if(time_to_start[0]>0){
            time_to_start[0]--;
            time_to_start[1]=59;
          }
        }
      }
    }

    if(state == WAIT_TO_SLEEP){
      if(time_to_stop>0)
        time_to_stop--;
    }

    // Watchdog
    if(state == POWER_ON && watchdog_restart_default!=0){
      if(watchdog_cpt_sec>0)
        watchdog_cpt_sec--;
      else{
        watchdog_cpt_sec = watchdog_cpt_default;

        if(watchdog_restart>0)
          watchdog_restart--;
        else{
         // hour, min, sec
          default_time_to_start[0] = 0;
          default_time_to_start[1] = 0;
          default_time_to_start[2] = 2; // 2s
          time_to_stop = default_time_to_stop;
          
          state = WAIT_TO_SLEEP;
          watchdog_restart = watchdog_restart_default;
        }
      }
    }
    

    TMR0H = 0x0B;
    TMR0L = 0xDC;
    TMR0IF_bit = 0;
  }

  // Interruption du TIMER3 (Led Puissance)
  else if (TMR3IF_bit){
    // LED Puissance
    if(start_led_puissance == 1){
      if (cpt_led_puissance > 0){
        LED_PUISSANCE = 0;
        cpt_led_puissance--;
      }
      else{
        LED_PUISSANCE = 1;
        cpt_led_puissance=led_puissance_delay;
      }
    }
    else{
      LED_PUISSANCE = 0;
    }

    // LED
    if(set_led_on == 1) // For ILS
      LED = 1;
    else{
      if (cpt_led > 0){
        LED = 0;
        cpt_led--;
      }
      else{
        LED = 1;
        cpt_led=led_delay;
      }
    }

    TMR3H = 0x3C;
    TMR3L = 0xB0;
    TMR3IF_bit = 0;
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
  SSPCON1.SSPM3 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled (1-> with S/P, 0 -> without)
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

        if(nb_rx_octet>1){
          Delay_us(30); // Wait P signal ?
          if(SSPSTAT.P == 1){
            i2c_read_data_from_buffer();
            nb_rx_octet = 0;
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
          nb_tx_octet++;
      }

    SSPCON1.CKP = 1;
    PIR1.SSPIF = 0; // reset SSP interrupt flag
  }
}
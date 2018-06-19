/*  PIC18F14K22  mikroC PRO for PIC v6.4
Oscillateur interne sur quartz 16MHZ

Hardware:
  18F14K22  DIP20,SOIC
  
  pin 1     VDD Alim +5V
  pin 2     OSC1/RA5
  pin 3     OSC2/RA4
  pin 4     RA3/MCR ---> sortie de l'opto HOA0901 SORTIE B (avec PULL UP)
  pin 5     RC5 ---> sortie PWM P1A, relié à l'entrée IN1 du pont L6203
  pin 6     RC4 ---> sortie PWM P1B (complémenté à P1A), relié à l'entrée IN2 du pont L6203
  pin 7     RC3
  pin 8     RC6 ---> sortie relié à l'entrée ENABLE du pont L6203
  pin 9     RC7
  pin 10    RB7
  pin 11    RB6 ---> SCL I2C clock input
  pin 12    RB5
  pin 13    RB4 ---> SDA I2C data output
  pin 14    RC2
  pin 15    RC1
  pin 16    RC0 ---> LED 1
  pin 17    RA4/INT2/T0CKI---> sortie de l'opto HOA0901 SORTIE A sur entrée de comptage du TIMER0
  pin 18    RA1/INT1 ---> interrupteur de butée de rentrée sur entrée INT1 (avec PULL UP)
  pin 19    RA0/INT0 ---> interrupteur de butée de sortie sur entrée INT0 (avec PULL UP)
  pin 20    VSS Alim 0V
*/

#define ADDRESS_I2C 0x38 // linux I2C Adresse
#define SIZE_RX_BUFFER 8
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

sbit SA at RA2_bit;
sbit SB at RA4_bit;
sbit LED1 at RC0_bit;

// Sensors
unsigned short optical_state;
int nb_pulse = 0;  // Nombre d'impulsions de la sortie de l'opto OPB461T11
unsigned short butee_out = 0;
unsigned short butee_in = 0;

// Motor
unsigned short motor_speed_in = 50; // 2 octets
unsigned short motor_speed_out = 50; // 2 octets
unsigned short motor_current_speed = 127; // 2 octets

// Regulation
int position_set_point = 0;
signed int error = 0;

// State machine
int system_on = 1;
unsigned short motor_on = 1;
enum robot_state {IDLE,RESET_OUT,REGULATION};
unsigned char state = RESET_OUT;

// Watchdog
unsigned short watchdog_restart = 60;
unsigned short watchdog_restart_default = 60; // 3 s

void i2c_read_data_from_buffer(){
    unsigned short i = 0;
    unsigned short nb_data = nb_rx_octet-1;

    for(i=0; i<nb_data; i++){
        switch(rxbuffer_tab[0]+i){
            case 0x00:
                system_on = (rxbuffer_tab[i+1]!=0x00);
                break;
            case 0x01:
                state = RESET_OUT;
                break;
            case 0x02:
                motor_on = (rxbuffer_tab[i+1]!=0x00);
                break;
            case 0x03:
                RC6_bit = (rxbuffer_tab[i+1]!=0x00);
                break;
            case 0x04:
                LED1 = (rxbuffer_tab[i+1]!=0x00);
                break;

            case 0x10:  // consigne de postion
                if(nb_data >= i+2){
                    position_set_point = 4*(rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8));
                    i++;
                }
                break;

            case 0x12:  // consigne de vitesse in
                if(nb_data >= i+2){
                    motor_speed_in = rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8);
                    i++;
                }
                break;
            case 0x14:  // consigne de vitesse out
                if(nb_data >= i+2){
                    motor_speed_out = rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8);
                    i++;
                }
                break;
            default:
                break;
        }
    }
    watchdog_restart = watchdog_restart_default;
}

/**
 * @brief Fonction qui créer la trame I2C
 * @param num
 */
void i2c_write_data_to_buffer(unsigned short nb_tx_octet){

    switch(rxbuffer_tab[0]+nb_tx_octet){
    case 0x00:
        SSPBUF = nb_pulse;
        break;
    case 0x01:
        SSPBUF = (nb_pulse >> 8);
        break;
    case 0x02:
        SSPBUF = (butee_out & 0b1)
                | ((butee_in & 0b1)<<1)
                | ((state & 0b11) <<2) 
                | ((system_on & 0b1) <<4)
                | ((motor_on & 0b1) << 5)
                | ((RC6_bit & 0b1) << 6);
        break;
    case 0x03:
        SSPBUF = position_set_point;
        break;
    case 0x04:
        SSPBUF = position_set_point >> 8;
        break;
    case 0x05:
        SSPBUF = motor_current_speed;
        break;
    case 0x06:
        SSPBUF = motor_speed_in;
        break;
    case 0x07:
        SSPBUF = motor_speed_in >> 8;
        break;
    case 0x08:
        SSPBUF = motor_speed_out;
        break;
    case 0x09:
        SSPBUF = motor_speed_out >> 8;
        break;
    default:
        SSPBUF = 0x00;
        break;
    }
}

/**
 * @brief Lecture des valeurs des butees
 */
void read_butee(){
    if (RA0_bit == 0)
        butee_out = 1;
    else
        butee_out = 0;
    if (RA1_bit == 0)
        butee_in = 1;
    else
        butee_in = 0;
}
/**
 * @brief Arret du Moteur
 */
void set_motor_cmd_stop(){
    if(motor_current_speed != 127){
        CCPR1 = 50; // Rapport cyclique à 50% pour stopper le moteur et garder du couple
        motor_current_speed = 127;
        RC6_bit = 1;
    }
}

/**
 * @brief Commande du moteur
 * @param i
 */
void set_motor_cmd(unsigned short speed){

    if(system_on==0 || motor_on == 0 || (butee_out == 1 && speed >= 127) || (butee_in == 1 && speed <= 127)){
        set_motor_cmd_stop();
    }
    else if(motor_current_speed != speed){
        motor_current_speed = speed;
        PWM1_Set_Duty(speed);
        CCP1CON=0b10001100; // Half-bridge output: P1A, P1B modulated with dead-band control
        PSTRCON.STRB = 1 ;  // Pour activer la sortie P1B
        PWM1_Start(); // Necessary ?
        RC6_bit = 1;  //Enable L6203
    }
    
}

/**
 * @brief set_motor_cmd_out
 * @param speed
 */
void set_motor_cmd_out(unsigned short speed){
    set_motor_cmd(127 + speed);
}

/**
 * @brief set_motor_cmd_in
 * @param speed
 */
void set_motor_cmd_in(unsigned short speed){
    set_motor_cmd(127 - speed);
}

/**
 * @brief Lecture de la fourche optique
 */
void read_optical_fork(){
    unsigned short new_state = SB<<1 | SA;  //  ou logique de RA3 et RA2

    switch(optical_state){
    case 0x00:
        if(new_state == 0x1)
            nb_pulse--;
        else if(new_state == 0x2)
            nb_pulse++;
        break;
    case 0x01:
        if(new_state == 0x3)
            nb_pulse--;
        else if(new_state == 0x0)
            nb_pulse++;
        break;
    case 0x02:
        if(new_state == 0x0)
            nb_pulse--;
        else if(new_state == 0x3)
            nb_pulse++;
        break;
    case 0x03:
        if(new_state == 0x1)
            nb_pulse++;
        else if(new_state == 0x2)
            nb_pulse--;
        break;
    default:
        break;
    }

    if (optical_state != new_state){
        TXREG = nb_pulse >> 8;
        TXREG = nb_pulse;
    }
    
    optical_state = new_state;  // store the current state value to optical_state value this value will be used in next call
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
 * @brief Initialisation des entrées sorties du PIC
 */
void init_io(){
    ANSEL = 0x00;
    ANSELH = 0x00;

    CM1CON0 = 0x00; // Not using the comparators
    CM2CON0 = 0x00; //

    TRISA0_bit = 1; // RA0 en entrée
    TRISA1_bit = 1; // RA1 en entrée
    TRISA2_bit = 1; // RA2 en entrée
    // TRISA3_bit = 1; // RA3 en entrée  // toujours en entrée MCLR/VPP
    TRISA4_bit = 1; // RA4 en entrée

    TRISA5_bit = 0; // RA5 en sortie

    INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit
    WPUA.WPUA0 = 1; // Pull-up enabled sur RA0, sur inter de butée haute
    WPUA.WPUA1 = 1; // Pull-up enabled sur RA1, sur inter de butée basse
    //WPUA.WPUA2 = 1; // Pull-up enabled sur RA2, sur sortie de l'opto HOA0901 (sortie collecteur ouvert)
    //WPUA.WPUA3 = 1; // Pull-up enabled sur RA3, sur sortie de l'opto HOA0901 (sortie collecteur ouvert)

    TRISB5_bit = 0; // RB5 en sortie
    TRISB7_bit = 0; // RB7 en sortie

    TRISC = 0xFF;
    TRISC0_bit = 0; // RC0 en sortie
    TRISC4_bit = 0; // RC4 en sortie
    TRISC5_bit = 0; // RC5 en sortie
    TRISC6_bit = 0; // RC6 en sortie
    TRISC7_bit = 0; // RC7 en sortie

    RC6_bit = 0;
}

/**
 * @brief main
 */
void main(){
    // Oscillateur interne de 16Mhz
    OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block

    init_io(); // Initialisation des I/O
    init_i2C(ADDRESS_I2C); // Initialisation de l'I2C en esclave
    init_timer0(); // Initialisation du TIMER0 toutes les 1 secondes

    // Initialisation de l'entrée d'interruption INT0 pour la butée haute
    INTCON2.INTEDG0 = 0; // Interrupt on falling edge
    INTCON.INT0IE = 1; //Enables the INT0 external interrupt
    INTCON.INT0IF = 0; // INT0 External Interrupt Flag bit

    // Initialisation de l'entrée d'interruption INT1 pour la butée basse
    INTCON2.INTEDG1 = 0; // Interrupt on falling edge
    INTCON3.INT1IE = 1; //Enables the INT0 external interrupt
    INTCON3.INT1IF = 0; // INT0 External Interrupt Flag bit

    // Initialisation d'interruption sur PORTA change sur les entrées RA2 et RA3
    // sorties A et B du codeur HOA901-12 sur les entrées RA2 et RA3
    //    INTCON.RABIE = 1; // RA and RB Port Change Interrupt Enable bit
    //    IOCA.IOCA2 = 1; // Interrupt-on-change enabled sur RA2
    //    IOCA.IOCA3 = 1; // Interrupt-on-change enabled sur RA3
    //    IOCB = 0x00;    // Interrupt-on-change disabled sur le PORTB
    //    INTCON2.RABIP = 1; //RA and RB Port Change Interrupt Priority bit, high priority

    INTCON3.INT1IP = 1; //INT1 External Interrupt Priority bit, INT0 always a high
    //priority interrupt source

    IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit, low priority
    RCON.IPEN = 1;  //Enable priority levels on interrupts
    INTCON.GIEH = 1; //enable all high-priority interrupts
    INTCON.GIEL = 1; //enable all low-priority interrupts

    INTCON.GIE = 1; // Global Interrupt Enable bit
    INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

    TMR0IE_bit = 1;  //Enable TIMER0
    TMR0ON_bit = 1; // Start TIMER1

    PWM1_Init(10000);  // Fréquence du PWM à 10Khz
    RC6_bit = 0;  //Disable L6203
    CCPR1 = 50; // Rapport cyclique à 50%
    motor_current_speed = 127;
    PWM1_Start();

    UART1_Init(115200);
    delay_ms(100);

    while(1){
        // Global actions
        read_butee();

        // State machine
        switch (state){
        case IDLE: // Idle state
            // debug_uart();
            if(system_on == 1)
                state = RESET_OUT;
            break;

        case RESET_OUT:
            //debug_uart();

            if (system_on == 0){
                state = IDLE;
            }
            else if (butee_out == 1){ // Sortie complète
                state = REGULATION;
                optical_state = SB<<1 | SA;  //  ou logique de RA3 et RA2, lecture du capteur pour initialiser la machine d'état
                nb_pulse = 0; // Reset Nb pulse (The reset is also done in the interrupt function)
                position_set_point = 0;
                LED1 = 1;
            }
            else{
                set_motor_cmd_out(motor_speed_out);
            }

            break;

        case REGULATION:
            //debug_uart();
            read_optical_fork();

            // Regulation
            error = position_set_point - nb_pulse;

            if(error > 0)
                set_motor_cmd_in(motor_speed_in);
            else if(error < 0)
                set_motor_cmd_out(motor_speed_out);
            else // position reached
                set_motor_cmd_stop();

            // State machine
            if(system_on == 0){
                state = IDLE;
                set_motor_cmd_stop();
            }
            break;
        default:
            break;
        }
    
        if(nb_rx_octet>1 && SSPSTAT.P == 1){
          nb_rx_octet = 0;
          i2c_read_data_from_buffer();
        }
    }
}

/**
 * @brief Fonction de gestion des interruptions:
 * interruption sur l'entrée INT0/RA0
 * interruption sur l'entrée INT1/RA1
 * interruption sur TIMER0 interruptions sur front descendant sur l'entrée INT0
 * interruption sur TIMER1 interruptions tous les 1ms pour commande moteur avec rampe
 * interruption sur TIMER3 interruptions tous les 100ms pour visu STATE0(tous les 500ms)
 * interruption sur le bus I2C
 */
void interrupt(){
    // Interruption sur l'entrée INT0/RA0, detection de butée de sortie
    if(INTCON.INT0IF == 1) {
        if (!RA0_bit){
            delay_ms(2);
            if (!RA0_bit){
                butee_out = 1;
                //nb_pulse = 0; // Reset pulse
                CCPR1 = 50;  // Rapport cyclique à 50% pour stopper le moteur et garder du couple (=> pose problème car dépend du sens !)
                motor_current_speed = 127;
            }
        }
        INTCON.INT0IF = 0;
    }

    // Interruption sur l'entrée INT1/RA1, detection de butée de rentree
    else if(INTCON3.INT1IF == 1) {
        if (!RA1_bit){
            delay_ms(2);
            if (!RA1_bit){
                butee_in = 1;
                CCPR1 = 50;  // Rapport cyclique à 50% pour stopper le moteur et garder du couple
                motor_current_speed = 127;
            }
        }
        INTCON3.INT1IF = 0;
    }

    else if (TMR0IF_bit){
        // Watchdog
        if(watchdog_restart>0)
          watchdog_restart--;  
        else{
          position_set_point = 0;
          watchdog_restart = watchdog_restart_default;
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

        if(nb_rx_octet>1){
          Delay_us(30); // Wait P signal ?
          if(SSPSTAT.P == 0)
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
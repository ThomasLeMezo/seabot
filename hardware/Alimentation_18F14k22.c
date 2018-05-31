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
contrôle la tension d'alimentation de 4 batteries, la valeur est retourner sur 
ordre via l'I2C.
La sortie RA4 commande trois LED de repérage via le circuit ZXLD1350.

14/03/18/ Implantation et essai du programme, seuil des batteries à corriger

*/


sbit BAT1 at LATC.B0; // entrée de controle de tension BAT1
sbit BAT2 at LATC.B1; // entrée de controle de tension BAT2
sbit BAT3 at LATC.B2; // entrée de controle de tension BAT2
sbit BAT4 at LATC.B3; // entrée de controle de tension BAT2
sbit ILS at LATA.B2; //  entrée de l'ILS

sbit LED1 at LATA.B1; // sortie LED1
sbit LED at LATA.B0; // sortie LED
sbit LED_PUISSANCE at LATA.B4; // sortie LED de puissance
sbit ALIM at LATA.B5; // sortie MOSFET de puissance, commande de l'alimentation

#define QUANTUM 0.015625;
#define WARNING_LOW_VOLTAGE 10.4
#define ADDRESS_I2C 0x72;

unsigned short Default = 0;
unsigned short POWER = 0;

unsigned short compteur1 = 0;
unsigned short compteur2 = 0;
unsigned short compteur3 = 0;
unsigned short i = 0;

// ILS
#define ILS_CPT_TIME 4
unsigned short ils_cpt = 4;
bool ils_removed = true;

// State Machine
enum power_state {IDLE,POWER_ON,SLEEP};

// Batteries
unsigned int battery_voltage[4];
bool battery_voltage_default[4];
bool battery_global_default = false;

// I2C
#define SIZE_RX_BUFFER 3
unsigned short rxbuffer_tab[SIZE_RX_BUFFER];
unsigned short j = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

// Flasher LED
unsigned short led_puissance_delay = 20; // 100ms * val (default = 2s)
unsigned short cpt_led_puissance = 20;


/**************************************************************************************************
* Fonction pour convertir les données I2C transmises du maître
* Le maître transmet trois octets à suivre:
*
*
* exemple: le maitre transmet les trames suivantes

Consigne on met en marche la sortie LED_PUISSANCE (LED_PUISSANCE=1):
I2C1_Start();
I2C1_Write(0x72);   // send byte via I2C  (device address + W) adresse 0x72
I2C1_Write(0x00);  //
I2C1_Write(0x02);  //
I2C1_Write(0x01);  //
I2C1_Stop();

Consigne on coupe la sortie LED_PUISSANCE (LED_PUISSANCE=0):
I2C1_Start();
I2C1_Write(0x72);   // send byte via I2C  (device address + W) adresse 0x72
I2C1_Write(0x00);  //
I2C1_Write(0x02);  //
I2C1_Write(0x00);  //
I2C1_Stop();


i2cdetect -y 2 pour la pcduino

adresse du circuit 0x72 --> 0111 0010
un i2cget ou un i2cset postionne le dernier bit à 1, suivant une lecture ou écriture

0011 1001 --> 0x39

i2cset -y 2 0x39 0x00 0x01 0x00 i --> on coupe l'alimentation générale ALIM (ALIM=0)
i2cset -y 2 0x39 0x00 0x02 0x00 i --> on coupe la sortie LED_PUISSANCE (LED_PUISSANCE=0)
i2cset -y 2 0x39 0x00 0x02 0x01 i --> on met en marche la sortie LED_PUISSANCE (LED_PUISSANCE=0)


Exemple de séquence en simulation sous Proteus:

S 0x72 0x00 0x01 0x00 P --> on coupe l'alimentation générale ALIM (ALIM=0)
S 0x72 0x00 0x02 0x00 P --> on coupe la sortie LED_PUISSANCE (LED_PUISSANCE=0)
S 0x72 0x00 0x02 0x01 P --> on met en marche la sortie LED_PUISSANCE (LED_PUISSANCE=1)


i2cget -y 2 0x39 0x01 w --> on demande la valeur de la tension de la batterie 1
i2cget -y 2 0x39 0x02 w --> on demande la valeur de la tension de la batterie 2
i2cget -y 2 0x39 0x03 w --> on demande la valeur de la tension de la batterie 3


**************************************************************************************************/

/**
 * @brief i2c_read_data_from_buffer
 */
void i2c_read_data_from_buffer(){
    switch(rxbuffer_tab[0]){
    case 0x01:  // alimentation
        if(rxbuffer_tab[1] == 0)
            state = IDLE;
        else
            state = POWER_ON;   
        break;
    case 0x02:  // led power
        if(rxbuffer_tab[1] == 0){
          TMR3ON_bit = 0; // Stop timer
          LED_PUISSANCE = 0;
        }
        else
          TMR3ON_bit = 1; // Start timer
        break;
    case 0x03:
        led_puissance_delay = rxbuffer_tab[1];
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
        SSPBUF = TMR3ON_bit;
    default:
        SSPBUF = 0x00;
        break;
    }
}

/**
 * @brief read batteries voltage
 */
void read_batteries_voltage(){
    battery_voltage[0] = ADC_Get_Sample(4);   // Get 10-bit results of AD conversion AN4 batterie 1
    // delay_ms(5);
    battery_voltage[1] = ADC_Get_Sample(5);   // Get 10-bit results of AD conversion AN5 batterie 2
    // delay_ms(5);
    battery_voltage[2] = ADC_Get_Sample(6);   // Get 10-bit results of AD conversion AN6 batterie 3
    // delay_ms(5);
    battery_voltage[3] = ADC_Get_Sample(7);   // Get 10-bit results of AD conversion AN7 batterie 4
    // delay_ms(5);
}

/**
 * @brief analyze batteries voltage
 * 3 cellules lithium --> 4.1v/éléments --> en fin de charge
 * 9v totalement déchargé
 */
void analyze_batteries_voltage(){
    battery_global_default = false;
    for(unsigned short i=0; i<4; i++){
        if((float)(battery_voltage[i]) * QUANTUM < WARNING_LOW_VOLTAGE){
            battery_voltage_default[i] = true;
            battery_global_default = true;
        }
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
 * @brief InitINT2
 * Fonction d'initialisation de l'entrée d'interruption INT2 pour la détection de fermeture/ouverture
 * de l'ILS, détection du front montant (ouverture de l'ILS).
 */
void init_io_ils(){
    INTCON2.INTEDG2 = 0; // Interrupt on falling edge
    INTCON3.INT2IE = 0; //Enables the INT2 external interrupt
    INTCON3.INT2IF = 0; // INT2 External Interrupt Flag bit
}

/**
 * @brief init_timer0
 * Fonction d'initialisation du TIMER0
 * Prescaler 1:128; TMR0 Preload = 3036; Actual Interrupt Time : 1 s
 */
void init_timer0(){
    //T0CON = 0x86; // TIMER0 ON time 2s
    T0CON = 0x85; // TIMER0 ON
    TMR0H = 0x0B;
    TMR0L = 0xDC;
    TMR0IE_bit = 0;
}

/**
 * @brief init_timer1
 * Fonction d'initialisation du TIMER1
 * Prescaler 1:8; TMR1 Preload = 15536; Actual Interrupt Time : 100 ms
 */
void init_timer1(){
    T1CON = 0x31;
    TMR1IF_bit = 0;
    TMR1H = 0x77;
    TMR1L = 0x48;
    TMR1IE_bit = 0;
}

/**
 * @brief init_timer2
 * Fonction d'initialisation du TIMER2
 * Prescaler 1:1; Postscaler 1:2; TMR2 Preload = 199; Actual Interrupt Time : 10 us
 */
void init_timer2(){
    T2CON = 0x08;
    TMR2IE_bit = 0;
    PR2 = 79;
    INTCON = 0xC0;
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
    TRISA0_bit = 0; // RA0 en sortie
    TRISA2_bit = 1; // RA2 en entrée
    TRISA4_bit = 0; // RA4 en sortie
    TRISA5_bit = 0; // RA5 en sortie

    TRISA1_bit = 0; // RA1 en sortie

    INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit
    WPUA.WPUA2 = 1; // Pull-up enabled sur RA2, sur inter de butée basse

    TRISB4_bit = 1; // RB4 en entrée
    TRISB6_bit = 1; // RB6 en entrée

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
    // Oscillateur interne de 16Mhz
    OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block

    init_io(); // Initialisation des I/O
    init_i2c(); // Initialisation de l'I2C en esclave
    init_io_ils(); // Initialisation de l'entrée d'interruption INT2 pour l'ILS
    init_timer0(); // Initialisation du TIMER0 toutes les 2 secondes
    init_timer1(); // Initialisation du TIMER1 toutes les 100ms
    init_timer3(); // Initialisation du TIMER3 toutes les 100ms

    ADC_Init();

    LED = 0; // sortie LED
    LED1 = 0;
    LED_PUISSANCE = 0; // sortie LED de puissance
    ALIM = 0; // sortie MOSFET de puissance, commande de l'alimentation
    POWER = 0;
    battery_global_default = false;

    UART1_Init(9600);

    TMR0IE_bit = 1;  //Enable TIMER0
    TMR1IE_bit = 1;  //Enable TIMER1
    TMR3IE_bit = 1;  //Enable TIMER3
    INTCON3.INT2IE = 1; //Enable the INT2 external interrupt
    PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
    INTCON.GIE = 1; // Global Interrupt Enable bit
    INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

    i = 0;
    j = 0;

    while(1){
        read_batteries_voltage();
        analyze_batteries_voltage();

        switch (state){
            case IDLE: // Idle state
                ALIM = 0;

                if(ILS==0){ // Magnet detected
                    ils_cpt--;
                    LED1=1;
                }
                else{
                    ils_cpt = ILS_CPT_TIME;
                    LED1=0;
                    ils_removed = true;
                }

                if(ils_removed && ils_cpt == 0){
                    ils_cpt = ILS_CPT_TIME;
                    state = POWER_ON;
                    ils_removed = false;
                }
                break;
            case POWER_ON:
                ALIM = 1;

                if(ILS==0){ // Magnet detected
                    ils_cpt--;
                    LED1=1;
                }
                else{
                    ils_cpt = ILS_CPT_TIME;
                    LED1=0;
                    ils_removed = true;
                }
                if(ils_removed && ils_cpt == 0){
                    ils_cpt = ILS_CPT_TIME;
                    state = IDLE;
                    ils_removed = false;
                }

                break;
            case SLEEP:

                break;
            default:
                break;
        }
        delay_ms(500);
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
            j = SSPBUF;
        }

        PIR1.SSPIF = 0; // reset SSP interrupt flag
    }
    if (PIR2.BCLIF)
        PIR2.BCLIF = 0;


    /// ************************************************** //
    /// ********************** ILS  ********************** //
    // Interruption sur l'entrée INT2/RA2, detection d'ouverture de l'ILS
    if(INTCON3.INT2IF == 1) {
        INTCON3.INT2IF = 0;
    }


    /// ************************************************** //
    /// ********************** TIMERS  ******************* //

    // Interruption du TIMER0 (1 ou 2 s)
    if (TMR0IF_bit){

        if (POWER==1)
          T0CON = 0x86; // TIMER0 ON time 2 secondes
        else
          T0CON = 0x85; // TIMER0 ON time 1 secondes

        TMR0H = 0x0B;
        TMR0L = 0xDC;

        if (battery_global_default == true){
            LED = 1;
            T1CON.TMR1ON = 1;
        }
        else LED =~LED;

        TMR0IF_bit = 0;
    }


    // Interruption du TIMER1 (100 ms) => battery default
    if (TMR1IF_bit){

        TMR1H = 0x77;
        TMR1L = 0x48;

        switch (Default){
        case 1: T1CON.TMR1ON = 0;
            LED = 0;
            break;
        case 2: compteur1++;
            if (compteur1 == 1){
                LED = 0;
            }
            if (compteur1 == 2){
                LED = 1;
            }
            if (compteur1 == 3){
                LED = 0;
                compteur1 = 0;
                T1CON.TMR1ON = 0;
            }
            break;
        case 3: compteur1++;
            if (compteur1 == 1){
                LED = 0;
            }
            if (compteur1 == 2){
                LED = 1;
            }
            if (compteur1 == 3){
                LED = 0;
            }
            if (compteur1 == 4){
                LED = 1;
            }
            if (compteur1 == 5){
                LED = 0;
                compteur1 = 0;
                T1CON.TMR1ON = 0;
            }
            break;
        case 4: compteur1++;
            if (compteur1 == 1){
                LED = 0;
            }
            if (compteur1 == 2){
                LED = 1;
            }
            if (compteur1 == 3){
                LED = 0;
            }
            if (compteur1 == 4){
                LED = 1;
            }
            if (compteur1 == 5){
                LED = 0;
            }
            if (compteur1 == 6){
                LED = 1;
            }
            if (compteur1 == 7){
                LED = 0;
                compteur1 = 0;
                T1CON.TMR1ON = 0;
            }
            break;

        }

        TMR1IF_bit = 0;
    }


    // Interruption du TIMER3 (Led Puissance)
    if (TMR3IF_bit){
        cpt_led_puissance--;
        if (cpt_led_puissance == 0){
            LED_PUISSANCE = 1;
            cpt_led_puissance=led_puissance_delay;
        }
        else 
          LED_PUISSANCE = 0;

        // Restart the timer
        TMR3H = 0x3C;
        TMR3L = 0xB0;
        TMR3IF_bit = 0;
    }
}

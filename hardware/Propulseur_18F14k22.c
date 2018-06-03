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

const Adresse_I2C = 0x70; //adresse I2C du circuit
unsigned short rxbuffer_I2C_Octet0 = 0;
unsigned short rxbuffer_I2C_Octet1 = 0;
unsigned short rxbuffer_I2C_Octet2 = 0;
unsigned short val1 = 0;
unsigned short val2 = 0;
unsigned short val3 = 0;

unsigned compteur1 = 0;
unsigned compteur2 = 0;
unsigned compteur3 = 0;
unsigned val_compteur1 = 0;
unsigned val_compteur2 = 0;
unsigned val_compteur3 = 0;
unsigned new_val_compteur1 = 0;
unsigned new_val_compteur2 = 0;
unsigned new_val_compteur3 = 0;
unsigned rxbuffer_I2C = 0;
unsigned i = 0;
unsigned j = 0;
unsigned trame_I2C = 0;


/**************************************************************************************************
* Fonction pour stopper les moteurs
**************************************************************************************************/

void Moteur_stop(){
  new_val_compteur1 = 150; // moteur 1 stop
  new_val_compteur2 = 150; // moteur 2 stop
  new_val_compteur3 = 150; // moteur 3 stop
}


/**************************************************************************************************
* Fonction pour convertir les données I2C transmises du maître
* Le maître transmet trois octets à suivre:
* octet0 = 0x00 --> numéro du moteur
* octet1 = 0x00 --> valeur de vitesse sur deux octets
* octet2 = 0x00 --> valeur de vitesse sur deux octets
*
*
* exemple: le maitre transmet les trames suivantes

Consigne Moteur 1 STOP:
I2C1_Start();
I2C1_Write(0x70);   // send byte via I2C  (device address + W) adresse 0x42
I2C1_Write(0x01);  //  le moteur 1 a pour consigne de vitesse 1500us, donc le moteur
I2C1_Write(0x05);  //  1 est stopé
I2C1_Write(0xDC);  //
I2C1_Stop();

Consigne Moteur 2 vitesse max en avant:
I2C1_Start();
I2C1_Write(0x70);   // send byte via I2C  (device address + W) adresse 0x42
I2C1_Write(0x02);  //  le moteur 2 a pour consigne de vitesse 1900us, donc le moteur
I2C1_Write(0x07);  //  2 est à sa vitesse max en avant
I2C1_Write(0x6C);  //
I2C1_Stop();

Consigne Moteur 3 vitesse max en arrière:
I2C1_Start();
I2C1_Write(0x70);   // send byte via I2C  (device address + W) adresse 0x42
I2C1_Write(0x03);  //  le moteur 3 a pour consigne de vitesse 1100us, donc le moteur
I2C1_Write(0x04);  //  3 est à sa vitesse max en arrière
I2C1_Write(0x4C);  //
I2C1_Stop();


i2cdetect -y 2 pour la pcduino

adresse du circuit 0x70 --> 0111 0000
un i2cget ou un i2cset postionne le dernier bit à 1, suivant une lecture ou écriture

0011 1000 --> 0x38

i2cset -y 2 0x38 0x01 0x05 0xDC i --> le moteur 1 a pour vitesse 5DC donc 1500us
i2cset -y 2 0x38 0x02 0x07 0x6C i --> le moteur 2 a pour vitesse 76C donc 1900us
i2cset -y 2 0x38 0x03 0x04 0x4C i --> le moteur 3 a pour vitesse 44c donc 1100us



Exemple de séquence en simulation sous Proteus:

S 0x70 0x01 0x07 0x6c P --> moteur 1 consigne 0x076C (1900us)
S 0x70 0x01 0x05 0xdc P --> moteur 1 consigne 0x05DC (1500us)
S 0x70 0x01 0x04 0x4c P --> moteur 1 consigne 0x044C (1100us)

Exemple de programme Python

import smbus
import time

bus=smbus.SMBus(2)
add=0x38
time_sleep = 0.35

#vitesse =  [1500, 1550,1600,1650,1700,1500,1450,1400,1390,1380,1500]
#vitesse = [1500, 1550]
#vitesse =  [1500, 1550,1600,1620,1500]
vitesse =  [1500, 1540,1560,1580,1600,1500]


#####

bus.write_i2c_block_data(add,0x00,[0x06,0x12])
time.sleep(time_sleep)

bus.write_i2c_block_data(add,0x01,[0x06,0x12])

bus.write_i2c_block_data(add,0x02,[0x05,0xAA])

for t in range(30):
        for v in vitesse:
                octet1 =  v >> 8
                octet2 =  v

                #bus.write_i2c_block_data(add,0x01,[0x06 ,0x12])   # 1554 us
                #bus.write_i2c_block_data(add,0x01,[0x05 ,0x78])     # on stop le moteur 1 --->1500 us

                time.sleep(time_sleep)

                #bus.write_i2c_block_data(add,0x01,[octet1,octet2])

                #bus.write_i2c_block_data(add,0x02,[octet1,octet2])

                bus.write_i2c_block_data(add,0x03,[octet1,octet2])
                #bus.write_i2c_block_data(add,0x01,[octet1,octet2])

bus.write_i2c_block_data(add,0x00,[0x06,0x12])



**************************************************************************************************/

void Convertion_I2C(){

  if (rxbuffer_I2C_Octet0 == 0){
     Moteur_stop();
     val1 = 1;
     val2 = 1;
     val3 = 1;
  }
  else{
    rxbuffer_I2C = ((rxbuffer_I2C_Octet1 << 8) | (rxbuffer_I2C_Octet2));

      if ((rxbuffer_I2C >= 1100) && (rxbuffer_I2C <= 1900)){

          switch (rxbuffer_I2C_Octet0) {

                     case 1 :  // moteur 1
                              new_val_compteur1 = rxbuffer_I2C / 10;
                              if (new_val_compteur1 != val_compteur1) val1 = 1;
                              break;
                     case 2 :  // moteur 2
                              new_val_compteur2 = rxbuffer_I2C / 10;
                              if (new_val_compteur1 != val_compteur1) val2 = 1;
                              break;
                     case 3 :  // moteur 3
                              new_val_compteur3 = rxbuffer_I2C / 10;
                              if (new_val_compteur1 != val_compteur1) val3 = 1;
                              break;
                     default: Moteur_stop();
            }

     }
   }
}


/**************************************************************************************************
* Fonction d'initialisation de l'I2C in slave mode
**************************************************************************************************/

void InitI2C(){

     SSPADD = Adresse_I2C; // Address Register, Get address (7bit). Lsb is read/write flag
     SSPCON1 = 0x3E; // SYNC SERIAL PORT CONTROL REGISTER
                     // bit 3-0 SSPM3:SSPM0: I2C Firmware Controlled Master mode,
                     // 7-bit address with START and STOP bit interrupts enabled
                     // bit 4 CKP: 1 = Enable clock
                     // bit 5 SSPEN: Enables the serial port and configures the SDA and SCL
                     // pins as the source of the serial port pins
     SSPCON2 = 0x00;
     SSPSTAT=0x00;

     SSPSTAT.SMP = 1; // 1 = Slew rate control disabled for standard speed mode
                      // (100 kHz and 1 MHz)
     SSPSTAT.CKE = 1; // 1 = Input levels conform to SMBus spec
  
     PIE1.SSPIE = 0; // Synchronous Serial Port Interrupt Enable bit
     PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
                     // a transmission/reception has taken place.

}


/**************************************************************************************************
* Fonction d'initialisation du TIMER0
* Prescaler 1:8; TMR0 Preload = 25536; Actual Interrupt Time : 20 ms
**************************************************************************************************/

void initTimer0(){

     T0CON = 0x82;
  
     TMR0H = 0x63;
     TMR0L = 0xC0;

     TMR0L = 0x38;

     TMR0IE_bit = 0;

}


/**************************************************************************************************
* Fonction d'initialisation du TIMER1
* Prescaler 1:1; TMR1 Preload = 65136; Actual Interrupt Time : 10 us
**************************************************************************************************/

void initTimer1(){

     T1CON = 0x00;
  
     TMR1IF_bit = 0;

     TMR1H = 0xFF;
     TMR1L = 0x80;

     TMR1IE_bit = 0;
     INTCON = 0xC0;

}


/**************************************************************************************************
* Fonction d'initialisation du TIMER2
* Prescaler 1:1; Postscaler 1:2; TMR2 Preload = 199; Actual Interrupt Time : 10 us
**************************************************************************************************/

void initTimer2(){

     T2CON = 0x08;

     TMR2IE_bit = 0;
     PR2 = 79;
     INTCON = 0xC0;

}


/**************************************************************************************************
* Fonction d'initialisation du TIMER3
* Prescaler 1:1; TMR1 Preload = 65136; Actual Interrupt Time : 10 US
**************************************************************************************************/

void initTimer3(){

     T3CON = 0x00;
  
     TMR3IF_bit = 0;

     TMR3H = 0xFF;
     TMR3L = 0x84;

     TMR3IE_bit = 0;
     INTCON = 0xC0;

}


/**************************************************************************************************
* Initialisation des entrées sorties du PIC
**************************************************************************************************/

void Init(){

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


/**************************************************************************************************
* Programme principal
**************************************************************************************************/

void Main(){

     Init(); // Initialisation des I/O
     InitI2C(); // Initialisation de l'I2C en esclave
     initTimer0(); // Initialisation du TIMER0 toutes les 20 ms
     initTimer1(); // Initialisation du TIMER1 toutes les 1us
     initTimer2(); // Initialisation du TIMER2 toutes les 1us
     initTimer3(); // Initialisation du TIMER3 toutes les 1us

     val_compteur1 = 150;  // moteur 1 stop
     val_compteur2 = 150;  // moteur 2 stop
     val_compteur3 = 150;  // moteur 3 stop
     new_val_compteur1 = 150;  // moteur 1 stop
     new_val_compteur2 = 150;  // moteur 2 stop
     new_val_compteur3 = 150;  // moteur 3 stop

     //UART1_Init(115200);
     LATC = 0;
 
     for (i=0; i< 90; i++){  // Tempo de 45 secondes pour faire monter Linux sur la carte PCDUINO
       LED =~ LED;
       delay_ms(500);
     }

     TMR0IE_bit = 1;
     TMR1IE_bit = 1;
     TMR2IE_bit = 1;
     TMR3IE_bit = 1;

     PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
     INTCON.GIE = 1; // Global Interrupt Enable bit
     INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

     i = 0;
     j = 0;

while(1){
        
         if (trame_I2C == 1){
            trame_I2C = 0;
            Convertion_I2C();
         }
        
}//While(1)

}// Main


/**************************************************************************************************
* Fonction de gestion des interruptions:
* interruption sur TIMER3 interruptions tous les 100ms pour visu STATE0(tous les 500ms)
* interruption sur le bus I2C
**************************************************************************************************/

void interrupt(){

// Interruption sur le bus I2C, le bus est en esclave

if (PIR1.SSPIF){  // I2C Interrupt

  PIR1.SSPIF = 0; // reset SSP interrupt flag

// transmit data to master
  if (SSPSTAT.R_W == 1){   // Read/Write bit Information, Read request from master

    SSPCON1.CKP = 1;        // Release SCL line
    j = SSPBUF;
    return;
  }

  if (SSPSTAT.BF == 0){ // Buffer Full Status bit, Data transmit complete, SSPBUF is empty

    j = SSPBUF;        // Nothing in buffer so exit
    return;

  }

  //recieve data from master
  if (SSPSTAT.R_W == 0){

    if (SSPSTAT.D_A == 0){
      j = SSPBUF;
      i = 0;
    }

    if (SSPSTAT.D_A == 1){    //1 = Indicates that the last byte received or transmitted was data

      if(i==0){
      rxbuffer_I2C_Octet0 = SSPBUF;  // get data octet1
      i++;
      }
      else if(i==1){
      rxbuffer_I2C_Octet1 = SSPBUF;  // get data octet2
      i++;
      }
      else if(i==2){
      rxbuffer_I2C_Octet2 = SSPBUF;  // get data octet2
      i=0;
      trame_I2C=1;
      }

      j = SSPBUF;       // read buffer to clear flag [address]
      return;
     }
  }

  j = SSPBUF;     // read buffer to clear flag [address]

  }

  
// Interruption TIMER0 toutes les 10us
  if (T0IE_bit && TMR0IF_bit){

    LED =~ LED;

    MOT1 = 1;

    if (val1 == 1){
      val_compteur1 = new_val_compteur1;
      val1 = 0;
     }
    if (val2 == 1){
      val_compteur2 = new_val_compteur2;
      val2 = 0;
     }
    if (val3 == 1){
      val_compteur3 = new_val_compteur3;
      val3 = 0;
     }

    T1CON = 0x01;

    TMR0H = 0x63;
    TMR0L = 0xC0;

    TMR0IF_bit = 0;

  }
 
  
// Interruption TIMER1 toutes les 10us

 if (TMR1IE_bit && TMR1IF_bit){

    compteur1++;

    if (compteur1 ==  val_compteur1){

      MOT1 = 0;
      compteur1 = 0;
      T1CON = 0x00;
      
      val_compteur2 = new_val_compteur2;
      MOT2 = 1;
      T2CON = 0x0C;
      
    }

    TMR1H = 0xFF;
    TMR1L = 0x80;

    TMR1IF_bit = 0;

   }


// Interruption TIMER3 toutes les 10us

 if (TMR3IE_bit && TMR3IF_bit){

    compteur3++;

    if (compteur3 ==  val_compteur3){

      MOT3 = 0;
      compteur3 = 0;
      T3CON = 0x00;
      
    }

    TMR3H = 0xFF;
    TMR3L = 0x84;

    TMR3IF_bit = 0;

   }


// Interruption TIMER2 toutes les us

 if (TMR2IE_bit && TMR2IF_bit){

    compteur2++;

    if (compteur2 ==  val_compteur2){

      MOT2 = 0;
      compteur2 = 0;
      PR2 = 79;
      TMR2 = 0;
      T2CON = 0x08;
      
      val_compteur3 = new_val_compteur3;
      MOT3 = 1;
      T3CON = 0x01;
      
    }

    TMR2IF_bit = 0;

   }

}
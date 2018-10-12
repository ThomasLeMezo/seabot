#line 1 "E:/hardware/Propulseur_18F14k22.c"
#line 59 "E:/hardware/Propulseur_18F14k22.c"
const unsigned short ADDRESS_I2C = 0x20;

unsigned short rxbuffer_tab[ 8 ];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

void init_i2c();

sbit MOT1 at LATC.B0;
sbit MOT2 at LATC.B1;
sbit MOT3 at LATC.B2;

sbit LED at LATA.B2;




unsigned short cmd_motor[3] = { 150 ,  150 ,  150 };


unsigned char cpt_motor_1 = 0;
unsigned char cpt_motor_2 = 0;
unsigned char cpt_motor_3 = 0;
unsigned int cpt_global =  2000 ;


unsigned short watchdog_restart = 3;
#line 92 "E:/hardware/Propulseur_18F14k22.c"
void i2c_read_data_from_buffer(){
 unsigned short k=0;
 unsigned short nb_motor=0;

 for(k=1; k<nb_rx_octet; k++){
 nb_motor = rxbuffer_tab[0] + k - 1;

 if(nb_motor<3){
 if(rxbuffer_tab[k]>=110 && rxbuffer_tab[k]<=190)
 cmd_motor[nb_motor] = rxbuffer_tab[k];
 else
 cmd_motor[nb_motor] =  150 ;
 }
 }
 watchdog_restart =  3 ;
}
#line 113 "E:/hardware/Propulseur_18F14k22.c"
void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
 SSPBUF = 0x00;
}
#line 122 "E:/hardware/Propulseur_18F14k22.c"
void init_timer0(){
 T0CON = 0x85;
 TMR0H = 0x0B;
 TMR0L = 0xDC;
 TMR0IE_bit = 0;
}
#line 134 "E:/hardware/Propulseur_18F14k22.c"
void init_timer1(){
 T1CON = 0x01;
 TMR1IF_bit = 0;
 TMR1H = 255;
 TMR1L = 136;
 TMR1IE_bit = 0;
 INTCON = 0xC0;
}
#line 148 "E:/hardware/Propulseur_18F14k22.c"
void init_io(){
 ANSEL = 0xFF;

 CM1CON0 = 0x00;
 CM2CON0 = 0x00;

 PORTA = 0;
 LATA = 0;

 TRISA = 0xFF;

 TRISA2_bit = 0;

 INTCON.RABIE = 0;
 INTCON2.RABPU = 1;

 TRISB5_bit = 0;
 TRISB7_bit = 0;

 TRISC = 0xFF;
 TRISC0_bit = 0;
 TRISC1_bit = 0;
 TRISC2_bit = 0;
}
#line 176 "E:/hardware/Propulseur_18F14k22.c"
void main(){

 init_io();
 init_i2c();
 init_timer0();
 init_timer1();

 UART1_Init(115200);
 LATC = 0;

 TMR0IE_bit = 1;
 TMR0ON_bit = 1;

 TMR1IE_bit = 1;
 TMR1ON_bit = 1;

 INTCON3.INT1IP = 1;


 IPR1.SSPIP = 0;
 RCON.IPEN = 1;
 INTCON.GIEH = 1;
 INTCON.GIEL = 1;

 INTCON.GIE = 1;
 INTCON.PEIE = 1;

 LED = 1;
 delay_ms(2000);


 while(1){
 if(cmd_motor[0] !=  150  || cmd_motor[1] !=  150  || cmd_motor[2] !=  150 )
 LED = 1;
 else
 LED = 0;

 delay_ms(250);
 }
}
#line 220 "E:/hardware/Propulseur_18F14k22.c"
void interrupt(){




 if (TMR1IF_bit){

 if(cpt_global==0){
 MOT1 = 1;
 MOT2 = 1;
 MOT3 = 1;
 cpt_global =  2000 ;

 cpt_motor_1 = cmd_motor[0];
 cpt_motor_2 = cmd_motor[1];
 cpt_motor_3 = cmd_motor[2];
 }
 else{
 cpt_global--;


 if(cpt_motor_1==0)
 MOT1 = 0;
 else
 cpt_motor_1--;


 if(cpt_motor_2==0)
 MOT2 = 0;
 else
 cpt_motor_2--;


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

 if(watchdog_restart>0)
 watchdog_restart--;
 else{
 cmd_motor[0] =  150 ;
 cmd_motor[1] =  150 ;
 cmd_motor[2] =  150 ;
 watchdog_restart =  3 ;
 }

 TMR0H = 0x0B;
 TMR0L = 0xDC;
 TMR0IF_bit = 0;
 }
}
#line 284 "E:/hardware/Propulseur_18F14k22.c"
void init_i2c(){


 TRISB4_bit = 1;
 TRISB6_bit = 1;


 PIE1.SSPIE = 1;
 PIR1.SSPIF = 0;

 PIR2.BCLIE = 1;
 PIR2.BCLIF = 0;


 SSPADD = (ADDRESS_I2C << 1);
 SSPMSK = 0xFF;



 SSPSTAT.SMP = 1;


 SSPSTAT.CKE = 1;


 SSPCON2 = 0x00;
 SSPCON2.GCEN = 0;
 SSPCON2.SEN = 1;


 SSPCON1.WCOL = 0;
 SSPCON1.SSPOV = 0;
 SSPCON1.CKP = 1;
 SSPCON1.SSPM3 = 0b1;
 SSPCON1.SSPM2 = 0b1;
 SSPCON1.SSPM1 = 0b1;
 SSPCON1.SSPM0 = 0b0;


 SSPCON1.SSPEN = 1;
}
#line 329 "E:/hardware/Propulseur_18F14k22.c"
void interrupt_low(){
 if (PIR1.SSPIF){

 if(SSPCON1.SSPOV || SSPCON1.WCOL){
 SSPCON1.SSPOV = 0;
 SSPCON1.WCOL = 0;
 tmp_rx = SSPBUF;
 }



 if (SSPSTAT.R_W == 0){
 if(SSPSTAT.P == 0){
 if (SSPSTAT.D_A == 0){
 nb_rx_octet = 0;
 tmp_rx = SSPBUF;
 }
 else{
 if(nb_rx_octet <  8 ){
 rxbuffer_tab[nb_rx_octet] = SSPBUF;
 nb_rx_octet++;
 }
 else{
 tmp_rx = SSPBUF;
 }
 }
 }

 if(nb_rx_octet>1){
 Delay_us(30);
 if(SSPSTAT.P == 1){
 i2c_read_data_from_buffer();
 nb_rx_octet = 0;
 }
 }
 }


 else{
 if(SSPSTAT.D_A == 0){
 nb_tx_octet = 0;
 tmp_rx = SSPBUF;
 }


 i2c_write_data_to_buffer(nb_tx_octet);
 nb_tx_octet++;
 }

 SSPCON1.CKP = 1;
 PIR1.SSPIF = 0;
 }
}

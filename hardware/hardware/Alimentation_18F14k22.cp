#line 1 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
#line 43 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
const unsigned short ADDRESS_I2C = 0x39;

unsigned short rxbuffer_tab[ 8 ];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

void init_i2c();

sbit BAT1 at PORTC.B0;
sbit BAT2 at PORTC.B1;
sbit BAT3 at PORTC.B2;
sbit BAT4 at PORTC.B3;
sbit ILS at PORTA.B2;

sbit LED1 at LATA.B1;
sbit LED at LATA.B0;
sbit LED_PUISSANCE at LATA.B4;
sbit ALIM at LATA.B5;



unsigned short ils_cpt = 4;
unsigned short ils_removed = 1;


enum power_state {IDLE,POWER_ON,WAIT_TO_SLEEP, SLEEP};
unsigned short state = IDLE;
unsigned int cpt_wait = 0;




unsigned int battery_voltage[4];
unsigned short battery_global_default = 0;


unsigned short start_led_puissance = 0;
unsigned short led_puissance_delay = 20;
unsigned short cpt_led_puissance = 20;


unsigned short led_delay = 100;
unsigned short cpt_led = 100;
unsigned short set_led_on = 0;


unsigned char time_to_start[3] = {0, 0, 5};
unsigned char time_to_stop = 60;

unsigned char default_time_to_start[3] = {0, 0, 5};
unsigned char default_time_to_stop = 60;

unsigned short start_time_to_stop = 0;
unsigned short start_time_to_power_on = 0;
unsigned short start_time_to_start = 0;

unsigned short k = 0;


unsigned int watchdog_restart = 3600;
unsigned int watchdog_restart_default = 3600;
#line 109 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void i2c_read_data_from_buffer(){
 unsigned short i = 0;

 for(i=0; i<(nb_rx_octet-1); i++){
 switch(rxbuffer_tab[0]+i){
 case 0x00:
 switch(rxbuffer_tab[i+1]){
 case 0x01:
 state = POWER_ON;
 break;
 case 0x02:
 time_to_stop = default_time_to_stop;
 start_time_to_stop = 1;
 state = WAIT_TO_SLEEP;
 break;
 default:
 break;
 }
 break;
 case 0x01:
 start_led_puissance = (rxbuffer_tab[i+1]!=0x00);
 break;
 case 0x02:
 led_puissance_delay = rxbuffer_tab[i+1];
 break;
 case 0x03:
 default_time_to_start[0] = rxbuffer_tab[i+1];
 break;
 case 0x04:
 default_time_to_start[1] = rxbuffer_tab[i+1];
 break;
 case 0x05:
 default_time_to_start[2] = rxbuffer_tab[i+1];
 break;
 case 0x06:
 default_time_to_stop = rxbuffer_tab[i+1];
 break;
 default:
 break;
 }
 }
}
#line 156 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
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
 case 0xC0:
 SSPBUF =  0x01 ;
 break;
 default:
 SSPBUF = 0x00;
 break;
 }
 watchdog_restart = watchdog_restart_default;
}
#line 201 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void read_batteries_voltage(){
 battery_voltage[0] = ADC_Get_Sample(4);
 battery_voltage[1] = ADC_Get_Sample(5);
 battery_voltage[2] = ADC_Get_Sample(6);
 battery_voltage[3] = ADC_Get_Sample(7);
}
#line 213 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void analyze_batteries_voltage(){
 unsigned short l = 0;
 battery_global_default = 0;

 for(l=0; l<4; l++){
 if(battery_voltage[l] <  665 )
 battery_global_default = 1;
 }
}
#line 228 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void init_timer0(){
 T0CON = 0x85;
 TMR0H = 0x0B;
 TMR0L = 0xDC;
 TMR0IE_bit = 0;
}
#line 240 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void init_timer3(){
 T3CON = 0x30;
 TMR3IF_bit = 0;
 TMR3H = 0x3C;
 TMR3L = 0xB0;
 TMR3IE_bit = 0;
}
#line 252 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void init_io(){
 ANSEL = 0xF0;

 CM1CON0 = 0x00;
 CM2CON0 = 0x00;



 TRISA = 0xFF;

 TRISA0_bit = 0;
 TRISA2_bit = 1;
 TRISA4_bit = 0;
 TRISA5_bit = 0;

 TRISA1_bit = 0;

 INTCON2.RABPU = 0;
 WPUA.WPUA2 = 1;

 TRISB5_bit = 1;
 TRISB7_bit = 0;

 TRISC = 0xFF;
 TRISC0_bit = 1;
 TRISC1_bit = 1;
 TRISC2_bit = 1;
 TRISC3_bit = 1;
}
#line 286 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void main(){

 OSCCON = 0b01110010;

 init_io();
 init_i2c();
 init_timer0();
 init_timer3();

 ADC_Init();

 LED = 0;
 LED1 = 0;
 LED_PUISSANCE = 0;
 ALIM = 0;
 battery_global_default = 0;

 UART1_Init(115200);

 delay_ms(1000);

 INTCON3.INT1IP = 0;


 RCON.IPEN = 1;
 IPR1.SSPIP = 0;
 INTCON.GIEH = 1;
 INTCON.GIEL = 1;

 INTCON.GIE = 1;
 INTCON.PEIE = 1;

 TMR0IE_bit = 1;
 TMR0ON_bit = 1;

 TMR3IE_bit = 1;
 TMR3ON_bit = 1;

 while(1){
 read_batteries_voltage();
 analyze_batteries_voltage();



 switch (state){
 case IDLE:
 ALIM = 0;
 led_delay = 50;
 start_led_puissance = 0;

 if(ILS==0){
 ils_cpt--;
 set_led_on = 1;
 }
 else{
 ils_cpt =  4 ;
 set_led_on = 0;
 ils_removed = 1;
 }

 if(ils_removed == 1 && ils_cpt == 0){
 ils_cpt =  4 ;
 state = POWER_ON;
 ils_removed = 0;
 set_led_on = 0;
 }
 break;

 case POWER_ON:
 ALIM = 1;
 if(battery_global_default == 1)
 led_delay = 5;
 else
 led_delay = 20;

 if(ILS==0){
 ils_cpt--;
 set_led_on = 1;
 }
 else{
 ils_cpt =  4 ;
 set_led_on = 0;
 ils_removed = 1;
 }

 if(ils_removed == 1 && ils_cpt == 0){
 ils_cpt =  4 ;
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
 led_delay = 600;
 if(time_to_start[0] == 0 && time_to_start[1] == 0 && time_to_start[2] == 0){
 state = POWER_ON;
 }
 break;

 default:
 state = POWER_ON;
 break;
 }

 for(cpt_wait=0; cpt_wait< 10000 ; cpt_wait++){
 delay_us(50);

 if(nb_rx_octet>1 && SSPSTAT.P == 1){
 i2c_read_data_from_buffer();
 nb_rx_octet = 0;
 }
 }
 }
}
#line 422 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
void interrupt(){




 if (TMR0IF_bit){


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


 if(state == POWER_ON){
 if(watchdog_restart>0)
 watchdog_restart--;
 else{

 default_time_to_start[0] = 0;
 default_time_to_start[1] = 0;
 default_time_to_start[2] = 2;
 time_to_stop = 10;

 state = WAIT_TO_SLEEP;
 watchdog_restart = watchdog_restart_default;
 }
 }


 TMR0H = 0x0B;
 TMR0L = 0xDC;
 TMR0IF_bit = 0;
 }


 else if (TMR3IF_bit){

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


 if(set_led_on == 1)
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
#line 515 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
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
 SSPCON1.SSPM3 = 0b0;
 SSPCON1.SSPM2 = 0b1;
 SSPCON1.SSPM1 = 0b1;
 SSPCON1.SSPM0 = 0b0;


 SSPCON1.SSPEN = 1;
}
#line 560 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/Alimentation_18F14k22.c"
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

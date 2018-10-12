#line 1 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
#line 29 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
const unsigned short ADDRESS_I2C = 0x38;

unsigned short rxbuffer_tab[ 8 ];
unsigned short tmp_rx = 0;
unsigned short nb_tx_octet = 0;
unsigned short nb_rx_octet = 0;

void init_i2c();

sbit SA at RA2_bit;
sbit SB at RA4_bit;
sbit LED1 at RC0_bit;
sbit LED2 at RC2_bit;


unsigned short optical_state;
int nb_pulse = 0;
unsigned short butee_out = 0;
unsigned short butee_in = 0;


unsigned short motor_speed_in = 50;
unsigned short motor_speed_out = 50;
unsigned short motor_current_speed = 127;


int position_set_point = 0;
signed int error = 0;
unsigned long int position_reached_max_value = 50000;
unsigned long int position_reached_cpt = 0;
unsigned short position_reached_enable = 1;
unsigned short error_interval = 3;


unsigned short motor_on = 1;
enum robot_state {RESET_OUT,REGULATION};
unsigned char state = RESET_OUT;


unsigned short watchdog_restart = 60;
unsigned short watchdog_restart_default = 60;

void i2c_read_data_from_buffer(){
 unsigned short i = 0;
 unsigned short nb_data = nb_rx_octet-1;

 for(i=0; i<nb_data; i++){
 switch(rxbuffer_tab[0]+i){
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
 case 0x05:
 error_interval = rxbuffer_tab[i+1];
 break;
 case 0x06:
 position_reached_enable = rxbuffer_tab[i+1];
 break;
 case 0x10:
 if(nb_data >= i+2){
 position_set_point = 4*(rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8));
 i++;
 }
 break;

 case 0x12:
 if(nb_data >= i+2){
 motor_speed_in = rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8);
 i++;
 }
 break;
 case 0x14:
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
#line 125 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
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
 | ((motor_on & 0b1) << 4)
 | ((RC6_bit & 0b1) << 5);
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
#line 171 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void read_butee(){
 if (RA0_bit == 0){
 butee_out = 1;
 LED1 = 1;
 }
 else{
 butee_out = 0;
 LED1 = 0;
 }
 if (RA1_bit == 0)
 butee_in = 1;
 else
 butee_in = 0;
}
#line 188 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void set_motor_cmd_stop(){
 if(motor_current_speed != 127){

 PWM1_Set_Duty(127);
 motor_current_speed = 127;
 RC6_bit = 1;
 }
}
#line 201 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void set_motor_cmd(unsigned short speed){
 if(motor_on == 0 || (butee_out == 1 && speed >= 127) || (butee_in == 1 && speed <= 127)){
 set_motor_cmd_stop();
 if(position_reached_enable == 1){
 if(position_reached_cpt>position_reached_max_value){
 RC6_bit = 0;
 }
 else
 position_reached_cpt++;
 }
 }
 else if(motor_current_speed != speed){
 position_reached_cpt = 0;
 motor_current_speed = speed;
 PWM1_Set_Duty(speed);
 RC6_bit = 1;
 }





}
#line 229 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void set_motor_cmd_out(unsigned short speed){
 set_motor_cmd(127 + speed);
}
#line 237 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void set_motor_cmd_in(unsigned short speed){
 set_motor_cmd(127 - speed);
}
#line 244 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void read_optical_fork(){
 unsigned short new_state = SB<<1 | SA;

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

 optical_state = new_state;
}
#line 289 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void init_timer0(){
 T0CON = 0x85;
 TMR0H = 0x0B;
 TMR0L = 0xDC;
 TMR0IE_bit = 0;
}
#line 299 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void init_io(){
 ANSEL = 0x00;
 ANSELH = 0x00;

 CM1CON0 = 0x00;
 CM2CON0 = 0x00;

 TRISA0_bit = 1;
 TRISA1_bit = 1;
 TRISA2_bit = 1;

 TRISA4_bit = 1;

 TRISA5_bit = 0;

 INTCON2.RABPU = 0;
 WPUA.WPUA0 = 1;
 WPUA.WPUA1 = 1;



 TRISB5_bit = 0;
 TRISB7_bit = 0;

 TRISC = 0xFF;
 TRISC0_bit = 0;
 TRISC2_bit = 0;
 TRISC4_bit = 0;
 TRISC5_bit = 0;
 TRISC6_bit = 0;
 TRISC7_bit = 0;

 RC6_bit = 0;
}
#line 337 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void main(){

 OSCCON = 0b01110010;

 init_io();
 init_i2C();
 init_timer0();


 INTCON2.INTEDG0 = 0;
 INTCON.INT0IE = 1;
 INTCON.INT0IF = 0;


 INTCON2.INTEDG1 = 0;
 INTCON3.INT1IE = 1;
 INTCON3.INT1IF = 0;









 INTCON3.INT1IP = 1;


 IPR1.SSPIP = 0;
 RCON.IPEN = 1;
 INTCON.GIEH = 1;
 INTCON.GIEL = 1;

 INTCON.GIE = 1;
 INTCON.PEIE = 1;

 TMR0IE_bit = 1;
 TMR0ON_bit = 1;

 PWM1_Init(10000);
 RC6_bit = 0;
 CCPR1 = 50;
 motor_current_speed = 127;
 PWM1_Start();





 UART1_Init(115200);
 delay_ms(100);

 while(1){

 read_butee();


 switch (state){
 case RESET_OUT:

 LED2 = 1;

 if (butee_out == 1){
 state = REGULATION;
 optical_state = SB<<1 | SA;
 nb_pulse = 0;
 position_set_point = 0;
 }
 else
 set_motor_cmd_out(motor_speed_out);

 break;

 case REGULATION:

 read_optical_fork();
 LED2 = 0;


 error = position_set_point - nb_pulse;

 if(error > error_interval)
 set_motor_cmd_in(motor_speed_in);
 else if(error < error_interval)
 set_motor_cmd_out(motor_speed_out);
 else
 set_motor_cmd_stop();

 break;
 default:
 break;
 }
 }
}
#line 442 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
void interrupt(){

 if(INTCON.INT0IF == 1) {
 if (!RA0_bit){
 delay_ms(2);
 if (!RA0_bit){
 butee_out = 1;
 CCPR1 = 50;

 }
 }
 INTCON.INT0IF = 0;
 }


 else if(INTCON3.INT1IF == 1) {
 if (!RA1_bit){
 delay_ms(2);
 if (!RA1_bit){
 butee_in = 1;
 CCPR1 = 50;

 }
 }
 INTCON3.INT1IF = 0;
 }

 else if (TMR0IF_bit){

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
#line 487 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
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
#line 532 "C:/users/lemezoth/My Documents/workspaceFlotteur/hardware/hardware/robot_18F14k22_test.c"
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

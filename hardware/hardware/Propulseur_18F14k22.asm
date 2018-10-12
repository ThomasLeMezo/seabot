
_i2c_read_data_from_buffer:

;Propulseur_18F14k22.c,92 :: 		void i2c_read_data_from_buffer(){
;Propulseur_18F14k22.c,93 :: 		unsigned short k=0;
	CLRF        i2c_read_data_from_buffer_k_L0+0 
	CLRF        i2c_read_data_from_buffer_nb_motor_L0+0 
;Propulseur_18F14k22.c,96 :: 		for(k=1; k<nb_rx_octet; k++){
	MOVLW       1
	MOVWF       i2c_read_data_from_buffer_k_L0+0 
L_i2c_read_data_from_buffer0:
	MOVF        _nb_rx_octet+0, 0 
	SUBWF       i2c_read_data_from_buffer_k_L0+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer1
;Propulseur_18F14k22.c,97 :: 		nb_motor = rxbuffer_tab[0] + k - 1;
	MOVF        i2c_read_data_from_buffer_k_L0+0, 0 
	ADDWF       _rxbuffer_tab+0, 0 
	MOVWF       R0 
	DECF        R0, 0 
	MOVWF       R1 
	MOVF        R1, 0 
	MOVWF       i2c_read_data_from_buffer_nb_motor_L0+0 
;Propulseur_18F14k22.c,99 :: 		if(nb_motor<3){
	MOVLW       3
	SUBWF       R1, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer3
;Propulseur_18F14k22.c,100 :: 		if(rxbuffer_tab[k]>=110 && rxbuffer_tab[k]<=190)
	MOVLW       _rxbuffer_tab+0
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	MOVWF       FSR0H 
	MOVF        i2c_read_data_from_buffer_k_L0+0, 0 
	ADDWF       FSR0, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR0H, 1 
	MOVLW       110
	SUBWF       POSTINC0+0, 0 
	BTFSS       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer6
	MOVLW       _rxbuffer_tab+0
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	MOVWF       FSR0H 
	MOVF        i2c_read_data_from_buffer_k_L0+0, 0 
	ADDWF       FSR0, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR0H, 1 
	MOVF        POSTINC0+0, 0 
	SUBLW       190
	BTFSS       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer6
L__i2c_read_data_from_buffer44:
;Propulseur_18F14k22.c,101 :: 		cmd_motor[nb_motor] = rxbuffer_tab[k];
	MOVLW       _cmd_motor+0
	MOVWF       FSR1 
	MOVLW       hi_addr(_cmd_motor+0)
	MOVWF       FSR1H 
	MOVF        i2c_read_data_from_buffer_nb_motor_L0+0, 0 
	ADDWF       FSR1, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR1H, 1 
	MOVLW       _rxbuffer_tab+0
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	MOVWF       FSR0H 
	MOVF        i2c_read_data_from_buffer_k_L0+0, 0 
	ADDWF       FSR0, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR0H, 1 
	MOVF        POSTINC0+0, 0 
	MOVWF       POSTINC1+0 
	GOTO        L_i2c_read_data_from_buffer7
L_i2c_read_data_from_buffer6:
;Propulseur_18F14k22.c,103 :: 		cmd_motor[nb_motor] = MOTOR_CMD_STOP;
	MOVLW       _cmd_motor+0
	MOVWF       FSR1 
	MOVLW       hi_addr(_cmd_motor+0)
	MOVWF       FSR1H 
	MOVF        i2c_read_data_from_buffer_nb_motor_L0+0, 0 
	ADDWF       FSR1, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR1H, 1 
	MOVLW       150
	MOVWF       POSTINC1+0 
L_i2c_read_data_from_buffer7:
;Propulseur_18F14k22.c,104 :: 		}
L_i2c_read_data_from_buffer3:
;Propulseur_18F14k22.c,96 :: 		for(k=1; k<nb_rx_octet; k++){
	INCF        i2c_read_data_from_buffer_k_L0+0, 1 
;Propulseur_18F14k22.c,105 :: 		}
	GOTO        L_i2c_read_data_from_buffer0
L_i2c_read_data_from_buffer1:
;Propulseur_18F14k22.c,106 :: 		watchdog_restart = WATCHDOG_RESTART_DEFAULT;
	MOVLW       3
	MOVWF       _watchdog_restart+0 
;Propulseur_18F14k22.c,107 :: 		}
L_end_i2c_read_data_from_buffer:
	RETURN      0
; end of _i2c_read_data_from_buffer

_i2c_write_data_to_buffer:

;Propulseur_18F14k22.c,113 :: 		void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
;Propulseur_18F14k22.c,114 :: 		SSPBUF = 0x00;
	CLRF        SSPBUF+0 
;Propulseur_18F14k22.c,115 :: 		}
L_end_i2c_write_data_to_buffer:
	RETURN      0
; end of _i2c_write_data_to_buffer

_init_timer0:

;Propulseur_18F14k22.c,122 :: 		void init_timer0(){
;Propulseur_18F14k22.c,123 :: 		T0CON = 0x85; // TIMER0 ON (1 s)
	MOVLW       133
	MOVWF       T0CON+0 
;Propulseur_18F14k22.c,124 :: 		TMR0H = 0x0B;
	MOVLW       11
	MOVWF       TMR0H+0 
;Propulseur_18F14k22.c,125 :: 		TMR0L = 0xDC;
	MOVLW       220
	MOVWF       TMR0L+0 
;Propulseur_18F14k22.c,126 :: 		TMR0IE_bit = 0;
	BCF         TMR0IE_bit+0, BitPos(TMR0IE_bit+0) 
;Propulseur_18F14k22.c,127 :: 		}
L_end_init_timer0:
	RETURN      0
; end of _init_timer0

_init_timer1:

;Propulseur_18F14k22.c,134 :: 		void init_timer1(){
;Propulseur_18F14k22.c,135 :: 		T1CON = 0x01;
	MOVLW       1
	MOVWF       T1CON+0 
;Propulseur_18F14k22.c,136 :: 		TMR1IF_bit = 0;
	BCF         TMR1IF_bit+0, BitPos(TMR1IF_bit+0) 
;Propulseur_18F14k22.c,137 :: 		TMR1H = 255;
	MOVLW       255
	MOVWF       TMR1H+0 
;Propulseur_18F14k22.c,138 :: 		TMR1L = 136;
	MOVLW       136
	MOVWF       TMR1L+0 
;Propulseur_18F14k22.c,139 :: 		TMR1IE_bit = 0;
	BCF         TMR1IE_bit+0, BitPos(TMR1IE_bit+0) 
;Propulseur_18F14k22.c,140 :: 		INTCON = 0xC0;
	MOVLW       192
	MOVWF       INTCON+0 
;Propulseur_18F14k22.c,141 :: 		}
L_end_init_timer1:
	RETURN      0
; end of _init_timer1

_init_io:

;Propulseur_18F14k22.c,148 :: 		void init_io(){
;Propulseur_18F14k22.c,149 :: 		ANSEL = 0xFF;
	MOVLW       255
	MOVWF       ANSEL+0 
;Propulseur_18F14k22.c,151 :: 		CM1CON0 = 0x00; // Not using the comparators
	CLRF        CM1CON0+0 
;Propulseur_18F14k22.c,152 :: 		CM2CON0 = 0x00; //
	CLRF        CM2CON0+0 
;Propulseur_18F14k22.c,154 :: 		PORTA = 0;
	CLRF        PORTA+0 
;Propulseur_18F14k22.c,155 :: 		LATA = 0;
	CLRF        LATA+0 
;Propulseur_18F14k22.c,157 :: 		TRISA = 0xFF;
	MOVLW       255
	MOVWF       TRISA+0 
;Propulseur_18F14k22.c,159 :: 		TRISA2_bit = 0; // RA2 en sortie
	BCF         TRISA2_bit+0, BitPos(TRISA2_bit+0) 
;Propulseur_18F14k22.c,161 :: 		INTCON.RABIE = 0;
	BCF         INTCON+0, 3 
;Propulseur_18F14k22.c,162 :: 		INTCON2.RABPU = 1; // PORTA and PORTB Pull-up disable bit
	BSF         INTCON2+0, 7 
;Propulseur_18F14k22.c,164 :: 		TRISB5_bit = 0; // RB5 en sortie
	BCF         TRISB5_bit+0, BitPos(TRISB5_bit+0) 
;Propulseur_18F14k22.c,165 :: 		TRISB7_bit = 0; // RB7 en sortie
	BCF         TRISB7_bit+0, BitPos(TRISB7_bit+0) 
;Propulseur_18F14k22.c,167 :: 		TRISC = 0xFF;
	MOVLW       255
	MOVWF       TRISC+0 
;Propulseur_18F14k22.c,168 :: 		TRISC0_bit = 0; // RC0 en sortie
	BCF         TRISC0_bit+0, BitPos(TRISC0_bit+0) 
;Propulseur_18F14k22.c,169 :: 		TRISC1_bit = 0; // RC1 en sortie
	BCF         TRISC1_bit+0, BitPos(TRISC1_bit+0) 
;Propulseur_18F14k22.c,170 :: 		TRISC2_bit = 0; // RC2 en sortie
	BCF         TRISC2_bit+0, BitPos(TRISC2_bit+0) 
;Propulseur_18F14k22.c,171 :: 		}
L_end_init_io:
	RETURN      0
; end of _init_io

_main:

;Propulseur_18F14k22.c,176 :: 		void main(){
;Propulseur_18F14k22.c,178 :: 		init_io(); // Initialisation des I/O
	CALL        _init_io+0, 0
;Propulseur_18F14k22.c,179 :: 		init_i2c(); // Initialisation de l'I2C en esclave
	CALL        _init_i2c+0, 0
;Propulseur_18F14k22.c,180 :: 		init_timer0(); // Initi TIMER0 toutes les 1s
	CALL        _init_timer0+0, 0
;Propulseur_18F14k22.c,181 :: 		init_timer1(); // Initialisation du TIMER1 toutes les 1us
	CALL        _init_timer1+0, 0
;Propulseur_18F14k22.c,183 :: 		UART1_Init(115200);
	BSF         BAUDCON+0, 3, 0
	CLRF        SPBRGH+0 
	MOVLW       138
	MOVWF       SPBRG+0 
	BSF         TXSTA+0, 2, 0
	CALL        _UART1_Init+0, 0
;Propulseur_18F14k22.c,184 :: 		LATC = 0;
	CLRF        LATC+0 
;Propulseur_18F14k22.c,186 :: 		TMR0IE_bit = 1; //Enable TIMER0
	BSF         TMR0IE_bit+0, BitPos(TMR0IE_bit+0) 
;Propulseur_18F14k22.c,187 :: 		TMR0ON_bit = 1; // Start TIMER0
	BSF         TMR0ON_bit+0, BitPos(TMR0ON_bit+0) 
;Propulseur_18F14k22.c,189 :: 		TMR1IE_bit = 1;
	BSF         TMR1IE_bit+0, BitPos(TMR1IE_bit+0) 
;Propulseur_18F14k22.c,190 :: 		TMR1ON_bit = 1; // Start TIMER1
	BSF         TMR1ON_bit+0, BitPos(TMR1ON_bit+0) 
;Propulseur_18F14k22.c,192 :: 		INTCON3.INT1IP = 1; //INT1 External Interrupt Priority bit, INT0 always a high
	BSF         INTCON3+0, 6 
;Propulseur_18F14k22.c,195 :: 		IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit, low priority
	BCF         IPR1+0, 3 
;Propulseur_18F14k22.c,196 :: 		RCON.IPEN = 1;  //Enable priority levels on interrupts
	BSF         RCON+0, 7 
;Propulseur_18F14k22.c,197 :: 		INTCON.GIEH = 1; //enable all high-priority interrupts
	BSF         INTCON+0, 7 
;Propulseur_18F14k22.c,198 :: 		INTCON.GIEL = 1; //enable all low-priority interrupts
	BSF         INTCON+0, 6 
;Propulseur_18F14k22.c,200 :: 		INTCON.GIE = 1; // Global Interrupt Enable bit
	BSF         INTCON+0, 7 
;Propulseur_18F14k22.c,201 :: 		INTCON.PEIE = 1; // Peripheral Interrupt Enable bit
	BSF         INTCON+0, 6 
;Propulseur_18F14k22.c,203 :: 		LED = 1;
	BSF         LATA+0, 2 
;Propulseur_18F14k22.c,204 :: 		delay_ms(2000);
	MOVLW       163
	MOVWF       R11, 0
	MOVLW       87
	MOVWF       R12, 0
	MOVLW       2
	MOVWF       R13, 0
L_main8:
	DECFSZ      R13, 1, 1
	BRA         L_main8
	DECFSZ      R12, 1, 1
	BRA         L_main8
	DECFSZ      R11, 1, 1
	BRA         L_main8
	NOP
;Propulseur_18F14k22.c,207 :: 		while(1){
L_main9:
;Propulseur_18F14k22.c,208 :: 		if(cmd_motor[0] != MOTOR_CMD_STOP || cmd_motor[1] != MOTOR_CMD_STOP || cmd_motor[2] != MOTOR_CMD_STOP)
	MOVF        _cmd_motor+0, 0 
	XORLW       150
	BTFSS       STATUS+0, 2 
	GOTO        L__main45
	MOVF        _cmd_motor+1, 0 
	XORLW       150
	BTFSS       STATUS+0, 2 
	GOTO        L__main45
	MOVF        _cmd_motor+2, 0 
	XORLW       150
	BTFSS       STATUS+0, 2 
	GOTO        L__main45
	GOTO        L_main13
L__main45:
;Propulseur_18F14k22.c,209 :: 		LED = 1;
	BSF         LATA+0, 2 
	GOTO        L_main14
L_main13:
;Propulseur_18F14k22.c,211 :: 		LED = 0;
	BCF         LATA+0, 2 
L_main14:
;Propulseur_18F14k22.c,213 :: 		delay_ms(250);
	MOVLW       21
	MOVWF       R11, 0
	MOVLW       75
	MOVWF       R12, 0
	MOVLW       190
	MOVWF       R13, 0
L_main15:
	DECFSZ      R13, 1, 1
	BRA         L_main15
	DECFSZ      R12, 1, 1
	BRA         L_main15
	DECFSZ      R11, 1, 1
	BRA         L_main15
	NOP
;Propulseur_18F14k22.c,214 :: 		}
	GOTO        L_main9
;Propulseur_18F14k22.c,215 :: 		}
L_end_main:
	GOTO        $+0
; end of _main

_interrupt:

;Propulseur_18F14k22.c,220 :: 		void interrupt(){
;Propulseur_18F14k22.c,225 :: 		if (TMR1IF_bit){
	BTFSS       TMR1IF_bit+0, BitPos(TMR1IF_bit+0) 
	GOTO        L_interrupt16
;Propulseur_18F14k22.c,227 :: 		if(cpt_global==0){
	MOVLW       0
	XORWF       _cpt_global+1, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__interrupt55
	MOVLW       0
	XORWF       _cpt_global+0, 0 
L__interrupt55:
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt17
;Propulseur_18F14k22.c,228 :: 		MOT1 = 1;
	BSF         LATC+0, 0 
;Propulseur_18F14k22.c,229 :: 		MOT2 = 1;
	BSF         LATC+0, 1 
;Propulseur_18F14k22.c,230 :: 		MOT3 = 1;
	BSF         LATC+0, 2 
;Propulseur_18F14k22.c,231 :: 		cpt_global = PWM_PERIOD;
	MOVLW       208
	MOVWF       _cpt_global+0 
	MOVLW       7
	MOVWF       _cpt_global+1 
;Propulseur_18F14k22.c,233 :: 		cpt_motor_1 = cmd_motor[0];
	MOVF        _cmd_motor+0, 0 
	MOVWF       _cpt_motor_1+0 
;Propulseur_18F14k22.c,234 :: 		cpt_motor_2 = cmd_motor[1];
	MOVF        _cmd_motor+1, 0 
	MOVWF       _cpt_motor_2+0 
;Propulseur_18F14k22.c,235 :: 		cpt_motor_3 = cmd_motor[2];
	MOVF        _cmd_motor+2, 0 
	MOVWF       _cpt_motor_3+0 
;Propulseur_18F14k22.c,236 :: 		}
	GOTO        L_interrupt18
L_interrupt17:
;Propulseur_18F14k22.c,238 :: 		cpt_global--;
	MOVLW       1
	SUBWF       _cpt_global+0, 1 
	MOVLW       0
	SUBWFB      _cpt_global+1, 1 
;Propulseur_18F14k22.c,241 :: 		if(cpt_motor_1==0)
	MOVF        _cpt_motor_1+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt19
;Propulseur_18F14k22.c,242 :: 		MOT1 = 0;
	BCF         LATC+0, 0 
	GOTO        L_interrupt20
L_interrupt19:
;Propulseur_18F14k22.c,244 :: 		cpt_motor_1--;
	DECF        _cpt_motor_1+0, 1 
L_interrupt20:
;Propulseur_18F14k22.c,247 :: 		if(cpt_motor_2==0)
	MOVF        _cpt_motor_2+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt21
;Propulseur_18F14k22.c,248 :: 		MOT2 = 0;
	BCF         LATC+0, 1 
	GOTO        L_interrupt22
L_interrupt21:
;Propulseur_18F14k22.c,250 :: 		cpt_motor_2--;
	DECF        _cpt_motor_2+0, 1 
L_interrupt22:
;Propulseur_18F14k22.c,253 :: 		if(cpt_motor_3==0)
	MOVF        _cpt_motor_3+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt23
;Propulseur_18F14k22.c,254 :: 		MOT3 = 0;
	BCF         LATC+0, 2 
	GOTO        L_interrupt24
L_interrupt23:
;Propulseur_18F14k22.c,256 :: 		cpt_motor_3--;
	DECF        _cpt_motor_3+0, 1 
L_interrupt24:
;Propulseur_18F14k22.c,257 :: 		}
L_interrupt18:
;Propulseur_18F14k22.c,259 :: 		TMR1H = 255;
	MOVLW       255
	MOVWF       TMR1H+0 
;Propulseur_18F14k22.c,260 :: 		TMR1L = 136;
	MOVLW       136
	MOVWF       TMR1L+0 
;Propulseur_18F14k22.c,261 :: 		TMR1IF_bit = 0;
	BCF         TMR1IF_bit+0, BitPos(TMR1IF_bit+0) 
;Propulseur_18F14k22.c,262 :: 		}
	GOTO        L_interrupt25
L_interrupt16:
;Propulseur_18F14k22.c,264 :: 		else if (TMR0IF_bit){
	BTFSS       TMR0IF_bit+0, BitPos(TMR0IF_bit+0) 
	GOTO        L_interrupt26
;Propulseur_18F14k22.c,266 :: 		if(watchdog_restart>0)
	MOVF        _watchdog_restart+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt27
;Propulseur_18F14k22.c,267 :: 		watchdog_restart--;
	DECF        _watchdog_restart+0, 1 
	GOTO        L_interrupt28
L_interrupt27:
;Propulseur_18F14k22.c,269 :: 		cmd_motor[0] = MOTOR_CMD_STOP;
	MOVLW       150
	MOVWF       _cmd_motor+0 
;Propulseur_18F14k22.c,270 :: 		cmd_motor[1] = MOTOR_CMD_STOP;
	MOVLW       150
	MOVWF       _cmd_motor+1 
;Propulseur_18F14k22.c,271 :: 		cmd_motor[2] = MOTOR_CMD_STOP;
	MOVLW       150
	MOVWF       _cmd_motor+2 
;Propulseur_18F14k22.c,272 :: 		watchdog_restart = WATCHDOG_RESTART_DEFAULT;
	MOVLW       3
	MOVWF       _watchdog_restart+0 
;Propulseur_18F14k22.c,273 :: 		}
L_interrupt28:
;Propulseur_18F14k22.c,275 :: 		TMR0H = 0x0B;
	MOVLW       11
	MOVWF       TMR0H+0 
;Propulseur_18F14k22.c,276 :: 		TMR0L = 0xDC;
	MOVLW       220
	MOVWF       TMR0L+0 
;Propulseur_18F14k22.c,277 :: 		TMR0IF_bit = 0;
	BCF         TMR0IF_bit+0, BitPos(TMR0IF_bit+0) 
;Propulseur_18F14k22.c,278 :: 		}
L_interrupt26:
L_interrupt25:
;Propulseur_18F14k22.c,279 :: 		}
L_end_interrupt:
L__interrupt54:
	RETFIE      1
; end of _interrupt

_init_i2c:

;Propulseur_18F14k22.c,284 :: 		void init_i2c(){
;Propulseur_18F14k22.c,287 :: 		TRISB4_bit = 1; // RB4 en entrée
	BSF         TRISB4_bit+0, BitPos(TRISB4_bit+0) 
;Propulseur_18F14k22.c,288 :: 		TRISB6_bit = 1; // RB6 en entrée
	BSF         TRISB6_bit+0, BitPos(TRISB6_bit+0) 
;Propulseur_18F14k22.c,291 :: 		PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
	BSF         PIE1+0, 3 
;Propulseur_18F14k22.c,292 :: 		PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
	BCF         PIR1+0, 3 
;Propulseur_18F14k22.c,294 :: 		PIR2.BCLIE = 1;
	BSF         PIR2+0, 3 
;Propulseur_18F14k22.c,295 :: 		PIR2.BCLIF = 0;
	BCF         PIR2+0, 3 
;Propulseur_18F14k22.c,298 :: 		SSPADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
	MOVLW       64
	MOVWF       SSPADD+0 
;Propulseur_18F14k22.c,299 :: 		SSPMSK = 0xFF; // A zero (‘0’) bit in the SSPMSK register has the effect of making
	MOVLW       255
	MOVWF       SSPMSK+0 
;Propulseur_18F14k22.c,303 :: 		SSPSTAT.SMP = 1; // Slew Rate Control bit
	BSF         SSPSTAT+0, 7 
;Propulseur_18F14k22.c,306 :: 		SSPSTAT.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)
	BSF         SSPSTAT+0, 6 
;Propulseur_18F14k22.c,309 :: 		SSPCON2 = 0x00;
	CLRF        SSPCON2+0 
;Propulseur_18F14k22.c,310 :: 		SSPCON2.GCEN = 0; // General Call Enable bit (0 = disabled)
	BCF         SSPCON2+0, 7 
;Propulseur_18F14k22.c,311 :: 		SSPCON2.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)
	BSF         SSPCON2+0, 0 
;Propulseur_18F14k22.c,314 :: 		SSPCON1.WCOL = 0; // Write Collision Detect bit
	BCF         SSPCON1+0, 7 
;Propulseur_18F14k22.c,315 :: 		SSPCON1.SSPOV = 0; // Receive Overflow Indicator bit
	BCF         SSPCON1+0, 6 
;Propulseur_18F14k22.c,316 :: 		SSPCON1.CKP = 1; // SCK Release Control bit (1=Release clock)
	BSF         SSPCON1+0, 4 
;Propulseur_18F14k22.c,317 :: 		SSPCON1.SSPM3 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 3 
;Propulseur_18F14k22.c,318 :: 		SSPCON1.SSPM2 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 2 
;Propulseur_18F14k22.c,319 :: 		SSPCON1.SSPM1 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 1 
;Propulseur_18F14k22.c,320 :: 		SSPCON1.SSPM0 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BCF         SSPCON1+0, 0 
;Propulseur_18F14k22.c,323 :: 		SSPCON1.SSPEN = 1; // Synchronous Serial Port Enable bit
	BSF         SSPCON1+0, 5 
;Propulseur_18F14k22.c,324 :: 		}
L_end_init_i2c:
	RETURN      0
; end of _init_i2c

_interrupt_low:
	MOVWF       ___Low_saveWREG+0 
	MOVF        STATUS+0, 0 
	MOVWF       ___Low_saveSTATUS+0 
	MOVF        BSR+0, 0 
	MOVWF       ___Low_saveBSR+0 

;Propulseur_18F14k22.c,329 :: 		void interrupt_low(){
;Propulseur_18F14k22.c,330 :: 		if (PIR1.SSPIF){  // I2C Interrupt
	BTFSS       PIR1+0, 3 
	GOTO        L_interrupt_low29
;Propulseur_18F14k22.c,332 :: 		if(SSPCON1.SSPOV || SSPCON1.WCOL){
	BTFSC       SSPCON1+0, 6 
	GOTO        L__interrupt_low46
	BTFSC       SSPCON1+0, 7 
	GOTO        L__interrupt_low46
	GOTO        L_interrupt_low32
L__interrupt_low46:
;Propulseur_18F14k22.c,333 :: 		SSPCON1.SSPOV = 0;
	BCF         SSPCON1+0, 6 
;Propulseur_18F14k22.c,334 :: 		SSPCON1.WCOL = 0;
	BCF         SSPCON1+0, 7 
;Propulseur_18F14k22.c,335 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Propulseur_18F14k22.c,336 :: 		}
L_interrupt_low32:
;Propulseur_18F14k22.c,340 :: 		if (SSPSTAT.R_W == 0){
	BTFSC       SSPSTAT+0, 2 
	GOTO        L_interrupt_low33
;Propulseur_18F14k22.c,341 :: 		if(SSPSTAT.P == 0){
	BTFSC       SSPSTAT+0, 4 
	GOTO        L_interrupt_low34
;Propulseur_18F14k22.c,342 :: 		if (SSPSTAT.D_A == 0){ // Address
	BTFSC       SSPSTAT+0, 5 
	GOTO        L_interrupt_low35
;Propulseur_18F14k22.c,343 :: 		nb_rx_octet = 0;
	CLRF        _nb_rx_octet+0 
;Propulseur_18F14k22.c,344 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Propulseur_18F14k22.c,345 :: 		}
	GOTO        L_interrupt_low36
L_interrupt_low35:
;Propulseur_18F14k22.c,347 :: 		if(nb_rx_octet < SIZE_RX_BUFFER){
	MOVLW       8
	SUBWF       _nb_rx_octet+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt_low37
;Propulseur_18F14k22.c,348 :: 		rxbuffer_tab[nb_rx_octet] = SSPBUF;
	MOVLW       _rxbuffer_tab+0
	MOVWF       FSR1 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	MOVWF       FSR1H 
	MOVF        _nb_rx_octet+0, 0 
	ADDWF       FSR1, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR1H, 1 
	MOVF        SSPBUF+0, 0 
	MOVWF       POSTINC1+0 
;Propulseur_18F14k22.c,349 :: 		nb_rx_octet++;
	INCF        _nb_rx_octet+0, 1 
;Propulseur_18F14k22.c,350 :: 		}
	GOTO        L_interrupt_low38
L_interrupt_low37:
;Propulseur_18F14k22.c,352 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Propulseur_18F14k22.c,353 :: 		}
L_interrupt_low38:
;Propulseur_18F14k22.c,354 :: 		}
L_interrupt_low36:
;Propulseur_18F14k22.c,355 :: 		}
L_interrupt_low34:
;Propulseur_18F14k22.c,357 :: 		if(nb_rx_octet>1){
	MOVF        _nb_rx_octet+0, 0 
	SUBLW       1
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt_low39
;Propulseur_18F14k22.c,358 :: 		Delay_us(30); // Wait P signal ?
	MOVLW       159
	MOVWF       R13, 0
L_interrupt_low40:
	DECFSZ      R13, 1, 1
	BRA         L_interrupt_low40
	NOP
	NOP
;Propulseur_18F14k22.c,359 :: 		if(SSPSTAT.P == 1){
	BTFSS       SSPSTAT+0, 4 
	GOTO        L_interrupt_low41
;Propulseur_18F14k22.c,360 :: 		i2c_read_data_from_buffer();
	CALL        _i2c_read_data_from_buffer+0, 0
;Propulseur_18F14k22.c,361 :: 		nb_rx_octet = 0;
	CLRF        _nb_rx_octet+0 
;Propulseur_18F14k22.c,362 :: 		}
L_interrupt_low41:
;Propulseur_18F14k22.c,363 :: 		}
L_interrupt_low39:
;Propulseur_18F14k22.c,364 :: 		}
	GOTO        L_interrupt_low42
L_interrupt_low33:
;Propulseur_18F14k22.c,368 :: 		if(SSPSTAT.D_A == 0){
	BTFSC       SSPSTAT+0, 5 
	GOTO        L_interrupt_low43
;Propulseur_18F14k22.c,369 :: 		nb_tx_octet = 0;
	CLRF        _nb_tx_octet+0 
;Propulseur_18F14k22.c,370 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Propulseur_18F14k22.c,371 :: 		}
L_interrupt_low43:
;Propulseur_18F14k22.c,374 :: 		i2c_write_data_to_buffer(nb_tx_octet);
	MOVF        _nb_tx_octet+0, 0 
	MOVWF       FARG_i2c_write_data_to_buffer_nb_tx_octet+0 
	CALL        _i2c_write_data_to_buffer+0, 0
;Propulseur_18F14k22.c,375 :: 		nb_tx_octet++;
	INCF        _nb_tx_octet+0, 1 
;Propulseur_18F14k22.c,376 :: 		}
L_interrupt_low42:
;Propulseur_18F14k22.c,378 :: 		SSPCON1.CKP = 1;
	BSF         SSPCON1+0, 4 
;Propulseur_18F14k22.c,379 :: 		PIR1.SSPIF = 0; // reset SSP interrupt flag
	BCF         PIR1+0, 3 
;Propulseur_18F14k22.c,380 :: 		}
L_interrupt_low29:
;Propulseur_18F14k22.c,381 :: 		}
L_end_interrupt_low:
L__interrupt_low58:
	MOVF        ___Low_saveBSR+0, 0 
	MOVWF       BSR+0 
	MOVF        ___Low_saveSTATUS+0, 0 
	MOVWF       STATUS+0 
	SWAPF       ___Low_saveWREG+0, 1 
	SWAPF       ___Low_saveWREG+0, 0 
	RETFIE      0
; end of _interrupt_low

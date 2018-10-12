
_i2c_read_data_from_buffer:

;robot_18F14k22_test.c,71 :: 		void i2c_read_data_from_buffer(){
;robot_18F14k22_test.c,72 :: 		unsigned short i = 0;
	CLRF        i2c_read_data_from_buffer_i_L0+0 
;robot_18F14k22_test.c,73 :: 		unsigned short nb_data = nb_rx_octet-1;
	DECF        _nb_rx_octet+0, 0 
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	SUBWFB      R1, 1 
	MOVF        R0, 0 
	MOVWF       R9 
;robot_18F14k22_test.c,75 :: 		for(i=0; i<nb_data; i++){
	CLRF        i2c_read_data_from_buffer_i_L0+0 
L_i2c_read_data_from_buffer0:
	MOVF        R9, 0 
	SUBWF       i2c_read_data_from_buffer_i_L0+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer1
;robot_18F14k22_test.c,76 :: 		switch(rxbuffer_tab[0]+i){
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDWF       _rxbuffer_tab+0, 0 
	MOVWF       R7 
	CLRF        R8 
	MOVLW       0
	ADDWFC      R8, 1 
	GOTO        L_i2c_read_data_from_buffer3
;robot_18F14k22_test.c,77 :: 		case 0x01:
L_i2c_read_data_from_buffer5:
;robot_18F14k22_test.c,78 :: 		state = RESET_OUT;
	CLRF        _state+0 
;robot_18F14k22_test.c,79 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,80 :: 		case 0x02:
L_i2c_read_data_from_buffer6:
;robot_18F14k22_test.c,81 :: 		motor_on = (rxbuffer_tab[i+1]!=0x00);
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	XORLW       0
	MOVLW       0
	BTFSS       STATUS+0, 2 
	MOVLW       1
	MOVWF       _motor_on+0 
;robot_18F14k22_test.c,82 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,83 :: 		case 0x03:
L_i2c_read_data_from_buffer7:
;robot_18F14k22_test.c,84 :: 		RC6_bit = (rxbuffer_tab[i+1]!=0x00);
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	XORLW       0
	MOVLW       0
	BTFSS       STATUS+0, 2 
	MOVLW       1
	MOVWF       R0 
	BTFSC       R0, 0 
	GOTO        L__i2c_read_data_from_buffer115
	BCF         RC6_bit+0, BitPos(RC6_bit+0) 
	GOTO        L__i2c_read_data_from_buffer116
L__i2c_read_data_from_buffer115:
	BSF         RC6_bit+0, BitPos(RC6_bit+0) 
L__i2c_read_data_from_buffer116:
;robot_18F14k22_test.c,85 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,86 :: 		case 0x04:
L_i2c_read_data_from_buffer8:
;robot_18F14k22_test.c,87 :: 		LED1 = (rxbuffer_tab[i+1]!=0x00);
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	XORLW       0
	MOVLW       0
	BTFSS       STATUS+0, 2 
	MOVLW       1
	MOVWF       R0 
	BTFSC       R0, 0 
	GOTO        L__i2c_read_data_from_buffer117
	BCF         RC0_bit+0, BitPos(RC0_bit+0) 
	GOTO        L__i2c_read_data_from_buffer118
L__i2c_read_data_from_buffer117:
	BSF         RC0_bit+0, BitPos(RC0_bit+0) 
L__i2c_read_data_from_buffer118:
;robot_18F14k22_test.c,88 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,89 :: 		case 0x05:
L_i2c_read_data_from_buffer9:
;robot_18F14k22_test.c,90 :: 		error_interval = rxbuffer_tab[i+1];
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       _error_interval+0 
;robot_18F14k22_test.c,91 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,92 :: 		case 0x06:
L_i2c_read_data_from_buffer10:
;robot_18F14k22_test.c,93 :: 		position_reached_enable = rxbuffer_tab[i+1];
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       _position_reached_enable+0 
;robot_18F14k22_test.c,94 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,95 :: 		case 0x10:  // consigne de postion
L_i2c_read_data_from_buffer11:
;robot_18F14k22_test.c,96 :: 		if(nb_data >= i+2){
	MOVLW       2
	ADDWF       i2c_read_data_from_buffer_i_L0+0, 0 
	MOVWF       R1 
	CLRF        R2 
	MOVLW       0
	ADDWFC      R2, 1 
	MOVLW       128
	MOVWF       R0 
	MOVLW       128
	XORWF       R2, 0 
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer119
	MOVF        R1, 0 
	SUBWF       R9, 0 
L__i2c_read_data_from_buffer119:
	BTFSS       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer12
;robot_18F14k22_test.c,97 :: 		position_set_point = 4*(rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8));
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R5 
	CLRF        R6 
	MOVLW       0
	ADDWFC      R6, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R5, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R6, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       R4 
	MOVLW       2
	ADDWF       i2c_read_data_from_buffer_i_L0+0, 0 
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       R3 
	MOVF        R3, 0 
	MOVWF       R1 
	CLRF        R0 
	MOVF        R0, 0 
	IORWF       R4, 0 
	MOVWF       _position_set_point+0 
	MOVLW       0
	IORWF       R1, 0 
	MOVWF       _position_set_point+1 
	RLCF        _position_set_point+0, 1 
	BCF         _position_set_point+0, 0 
	RLCF        _position_set_point+1, 1 
	RLCF        _position_set_point+0, 1 
	BCF         _position_set_point+0, 0 
	RLCF        _position_set_point+1, 1 
;robot_18F14k22_test.c,98 :: 		i++;
	MOVF        R5, 0 
	MOVWF       i2c_read_data_from_buffer_i_L0+0 
;robot_18F14k22_test.c,99 :: 		}
L_i2c_read_data_from_buffer12:
;robot_18F14k22_test.c,100 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,102 :: 		case 0x12:  // consigne de vitesse in
L_i2c_read_data_from_buffer13:
;robot_18F14k22_test.c,103 :: 		if(nb_data >= i+2){
	MOVLW       2
	ADDWF       i2c_read_data_from_buffer_i_L0+0, 0 
	MOVWF       R1 
	CLRF        R2 
	MOVLW       0
	ADDWFC      R2, 1 
	MOVLW       128
	MOVWF       R0 
	MOVLW       128
	XORWF       R2, 0 
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer120
	MOVF        R1, 0 
	SUBWF       R9, 0 
L__i2c_read_data_from_buffer120:
	BTFSS       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer14
;robot_18F14k22_test.c,104 :: 		motor_speed_in = rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8);
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       _motor_speed_in+0 
;robot_18F14k22_test.c,105 :: 		i++;
	MOVF        R0, 0 
	MOVWF       i2c_read_data_from_buffer_i_L0+0 
;robot_18F14k22_test.c,106 :: 		}
L_i2c_read_data_from_buffer14:
;robot_18F14k22_test.c,107 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,108 :: 		case 0x14:  // consigne de vitesse out
L_i2c_read_data_from_buffer15:
;robot_18F14k22_test.c,109 :: 		if(nb_data >= i+2){
	MOVLW       2
	ADDWF       i2c_read_data_from_buffer_i_L0+0, 0 
	MOVWF       R1 
	CLRF        R2 
	MOVLW       0
	ADDWFC      R2, 1 
	MOVLW       128
	MOVWF       R0 
	MOVLW       128
	XORWF       R2, 0 
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer121
	MOVF        R1, 0 
	SUBWF       R9, 0 
L__i2c_read_data_from_buffer121:
	BTFSS       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer16
;robot_18F14k22_test.c,110 :: 		motor_speed_out = rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8);
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       _motor_speed_out+0 
;robot_18F14k22_test.c,111 :: 		i++;
	MOVF        R0, 0 
	MOVWF       i2c_read_data_from_buffer_i_L0+0 
;robot_18F14k22_test.c,112 :: 		}
L_i2c_read_data_from_buffer16:
;robot_18F14k22_test.c,113 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,114 :: 		default:
L_i2c_read_data_from_buffer17:
;robot_18F14k22_test.c,115 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;robot_18F14k22_test.c,116 :: 		}
L_i2c_read_data_from_buffer3:
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer122
	MOVLW       1
	XORWF       R7, 0 
L__i2c_read_data_from_buffer122:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer5
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer123
	MOVLW       2
	XORWF       R7, 0 
L__i2c_read_data_from_buffer123:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer6
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer124
	MOVLW       3
	XORWF       R7, 0 
L__i2c_read_data_from_buffer124:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer7
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer125
	MOVLW       4
	XORWF       R7, 0 
L__i2c_read_data_from_buffer125:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer8
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer126
	MOVLW       5
	XORWF       R7, 0 
L__i2c_read_data_from_buffer126:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer9
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer127
	MOVLW       6
	XORWF       R7, 0 
L__i2c_read_data_from_buffer127:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer10
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer128
	MOVLW       16
	XORWF       R7, 0 
L__i2c_read_data_from_buffer128:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer11
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer129
	MOVLW       18
	XORWF       R7, 0 
L__i2c_read_data_from_buffer129:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer13
	MOVLW       0
	XORWF       R8, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer130
	MOVLW       20
	XORWF       R7, 0 
L__i2c_read_data_from_buffer130:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer15
	GOTO        L_i2c_read_data_from_buffer17
L_i2c_read_data_from_buffer4:
;robot_18F14k22_test.c,75 :: 		for(i=0; i<nb_data; i++){
	INCF        i2c_read_data_from_buffer_i_L0+0, 1 
;robot_18F14k22_test.c,117 :: 		}
	GOTO        L_i2c_read_data_from_buffer0
L_i2c_read_data_from_buffer1:
;robot_18F14k22_test.c,118 :: 		watchdog_restart = watchdog_restart_default;
	MOVF        _watchdog_restart_default+0, 0 
	MOVWF       _watchdog_restart+0 
;robot_18F14k22_test.c,119 :: 		}
L_end_i2c_read_data_from_buffer:
	RETURN      0
; end of _i2c_read_data_from_buffer

_i2c_write_data_to_buffer:

;robot_18F14k22_test.c,125 :: 		void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
;robot_18F14k22_test.c,127 :: 		switch(rxbuffer_tab[0]+nb_tx_octet){
	MOVF        FARG_i2c_write_data_to_buffer_nb_tx_octet+0, 0 
	ADDWF       _rxbuffer_tab+0, 0 
	MOVWF       R5 
	CLRF        R6 
	MOVLW       0
	ADDWFC      R6, 1 
	GOTO        L_i2c_write_data_to_buffer18
;robot_18F14k22_test.c,128 :: 		case 0x00:
L_i2c_write_data_to_buffer20:
;robot_18F14k22_test.c,129 :: 		SSPBUF = nb_pulse;
	MOVF        _nb_pulse+0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,130 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,131 :: 		case 0x01:
L_i2c_write_data_to_buffer21:
;robot_18F14k22_test.c,132 :: 		SSPBUF = (nb_pulse >> 8);
	MOVF        _nb_pulse+1, 0 
	MOVWF       R0 
	MOVLW       0
	BTFSC       _nb_pulse+1, 7 
	MOVLW       255
	MOVWF       R1 
	MOVF        R0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,133 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,134 :: 		case 0x02:
L_i2c_write_data_to_buffer22:
;robot_18F14k22_test.c,135 :: 		SSPBUF = (butee_out & 0b1)
	MOVLW       1
	ANDWF       _butee_out+0, 0 
	MOVWF       R3 
;robot_18F14k22_test.c,136 :: 		| ((butee_in & 0b1)<<1)
	MOVLW       1
	ANDWF       _butee_in+0, 0 
	MOVWF       R2 
	MOVF        R2, 0 
	MOVWF       R0 
	RLCF        R0, 1 
	BCF         R0, 0 
	MOVF        R0, 0 
	IORWF       R3, 0 
	MOVWF       R4 
;robot_18F14k22_test.c,137 :: 		| ((state & 0b11) <<2)
	MOVLW       3
	ANDWF       _state+0, 0 
	MOVWF       R3 
	MOVF        R3, 0 
	MOVWF       R0 
	MOVLW       0
	MOVWF       R1 
	RLCF        R0, 1 
	BCF         R0, 0 
	RLCF        R1, 1 
	RLCF        R0, 1 
	BCF         R0, 0 
	RLCF        R1, 1 
	MOVF        R0, 0 
	IORWF       R4, 1 
;robot_18F14k22_test.c,138 :: 		| ((motor_on & 0b1) << 4)
	MOVLW       1
	ANDWF       _motor_on+0, 0 
	MOVWF       R3 
	MOVLW       4
	MOVWF       R2 
	MOVF        R3, 0 
	MOVWF       R0 
	MOVLW       0
	MOVWF       R1 
	MOVF        R2, 0 
L__i2c_write_data_to_buffer132:
	BZ          L__i2c_write_data_to_buffer133
	RLCF        R0, 1 
	BCF         R0, 0 
	RLCF        R1, 1 
	ADDLW       255
	GOTO        L__i2c_write_data_to_buffer132
L__i2c_write_data_to_buffer133:
	MOVF        R0, 0 
	IORWF       R4, 1 
;robot_18F14k22_test.c,139 :: 		| ((RC6_bit & 0b1) << 5);
	CLRF        R0 
	BTFSC       RC6_bit+0, BitPos(RC6_bit+0) 
	INCF        R0, 1 
	MOVLW       1
	ANDWF       R0, 0 
	MOVWF       R3 
	MOVLW       5
	MOVWF       R2 
	MOVF        R3, 0 
	MOVWF       R0 
	MOVLW       0
	MOVWF       R1 
	MOVF        R2, 0 
L__i2c_write_data_to_buffer134:
	BZ          L__i2c_write_data_to_buffer135
	RLCF        R0, 1 
	BCF         R0, 0 
	RLCF        R1, 1 
	ADDLW       255
	GOTO        L__i2c_write_data_to_buffer134
L__i2c_write_data_to_buffer135:
	MOVF        R0, 0 
	IORWF       R4, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,140 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,141 :: 		case 0x03:
L_i2c_write_data_to_buffer23:
;robot_18F14k22_test.c,142 :: 		SSPBUF = position_set_point;
	MOVF        _position_set_point+0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,143 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,144 :: 		case 0x04:
L_i2c_write_data_to_buffer24:
;robot_18F14k22_test.c,145 :: 		SSPBUF = position_set_point >> 8;
	MOVF        _position_set_point+1, 0 
	MOVWF       R0 
	MOVLW       0
	BTFSC       _position_set_point+1, 7 
	MOVLW       255
	MOVWF       R1 
	MOVF        R0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,146 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,147 :: 		case 0x05:
L_i2c_write_data_to_buffer25:
;robot_18F14k22_test.c,148 :: 		SSPBUF = motor_current_speed;
	MOVF        _motor_current_speed+0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,149 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,150 :: 		case 0x06:
L_i2c_write_data_to_buffer26:
;robot_18F14k22_test.c,151 :: 		SSPBUF = motor_speed_in;
	MOVF        _motor_speed_in+0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,152 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,153 :: 		case 0x07:
L_i2c_write_data_to_buffer27:
;robot_18F14k22_test.c,154 :: 		SSPBUF = motor_speed_in >> 8;
	CLRF        SSPBUF+0 
;robot_18F14k22_test.c,155 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,156 :: 		case 0x08:
L_i2c_write_data_to_buffer28:
;robot_18F14k22_test.c,157 :: 		SSPBUF = motor_speed_out;
	MOVF        _motor_speed_out+0, 0 
	MOVWF       SSPBUF+0 
;robot_18F14k22_test.c,158 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,159 :: 		case 0x09:
L_i2c_write_data_to_buffer29:
;robot_18F14k22_test.c,160 :: 		SSPBUF = motor_speed_out >> 8;
	CLRF        SSPBUF+0 
;robot_18F14k22_test.c,161 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,162 :: 		default:
L_i2c_write_data_to_buffer30:
;robot_18F14k22_test.c,163 :: 		SSPBUF = 0x00;
	CLRF        SSPBUF+0 
;robot_18F14k22_test.c,164 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;robot_18F14k22_test.c,165 :: 		}
L_i2c_write_data_to_buffer18:
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer136
	MOVLW       0
	XORWF       R5, 0 
L__i2c_write_data_to_buffer136:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer20
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer137
	MOVLW       1
	XORWF       R5, 0 
L__i2c_write_data_to_buffer137:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer21
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer138
	MOVLW       2
	XORWF       R5, 0 
L__i2c_write_data_to_buffer138:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer22
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer139
	MOVLW       3
	XORWF       R5, 0 
L__i2c_write_data_to_buffer139:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer23
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer140
	MOVLW       4
	XORWF       R5, 0 
L__i2c_write_data_to_buffer140:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer24
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer141
	MOVLW       5
	XORWF       R5, 0 
L__i2c_write_data_to_buffer141:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer25
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer142
	MOVLW       6
	XORWF       R5, 0 
L__i2c_write_data_to_buffer142:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer26
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer143
	MOVLW       7
	XORWF       R5, 0 
L__i2c_write_data_to_buffer143:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer27
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer144
	MOVLW       8
	XORWF       R5, 0 
L__i2c_write_data_to_buffer144:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer28
	MOVLW       0
	XORWF       R6, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer145
	MOVLW       9
	XORWF       R5, 0 
L__i2c_write_data_to_buffer145:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer29
	GOTO        L_i2c_write_data_to_buffer30
L_i2c_write_data_to_buffer19:
;robot_18F14k22_test.c,166 :: 		}
L_end_i2c_write_data_to_buffer:
	RETURN      0
; end of _i2c_write_data_to_buffer

_read_butee:

;robot_18F14k22_test.c,171 :: 		void read_butee(){
;robot_18F14k22_test.c,172 :: 		if (RA0_bit == 0){
	BTFSC       RA0_bit+0, BitPos(RA0_bit+0) 
	GOTO        L_read_butee31
;robot_18F14k22_test.c,173 :: 		butee_out = 1;
	MOVLW       1
	MOVWF       _butee_out+0 
;robot_18F14k22_test.c,174 :: 		LED1 = 1;
	BSF         RC0_bit+0, BitPos(RC0_bit+0) 
;robot_18F14k22_test.c,175 :: 		}
	GOTO        L_read_butee32
L_read_butee31:
;robot_18F14k22_test.c,177 :: 		butee_out = 0;
	CLRF        _butee_out+0 
;robot_18F14k22_test.c,178 :: 		LED1 = 0;
	BCF         RC0_bit+0, BitPos(RC0_bit+0) 
;robot_18F14k22_test.c,179 :: 		}
L_read_butee32:
;robot_18F14k22_test.c,180 :: 		if (RA1_bit == 0)
	BTFSC       RA1_bit+0, BitPos(RA1_bit+0) 
	GOTO        L_read_butee33
;robot_18F14k22_test.c,181 :: 		butee_in = 1;
	MOVLW       1
	MOVWF       _butee_in+0 
	GOTO        L_read_butee34
L_read_butee33:
;robot_18F14k22_test.c,183 :: 		butee_in = 0;
	CLRF        _butee_in+0 
L_read_butee34:
;robot_18F14k22_test.c,184 :: 		}
L_end_read_butee:
	RETURN      0
; end of _read_butee

_set_motor_cmd_stop:

;robot_18F14k22_test.c,188 :: 		void set_motor_cmd_stop(){
;robot_18F14k22_test.c,189 :: 		if(motor_current_speed != 127){
	MOVF        _motor_current_speed+0, 0 
	XORLW       127
	BTFSC       STATUS+0, 2 
	GOTO        L_set_motor_cmd_stop35
;robot_18F14k22_test.c,191 :: 		PWM1_Set_Duty(127);
	MOVLW       127
	MOVWF       FARG_PWM1_Set_Duty_new_duty+0 
	CALL        _PWM1_Set_Duty+0, 0
;robot_18F14k22_test.c,192 :: 		motor_current_speed = 127;
	MOVLW       127
	MOVWF       _motor_current_speed+0 
;robot_18F14k22_test.c,193 :: 		RC6_bit = 1;
	BSF         RC6_bit+0, BitPos(RC6_bit+0) 
;robot_18F14k22_test.c,194 :: 		}
L_set_motor_cmd_stop35:
;robot_18F14k22_test.c,195 :: 		}
L_end_set_motor_cmd_stop:
	RETURN      0
; end of _set_motor_cmd_stop

_set_motor_cmd:

;robot_18F14k22_test.c,201 :: 		void set_motor_cmd(unsigned short speed){
;robot_18F14k22_test.c,202 :: 		if(motor_on == 0 || (butee_out == 1 && speed >= 127) || (butee_in == 1 && speed <= 127)){
	MOVF        _motor_on+0, 0 
	XORLW       0
	BTFSC       STATUS+0, 2 
	GOTO        L__set_motor_cmd110
	MOVF        _butee_out+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L__set_motor_cmd112
	MOVLW       127
	SUBWF       FARG_set_motor_cmd_speed+0, 0 
	BTFSS       STATUS+0, 0 
	GOTO        L__set_motor_cmd112
	GOTO        L__set_motor_cmd110
L__set_motor_cmd112:
	MOVF        _butee_in+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L__set_motor_cmd111
	MOVF        FARG_set_motor_cmd_speed+0, 0 
	SUBLW       127
	BTFSS       STATUS+0, 0 
	GOTO        L__set_motor_cmd111
	GOTO        L__set_motor_cmd110
L__set_motor_cmd111:
	GOTO        L_set_motor_cmd42
L__set_motor_cmd110:
;robot_18F14k22_test.c,203 :: 		set_motor_cmd_stop();
	CALL        _set_motor_cmd_stop+0, 0
;robot_18F14k22_test.c,204 :: 		if(position_reached_enable == 1){
	MOVF        _position_reached_enable+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_set_motor_cmd43
;robot_18F14k22_test.c,205 :: 		if(position_reached_cpt>position_reached_max_value){
	MOVF        _position_reached_cpt+3, 0 
	SUBWF       _position_reached_max_value+3, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__set_motor_cmd149
	MOVF        _position_reached_cpt+2, 0 
	SUBWF       _position_reached_max_value+2, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__set_motor_cmd149
	MOVF        _position_reached_cpt+1, 0 
	SUBWF       _position_reached_max_value+1, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__set_motor_cmd149
	MOVF        _position_reached_cpt+0, 0 
	SUBWF       _position_reached_max_value+0, 0 
L__set_motor_cmd149:
	BTFSC       STATUS+0, 0 
	GOTO        L_set_motor_cmd44
;robot_18F14k22_test.c,206 :: 		RC6_bit = 0;
	BCF         RC6_bit+0, BitPos(RC6_bit+0) 
;robot_18F14k22_test.c,207 :: 		}
	GOTO        L_set_motor_cmd45
L_set_motor_cmd44:
;robot_18F14k22_test.c,209 :: 		position_reached_cpt++;
	MOVLW       1
	ADDWF       _position_reached_cpt+0, 1 
	MOVLW       0
	ADDWFC      _position_reached_cpt+1, 1 
	ADDWFC      _position_reached_cpt+2, 1 
	ADDWFC      _position_reached_cpt+3, 1 
L_set_motor_cmd45:
;robot_18F14k22_test.c,210 :: 		}
L_set_motor_cmd43:
;robot_18F14k22_test.c,211 :: 		}
	GOTO        L_set_motor_cmd46
L_set_motor_cmd42:
;robot_18F14k22_test.c,212 :: 		else if(motor_current_speed != speed){
	MOVF        _motor_current_speed+0, 0 
	XORWF       FARG_set_motor_cmd_speed+0, 0 
	BTFSC       STATUS+0, 2 
	GOTO        L_set_motor_cmd47
;robot_18F14k22_test.c,213 :: 		position_reached_cpt = 0;
	CLRF        _position_reached_cpt+0 
	CLRF        _position_reached_cpt+1 
	CLRF        _position_reached_cpt+2 
	CLRF        _position_reached_cpt+3 
;robot_18F14k22_test.c,214 :: 		motor_current_speed = speed;
	MOVF        FARG_set_motor_cmd_speed+0, 0 
	MOVWF       _motor_current_speed+0 
;robot_18F14k22_test.c,215 :: 		PWM1_Set_Duty(speed);
	MOVF        FARG_set_motor_cmd_speed+0, 0 
	MOVWF       FARG_PWM1_Set_Duty_new_duty+0 
	CALL        _PWM1_Set_Duty+0, 0
;robot_18F14k22_test.c,216 :: 		RC6_bit = 1;  //Enable L6203
	BSF         RC6_bit+0, BitPos(RC6_bit+0) 
;robot_18F14k22_test.c,217 :: 		}
L_set_motor_cmd47:
L_set_motor_cmd46:
;robot_18F14k22_test.c,223 :: 		}
L_end_set_motor_cmd:
	RETURN      0
; end of _set_motor_cmd

_set_motor_cmd_out:

;robot_18F14k22_test.c,229 :: 		void set_motor_cmd_out(unsigned short speed){
;robot_18F14k22_test.c,230 :: 		set_motor_cmd(127 + speed);
	MOVF        FARG_set_motor_cmd_out_speed+0, 0 
	ADDLW       127
	MOVWF       FARG_set_motor_cmd_speed+0 
	CALL        _set_motor_cmd+0, 0
;robot_18F14k22_test.c,231 :: 		}
L_end_set_motor_cmd_out:
	RETURN      0
; end of _set_motor_cmd_out

_set_motor_cmd_in:

;robot_18F14k22_test.c,237 :: 		void set_motor_cmd_in(unsigned short speed){
;robot_18F14k22_test.c,238 :: 		set_motor_cmd(127 - speed);
	MOVF        FARG_set_motor_cmd_in_speed+0, 0 
	SUBLW       127
	MOVWF       FARG_set_motor_cmd_speed+0 
	CALL        _set_motor_cmd+0, 0
;robot_18F14k22_test.c,239 :: 		}
L_end_set_motor_cmd_in:
	RETURN      0
; end of _set_motor_cmd_in

_read_optical_fork:

;robot_18F14k22_test.c,244 :: 		void read_optical_fork(){
;robot_18F14k22_test.c,245 :: 		unsigned short new_state = SB<<1 | SA;  //  ou logique de RA3 et RA2
	CLRF        R1 
	BTFSC       RA4_bit+0, BitPos(RA4_bit+0) 
	INCF        R1, 1 
	MOVF        R1, 0 
	MOVWF       R2 
	MOVLW       0
	MOVWF       R3 
	RLCF        R2, 1 
	BCF         R2, 0 
	RLCF        R3, 1 
	CLRF        R0 
	BTFSC       RA2_bit+0, BitPos(RA2_bit+0) 
	INCF        R0, 1 
	MOVLW       0
	MOVWF       R1 
	MOVF        R2, 0 
	IORWF       R0, 1 
	MOVF        R3, 0 
	IORWF       R1, 1 
	MOVF        R0, 0 
	MOVWF       R4 
;robot_18F14k22_test.c,247 :: 		switch(optical_state){
	GOTO        L_read_optical_fork48
;robot_18F14k22_test.c,248 :: 		case 0x00:
L_read_optical_fork50:
;robot_18F14k22_test.c,249 :: 		if(new_state == 0x1)
	MOVF        R4, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork51
;robot_18F14k22_test.c,250 :: 		nb_pulse--;
	MOVLW       1
	SUBWF       _nb_pulse+0, 1 
	MOVLW       0
	SUBWFB      _nb_pulse+1, 1 
	GOTO        L_read_optical_fork52
L_read_optical_fork51:
;robot_18F14k22_test.c,251 :: 		else if(new_state == 0x2)
	MOVF        R4, 0 
	XORLW       2
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork53
;robot_18F14k22_test.c,252 :: 		nb_pulse++;
	INFSNZ      _nb_pulse+0, 1 
	INCF        _nb_pulse+1, 1 
L_read_optical_fork53:
L_read_optical_fork52:
;robot_18F14k22_test.c,253 :: 		break;
	GOTO        L_read_optical_fork49
;robot_18F14k22_test.c,254 :: 		case 0x01:
L_read_optical_fork54:
;robot_18F14k22_test.c,255 :: 		if(new_state == 0x3)
	MOVF        R4, 0 
	XORLW       3
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork55
;robot_18F14k22_test.c,256 :: 		nb_pulse--;
	MOVLW       1
	SUBWF       _nb_pulse+0, 1 
	MOVLW       0
	SUBWFB      _nb_pulse+1, 1 
	GOTO        L_read_optical_fork56
L_read_optical_fork55:
;robot_18F14k22_test.c,257 :: 		else if(new_state == 0x0)
	MOVF        R4, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork57
;robot_18F14k22_test.c,258 :: 		nb_pulse++;
	INFSNZ      _nb_pulse+0, 1 
	INCF        _nb_pulse+1, 1 
L_read_optical_fork57:
L_read_optical_fork56:
;robot_18F14k22_test.c,259 :: 		break;
	GOTO        L_read_optical_fork49
;robot_18F14k22_test.c,260 :: 		case 0x02:
L_read_optical_fork58:
;robot_18F14k22_test.c,261 :: 		if(new_state == 0x0)
	MOVF        R4, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork59
;robot_18F14k22_test.c,262 :: 		nb_pulse--;
	MOVLW       1
	SUBWF       _nb_pulse+0, 1 
	MOVLW       0
	SUBWFB      _nb_pulse+1, 1 
	GOTO        L_read_optical_fork60
L_read_optical_fork59:
;robot_18F14k22_test.c,263 :: 		else if(new_state == 0x3)
	MOVF        R4, 0 
	XORLW       3
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork61
;robot_18F14k22_test.c,264 :: 		nb_pulse++;
	INFSNZ      _nb_pulse+0, 1 
	INCF        _nb_pulse+1, 1 
L_read_optical_fork61:
L_read_optical_fork60:
;robot_18F14k22_test.c,265 :: 		break;
	GOTO        L_read_optical_fork49
;robot_18F14k22_test.c,266 :: 		case 0x03:
L_read_optical_fork62:
;robot_18F14k22_test.c,267 :: 		if(new_state == 0x1)
	MOVF        R4, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork63
;robot_18F14k22_test.c,268 :: 		nb_pulse++;
	INFSNZ      _nb_pulse+0, 1 
	INCF        _nb_pulse+1, 1 
	GOTO        L_read_optical_fork64
L_read_optical_fork63:
;robot_18F14k22_test.c,269 :: 		else if(new_state == 0x2)
	MOVF        R4, 0 
	XORLW       2
	BTFSS       STATUS+0, 2 
	GOTO        L_read_optical_fork65
;robot_18F14k22_test.c,270 :: 		nb_pulse--;
	MOVLW       1
	SUBWF       _nb_pulse+0, 1 
	MOVLW       0
	SUBWFB      _nb_pulse+1, 1 
L_read_optical_fork65:
L_read_optical_fork64:
;robot_18F14k22_test.c,271 :: 		break;
	GOTO        L_read_optical_fork49
;robot_18F14k22_test.c,272 :: 		default:
L_read_optical_fork66:
;robot_18F14k22_test.c,273 :: 		break;
	GOTO        L_read_optical_fork49
;robot_18F14k22_test.c,274 :: 		}
L_read_optical_fork48:
	MOVF        _optical_state+0, 0 
	XORLW       0
	BTFSC       STATUS+0, 2 
	GOTO        L_read_optical_fork50
	MOVF        _optical_state+0, 0 
	XORLW       1
	BTFSC       STATUS+0, 2 
	GOTO        L_read_optical_fork54
	MOVF        _optical_state+0, 0 
	XORLW       2
	BTFSC       STATUS+0, 2 
	GOTO        L_read_optical_fork58
	MOVF        _optical_state+0, 0 
	XORLW       3
	BTFSC       STATUS+0, 2 
	GOTO        L_read_optical_fork62
	GOTO        L_read_optical_fork66
L_read_optical_fork49:
;robot_18F14k22_test.c,276 :: 		if (optical_state != new_state){
	MOVF        _optical_state+0, 0 
	XORWF       R4, 0 
	BTFSC       STATUS+0, 2 
	GOTO        L_read_optical_fork67
;robot_18F14k22_test.c,277 :: 		TXREG = nb_pulse >> 8;
	MOVF        _nb_pulse+1, 0 
	MOVWF       R0 
	MOVLW       0
	BTFSC       _nb_pulse+1, 7 
	MOVLW       255
	MOVWF       R1 
	MOVF        R0, 0 
	MOVWF       TXREG+0 
;robot_18F14k22_test.c,278 :: 		TXREG = nb_pulse;
	MOVF        _nb_pulse+0, 0 
	MOVWF       TXREG+0 
;robot_18F14k22_test.c,279 :: 		}
L_read_optical_fork67:
;robot_18F14k22_test.c,281 :: 		optical_state = new_state;  // store the current state value to optical_state value this value will be used in next call
	MOVF        R4, 0 
	MOVWF       _optical_state+0 
;robot_18F14k22_test.c,282 :: 		}
L_end_read_optical_fork:
	RETURN      0
; end of _read_optical_fork

_init_timer0:

;robot_18F14k22_test.c,289 :: 		void init_timer0(){
;robot_18F14k22_test.c,290 :: 		T0CON = 0x85; // TIMER0 ON (1 s)
	MOVLW       133
	MOVWF       T0CON+0 
;robot_18F14k22_test.c,291 :: 		TMR0H = 0x0B;
	MOVLW       11
	MOVWF       TMR0H+0 
;robot_18F14k22_test.c,292 :: 		TMR0L = 0xDC;
	MOVLW       220
	MOVWF       TMR0L+0 
;robot_18F14k22_test.c,293 :: 		TMR0IE_bit = 0;
	BCF         TMR0IE_bit+0, BitPos(TMR0IE_bit+0) 
;robot_18F14k22_test.c,294 :: 		}
L_end_init_timer0:
	RETURN      0
; end of _init_timer0

_init_io:

;robot_18F14k22_test.c,299 :: 		void init_io(){
;robot_18F14k22_test.c,300 :: 		ANSEL = 0x00;
	CLRF        ANSEL+0 
;robot_18F14k22_test.c,301 :: 		ANSELH = 0x00;
	CLRF        ANSELH+0 
;robot_18F14k22_test.c,303 :: 		CM1CON0 = 0x00; // Not using the comparators
	CLRF        CM1CON0+0 
;robot_18F14k22_test.c,304 :: 		CM2CON0 = 0x00; //
	CLRF        CM2CON0+0 
;robot_18F14k22_test.c,306 :: 		TRISA0_bit = 1; // RA0 en entrée
	BSF         TRISA0_bit+0, BitPos(TRISA0_bit+0) 
;robot_18F14k22_test.c,307 :: 		TRISA1_bit = 1; // RA1 en entrée
	BSF         TRISA1_bit+0, BitPos(TRISA1_bit+0) 
;robot_18F14k22_test.c,308 :: 		TRISA2_bit = 1; // RA2 en entrée
	BSF         TRISA2_bit+0, BitPos(TRISA2_bit+0) 
;robot_18F14k22_test.c,310 :: 		TRISA4_bit = 1; // RA4 en entrée
	BSF         TRISA4_bit+0, BitPos(TRISA4_bit+0) 
;robot_18F14k22_test.c,312 :: 		TRISA5_bit = 0; // RA5 en sortie
	BCF         TRISA5_bit+0, BitPos(TRISA5_bit+0) 
;robot_18F14k22_test.c,314 :: 		INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit
	BCF         INTCON2+0, 7 
;robot_18F14k22_test.c,315 :: 		WPUA.WPUA0 = 1; // Pull-up enabled sur RA0, sur inter de butée haute
	BSF         WPUA+0, 0 
;robot_18F14k22_test.c,316 :: 		WPUA.WPUA1 = 1; // Pull-up enabled sur RA1, sur inter de butée basse
	BSF         WPUA+0, 1 
;robot_18F14k22_test.c,320 :: 		TRISB5_bit = 0; // RB5 en sortie
	BCF         TRISB5_bit+0, BitPos(TRISB5_bit+0) 
;robot_18F14k22_test.c,321 :: 		TRISB7_bit = 0; // RB7 en sortie
	BCF         TRISB7_bit+0, BitPos(TRISB7_bit+0) 
;robot_18F14k22_test.c,323 :: 		TRISC = 0xFF;
	MOVLW       255
	MOVWF       TRISC+0 
;robot_18F14k22_test.c,324 :: 		TRISC0_bit = 0; // RC0 en sortie
	BCF         TRISC0_bit+0, BitPos(TRISC0_bit+0) 
;robot_18F14k22_test.c,325 :: 		TRISC2_bit = 0; // RC0 en sortie
	BCF         TRISC2_bit+0, BitPos(TRISC2_bit+0) 
;robot_18F14k22_test.c,326 :: 		TRISC4_bit = 0; // RC4 en sortie
	BCF         TRISC4_bit+0, BitPos(TRISC4_bit+0) 
;robot_18F14k22_test.c,327 :: 		TRISC5_bit = 0; // RC5 en sortie
	BCF         TRISC5_bit+0, BitPos(TRISC5_bit+0) 
;robot_18F14k22_test.c,328 :: 		TRISC6_bit = 0; // RC6 en sortie
	BCF         TRISC6_bit+0, BitPos(TRISC6_bit+0) 
;robot_18F14k22_test.c,329 :: 		TRISC7_bit = 0; // RC7 en sortie
	BCF         TRISC7_bit+0, BitPos(TRISC7_bit+0) 
;robot_18F14k22_test.c,331 :: 		RC6_bit = 0;
	BCF         RC6_bit+0, BitPos(RC6_bit+0) 
;robot_18F14k22_test.c,332 :: 		}
L_end_init_io:
	RETURN      0
; end of _init_io

_main:

;robot_18F14k22_test.c,337 :: 		void main(){
;robot_18F14k22_test.c,339 :: 		OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block
	MOVLW       114
	MOVWF       OSCCON+0 
;robot_18F14k22_test.c,341 :: 		init_io(); // Initialisation des I/O
	CALL        _init_io+0, 0
;robot_18F14k22_test.c,342 :: 		init_i2C(); // Initialisation de l'I2C en esclave
	CALL        _init_i2c+0, 0
;robot_18F14k22_test.c,343 :: 		init_timer0(); // Initialisation du TIMER0 toutes les 1 secondes
	CALL        _init_timer0+0, 0
;robot_18F14k22_test.c,346 :: 		INTCON2.INTEDG0 = 0; // Interrupt on falling edge
	BCF         INTCON2+0, 6 
;robot_18F14k22_test.c,347 :: 		INTCON.INT0IE = 1; //Enables the INT0 external interrupt
	BSF         INTCON+0, 4 
;robot_18F14k22_test.c,348 :: 		INTCON.INT0IF = 0; // INT0 External Interrupt Flag bit
	BCF         INTCON+0, 1 
;robot_18F14k22_test.c,351 :: 		INTCON2.INTEDG1 = 0; // Interrupt on falling edge
	BCF         INTCON2+0, 5 
;robot_18F14k22_test.c,352 :: 		INTCON3.INT1IE = 1; //Enables the INT0 external interrupt
	BSF         INTCON3+0, 3 
;robot_18F14k22_test.c,353 :: 		INTCON3.INT1IF = 0; // INT0 External Interrupt Flag bit
	BCF         INTCON3+0, 0 
;robot_18F14k22_test.c,363 :: 		INTCON3.INT1IP = 1; //INT1 External Interrupt Priority bit, INT0 always a high
	BSF         INTCON3+0, 6 
;robot_18F14k22_test.c,366 :: 		IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit, low priority
	BCF         IPR1+0, 3 
;robot_18F14k22_test.c,367 :: 		RCON.IPEN = 1;  //Enable priority levels on interrupts
	BSF         RCON+0, 7 
;robot_18F14k22_test.c,368 :: 		INTCON.GIEH = 1; //enable all high-priority interrupts
	BSF         INTCON+0, 7 
;robot_18F14k22_test.c,369 :: 		INTCON.GIEL = 1; //enable all low-priority interrupts
	BSF         INTCON+0, 6 
;robot_18F14k22_test.c,371 :: 		INTCON.GIE = 1; // Global Interrupt Enable bit
	BSF         INTCON+0, 7 
;robot_18F14k22_test.c,372 :: 		INTCON.PEIE = 1; // Peripheral Interrupt Enable bit
	BSF         INTCON+0, 6 
;robot_18F14k22_test.c,374 :: 		TMR0IE_bit = 1;  //Enable TIMER0
	BSF         TMR0IE_bit+0, BitPos(TMR0IE_bit+0) 
;robot_18F14k22_test.c,375 :: 		TMR0ON_bit = 1; // Start TIMER1
	BSF         TMR0ON_bit+0, BitPos(TMR0ON_bit+0) 
;robot_18F14k22_test.c,377 :: 		PWM1_Init(10000);  // Fréquence du PWM à 10Khz
	BSF         T2CON+0, 0, 0
	BCF         T2CON+0, 1, 0
	MOVLW       99
	MOVWF       PR2+0, 0
	CALL        _PWM1_Init+0, 0
;robot_18F14k22_test.c,378 :: 		RC6_bit = 0;  //Disable L6203
	BCF         RC6_bit+0, BitPos(RC6_bit+0) 
;robot_18F14k22_test.c,379 :: 		CCPR1 = 50; // Rapport cyclique à 50%
	MOVLW       50
	MOVWF       CCPR1+0 
	MOVLW       0
	MOVWF       CCPR1+1 
;robot_18F14k22_test.c,380 :: 		motor_current_speed = 127;
	MOVLW       127
	MOVWF       _motor_current_speed+0 
;robot_18F14k22_test.c,381 :: 		PWM1_Start();
	CALL        _PWM1_Start+0, 0
;robot_18F14k22_test.c,387 :: 		UART1_Init(115200);
	BSF         BAUDCON+0, 3, 0
	CLRF        SPBRGH+0 
	MOVLW       34
	MOVWF       SPBRG+0 
	BSF         TXSTA+0, 2, 0
	CALL        _UART1_Init+0, 0
;robot_18F14k22_test.c,388 :: 		delay_ms(100);
	MOVLW       3
	MOVWF       R11, 0
	MOVLW       8
	MOVWF       R12, 0
	MOVLW       119
	MOVWF       R13, 0
L_main68:
	DECFSZ      R13, 1, 1
	BRA         L_main68
	DECFSZ      R12, 1, 1
	BRA         L_main68
	DECFSZ      R11, 1, 1
	BRA         L_main68
;robot_18F14k22_test.c,390 :: 		while(1){
L_main69:
;robot_18F14k22_test.c,392 :: 		read_butee();
	CALL        _read_butee+0, 0
;robot_18F14k22_test.c,395 :: 		switch (state){
	GOTO        L_main71
;robot_18F14k22_test.c,396 :: 		case RESET_OUT:
L_main73:
;robot_18F14k22_test.c,398 :: 		LED2 = 1;
	BSF         RC2_bit+0, BitPos(RC2_bit+0) 
;robot_18F14k22_test.c,400 :: 		if (butee_out == 1){ // Sortie complète
	MOVF        _butee_out+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_main74
;robot_18F14k22_test.c,401 :: 		state = REGULATION;
	MOVLW       1
	MOVWF       _state+0 
;robot_18F14k22_test.c,402 :: 		optical_state = SB<<1 | SA;  //  ou logique de RA3 et RA2, lecture du capteur pour initialiser la machine d'état
	MOVLW       0
	BTFSC       RA4_bit+0, BitPos(RA4_bit+0) 
	MOVLW       1
	MOVWF       _optical_state+0 
	RLCF        _optical_state+0, 1 
	BCF         _optical_state+0, 0 
	CLRF        R0 
	BTFSC       RA2_bit+0, BitPos(RA2_bit+0) 
	INCF        R0, 1 
	MOVF        R0, 0 
	IORWF       _optical_state+0, 1 
;robot_18F14k22_test.c,403 :: 		nb_pulse = 0; // Reset Nb pulse (The reset is also done in the interrupt function)
	CLRF        _nb_pulse+0 
	CLRF        _nb_pulse+1 
;robot_18F14k22_test.c,404 :: 		position_set_point = 0;
	CLRF        _position_set_point+0 
	CLRF        _position_set_point+1 
;robot_18F14k22_test.c,405 :: 		}
	GOTO        L_main75
L_main74:
;robot_18F14k22_test.c,407 :: 		set_motor_cmd_out(motor_speed_out);
	MOVF        _motor_speed_out+0, 0 
	MOVWF       FARG_set_motor_cmd_out_speed+0 
	CALL        _set_motor_cmd_out+0, 0
L_main75:
;robot_18F14k22_test.c,409 :: 		break;
	GOTO        L_main72
;robot_18F14k22_test.c,411 :: 		case REGULATION:
L_main76:
;robot_18F14k22_test.c,413 :: 		read_optical_fork();
	CALL        _read_optical_fork+0, 0
;robot_18F14k22_test.c,414 :: 		LED2 = 0;
	BCF         RC2_bit+0, BitPos(RC2_bit+0) 
;robot_18F14k22_test.c,417 :: 		error = position_set_point - nb_pulse;
	MOVF        _nb_pulse+0, 0 
	SUBWF       _position_set_point+0, 0 
	MOVWF       R1 
	MOVF        _nb_pulse+1, 0 
	SUBWFB      _position_set_point+1, 0 
	MOVWF       R2 
	MOVF        R1, 0 
	MOVWF       _error+0 
	MOVF        R2, 0 
	MOVWF       _error+1 
;robot_18F14k22_test.c,419 :: 		if(error > error_interval)
	MOVLW       128
	MOVWF       R0 
	MOVLW       128
	XORWF       R2, 0 
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__main156
	MOVF        R1, 0 
	SUBWF       _error_interval+0, 0 
L__main156:
	BTFSC       STATUS+0, 0 
	GOTO        L_main77
;robot_18F14k22_test.c,420 :: 		set_motor_cmd_in(motor_speed_in);
	MOVF        _motor_speed_in+0, 0 
	MOVWF       FARG_set_motor_cmd_in_speed+0 
	CALL        _set_motor_cmd_in+0, 0
	GOTO        L_main78
L_main77:
;robot_18F14k22_test.c,421 :: 		else if(error < error_interval)
	MOVLW       128
	XORWF       _error+1, 0 
	MOVWF       R0 
	MOVLW       128
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__main157
	MOVF        _error_interval+0, 0 
	SUBWF       _error+0, 0 
L__main157:
	BTFSC       STATUS+0, 0 
	GOTO        L_main79
;robot_18F14k22_test.c,422 :: 		set_motor_cmd_out(motor_speed_out);
	MOVF        _motor_speed_out+0, 0 
	MOVWF       FARG_set_motor_cmd_out_speed+0 
	CALL        _set_motor_cmd_out+0, 0
	GOTO        L_main80
L_main79:
;robot_18F14k22_test.c,424 :: 		set_motor_cmd_stop();
	CALL        _set_motor_cmd_stop+0, 0
L_main80:
L_main78:
;robot_18F14k22_test.c,426 :: 		break;
	GOTO        L_main72
;robot_18F14k22_test.c,427 :: 		default:
L_main81:
;robot_18F14k22_test.c,428 :: 		break;
	GOTO        L_main72
;robot_18F14k22_test.c,429 :: 		}
L_main71:
	MOVF        _state+0, 0 
	XORLW       0
	BTFSC       STATUS+0, 2 
	GOTO        L_main73
	MOVF        _state+0, 0 
	XORLW       1
	BTFSC       STATUS+0, 2 
	GOTO        L_main76
	GOTO        L_main81
L_main72:
;robot_18F14k22_test.c,430 :: 		}
	GOTO        L_main69
;robot_18F14k22_test.c,431 :: 		}
L_end_main:
	GOTO        $+0
; end of _main

_interrupt:

;robot_18F14k22_test.c,442 :: 		void interrupt(){
;robot_18F14k22_test.c,444 :: 		if(INTCON.INT0IF == 1) {
	BTFSS       INTCON+0, 1 
	GOTO        L_interrupt82
;robot_18F14k22_test.c,445 :: 		if (!RA0_bit){
	BTFSC       RA0_bit+0, BitPos(RA0_bit+0) 
	GOTO        L_interrupt83
;robot_18F14k22_test.c,446 :: 		delay_ms(2);
	MOVLW       11
	MOVWF       R12, 0
	MOVLW       98
	MOVWF       R13, 0
L_interrupt84:
	DECFSZ      R13, 1, 1
	BRA         L_interrupt84
	DECFSZ      R12, 1, 1
	BRA         L_interrupt84
	NOP
;robot_18F14k22_test.c,447 :: 		if (!RA0_bit){
	BTFSC       RA0_bit+0, BitPos(RA0_bit+0) 
	GOTO        L_interrupt85
;robot_18F14k22_test.c,448 :: 		butee_out = 1;
	MOVLW       1
	MOVWF       _butee_out+0 
;robot_18F14k22_test.c,449 :: 		CCPR1 = 50;  // Rapport cyclique à 50% pour stopper le moteur et garder du couple (=> pose problème car dépend du sens !)
	MOVLW       50
	MOVWF       CCPR1+0 
	MOVLW       0
	MOVWF       CCPR1+1 
;robot_18F14k22_test.c,451 :: 		}
L_interrupt85:
;robot_18F14k22_test.c,452 :: 		}
L_interrupt83:
;robot_18F14k22_test.c,453 :: 		INTCON.INT0IF = 0;
	BCF         INTCON+0, 1 
;robot_18F14k22_test.c,454 :: 		}
	GOTO        L_interrupt86
L_interrupt82:
;robot_18F14k22_test.c,457 :: 		else if(INTCON3.INT1IF == 1) {
	BTFSS       INTCON3+0, 0 
	GOTO        L_interrupt87
;robot_18F14k22_test.c,458 :: 		if (!RA1_bit){
	BTFSC       RA1_bit+0, BitPos(RA1_bit+0) 
	GOTO        L_interrupt88
;robot_18F14k22_test.c,459 :: 		delay_ms(2);
	MOVLW       11
	MOVWF       R12, 0
	MOVLW       98
	MOVWF       R13, 0
L_interrupt89:
	DECFSZ      R13, 1, 1
	BRA         L_interrupt89
	DECFSZ      R12, 1, 1
	BRA         L_interrupt89
	NOP
;robot_18F14k22_test.c,460 :: 		if (!RA1_bit){
	BTFSC       RA1_bit+0, BitPos(RA1_bit+0) 
	GOTO        L_interrupt90
;robot_18F14k22_test.c,461 :: 		butee_in = 1;
	MOVLW       1
	MOVWF       _butee_in+0 
;robot_18F14k22_test.c,462 :: 		CCPR1 = 50;  // Rapport cyclique à 50% pour stopper le moteur et garder du couple
	MOVLW       50
	MOVWF       CCPR1+0 
	MOVLW       0
	MOVWF       CCPR1+1 
;robot_18F14k22_test.c,464 :: 		}
L_interrupt90:
;robot_18F14k22_test.c,465 :: 		}
L_interrupt88:
;robot_18F14k22_test.c,466 :: 		INTCON3.INT1IF = 0;
	BCF         INTCON3+0, 0 
;robot_18F14k22_test.c,467 :: 		}
	GOTO        L_interrupt91
L_interrupt87:
;robot_18F14k22_test.c,469 :: 		else if (TMR0IF_bit){
	BTFSS       TMR0IF_bit+0, BitPos(TMR0IF_bit+0) 
	GOTO        L_interrupt92
;robot_18F14k22_test.c,471 :: 		if(watchdog_restart>0)
	MOVF        _watchdog_restart+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt93
;robot_18F14k22_test.c,472 :: 		watchdog_restart--;
	DECF        _watchdog_restart+0, 1 
	GOTO        L_interrupt94
L_interrupt93:
;robot_18F14k22_test.c,474 :: 		position_set_point = 0;
	CLRF        _position_set_point+0 
	CLRF        _position_set_point+1 
;robot_18F14k22_test.c,475 :: 		watchdog_restart = watchdog_restart_default;
	MOVF        _watchdog_restart_default+0, 0 
	MOVWF       _watchdog_restart+0 
;robot_18F14k22_test.c,476 :: 		}
L_interrupt94:
;robot_18F14k22_test.c,478 :: 		TMR0H = 0x0B;
	MOVLW       11
	MOVWF       TMR0H+0 
;robot_18F14k22_test.c,479 :: 		TMR0L = 0xDC;
	MOVLW       220
	MOVWF       TMR0L+0 
;robot_18F14k22_test.c,480 :: 		TMR0IF_bit = 0;
	BCF         TMR0IF_bit+0, BitPos(TMR0IF_bit+0) 
;robot_18F14k22_test.c,481 :: 		}
L_interrupt92:
L_interrupt91:
L_interrupt86:
;robot_18F14k22_test.c,482 :: 		}
L_end_interrupt:
L__interrupt159:
	RETFIE      1
; end of _interrupt

_init_i2c:

;robot_18F14k22_test.c,487 :: 		void init_i2c(){
;robot_18F14k22_test.c,490 :: 		TRISB4_bit = 1; // RB4 en entrée
	BSF         TRISB4_bit+0, BitPos(TRISB4_bit+0) 
;robot_18F14k22_test.c,491 :: 		TRISB6_bit = 1; // RB6 en entrée
	BSF         TRISB6_bit+0, BitPos(TRISB6_bit+0) 
;robot_18F14k22_test.c,494 :: 		PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
	BSF         PIE1+0, 3 
;robot_18F14k22_test.c,495 :: 		PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
	BCF         PIR1+0, 3 
;robot_18F14k22_test.c,497 :: 		PIR2.BCLIE = 1;
	BSF         PIR2+0, 3 
;robot_18F14k22_test.c,498 :: 		PIR2.BCLIF = 0;
	BCF         PIR2+0, 3 
;robot_18F14k22_test.c,501 :: 		SSPADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
	MOVLW       112
	MOVWF       SSPADD+0 
;robot_18F14k22_test.c,502 :: 		SSPMSK = 0xFF; // A zero (‘0’) bit in the SSPMSK register has the effect of making
	MOVLW       255
	MOVWF       SSPMSK+0 
;robot_18F14k22_test.c,506 :: 		SSPSTAT.SMP = 1; // Slew Rate Control bit
	BSF         SSPSTAT+0, 7 
;robot_18F14k22_test.c,509 :: 		SSPSTAT.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)
	BSF         SSPSTAT+0, 6 
;robot_18F14k22_test.c,512 :: 		SSPCON2 = 0x00;
	CLRF        SSPCON2+0 
;robot_18F14k22_test.c,513 :: 		SSPCON2.GCEN = 0; // General Call Enable bit (0 = disabled)
	BCF         SSPCON2+0, 7 
;robot_18F14k22_test.c,514 :: 		SSPCON2.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)
	BSF         SSPCON2+0, 0 
;robot_18F14k22_test.c,517 :: 		SSPCON1.WCOL = 0; // Write Collision Detect bit
	BCF         SSPCON1+0, 7 
;robot_18F14k22_test.c,518 :: 		SSPCON1.SSPOV = 0; // Receive Overflow Indicator bit
	BCF         SSPCON1+0, 6 
;robot_18F14k22_test.c,519 :: 		SSPCON1.CKP = 1; // SCK Release Control bit (1=Release clock)
	BSF         SSPCON1+0, 4 
;robot_18F14k22_test.c,520 :: 		SSPCON1.SSPM3 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 3 
;robot_18F14k22_test.c,521 :: 		SSPCON1.SSPM2 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 2 
;robot_18F14k22_test.c,522 :: 		SSPCON1.SSPM1 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 1 
;robot_18F14k22_test.c,523 :: 		SSPCON1.SSPM0 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BCF         SSPCON1+0, 0 
;robot_18F14k22_test.c,526 :: 		SSPCON1.SSPEN = 1; // Synchronous Serial Port Enable bit
	BSF         SSPCON1+0, 5 
;robot_18F14k22_test.c,527 :: 		}
L_end_init_i2c:
	RETURN      0
; end of _init_i2c

_interrupt_low:
	MOVWF       ___Low_saveWREG+0 
	MOVF        STATUS+0, 0 
	MOVWF       ___Low_saveSTATUS+0 
	MOVF        BSR+0, 0 
	MOVWF       ___Low_saveBSR+0 

;robot_18F14k22_test.c,532 :: 		void interrupt_low(){
;robot_18F14k22_test.c,533 :: 		if (PIR1.SSPIF){  // I2C Interrupt
	BTFSS       PIR1+0, 3 
	GOTO        L_interrupt_low95
;robot_18F14k22_test.c,535 :: 		if(SSPCON1.SSPOV || SSPCON1.WCOL){
	BTFSC       SSPCON1+0, 6 
	GOTO        L__interrupt_low113
	BTFSC       SSPCON1+0, 7 
	GOTO        L__interrupt_low113
	GOTO        L_interrupt_low98
L__interrupt_low113:
;robot_18F14k22_test.c,536 :: 		SSPCON1.SSPOV = 0;
	BCF         SSPCON1+0, 6 
;robot_18F14k22_test.c,537 :: 		SSPCON1.WCOL = 0;
	BCF         SSPCON1+0, 7 
;robot_18F14k22_test.c,538 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;robot_18F14k22_test.c,539 :: 		}
L_interrupt_low98:
;robot_18F14k22_test.c,543 :: 		if (SSPSTAT.R_W == 0){
	BTFSC       SSPSTAT+0, 2 
	GOTO        L_interrupt_low99
;robot_18F14k22_test.c,544 :: 		if(SSPSTAT.P == 0){
	BTFSC       SSPSTAT+0, 4 
	GOTO        L_interrupt_low100
;robot_18F14k22_test.c,545 :: 		if (SSPSTAT.D_A == 0){ // Address
	BTFSC       SSPSTAT+0, 5 
	GOTO        L_interrupt_low101
;robot_18F14k22_test.c,546 :: 		nb_rx_octet = 0;
	CLRF        _nb_rx_octet+0 
;robot_18F14k22_test.c,547 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;robot_18F14k22_test.c,548 :: 		}
	GOTO        L_interrupt_low102
L_interrupt_low101:
;robot_18F14k22_test.c,550 :: 		if(nb_rx_octet < SIZE_RX_BUFFER){
	MOVLW       8
	SUBWF       _nb_rx_octet+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt_low103
;robot_18F14k22_test.c,551 :: 		rxbuffer_tab[nb_rx_octet] = SSPBUF;
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
;robot_18F14k22_test.c,552 :: 		nb_rx_octet++;
	INCF        _nb_rx_octet+0, 1 
;robot_18F14k22_test.c,553 :: 		}
	GOTO        L_interrupt_low104
L_interrupt_low103:
;robot_18F14k22_test.c,555 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;robot_18F14k22_test.c,556 :: 		}
L_interrupt_low104:
;robot_18F14k22_test.c,557 :: 		}
L_interrupt_low102:
;robot_18F14k22_test.c,558 :: 		}
L_interrupt_low100:
;robot_18F14k22_test.c,560 :: 		if(nb_rx_octet>1){
	MOVF        _nb_rx_octet+0, 0 
	SUBLW       1
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt_low105
;robot_18F14k22_test.c,561 :: 		Delay_us(30); // Wait P signal ?
	MOVLW       39
	MOVWF       R13, 0
L_interrupt_low106:
	DECFSZ      R13, 1, 1
	BRA         L_interrupt_low106
	NOP
	NOP
;robot_18F14k22_test.c,562 :: 		if(SSPSTAT.P == 1){
	BTFSS       SSPSTAT+0, 4 
	GOTO        L_interrupt_low107
;robot_18F14k22_test.c,563 :: 		i2c_read_data_from_buffer();
	CALL        _i2c_read_data_from_buffer+0, 0
;robot_18F14k22_test.c,564 :: 		nb_rx_octet = 0;
	CLRF        _nb_rx_octet+0 
;robot_18F14k22_test.c,565 :: 		}
L_interrupt_low107:
;robot_18F14k22_test.c,566 :: 		}
L_interrupt_low105:
;robot_18F14k22_test.c,567 :: 		}
	GOTO        L_interrupt_low108
L_interrupt_low99:
;robot_18F14k22_test.c,571 :: 		if(SSPSTAT.D_A == 0){
	BTFSC       SSPSTAT+0, 5 
	GOTO        L_interrupt_low109
;robot_18F14k22_test.c,572 :: 		nb_tx_octet = 0;
	CLRF        _nb_tx_octet+0 
;robot_18F14k22_test.c,573 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;robot_18F14k22_test.c,574 :: 		}
L_interrupt_low109:
;robot_18F14k22_test.c,577 :: 		i2c_write_data_to_buffer(nb_tx_octet);
	MOVF        _nb_tx_octet+0, 0 
	MOVWF       FARG_i2c_write_data_to_buffer_nb_tx_octet+0 
	CALL        _i2c_write_data_to_buffer+0, 0
;robot_18F14k22_test.c,578 :: 		nb_tx_octet++;
	INCF        _nb_tx_octet+0, 1 
;robot_18F14k22_test.c,579 :: 		}
L_interrupt_low108:
;robot_18F14k22_test.c,581 :: 		SSPCON1.CKP = 1;
	BSF         SSPCON1+0, 4 
;robot_18F14k22_test.c,582 :: 		PIR1.SSPIF = 0; // reset SSP interrupt flag
	BCF         PIR1+0, 3 
;robot_18F14k22_test.c,583 :: 		}
L_interrupt_low95:
;robot_18F14k22_test.c,584 :: 		}
L_end_interrupt_low:
L__interrupt_low162:
	MOVF        ___Low_saveBSR+0, 0 
	MOVWF       BSR+0 
	MOVF        ___Low_saveSTATUS+0, 0 
	MOVWF       STATUS+0 
	SWAPF       ___Low_saveWREG+0, 1 
	SWAPF       ___Low_saveWREG+0, 0 
	RETFIE      0
; end of _interrupt_low

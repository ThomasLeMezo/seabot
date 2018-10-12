
_i2c_read_data_from_buffer:

;Alimentation_18F14k22.c,109 :: 		void i2c_read_data_from_buffer(){
;Alimentation_18F14k22.c,110 :: 		unsigned short i = 0;
	CLRF        i2c_read_data_from_buffer_i_L0+0 
;Alimentation_18F14k22.c,112 :: 		for(i=0; i<(nb_rx_octet-1); i++){
	CLRF        i2c_read_data_from_buffer_i_L0+0 
L_i2c_read_data_from_buffer0:
	DECF        _nb_rx_octet+0, 0 
	MOVWF       R1 
	CLRF        R2 
	MOVLW       0
	SUBWFB      R2, 1 
	MOVLW       128
	MOVWF       R0 
	MOVLW       128
	XORWF       R2, 0 
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer112
	MOVF        R1, 0 
	SUBWF       i2c_read_data_from_buffer_i_L0+0, 0 
L__i2c_read_data_from_buffer112:
	BTFSC       STATUS+0, 0 
	GOTO        L_i2c_read_data_from_buffer1
;Alimentation_18F14k22.c,113 :: 		switch(rxbuffer_tab[0]+i){
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDWF       _rxbuffer_tab+0, 0 
	MOVWF       R4 
	CLRF        R5 
	MOVLW       0
	ADDWFC      R5, 1 
	GOTO        L_i2c_read_data_from_buffer3
;Alimentation_18F14k22.c,114 :: 		case 0x00:  // alimentation
L_i2c_read_data_from_buffer5:
;Alimentation_18F14k22.c,115 :: 		switch(rxbuffer_tab[i+1]){
	MOVF        i2c_read_data_from_buffer_i_L0+0, 0 
	ADDLW       1
	MOVWF       R0 
	CLRF        R1 
	MOVLW       0
	ADDWFC      R1, 1 
	MOVLW       _rxbuffer_tab+0
	ADDWF       R0, 0 
	MOVWF       R2 
	MOVLW       hi_addr(_rxbuffer_tab+0)
	ADDWFC      R1, 0 
	MOVWF       R3 
	GOTO        L_i2c_read_data_from_buffer6
;Alimentation_18F14k22.c,116 :: 		case 0x01:
L_i2c_read_data_from_buffer8:
;Alimentation_18F14k22.c,117 :: 		state = POWER_ON;
	MOVLW       1
	MOVWF       _state+0 
;Alimentation_18F14k22.c,118 :: 		break;
	GOTO        L_i2c_read_data_from_buffer7
;Alimentation_18F14k22.c,119 :: 		case 0x02:
L_i2c_read_data_from_buffer9:
;Alimentation_18F14k22.c,120 :: 		time_to_stop = default_time_to_stop;  // Go to Sleep mode
	MOVF        _default_time_to_stop+0, 0 
	MOVWF       _time_to_stop+0 
;Alimentation_18F14k22.c,121 :: 		start_time_to_stop = 1;
	MOVLW       1
	MOVWF       _start_time_to_stop+0 
;Alimentation_18F14k22.c,122 :: 		state = WAIT_TO_SLEEP;
	MOVLW       2
	MOVWF       _state+0 
;Alimentation_18F14k22.c,123 :: 		break;
	GOTO        L_i2c_read_data_from_buffer7
;Alimentation_18F14k22.c,124 :: 		default:
L_i2c_read_data_from_buffer10:
;Alimentation_18F14k22.c,125 :: 		break;
	GOTO        L_i2c_read_data_from_buffer7
;Alimentation_18F14k22.c,126 :: 		}
L_i2c_read_data_from_buffer6:
	MOVFF       R2, FSR0
	MOVFF       R3, FSR0H
	MOVF        POSTINC0+0, 0 
	XORLW       1
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer8
	MOVFF       R2, FSR0
	MOVFF       R3, FSR0H
	MOVF        POSTINC0+0, 0 
	XORLW       2
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer9
	GOTO        L_i2c_read_data_from_buffer10
L_i2c_read_data_from_buffer7:
;Alimentation_18F14k22.c,127 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,128 :: 		case 0x01:  // led power
L_i2c_read_data_from_buffer11:
;Alimentation_18F14k22.c,129 :: 		start_led_puissance = (rxbuffer_tab[i+1]!=0x00);
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
	MOVWF       _start_led_puissance+0 
;Alimentation_18F14k22.c,130 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,131 :: 		case 0x02:
L_i2c_read_data_from_buffer12:
;Alimentation_18F14k22.c,132 :: 		led_puissance_delay = rxbuffer_tab[i+1];
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
	MOVWF       _led_puissance_delay+0 
;Alimentation_18F14k22.c,133 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,134 :: 		case 0x03:
L_i2c_read_data_from_buffer13:
;Alimentation_18F14k22.c,135 :: 		default_time_to_start[0] = rxbuffer_tab[i+1]; // hours
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
	MOVWF       _default_time_to_start+0 
;Alimentation_18F14k22.c,136 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,137 :: 		case 0x04:
L_i2c_read_data_from_buffer14:
;Alimentation_18F14k22.c,138 :: 		default_time_to_start[1] = rxbuffer_tab[i+1]; // min
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
	MOVWF       _default_time_to_start+1 
;Alimentation_18F14k22.c,139 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,140 :: 		case 0x05:
L_i2c_read_data_from_buffer15:
;Alimentation_18F14k22.c,141 :: 		default_time_to_start[2] = rxbuffer_tab[i+1]; // sec
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
	MOVWF       _default_time_to_start+2 
;Alimentation_18F14k22.c,142 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,143 :: 		case 0x06:
L_i2c_read_data_from_buffer16:
;Alimentation_18F14k22.c,144 :: 		default_time_to_stop = rxbuffer_tab[i+1]; // sec
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
	MOVWF       _default_time_to_stop+0 
;Alimentation_18F14k22.c,145 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,146 :: 		default:
L_i2c_read_data_from_buffer17:
;Alimentation_18F14k22.c,147 :: 		break;
	GOTO        L_i2c_read_data_from_buffer4
;Alimentation_18F14k22.c,148 :: 		}
L_i2c_read_data_from_buffer3:
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer113
	MOVLW       0
	XORWF       R4, 0 
L__i2c_read_data_from_buffer113:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer5
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer114
	MOVLW       1
	XORWF       R4, 0 
L__i2c_read_data_from_buffer114:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer11
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer115
	MOVLW       2
	XORWF       R4, 0 
L__i2c_read_data_from_buffer115:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer12
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer116
	MOVLW       3
	XORWF       R4, 0 
L__i2c_read_data_from_buffer116:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer13
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer117
	MOVLW       4
	XORWF       R4, 0 
L__i2c_read_data_from_buffer117:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer14
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer118
	MOVLW       5
	XORWF       R4, 0 
L__i2c_read_data_from_buffer118:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer15
	MOVLW       0
	XORWF       R5, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_read_data_from_buffer119
	MOVLW       6
	XORWF       R4, 0 
L__i2c_read_data_from_buffer119:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_read_data_from_buffer16
	GOTO        L_i2c_read_data_from_buffer17
L_i2c_read_data_from_buffer4:
;Alimentation_18F14k22.c,112 :: 		for(i=0; i<(nb_rx_octet-1); i++){
	INCF        i2c_read_data_from_buffer_i_L0+0, 1 
;Alimentation_18F14k22.c,149 :: 		}
	GOTO        L_i2c_read_data_from_buffer0
L_i2c_read_data_from_buffer1:
;Alimentation_18F14k22.c,150 :: 		}
L_end_i2c_read_data_from_buffer:
	RETURN      0
; end of _i2c_read_data_from_buffer

_i2c_write_data_to_buffer:

;Alimentation_18F14k22.c,156 :: 		void i2c_write_data_to_buffer(unsigned short nb_tx_octet){
;Alimentation_18F14k22.c,157 :: 		switch(rxbuffer_tab[0]+nb_tx_octet){
	MOVF        FARG_i2c_write_data_to_buffer_nb_tx_octet+0, 0 
	ADDWF       _rxbuffer_tab+0, 0 
	MOVWF       R3 
	CLRF        R4 
	MOVLW       0
	ADDWFC      R4, 1 
	GOTO        L_i2c_write_data_to_buffer18
;Alimentation_18F14k22.c,158 :: 		case 0x00:
L_i2c_write_data_to_buffer20:
;Alimentation_18F14k22.c,159 :: 		SSPBUF = battery_voltage[0];
	MOVF        _battery_voltage+0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,160 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,161 :: 		case 0x01:
L_i2c_write_data_to_buffer21:
;Alimentation_18F14k22.c,162 :: 		SSPBUF = battery_voltage[0] >> 8;
	MOVF        _battery_voltage+1, 0 
	MOVWF       R0 
	CLRF        R1 
	MOVF        R0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,163 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,164 :: 		case 0x02:
L_i2c_write_data_to_buffer22:
;Alimentation_18F14k22.c,165 :: 		SSPBUF = battery_voltage[1];
	MOVF        _battery_voltage+2, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,166 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,167 :: 		case 0x03:
L_i2c_write_data_to_buffer23:
;Alimentation_18F14k22.c,168 :: 		SSPBUF = battery_voltage[1] >> 8;
	MOVF        _battery_voltage+3, 0 
	MOVWF       R0 
	CLRF        R1 
	MOVF        R0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,169 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,170 :: 		case 0x04:
L_i2c_write_data_to_buffer24:
;Alimentation_18F14k22.c,171 :: 		SSPBUF = battery_voltage[2];
	MOVF        _battery_voltage+4, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,172 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,173 :: 		case 0x05:
L_i2c_write_data_to_buffer25:
;Alimentation_18F14k22.c,174 :: 		SSPBUF = battery_voltage[2] >> 8;
	MOVF        _battery_voltage+5, 0 
	MOVWF       R0 
	CLRF        R1 
	MOVF        R0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,175 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,176 :: 		case 0x06:
L_i2c_write_data_to_buffer26:
;Alimentation_18F14k22.c,177 :: 		SSPBUF = battery_voltage[3];
	MOVF        _battery_voltage+6, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,178 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,179 :: 		case 0x07:
L_i2c_write_data_to_buffer27:
;Alimentation_18F14k22.c,180 :: 		SSPBUF = battery_voltage[3] >> 8;
	MOVF        _battery_voltage+7, 0 
	MOVWF       R0 
	CLRF        R1 
	MOVF        R0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,181 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,182 :: 		case 0x08:
L_i2c_write_data_to_buffer28:
;Alimentation_18F14k22.c,183 :: 		SSPBUF = start_led_puissance;
	MOVF        _start_led_puissance+0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,184 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,185 :: 		case 0x09:
L_i2c_write_data_to_buffer29:
;Alimentation_18F14k22.c,186 :: 		SSPBUF = state;
	MOVF        _state+0, 0 
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,187 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,188 :: 		case 0xC0:
L_i2c_write_data_to_buffer30:
;Alimentation_18F14k22.c,189 :: 		SSPBUF = CODE_VERSION;
	MOVLW       1
	MOVWF       SSPBUF+0 
;Alimentation_18F14k22.c,190 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,191 :: 		default:
L_i2c_write_data_to_buffer31:
;Alimentation_18F14k22.c,192 :: 		SSPBUF = 0x00;
	CLRF        SSPBUF+0 
;Alimentation_18F14k22.c,193 :: 		break;
	GOTO        L_i2c_write_data_to_buffer19
;Alimentation_18F14k22.c,194 :: 		}
L_i2c_write_data_to_buffer18:
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer121
	MOVLW       0
	XORWF       R3, 0 
L__i2c_write_data_to_buffer121:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer20
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer122
	MOVLW       1
	XORWF       R3, 0 
L__i2c_write_data_to_buffer122:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer21
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer123
	MOVLW       2
	XORWF       R3, 0 
L__i2c_write_data_to_buffer123:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer22
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer124
	MOVLW       3
	XORWF       R3, 0 
L__i2c_write_data_to_buffer124:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer23
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer125
	MOVLW       4
	XORWF       R3, 0 
L__i2c_write_data_to_buffer125:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer24
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer126
	MOVLW       5
	XORWF       R3, 0 
L__i2c_write_data_to_buffer126:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer25
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer127
	MOVLW       6
	XORWF       R3, 0 
L__i2c_write_data_to_buffer127:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer26
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer128
	MOVLW       7
	XORWF       R3, 0 
L__i2c_write_data_to_buffer128:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer27
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer129
	MOVLW       8
	XORWF       R3, 0 
L__i2c_write_data_to_buffer129:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer28
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer130
	MOVLW       9
	XORWF       R3, 0 
L__i2c_write_data_to_buffer130:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer29
	MOVLW       0
	XORWF       R4, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__i2c_write_data_to_buffer131
	MOVLW       192
	XORWF       R3, 0 
L__i2c_write_data_to_buffer131:
	BTFSC       STATUS+0, 2 
	GOTO        L_i2c_write_data_to_buffer30
	GOTO        L_i2c_write_data_to_buffer31
L_i2c_write_data_to_buffer19:
;Alimentation_18F14k22.c,195 :: 		watchdog_restart = watchdog_restart_default;
	MOVF        _watchdog_restart_default+0, 0 
	MOVWF       _watchdog_restart+0 
	MOVF        _watchdog_restart_default+1, 0 
	MOVWF       _watchdog_restart+1 
;Alimentation_18F14k22.c,196 :: 		}
L_end_i2c_write_data_to_buffer:
	RETURN      0
; end of _i2c_write_data_to_buffer

_read_batteries_voltage:

;Alimentation_18F14k22.c,201 :: 		void read_batteries_voltage(){
;Alimentation_18F14k22.c,202 :: 		battery_voltage[0] = ADC_Get_Sample(4);   // Get 10-bit results of AD conversion AN4 batterie 1
	MOVLW       4
	MOVWF       FARG_ADC_Get_Sample_channel+0 
	CALL        _ADC_Get_Sample+0, 0
	MOVF        R0, 0 
	MOVWF       _battery_voltage+0 
	MOVF        R1, 0 
	MOVWF       _battery_voltage+1 
;Alimentation_18F14k22.c,203 :: 		battery_voltage[1] = ADC_Get_Sample(5);   // Get 10-bit results of AD conversion AN5 batterie 2
	MOVLW       5
	MOVWF       FARG_ADC_Get_Sample_channel+0 
	CALL        _ADC_Get_Sample+0, 0
	MOVF        R0, 0 
	MOVWF       _battery_voltage+2 
	MOVF        R1, 0 
	MOVWF       _battery_voltage+3 
;Alimentation_18F14k22.c,204 :: 		battery_voltage[2] = ADC_Get_Sample(6);   // Get 10-bit results of AD conversion AN6 batterie 3
	MOVLW       6
	MOVWF       FARG_ADC_Get_Sample_channel+0 
	CALL        _ADC_Get_Sample+0, 0
	MOVF        R0, 0 
	MOVWF       _battery_voltage+4 
	MOVF        R1, 0 
	MOVWF       _battery_voltage+5 
;Alimentation_18F14k22.c,205 :: 		battery_voltage[3] = ADC_Get_Sample(7);   // Get 10-bit results of AD conversion AN7 batterie 4
	MOVLW       7
	MOVWF       FARG_ADC_Get_Sample_channel+0 
	CALL        _ADC_Get_Sample+0, 0
	MOVF        R0, 0 
	MOVWF       _battery_voltage+6 
	MOVF        R1, 0 
	MOVWF       _battery_voltage+7 
;Alimentation_18F14k22.c,206 :: 		}
L_end_read_batteries_voltage:
	RETURN      0
; end of _read_batteries_voltage

_analyze_batteries_voltage:

;Alimentation_18F14k22.c,213 :: 		void analyze_batteries_voltage(){
;Alimentation_18F14k22.c,214 :: 		unsigned short l = 0;
	CLRF        analyze_batteries_voltage_l_L0+0 
;Alimentation_18F14k22.c,215 :: 		battery_global_default = 0;
	CLRF        _battery_global_default+0 
;Alimentation_18F14k22.c,217 :: 		for(l=0; l<4; l++){
	CLRF        analyze_batteries_voltage_l_L0+0 
L_analyze_batteries_voltage32:
	MOVLW       4
	SUBWF       analyze_batteries_voltage_l_L0+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_analyze_batteries_voltage33
;Alimentation_18F14k22.c,218 :: 		if(battery_voltage[l] < WARNING_LOW_VOLTAGE)
	MOVF        analyze_batteries_voltage_l_L0+0, 0 
	MOVWF       R0 
	MOVLW       0
	MOVWF       R1 
	RLCF        R0, 1 
	BCF         R0, 0 
	RLCF        R1, 1 
	MOVLW       _battery_voltage+0
	ADDWF       R0, 0 
	MOVWF       FSR0 
	MOVLW       hi_addr(_battery_voltage+0)
	ADDWFC      R1, 0 
	MOVWF       FSR0H 
	MOVF        POSTINC0+0, 0 
	MOVWF       R1 
	MOVF        POSTINC0+0, 0 
	MOVWF       R2 
	MOVLW       2
	SUBWF       R2, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__analyze_batteries_voltage134
	MOVLW       153
	SUBWF       R1, 0 
L__analyze_batteries_voltage134:
	BTFSC       STATUS+0, 0 
	GOTO        L_analyze_batteries_voltage35
;Alimentation_18F14k22.c,219 :: 		battery_global_default = 1;
	MOVLW       1
	MOVWF       _battery_global_default+0 
L_analyze_batteries_voltage35:
;Alimentation_18F14k22.c,217 :: 		for(l=0; l<4; l++){
	INCF        analyze_batteries_voltage_l_L0+0, 1 
;Alimentation_18F14k22.c,220 :: 		}
	GOTO        L_analyze_batteries_voltage32
L_analyze_batteries_voltage33:
;Alimentation_18F14k22.c,221 :: 		}
L_end_analyze_batteries_voltage:
	RETURN      0
; end of _analyze_batteries_voltage

_init_timer0:

;Alimentation_18F14k22.c,228 :: 		void init_timer0(){
;Alimentation_18F14k22.c,229 :: 		T0CON = 0x85; // TIMER0 ON (1 s)
	MOVLW       133
	MOVWF       T0CON+0 
;Alimentation_18F14k22.c,230 :: 		TMR0H = 0x0B;
	MOVLW       11
	MOVWF       TMR0H+0 
;Alimentation_18F14k22.c,231 :: 		TMR0L = 0xDC;
	MOVLW       220
	MOVWF       TMR0L+0 
;Alimentation_18F14k22.c,232 :: 		TMR0IE_bit = 0;
	BCF         TMR0IE_bit+0, BitPos(TMR0IE_bit+0) 
;Alimentation_18F14k22.c,233 :: 		}
L_end_init_timer0:
	RETURN      0
; end of _init_timer0

_init_timer3:

;Alimentation_18F14k22.c,240 :: 		void init_timer3(){
;Alimentation_18F14k22.c,241 :: 		T3CON = 0x30;
	MOVLW       48
	MOVWF       T3CON+0 
;Alimentation_18F14k22.c,242 :: 		TMR3IF_bit = 0;
	BCF         TMR3IF_bit+0, BitPos(TMR3IF_bit+0) 
;Alimentation_18F14k22.c,243 :: 		TMR3H = 0x3C;
	MOVLW       60
	MOVWF       TMR3H+0 
;Alimentation_18F14k22.c,244 :: 		TMR3L = 0xB0;
	MOVLW       176
	MOVWF       TMR3L+0 
;Alimentation_18F14k22.c,245 :: 		TMR3IE_bit = 0;
	BCF         TMR3IE_bit+0, BitPos(TMR3IE_bit+0) 
;Alimentation_18F14k22.c,246 :: 		}
L_end_init_timer3:
	RETURN      0
; end of _init_timer3

_init_io:

;Alimentation_18F14k22.c,252 :: 		void init_io(){
;Alimentation_18F14k22.c,253 :: 		ANSEL = 0xF0;  // Set RC0,RC1,RC2,RC3 to analog (AN4,AN5,AN6,AN7)
	MOVLW       240
	MOVWF       ANSEL+0 
;Alimentation_18F14k22.c,255 :: 		CM1CON0 = 0x00; // Not using the comparators
	CLRF        CM1CON0+0 
;Alimentation_18F14k22.c,256 :: 		CM2CON0 = 0x00;
	CLRF        CM2CON0+0 
;Alimentation_18F14k22.c,260 :: 		TRISA = 0xFF;
	MOVLW       255
	MOVWF       TRISA+0 
;Alimentation_18F14k22.c,262 :: 		TRISA0_bit = 0; // RA0 en sortie
	BCF         TRISA0_bit+0, BitPos(TRISA0_bit+0) 
;Alimentation_18F14k22.c,263 :: 		TRISA2_bit = 1; // RA2 en entrée
	BSF         TRISA2_bit+0, BitPos(TRISA2_bit+0) 
;Alimentation_18F14k22.c,264 :: 		TRISA4_bit = 0; // RA4 en sortie
	BCF         TRISA4_bit+0, BitPos(TRISA4_bit+0) 
;Alimentation_18F14k22.c,265 :: 		TRISA5_bit = 0; // RA5 en sortie
	BCF         TRISA5_bit+0, BitPos(TRISA5_bit+0) 
;Alimentation_18F14k22.c,267 :: 		TRISA1_bit = 0; // RA1 en sortie
	BCF         TRISA1_bit+0, BitPos(TRISA1_bit+0) 
;Alimentation_18F14k22.c,269 :: 		INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit
	BCF         INTCON2+0, 7 
;Alimentation_18F14k22.c,270 :: 		WPUA.WPUA2 = 1; // Pull-up enabled sur RA2
	BSF         WPUA+0, 2 
;Alimentation_18F14k22.c,272 :: 		TRISB5_bit = 1; // RB5 en entrée
	BSF         TRISB5_bit+0, BitPos(TRISB5_bit+0) 
;Alimentation_18F14k22.c,273 :: 		TRISB7_bit = 0; // RB6 en sortie
	BCF         TRISB7_bit+0, BitPos(TRISB7_bit+0) 
;Alimentation_18F14k22.c,275 :: 		TRISC = 0xFF;
	MOVLW       255
	MOVWF       TRISC+0 
;Alimentation_18F14k22.c,276 :: 		TRISC0_bit = 1; // RC0 en entree voie AN4
	BSF         TRISC0_bit+0, BitPos(TRISC0_bit+0) 
;Alimentation_18F14k22.c,277 :: 		TRISC1_bit = 1; // RC1 en entree voie AN5
	BSF         TRISC1_bit+0, BitPos(TRISC1_bit+0) 
;Alimentation_18F14k22.c,278 :: 		TRISC2_bit = 1; // RC2 en entree voie AN6
	BSF         TRISC2_bit+0, BitPos(TRISC2_bit+0) 
;Alimentation_18F14k22.c,279 :: 		TRISC3_bit = 1; // RC3 en entree voie AN7
	BSF         TRISC3_bit+0, BitPos(TRISC3_bit+0) 
;Alimentation_18F14k22.c,280 :: 		}
L_end_init_io:
	RETURN      0
; end of _init_io

_main:

;Alimentation_18F14k22.c,286 :: 		void main(){
;Alimentation_18F14k22.c,288 :: 		OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block
	MOVLW       114
	MOVWF       OSCCON+0 
;Alimentation_18F14k22.c,290 :: 		init_io(); // Initialisation des I/O
	CALL        _init_io+0, 0
;Alimentation_18F14k22.c,291 :: 		init_i2c(); // Initialisation de l'I2C en esclave
	CALL        _init_i2c+0, 0
;Alimentation_18F14k22.c,292 :: 		init_timer0(); // Initialisation du TIMER0 toutes les 1 secondes
	CALL        _init_timer0+0, 0
;Alimentation_18F14k22.c,293 :: 		init_timer3(); // Initialisation du TIMER3 toutes les 100ms
	CALL        _init_timer3+0, 0
;Alimentation_18F14k22.c,295 :: 		ADC_Init();
	CALL        _ADC_Init+0, 0
;Alimentation_18F14k22.c,297 :: 		LED = 0; // sortie LED
	BCF         LATA+0, 0 
;Alimentation_18F14k22.c,298 :: 		LED1 = 0;
	BCF         LATA+0, 1 
;Alimentation_18F14k22.c,299 :: 		LED_PUISSANCE = 0; // sortie LED de puissance
	BCF         LATA+0, 4 
;Alimentation_18F14k22.c,300 :: 		ALIM = 0; // sortie MOSFET de puissance, commande de l'alimentation
	BCF         LATA+0, 5 
;Alimentation_18F14k22.c,301 :: 		battery_global_default = 0;
	CLRF        _battery_global_default+0 
;Alimentation_18F14k22.c,303 :: 		UART1_Init(115200);
	BSF         BAUDCON+0, 3, 0
	CLRF        SPBRGH+0 
	MOVLW       34
	MOVWF       SPBRG+0 
	BSF         TXSTA+0, 2, 0
	CALL        _UART1_Init+0, 0
;Alimentation_18F14k22.c,305 :: 		delay_ms(1000);
	MOVLW       21
	MOVWF       R11, 0
	MOVLW       75
	MOVWF       R12, 0
	MOVLW       190
	MOVWF       R13, 0
L_main36:
	DECFSZ      R13, 1, 1
	BRA         L_main36
	DECFSZ      R12, 1, 1
	BRA         L_main36
	DECFSZ      R11, 1, 1
	BRA         L_main36
	NOP
;Alimentation_18F14k22.c,307 :: 		INTCON3.INT1IP = 0; //INT1 External Interrupt Priority bit, INT0 always a high
	BCF         INTCON3+0, 6 
;Alimentation_18F14k22.c,310 :: 		RCON.IPEN = 1;  //Enable priority levels on interrupts
	BSF         RCON+0, 7 
;Alimentation_18F14k22.c,311 :: 		IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit (low priority = 0)
	BCF         IPR1+0, 3 
;Alimentation_18F14k22.c,312 :: 		INTCON.GIEH = 1; //enable all high-priority interrupts
	BSF         INTCON+0, 7 
;Alimentation_18F14k22.c,313 :: 		INTCON.GIEL = 1; //enable all low-priority interrupts
	BSF         INTCON+0, 6 
;Alimentation_18F14k22.c,315 :: 		INTCON.GIE = 1; // Global Interrupt Enable bit
	BSF         INTCON+0, 7 
;Alimentation_18F14k22.c,316 :: 		INTCON.PEIE = 1; // Peripheral Interrupt Enable bit
	BSF         INTCON+0, 6 
;Alimentation_18F14k22.c,318 :: 		TMR0IE_bit = 1;  //Enable TIMER0
	BSF         TMR0IE_bit+0, BitPos(TMR0IE_bit+0) 
;Alimentation_18F14k22.c,319 :: 		TMR0ON_bit = 1; // Start TIMER1
	BSF         TMR0ON_bit+0, BitPos(TMR0ON_bit+0) 
;Alimentation_18F14k22.c,321 :: 		TMR3IE_bit = 1;  //Enable TIMER3
	BSF         TMR3IE_bit+0, BitPos(TMR3IE_bit+0) 
;Alimentation_18F14k22.c,322 :: 		TMR3ON_bit = 1; // Start TIMER3
	BSF         TMR3ON_bit+0, BitPos(TMR3ON_bit+0) 
;Alimentation_18F14k22.c,324 :: 		while(1){
L_main37:
;Alimentation_18F14k22.c,325 :: 		read_batteries_voltage();
	CALL        _read_batteries_voltage+0, 0
;Alimentation_18F14k22.c,326 :: 		analyze_batteries_voltage();
	CALL        _analyze_batteries_voltage+0, 0
;Alimentation_18F14k22.c,330 :: 		switch (state){
	GOTO        L_main39
;Alimentation_18F14k22.c,331 :: 		case IDLE: // Idle state
L_main41:
;Alimentation_18F14k22.c,332 :: 		ALIM = 0;
	BCF         LATA+0, 5 
;Alimentation_18F14k22.c,333 :: 		led_delay = 50;
	MOVLW       50
	MOVWF       _led_delay+0 
;Alimentation_18F14k22.c,334 :: 		start_led_puissance = 0;
	CLRF        _start_led_puissance+0 
;Alimentation_18F14k22.c,336 :: 		if(ILS==0){ // Magnet detected
	BTFSC       PORTA+0, 2 
	GOTO        L_main42
;Alimentation_18F14k22.c,337 :: 		ils_cpt--;
	DECF        _ils_cpt+0, 1 
;Alimentation_18F14k22.c,338 :: 		set_led_on = 1;
	MOVLW       1
	MOVWF       _set_led_on+0 
;Alimentation_18F14k22.c,339 :: 		}
	GOTO        L_main43
L_main42:
;Alimentation_18F14k22.c,341 :: 		ils_cpt = ILS_CPT_TIME;
	MOVLW       4
	MOVWF       _ils_cpt+0 
;Alimentation_18F14k22.c,342 :: 		set_led_on = 0;
	CLRF        _set_led_on+0 
;Alimentation_18F14k22.c,343 :: 		ils_removed = 1;
	MOVLW       1
	MOVWF       _ils_removed+0 
;Alimentation_18F14k22.c,344 :: 		}
L_main43:
;Alimentation_18F14k22.c,346 :: 		if(ils_removed == 1 && ils_cpt == 0){
	MOVF        _ils_removed+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_main46
	MOVF        _ils_cpt+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_main46
L__main109:
;Alimentation_18F14k22.c,347 :: 		ils_cpt = ILS_CPT_TIME;
	MOVLW       4
	MOVWF       _ils_cpt+0 
;Alimentation_18F14k22.c,348 :: 		state = POWER_ON;
	MOVLW       1
	MOVWF       _state+0 
;Alimentation_18F14k22.c,349 :: 		ils_removed = 0;
	CLRF        _ils_removed+0 
;Alimentation_18F14k22.c,350 :: 		set_led_on = 0;
	CLRF        _set_led_on+0 
;Alimentation_18F14k22.c,351 :: 		}
L_main46:
;Alimentation_18F14k22.c,352 :: 		break;
	GOTO        L_main40
;Alimentation_18F14k22.c,354 :: 		case POWER_ON:
L_main47:
;Alimentation_18F14k22.c,355 :: 		ALIM = 1;
	BSF         LATA+0, 5 
;Alimentation_18F14k22.c,356 :: 		if(battery_global_default == 1)
	MOVF        _battery_global_default+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_main48
;Alimentation_18F14k22.c,357 :: 		led_delay = 5; // 0.5 sec
	MOVLW       5
	MOVWF       _led_delay+0 
	GOTO        L_main49
L_main48:
;Alimentation_18F14k22.c,359 :: 		led_delay = 20; // 5 sec
	MOVLW       20
	MOVWF       _led_delay+0 
L_main49:
;Alimentation_18F14k22.c,361 :: 		if(ILS==0){ // Magnet detected
	BTFSC       PORTA+0, 2 
	GOTO        L_main50
;Alimentation_18F14k22.c,362 :: 		ils_cpt--;
	DECF        _ils_cpt+0, 1 
;Alimentation_18F14k22.c,363 :: 		set_led_on = 1;
	MOVLW       1
	MOVWF       _set_led_on+0 
;Alimentation_18F14k22.c,364 :: 		}
	GOTO        L_main51
L_main50:
;Alimentation_18F14k22.c,366 :: 		ils_cpt = ILS_CPT_TIME;
	MOVLW       4
	MOVWF       _ils_cpt+0 
;Alimentation_18F14k22.c,367 :: 		set_led_on = 0;
	CLRF        _set_led_on+0 
;Alimentation_18F14k22.c,368 :: 		ils_removed = 1;
	MOVLW       1
	MOVWF       _ils_removed+0 
;Alimentation_18F14k22.c,369 :: 		}
L_main51:
;Alimentation_18F14k22.c,371 :: 		if(ils_removed == 1 && ils_cpt == 0){
	MOVF        _ils_removed+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_main54
	MOVF        _ils_cpt+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_main54
L__main108:
;Alimentation_18F14k22.c,372 :: 		ils_cpt = ILS_CPT_TIME;
	MOVLW       4
	MOVWF       _ils_cpt+0 
;Alimentation_18F14k22.c,373 :: 		state = IDLE;
	CLRF        _state+0 
;Alimentation_18F14k22.c,374 :: 		ils_removed = 0;
	CLRF        _ils_removed+0 
;Alimentation_18F14k22.c,375 :: 		set_led_on = 0;
	CLRF        _set_led_on+0 
;Alimentation_18F14k22.c,376 :: 		}
L_main54:
;Alimentation_18F14k22.c,378 :: 		break;
	GOTO        L_main40
;Alimentation_18F14k22.c,380 :: 		case WAIT_TO_SLEEP:
L_main55:
;Alimentation_18F14k22.c,382 :: 		ALIM = 1;
	BSF         LATA+0, 5 
;Alimentation_18F14k22.c,383 :: 		led_delay = 1;
	MOVLW       1
	MOVWF       _led_delay+0 
;Alimentation_18F14k22.c,384 :: 		if(time_to_stop==0){
	MOVF        _time_to_stop+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_main56
;Alimentation_18F14k22.c,385 :: 		for(k=0; k<3; k++)
	CLRF        _k+0 
L_main57:
	MOVLW       3
	SUBWF       _k+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_main58
;Alimentation_18F14k22.c,386 :: 		time_to_start[k] = default_time_to_start[k];
	MOVLW       _time_to_start+0
	MOVWF       FSR1 
	MOVLW       hi_addr(_time_to_start+0)
	MOVWF       FSR1H 
	MOVF        _k+0, 0 
	ADDWF       FSR1, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR1H, 1 
	MOVLW       _default_time_to_start+0
	MOVWF       FSR0 
	MOVLW       hi_addr(_default_time_to_start+0)
	MOVWF       FSR0H 
	MOVF        _k+0, 0 
	ADDWF       FSR0, 1 
	BTFSC       STATUS+0, 0 
	INCF        FSR0H, 1 
	MOVF        POSTINC0+0, 0 
	MOVWF       POSTINC1+0 
;Alimentation_18F14k22.c,385 :: 		for(k=0; k<3; k++)
	INCF        _k+0, 1 
;Alimentation_18F14k22.c,386 :: 		time_to_start[k] = default_time_to_start[k];
	GOTO        L_main57
L_main58:
;Alimentation_18F14k22.c,387 :: 		state = SLEEP;
	MOVLW       3
	MOVWF       _state+0 
;Alimentation_18F14k22.c,388 :: 		time_to_stop = default_time_to_stop;
	MOVF        _default_time_to_stop+0, 0 
	MOVWF       _time_to_stop+0 
;Alimentation_18F14k22.c,389 :: 		}
L_main56:
;Alimentation_18F14k22.c,390 :: 		break;
	GOTO        L_main40
;Alimentation_18F14k22.c,392 :: 		case SLEEP:
L_main60:
;Alimentation_18F14k22.c,393 :: 		ALIM = 0;
	BCF         LATA+0, 5 
;Alimentation_18F14k22.c,394 :: 		led_delay = 600;
	MOVLW       88
	MOVWF       _led_delay+0 
;Alimentation_18F14k22.c,395 :: 		if(time_to_start[0] == 0 && time_to_start[1] == 0 && time_to_start[2] == 0){
	MOVF        _time_to_start+0, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_main63
	MOVF        _time_to_start+1, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_main63
	MOVF        _time_to_start+2, 0 
	XORLW       0
	BTFSS       STATUS+0, 2 
	GOTO        L_main63
L__main107:
;Alimentation_18F14k22.c,396 :: 		state = POWER_ON;
	MOVLW       1
	MOVWF       _state+0 
;Alimentation_18F14k22.c,397 :: 		}
L_main63:
;Alimentation_18F14k22.c,398 :: 		break;
	GOTO        L_main40
;Alimentation_18F14k22.c,400 :: 		default:
L_main64:
;Alimentation_18F14k22.c,401 :: 		state = POWER_ON;
	MOVLW       1
	MOVWF       _state+0 
;Alimentation_18F14k22.c,402 :: 		break;
	GOTO        L_main40
;Alimentation_18F14k22.c,403 :: 		}
L_main39:
	MOVF        _state+0, 0 
	XORLW       0
	BTFSC       STATUS+0, 2 
	GOTO        L_main41
	MOVF        _state+0, 0 
	XORLW       1
	BTFSC       STATUS+0, 2 
	GOTO        L_main47
	MOVF        _state+0, 0 
	XORLW       2
	BTFSC       STATUS+0, 2 
	GOTO        L_main55
	MOVF        _state+0, 0 
	XORLW       3
	BTFSC       STATUS+0, 2 
	GOTO        L_main60
	GOTO        L_main64
L_main40:
;Alimentation_18F14k22.c,405 :: 		for(cpt_wait=0; cpt_wait<WAIT_LOOP; cpt_wait++){
	CLRF        _cpt_wait+0 
	CLRF        _cpt_wait+1 
L_main65:
	MOVLW       39
	SUBWF       _cpt_wait+1, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__main139
	MOVLW       16
	SUBWF       _cpt_wait+0, 0 
L__main139:
	BTFSC       STATUS+0, 0 
	GOTO        L_main66
;Alimentation_18F14k22.c,406 :: 		delay_us(50);
	MOVLW       66
	MOVWF       R13, 0
L_main68:
	DECFSZ      R13, 1, 1
	BRA         L_main68
	NOP
;Alimentation_18F14k22.c,408 :: 		if(nb_rx_octet>1 && SSPSTAT.P == 1){
	MOVF        _nb_rx_octet+0, 0 
	SUBLW       1
	BTFSC       STATUS+0, 0 
	GOTO        L_main71
	BTFSS       SSPSTAT+0, 4 
	GOTO        L_main71
L__main106:
;Alimentation_18F14k22.c,409 :: 		i2c_read_data_from_buffer();
	CALL        _i2c_read_data_from_buffer+0, 0
;Alimentation_18F14k22.c,410 :: 		nb_rx_octet = 0;
	CLRF        _nb_rx_octet+0 
;Alimentation_18F14k22.c,411 :: 		}
L_main71:
;Alimentation_18F14k22.c,405 :: 		for(cpt_wait=0; cpt_wait<WAIT_LOOP; cpt_wait++){
	INFSNZ      _cpt_wait+0, 1 
	INCF        _cpt_wait+1, 1 
;Alimentation_18F14k22.c,412 :: 		}
	GOTO        L_main65
L_main66:
;Alimentation_18F14k22.c,413 :: 		}
	GOTO        L_main37
;Alimentation_18F14k22.c,414 :: 		}
L_end_main:
	GOTO        $+0
; end of _main

_interrupt:

;Alimentation_18F14k22.c,422 :: 		void interrupt(){
;Alimentation_18F14k22.c,427 :: 		if (TMR0IF_bit){
	BTFSS       TMR0IF_bit+0, BitPos(TMR0IF_bit+0) 
	GOTO        L_interrupt72
;Alimentation_18F14k22.c,430 :: 		if(state == SLEEP){
	MOVF        _state+0, 0 
	XORLW       3
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt73
;Alimentation_18F14k22.c,431 :: 		if(time_to_start[2]>0){
	MOVF        _time_to_start+2, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt74
;Alimentation_18F14k22.c,432 :: 		time_to_start[2]--;
	DECF        _time_to_start+2, 1 
;Alimentation_18F14k22.c,433 :: 		}
	GOTO        L_interrupt75
L_interrupt74:
;Alimentation_18F14k22.c,435 :: 		if(time_to_start[1]>0){
	MOVF        _time_to_start+1, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt76
;Alimentation_18F14k22.c,436 :: 		time_to_start[1]--;
	DECF        _time_to_start+1, 1 
;Alimentation_18F14k22.c,437 :: 		time_to_start[2]=59;
	MOVLW       59
	MOVWF       _time_to_start+2 
;Alimentation_18F14k22.c,438 :: 		}
	GOTO        L_interrupt77
L_interrupt76:
;Alimentation_18F14k22.c,440 :: 		if(time_to_start[0]>0){
	MOVF        _time_to_start+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt78
;Alimentation_18F14k22.c,441 :: 		time_to_start[0]--;
	DECF        _time_to_start+0, 1 
;Alimentation_18F14k22.c,442 :: 		time_to_start[1]=59;
	MOVLW       59
	MOVWF       _time_to_start+1 
;Alimentation_18F14k22.c,443 :: 		}
L_interrupt78:
;Alimentation_18F14k22.c,444 :: 		}
L_interrupt77:
;Alimentation_18F14k22.c,445 :: 		}
L_interrupt75:
;Alimentation_18F14k22.c,446 :: 		}
L_interrupt73:
;Alimentation_18F14k22.c,448 :: 		if(state == WAIT_TO_SLEEP){
	MOVF        _state+0, 0 
	XORLW       2
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt79
;Alimentation_18F14k22.c,449 :: 		if(time_to_stop>0)
	MOVF        _time_to_stop+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt80
;Alimentation_18F14k22.c,450 :: 		time_to_stop--;
	DECF        _time_to_stop+0, 1 
L_interrupt80:
;Alimentation_18F14k22.c,451 :: 		}
L_interrupt79:
;Alimentation_18F14k22.c,454 :: 		if(state == POWER_ON){
	MOVF        _state+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt81
;Alimentation_18F14k22.c,455 :: 		if(watchdog_restart>0)
	MOVLW       0
	MOVWF       R0 
	MOVF        _watchdog_restart+1, 0 
	SUBWF       R0, 0 
	BTFSS       STATUS+0, 2 
	GOTO        L__interrupt142
	MOVF        _watchdog_restart+0, 0 
	SUBLW       0
L__interrupt142:
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt82
;Alimentation_18F14k22.c,456 :: 		watchdog_restart--;
	MOVLW       1
	SUBWF       _watchdog_restart+0, 1 
	MOVLW       0
	SUBWFB      _watchdog_restart+1, 1 
	GOTO        L_interrupt83
L_interrupt82:
;Alimentation_18F14k22.c,459 :: 		default_time_to_start[0] = 0;
	CLRF        _default_time_to_start+0 
;Alimentation_18F14k22.c,460 :: 		default_time_to_start[1] = 0;
	CLRF        _default_time_to_start+1 
;Alimentation_18F14k22.c,461 :: 		default_time_to_start[2] = 2; // 2s
	MOVLW       2
	MOVWF       _default_time_to_start+2 
;Alimentation_18F14k22.c,462 :: 		time_to_stop = 10;
	MOVLW       10
	MOVWF       _time_to_stop+0 
;Alimentation_18F14k22.c,464 :: 		state = WAIT_TO_SLEEP;
	MOVLW       2
	MOVWF       _state+0 
;Alimentation_18F14k22.c,465 :: 		watchdog_restart = watchdog_restart_default;
	MOVF        _watchdog_restart_default+0, 0 
	MOVWF       _watchdog_restart+0 
	MOVF        _watchdog_restart_default+1, 0 
	MOVWF       _watchdog_restart+1 
;Alimentation_18F14k22.c,466 :: 		}
L_interrupt83:
;Alimentation_18F14k22.c,467 :: 		}
L_interrupt81:
;Alimentation_18F14k22.c,470 :: 		TMR0H = 0x0B;
	MOVLW       11
	MOVWF       TMR0H+0 
;Alimentation_18F14k22.c,471 :: 		TMR0L = 0xDC;
	MOVLW       220
	MOVWF       TMR0L+0 
;Alimentation_18F14k22.c,472 :: 		TMR0IF_bit = 0;
	BCF         TMR0IF_bit+0, BitPos(TMR0IF_bit+0) 
;Alimentation_18F14k22.c,473 :: 		}
	GOTO        L_interrupt84
L_interrupt72:
;Alimentation_18F14k22.c,476 :: 		else if (TMR3IF_bit){
	BTFSS       TMR3IF_bit+0, BitPos(TMR3IF_bit+0) 
	GOTO        L_interrupt85
;Alimentation_18F14k22.c,478 :: 		if(start_led_puissance == 1){
	MOVF        _start_led_puissance+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt86
;Alimentation_18F14k22.c,479 :: 		if (cpt_led_puissance > 0){
	MOVF        _cpt_led_puissance+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt87
;Alimentation_18F14k22.c,480 :: 		LED_PUISSANCE = 0;
	BCF         LATA+0, 4 
;Alimentation_18F14k22.c,481 :: 		cpt_led_puissance--;
	DECF        _cpt_led_puissance+0, 1 
;Alimentation_18F14k22.c,482 :: 		}
	GOTO        L_interrupt88
L_interrupt87:
;Alimentation_18F14k22.c,484 :: 		LED_PUISSANCE = 1;
	BSF         LATA+0, 4 
;Alimentation_18F14k22.c,485 :: 		cpt_led_puissance=led_puissance_delay;
	MOVF        _led_puissance_delay+0, 0 
	MOVWF       _cpt_led_puissance+0 
;Alimentation_18F14k22.c,486 :: 		}
L_interrupt88:
;Alimentation_18F14k22.c,487 :: 		}
	GOTO        L_interrupt89
L_interrupt86:
;Alimentation_18F14k22.c,489 :: 		LED_PUISSANCE = 0;
	BCF         LATA+0, 4 
;Alimentation_18F14k22.c,490 :: 		}
L_interrupt89:
;Alimentation_18F14k22.c,493 :: 		if(set_led_on == 1) // For ILS
	MOVF        _set_led_on+0, 0 
	XORLW       1
	BTFSS       STATUS+0, 2 
	GOTO        L_interrupt90
;Alimentation_18F14k22.c,494 :: 		LED = 1;
	BSF         LATA+0, 0 
	GOTO        L_interrupt91
L_interrupt90:
;Alimentation_18F14k22.c,496 :: 		if (cpt_led > 0){
	MOVF        _cpt_led+0, 0 
	SUBLW       0
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt92
;Alimentation_18F14k22.c,497 :: 		LED = 0;
	BCF         LATA+0, 0 
;Alimentation_18F14k22.c,498 :: 		cpt_led--;
	DECF        _cpt_led+0, 1 
;Alimentation_18F14k22.c,499 :: 		}
	GOTO        L_interrupt93
L_interrupt92:
;Alimentation_18F14k22.c,501 :: 		LED = 1;
	BSF         LATA+0, 0 
;Alimentation_18F14k22.c,502 :: 		cpt_led=led_delay;
	MOVF        _led_delay+0, 0 
	MOVWF       _cpt_led+0 
;Alimentation_18F14k22.c,503 :: 		}
L_interrupt93:
;Alimentation_18F14k22.c,504 :: 		}
L_interrupt91:
;Alimentation_18F14k22.c,506 :: 		TMR3H = 0x3C;
	MOVLW       60
	MOVWF       TMR3H+0 
;Alimentation_18F14k22.c,507 :: 		TMR3L = 0xB0;
	MOVLW       176
	MOVWF       TMR3L+0 
;Alimentation_18F14k22.c,508 :: 		TMR3IF_bit = 0;
	BCF         TMR3IF_bit+0, BitPos(TMR3IF_bit+0) 
;Alimentation_18F14k22.c,509 :: 		}
L_interrupt85:
L_interrupt84:
;Alimentation_18F14k22.c,510 :: 		}
L_end_interrupt:
L__interrupt141:
	RETFIE      1
; end of _interrupt

_init_i2c:

;Alimentation_18F14k22.c,515 :: 		void init_i2c(){
;Alimentation_18F14k22.c,518 :: 		TRISB4_bit = 1; // RB4 en entrée
	BSF         TRISB4_bit+0, BitPos(TRISB4_bit+0) 
;Alimentation_18F14k22.c,519 :: 		TRISB6_bit = 1; // RB6 en entrée
	BSF         TRISB6_bit+0, BitPos(TRISB6_bit+0) 
;Alimentation_18F14k22.c,522 :: 		PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
	BSF         PIE1+0, 3 
;Alimentation_18F14k22.c,523 :: 		PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
	BCF         PIR1+0, 3 
;Alimentation_18F14k22.c,525 :: 		PIR2.BCLIE = 1;
	BSF         PIR2+0, 3 
;Alimentation_18F14k22.c,526 :: 		PIR2.BCLIF = 0;
	BCF         PIR2+0, 3 
;Alimentation_18F14k22.c,529 :: 		SSPADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
	MOVLW       114
	MOVWF       SSPADD+0 
;Alimentation_18F14k22.c,530 :: 		SSPMSK = 0xFF; // A zero (‘0’) bit in the SSPMSK register has the effect of making
	MOVLW       255
	MOVWF       SSPMSK+0 
;Alimentation_18F14k22.c,534 :: 		SSPSTAT.SMP = 1; // Slew Rate Control bit
	BSF         SSPSTAT+0, 7 
;Alimentation_18F14k22.c,537 :: 		SSPSTAT.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)
	BSF         SSPSTAT+0, 6 
;Alimentation_18F14k22.c,540 :: 		SSPCON2 = 0x00;
	CLRF        SSPCON2+0 
;Alimentation_18F14k22.c,541 :: 		SSPCON2.GCEN = 0; // General Call Enable bit (0 = disabled)
	BCF         SSPCON2+0, 7 
;Alimentation_18F14k22.c,542 :: 		SSPCON2.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)
	BSF         SSPCON2+0, 0 
;Alimentation_18F14k22.c,545 :: 		SSPCON1.WCOL = 0; // Write Collision Detect bit
	BCF         SSPCON1+0, 7 
;Alimentation_18F14k22.c,546 :: 		SSPCON1.SSPOV = 0; // Receive Overflow Indicator bit
	BCF         SSPCON1+0, 6 
;Alimentation_18F14k22.c,547 :: 		SSPCON1.CKP = 1; // SCK Release Control bit (1=Release clock)
	BSF         SSPCON1+0, 4 
;Alimentation_18F14k22.c,548 :: 		SSPCON1.SSPM3 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled (1-> with S/P, 0 -> without)
	BCF         SSPCON1+0, 3 
;Alimentation_18F14k22.c,549 :: 		SSPCON1.SSPM2 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 2 
;Alimentation_18F14k22.c,550 :: 		SSPCON1.SSPM1 = 0b1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BSF         SSPCON1+0, 1 
;Alimentation_18F14k22.c,551 :: 		SSPCON1.SSPM0 = 0b0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
	BCF         SSPCON1+0, 0 
;Alimentation_18F14k22.c,554 :: 		SSPCON1.SSPEN = 1; // Synchronous Serial Port Enable bit
	BSF         SSPCON1+0, 5 
;Alimentation_18F14k22.c,555 :: 		}
L_end_init_i2c:
	RETURN      0
; end of _init_i2c

_interrupt_low:
	MOVWF       ___Low_saveWREG+0 
	MOVF        STATUS+0, 0 
	MOVWF       ___Low_saveSTATUS+0 
	MOVF        BSR+0, 0 
	MOVWF       ___Low_saveBSR+0 

;Alimentation_18F14k22.c,560 :: 		void interrupt_low(){
;Alimentation_18F14k22.c,561 :: 		if (PIR1.SSPIF){  // I2C Interrupt
	BTFSS       PIR1+0, 3 
	GOTO        L_interrupt_low94
;Alimentation_18F14k22.c,563 :: 		if(SSPCON1.SSPOV || SSPCON1.WCOL){
	BTFSC       SSPCON1+0, 6 
	GOTO        L__interrupt_low110
	BTFSC       SSPCON1+0, 7 
	GOTO        L__interrupt_low110
	GOTO        L_interrupt_low97
L__interrupt_low110:
;Alimentation_18F14k22.c,564 :: 		SSPCON1.SSPOV = 0;
	BCF         SSPCON1+0, 6 
;Alimentation_18F14k22.c,565 :: 		SSPCON1.WCOL = 0;
	BCF         SSPCON1+0, 7 
;Alimentation_18F14k22.c,566 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Alimentation_18F14k22.c,567 :: 		}
L_interrupt_low97:
;Alimentation_18F14k22.c,571 :: 		if (SSPSTAT.R_W == 0){
	BTFSC       SSPSTAT+0, 2 
	GOTO        L_interrupt_low98
;Alimentation_18F14k22.c,572 :: 		if(SSPSTAT.P == 0){
	BTFSC       SSPSTAT+0, 4 
	GOTO        L_interrupt_low99
;Alimentation_18F14k22.c,573 :: 		if (SSPSTAT.D_A == 0){ // Address
	BTFSC       SSPSTAT+0, 5 
	GOTO        L_interrupt_low100
;Alimentation_18F14k22.c,574 :: 		nb_rx_octet = 0;
	CLRF        _nb_rx_octet+0 
;Alimentation_18F14k22.c,575 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Alimentation_18F14k22.c,576 :: 		}
	GOTO        L_interrupt_low101
L_interrupt_low100:
;Alimentation_18F14k22.c,578 :: 		if(nb_rx_octet < SIZE_RX_BUFFER){
	MOVLW       8
	SUBWF       _nb_rx_octet+0, 0 
	BTFSC       STATUS+0, 0 
	GOTO        L_interrupt_low102
;Alimentation_18F14k22.c,579 :: 		rxbuffer_tab[nb_rx_octet] = SSPBUF;
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
;Alimentation_18F14k22.c,580 :: 		nb_rx_octet++;
	INCF        _nb_rx_octet+0, 1 
;Alimentation_18F14k22.c,581 :: 		}
	GOTO        L_interrupt_low103
L_interrupt_low102:
;Alimentation_18F14k22.c,583 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Alimentation_18F14k22.c,584 :: 		}
L_interrupt_low103:
;Alimentation_18F14k22.c,585 :: 		}
L_interrupt_low101:
;Alimentation_18F14k22.c,586 :: 		}
L_interrupt_low99:
;Alimentation_18F14k22.c,595 :: 		}
	GOTO        L_interrupt_low104
L_interrupt_low98:
;Alimentation_18F14k22.c,599 :: 		if(SSPSTAT.D_A == 0){
	BTFSC       SSPSTAT+0, 5 
	GOTO        L_interrupt_low105
;Alimentation_18F14k22.c,600 :: 		nb_tx_octet = 0;
	CLRF        _nb_tx_octet+0 
;Alimentation_18F14k22.c,601 :: 		tmp_rx = SSPBUF;
	MOVF        SSPBUF+0, 0 
	MOVWF       _tmp_rx+0 
;Alimentation_18F14k22.c,602 :: 		}
L_interrupt_low105:
;Alimentation_18F14k22.c,605 :: 		i2c_write_data_to_buffer(nb_tx_octet);
	MOVF        _nb_tx_octet+0, 0 
	MOVWF       FARG_i2c_write_data_to_buffer_nb_tx_octet+0 
	CALL        _i2c_write_data_to_buffer+0, 0
;Alimentation_18F14k22.c,606 :: 		nb_tx_octet++;
	INCF        _nb_tx_octet+0, 1 
;Alimentation_18F14k22.c,607 :: 		}
L_interrupt_low104:
;Alimentation_18F14k22.c,609 :: 		SSPCON1.CKP = 1;
	BSF         SSPCON1+0, 4 
;Alimentation_18F14k22.c,610 :: 		PIR1.SSPIF = 0; // reset SSP interrupt flag
	BCF         PIR1+0, 3 
;Alimentation_18F14k22.c,611 :: 		}
L_interrupt_low94:
;Alimentation_18F14k22.c,612 :: 		}
L_end_interrupt_low:
L__interrupt_low145:
	MOVF        ___Low_saveBSR+0, 0 
	MOVWF       BSR+0 
	MOVF        ___Low_saveSTATUS+0, 0 
	MOVWF       STATUS+0 
	SWAPF       ___Low_saveWREG+0, 1 
	SWAPF       ___Low_saveWREG+0, 0 
	RETFIE      0
; end of _interrupt_low

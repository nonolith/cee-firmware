// Driver for MCP4922 DAC on USARTC1
// (C) 2012 Nonolith Labs
// Authors:
//   Kevin Mehall
//   Ian Daniher
// Licensed under the terms of the GNU GPLv3+


/// Configures the XMEGA's USARTC1 to talk to the digital-analog converter.
void DAC_init(void){
	PORTD.DIRSET = DAC_SHDN;
	PORTD.OUTSET = DAC_SHDN;
	PORTC.DIRSET = LDAC | CS | SCK | TXD1;
	USARTC1.CTRLC = USART_CMODE_MSPI_gc; // SPI master, MSB first, sample on rising clock (UCPHA=0)
	USARTC1.BAUDCTRLA =  0;  // 16MHz SPI clock. XMEGA AU manual 23.15.6 & 23.3.1
	USARTC1.BAUDCTRLB =  0;
	USARTC1.CTRLB = USART_TXEN_bm; // enable TX
	PORTC.OUTSET = CS;
	PORTC.OUTCLR = LDAC | CS; // LDAC, SCK low
}

/// Blocking write a value to a specified channel of the DAC with specified flags.
void DAC_write(uint8_t flags, uint16_t value){
	PORTC.OUTCLR = CS;
	USARTC1.DATA = ((flags<<4) & 0xF0) | ((value >> 8) & 0x0F); // munge channel, flags, and four MSB of the value into a single byte
	while(!(USARTC1.STATUS & USART_DREIF_bm)); // wait until we can write another byte
	USARTC1.STATUS = USART_TXCIF_bm; // clear TX complete flag
	USARTC1.DATA = value & 0xFF;
	while(!(USARTC1.STATUS & USART_TXCIF_bm)); // wait for TX complete flag
	PORTC.OUTSET = CS;
}


// Configuration nibbles for the DAC
uint8_t DAC_data_config[2];

/// Buffer for the pending data to be written to the DAC
uint8_t DAC_data[2];

/// Generate the DAC flags nibble based on the mode
#define MODE_TO_DACFLAGS(m)  ((((m)!=DISABLED)?DACFLAG_ENABLE:0) \
                            |(((m)==SIMV)?DACFLAG_NO_MULT_REF:0))

/// Update the config nibbles to DAC_data based on the new flags
inline void DAC_config(uint8_t mode_a, uint8_t mode_b){
	DAC_data_config[0] = MODE_TO_DACFLAGS(mode_a) << 4;
	DAC_data_config[1] = (DACFLAG_CHANNEL | MODE_TO_DACFLAGS(mode_b)) << 4;
}

/// Begin a non-blocking DAC write of the data from an OUT_Sample
inline void DAC_startWrite(OUT_sample* s){
	// Put out_sample into DAC_data
	uint8_t data0 = DAC_data_config[0] | (s->bh_ah & 0x0F);
	uint8_t data1 = s->al;
	DAC_data[0] = DAC_data_config[1] | (s->bh_ah >> 4);
	DAC_data[1] = s->bl;
	
	dac_write_state = 0;
	CS_LO();
	USARTC1.DATA =  data0;
	USARTC1.DATA =  data1;
	USARTC1.STATUS = USART_TXCIF_bm; // clear TXC
	USARTC1.CTRLA = USART_TXCINTLVL_HI_gc; // enable TXC
}

ISR(USARTC1_TXC_vect){
	CS_HI();
	if (!dac_write_state){
		CS_LO();
		USARTC1.DATA =  DAC_data[0]; // write byte 2
		USARTC1.DATA =  DAC_data[1]; // write byte 3
		dac_write_state = 1;
	}
}

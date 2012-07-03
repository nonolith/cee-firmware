// Firmware for CEE
// http://nonolithlabs.com
// (C) 2012 Nonolith Labs
// Authors:
//	Kevin Mehall
//	Ian Daniher
// Licensed under the terms of the GNU GPLv3+

#include <Common.h>
#include <usb.h>
#include <avr/eeprom.h>
#include <usb_pipe.h>

#include "cee.h"
#include "hardware.h"
#include "dac.c"

#define IN_ADDR  (0x81|USB_EP_PP)
#define OUT_ADDR (0x02|USB_EP_PP)

PIPE(in_pipe, 64, 40);
PIPE(out_pipe, 32, 40);

unsigned char in_seqno = 0;

int main(void){
	configHardware();
	
	//DEBUG: event out
	
	PORTD.DIRSET = (1<<4);
	PORTCFG.CLKEVOUT = PORTCFG_EVOUT_PD7_gc | PORTCFG_CLKEVPIN_PIN4_gc;
	PORTCFG.EVOUTSEL = PORTCFG_EVOUTSEL_0_gc;
	EVSYS.CH0MUX = EVSYS_CHMUX_ADCA_CH3_gc;
	
	
	PORTE.DIRSET = (1<<0) | (1<<1);
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();	
	
	for (;;){
		USB_Task(); // lower-priority USB polling, like control requests
		pollSamplingEndpoints();
	}
}

void EVENT_USB_Device_ConfigurationChanged(uint8_t configuration){
	USB_ep_init(IN_ADDR, USB_EP_TYPE_BULK_gc, 64);
	USB_ep_init(OUT_ADDR, USB_EP_TYPE_BULK_gc, 32);
}

/* Read the voltage and current from the two channels, pulling the latest samples off "ADCA.CHx.RES" registers. */
inline void readADC(IN_sample* s){
	uint8_t A_Il = ADCA.CH0RESL, A_Ih = ADCA.CH0RESH; // low and high bytes for channel A stream I
	uint8_t A_Vl = ADCA.CH1RESL, A_Vh = ADCA.CH1RESH;

	uint8_t B_Vl = ADCA.CH2RESL, B_Vh = ADCA.CH2RESH;
	uint8_t B_Il = ADCA.CH3RESL, B_Ih = ADCA.CH3RESH;
	
	GCC_FORCE_POINTER_ACCESS(s);
	
	s->avl = A_Vl;
	s->ail = A_Il;
	s->aih_avh = (A_Ih << 4) | (A_Vh&0x0f);

	s->bvl = B_Vl;
	s->bil = B_Il;
	s->bih_bvh = (B_Ih << 4) | (B_Vh&0x0f);
}

bool sampling = 0;
uint8_t sampleIndex = 0; // sample index within packet to be written next
bool havePacket = 0;
uint8_t sampleFlags = 0;
chan_mode modeA = DISABLED;
chan_mode modeB = DISABLED;
IN_packet *inPacket;
OUT_packet *outPacket;

void configureSampling(uint16_t mode, uint16_t period){
	TCC0.INTCTRLA = TC_OVFINTLVL_OFF_gc;
	TCC0.CTRLA = 0;
	sampling = 0;
	sampleIndex = 0;
	havePacket = 0;
	sampleFlags = 0;
	modeA = DISABLED;
	modeB = DISABLED;
	
	if (mode == 1 /*&& period > 80*/){
		usb_pipe_reset(IN_ADDR, &in_pipe);
		usb_pipe_reset(OUT_ADDR, &out_pipe);
		TCC0.CTRLA = TC_CLKSEL_DIV8_gc; // 4Mhz
		TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc; // interrupt on timer overflow
		TCC0.PER = period;
		TCC0.CNT = 0;
		sampling = 1;
	}else{
		configChannelA(DISABLED);
		configChannelB(DISABLED);
		PORTR.OUTCLR = 1 << 1; // LED off
	}
}

static inline void pollSamplingEndpoints(){
	if (sampling){
		usb_pipe_handle(IN_ADDR, &in_pipe);
		usb_pipe_handle(OUT_ADDR, &out_pipe);
	}
}

void switchMode(void){
	TCC0.CTRLA = 0; // Stop the timer
		
	// TODO: de-glitch by disabling OPA567 when switching mode

	modeA = outPacket->mode_a;
	modeB = outPacket->mode_b;

	DAC_config(modeA, modeB);
	
	// TODO: write first sample before enabling outputs

	configChannelA(modeA);
	configChannelB(modeB);
	
	TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCC0.CNT=1;
}

ISR(TCC0_OVF_vect){
	PORTE.OUTSET = 1;
	
	if (!havePacket){
		if (pipe_can_write(&in_pipe)>0 && pipe_can_read(&out_pipe)>0){
			PORTR.OUTSET = 1 << 1; // LED on
			havePacket = 1;
			inPacket = (IN_packet *) pipe_write_ptr(&in_pipe);
			outPacket = (OUT_packet *) pipe_read_ptr(&out_pipe);
			sampleIndex = 0;

			if (outPacket->mode_a != modeA || outPacket->mode_b != modeB)
				return switchMode();
		}else{
			// TODO: stop timer
			PORTR.OUTCLR = 1 << 1; // LED off
			sampleFlags |= FLAG_PACKET_DROPPED;
			return;
		}
	}
	
	DAC_startWrite(&(outPacket->data[sampleIndex]));	 // start SPI write
	readADC(&(inPacket->data[sampleIndex]));
	
	uint8_t i = sampleIndex++;
	
	if (i == 5){
		// fill header when there's nothing else going on
		inPacket->mode_a = outPacket->mode_a;
		inPacket->mode_b = outPacket->mode_b;
		inPacket->flags = sampleFlags;
		sampleFlags = 0;
	} else if (i >= 9){
		sampleIndex = 0;
		havePacket = 0;
		pipe_done_read(&out_pipe);
		pipe_done_write(&in_pipe);
	}
	PORTE.OUTCLR = 1;
}

/* Use the XMEGA's internal DAC to configure the hard current limit. */
void configISET(void){
	DACB.CTRLA |= DAC_CH0EN_bm | DAC_CH1EN_bm | DAC_ENABLE_bm;
	DACB.CTRLB |= DAC_CHSEL_DUAL_gc; 
	DACB.CTRLC |= DAC_REFSEL_AREFA_gc; // 2.5VREF
	DACB.CH1DATA = 0x6B7; // sane default for OPA567-level current limiting 
	DACB.CH0DATA = 0x6B7; // 0x6B7/0xFFF*2.5V = 1.05V, 9800*(1.18V-1.05)/560O = 0.227
}

/* Configure the pin modes for the switches and opamps. */
void initChannels(void){
	PORTD.DIRSET = SHDN_INS_A | SWMODE_A | SHDN_INS_B | EN_OPA_A;
	PORTC.DIRSET = SWMODE_B | EN_OPA_B;
	PORTD.DIRCLR = TFLAG_A;
	PORTC.DIRCLR = TFLAG_B;
	PORTB.DIRSET = ISET_A | ISET_B;
	PORTD.OUTCLR = EN_OPA_A;
	PORTC.OUTCLR = EN_OPA_B;
	configISET();
}

/* Configure the shutdown/enable pin states and set the SPDT switch states. */
void configChannelA(chan_mode state){
	switch (state) {
		case SVMI: // source voltage, measure current
			PORTD.OUTSET = SWMODE_A | EN_OPA_A;
			PORTD.OUTCLR = SHDN_INS_A;
			break;
		case SIMV: // source current, measure voltage
			PORTD.OUTSET = EN_OPA_A;
			PORTD.OUTCLR = SWMODE_A | SHDN_INS_A;
			break;
		case DISABLED: // high impedance 
			PORTD.OUTSET = SHDN_INS_A;
			PORTD.OUTCLR = SWMODE_A | EN_OPA_A;
			break;
		case CALIBRATE: // MAX9919F on, OPA567 off - used to characterize the '9919
			PORTD.OUTCLR = SHDN_INS_A | EN_OPA_A;
			PORTD.OUTSET = SWMODE_A;
			break;
	}
}

void configChannelB(chan_mode state){
	switch (state) {
		case SVMI:
			PORTC.OUTSET = SWMODE_B | EN_OPA_B;
			PORTD.OUTCLR = SHDN_INS_B;
			break;
		case SIMV:
			PORTC.OUTSET = EN_OPA_B;
			PORTC.OUTCLR = SWMODE_B;
			PORTD.OUTCLR = SHDN_INS_B;
			break;
		case DISABLED:
			PORTC.OUTCLR = SWMODE_B | EN_OPA_B;
			PORTD.OUTSET = SHDN_INS_B;
			break;
		case CALIBRATE:
			PORTD.OUTCLR = SHDN_INS_B;
			PORTC.OUTCLR = EN_OPA_B;
			PORTC.OUTSET = SWMODE_A;
		}
}



/* Take a channel, state, and value and configure the switches, shutdowns, and DACs. High level abstraction. */
void writeChannel(uint8_t channel, uint8_t state, uint16_t value){
	uint8_t dacflags = 0;
	if (channel) dacflags |= DACFLAG_CHANNEL;
	if (state != DISABLED) dacflags |= DACFLAG_ENABLE;
	if (state == SIMV) dacflags |= DACFLAG_NO_MULT_REF;

	if (channel == 0) configChannelA(state);
	else              configChannelB(state);

	DAC_write(dacflags, value);
}

/* Configures the board hardware and chip peripherals for the project's functionality. */
void configHardware(void){
	USB_ConfigureClock();
	PORTR.DIRSET = 1 << 1;
	PORTR.OUTSET = 1 << 1;
	_delay_ms(50);
	PORTR.OUTCLR = 1 << 1;
	DAC_init();
	initADC();
	initChannels();
	USB_Init();
	
	// Configure the timer to toggle LDAC
	TCC0.CTRLB = TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCC0.CCD = 1;
}

/* Configure the ADC to 12b, differential w/ gain, signed mode with a 2.5VREF. */
void initADC(void){
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | 1 << ADC_CONMODE_bp | 0 << ADC_IMPMODE_bp | ADC_CURRLIMIT_NO_gc;
	ADCA.REFCTRL = ADC_REFSEL_AREFA_gc; // use 2.5VREF at AREFA
	ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc; // ADC CLK = 2MHz
	ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVACT_SYNCHSWEEP_gc | ADC_EVSEL_7_gc;
	EVSYS.CH7MUX = EVSYS_CHMUX_TCC0_CCB_gc;
	TCC0.CCB = 20;
	 
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | ADC_CH_GAIN_2X_gc; // channel A stream I
	ADCA.CH1.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | ADC_CH_GAIN_1X_gc; // channel A stream V
	ADCA.CH2.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | ADC_CH_GAIN_1X_gc; // channel B stream V
	ADCA.CH3.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | ADC_CH_GAIN_2X_gc; // channel B stream I
	ADCA.CH0.MUXCTRL = ADC_CH_MUXNEG_PIN5_gc | ADC_CH_MUXPOS_PIN1_gc; // 1.25VREF vs VS-A
	ADCA.CH1.MUXCTRL = ADC_CH_MUXNEG_PIN4_gc |  ADC_CH_MUXPOS_PIN2_gc; // INTGND vs ADC-A
	ADCA.CH2.MUXCTRL = ADC_CH_MUXNEG_PIN4_gc | ADC_CH_MUXPOS_PIN6_gc; // INTGND vs ADC-B
	ADCA.CH3.MUXCTRL = ADC_CH_MUXNEG_PIN5_gc | ADC_CH_MUXPOS_PIN7_gc; // 1.25VREF vs VS-B
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc; // apply the factory-programmed calibration values
	ADCA.CALL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	ADCA.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
	ADCA.CTRLA = ADC_ENABLE_bm;
}

#define xstringify(s) stringify(s)
#define stringify(s) #s

const char PROGMEM hwversion[] = xstringify(HW_VERSION);
const char PROGMEM fwversion[] = xstringify(FW_VERSION);

uint8_t usb_cmd = 0;
uint8_t cmd_data = 0;

/** Event handler for the library USB Control Request reception event. */
bool EVENT_USB_Device_ControlRequest(USB_Request_Header_t* req){
	usb_cmd = 0;
	if ((req->bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_VENDOR){
		switch(req->bRequest){
			case 0x00: // Info
				if (req->wIndex == 0){
					USB_ep0_send_progmem((uint8_t*)hwversion, sizeof(hwversion));
				}else if (req->wIndex == 1){
					USB_ep0_send_progmem((uint8_t*)fwversion, sizeof(fwversion));
				}
				
				return true;
				
			case 0xA0: // read ADC
				readADC((IN_sample *) ep0_buf_in);
				USB_ep0_send(sizeof(IN_sample));
				return true;
				
			case 0xAA: // write to channel A
				writeChannel(0, req->wIndex, req->wValue);
				USB_ep0_send(0);
				return true;
				
			case 0xAB: // write to channel B
				writeChannel(1, req->wIndex, req->wValue);
				USB_ep0_send(0);
				return true;
				
			case 0x65: // set gains
				switch (req->wIndex){
					case 0x00:
				    	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | req->wValue; // VS-A
						break;
					case 0x01:
				    	ADCA.CH1.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | req->wValue; // ADC-A
						break;
					case 0x02:
				    	ADCA.CH2.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | req->wValue; // ADC-B
						break;
					case 0x03:
				    	ADCA.CH3.CTRL = ADC_CH_INPUTMODE_DIFFWGAIN_gc | req->wValue; // VS-B
						break;
				}
				USB_ep0_send(0);
				return true;
				
			case 0x15: // ISet
				DACB.CH0DATA = req->wValue;
				DACB.CH1DATA = req->wIndex;
				USB_ep0_send(0);
				return true;
				
			case 0x80: // Configure sampling	
				configureSampling(req->wIndex /*mode*/ , req->wValue /*period*/);
				USB_ep0_send(0);
				return true;
				
			case 0xE0: // Read EEPROM
				eeprom_read_block(ep0_buf_in, (void*)(req->wIndex*64), 64);
				USB_ep0_send(64);
				return true;
				
			case 0xE1: // Write EEPROM
				usb_cmd = req->bRequest;
				cmd_data = req->wIndex;
				USB_ep0_send(0);
				return true; // Wait for OUT data (expecting an OUT transfer)
				
			case 0xBB: // disconnect from USB, jump to bootloader
				USB_enter_bootloader();
				return true;
		}
	}
	return false;
}

void EVENT_USB_Device_ControlOUT(uint8_t* buf, uint8_t count){
	switch (usb_cmd){
		case 0xE1: // Write EEPROM
			eeprom_update_block(buf, (void*)(cmd_data*64), count);
			break;
	}
}

// Hardware-specific definitions
// (C) 2012 Nonolith Labs
// Authors:
//	Kevin Mehall
//	Ian Daniher
// Licensed under the terms of the GNU GPLv3+


// MCP4922 flags
#define DACFLAG_CHANNEL (1<<3)
#define DACFLAG_BUF     (1<<2)
#define DACFLAG_NO_MULT_REF (1<<1)
#define DACFLAG_ENABLE  (1<<0)    

// DAC-specific pinmappings
#define DAC_SHDN  (1 << 2)
#define LDAC	(1 << 3)
#define CS	(1 << 4)
#define SCK (1 << 5)
#define TXD1 (1 << 7)

// PORTC is mapped to VPORT0 for speed
inline void CS_LO(void) ATTR_ALWAYS_INLINE;
inline void CS_LO(void){VPORT0.OUT &= ~CS;}
inline void CS_HI(void) ATTR_ALWAYS_INLINE;
inline void CS_HI(void){VPORT0.OUT |= CS;}


// generic pinmappings
#define SHDN_INS_A	(1 << 5)
#define SWMODE_A	(1 << 3)
#define SWMODE_B	(1 << 2)
#define SHDN_INS_B	(1 << 1)
#define EN_OPA_A	(1 << 0)
#define EN_OPA_B	(1 << 0)
#define TFLAG_A	(1 << 4)
#define	TFLAG_B	(1 << 1)
#define ISET_B	(1 << 3)
#define ISET_A	(1 << 2)

/* Configure the shutdown/enable pin states and set the SPDT switch states. */
inline void configChannelA(chan_mode state) ATTR_ALWAYS_INLINE;
inline void configChannelA(chan_mode state){
	switch (state) {
		case SVMI: // source voltage, measure current
			PORTD.OUTSET = SWMODE_A;
			PORTD.OUTCLR = SHDN_INS_A;
			break;
		case SIMV: // source current, measure voltage
			PORTD.OUTCLR = SWMODE_A | SHDN_INS_A;
			break;
		case DISABLED: // high impedance 
			PORTD.OUTSET = SHDN_INS_A;
			PORTD.OUTCLR = SWMODE_A;
			break;
		case CALIBRATE: // MAX9919F on, OPA567 off - used to characterize the '9919
			PORTD.OUTCLR = SHDN_INS_A;
			PORTD.OUTSET = SWMODE_A;
			break;
	}
}

inline void configChannelB(chan_mode state) ATTR_ALWAYS_INLINE;
inline void configChannelB(chan_mode state){
	switch (state) {
		case SVMI:
			PORTC.OUTSET = SWMODE_B;
			PORTD.OUTCLR = SHDN_INS_B;
			break;
		case SIMV:
			PORTC.OUTCLR = SWMODE_B;
			PORTD.OUTCLR = SHDN_INS_B;
			break;
		case DISABLED:
			PORTC.OUTCLR = SWMODE_B;
			PORTD.OUTSET = SHDN_INS_B;
			break;
		case CALIBRATE:
			PORTD.OUTCLR = SHDN_INS_B;
			PORTC.OUTSET = SWMODE_A;
		}
}

inline void enableOutA(chan_mode state){
	switch (state) {
		case SVMI:
		case SIMV:
			PORTD.OUTSET = EN_OPA_A;
			break;
		case DISABLED:
		case CALIBRATE:
			PORTD.OUTCLR = EN_OPA_A;
			break;
	}
}

inline void enableOutB(chan_mode state){
	switch (state) {
		case SVMI:
		case SIMV:
			PORTC.OUTSET = EN_OPA_B;
			break;
		case DISABLED:
		case CALIBRATE:
			PORTC.OUTCLR = EN_OPA_B;
			break;
	}
}
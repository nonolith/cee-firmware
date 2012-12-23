// Firmware for CEE
// http://nonolithlabs.com
// (C) 2012 Nonolith Labs
// Authors:
//	Kevin Mehall
//	Ian Daniher
// Licensed under the terms of the GNU GPLv3+
#pragma once
#define F_CPU 32000000UL

// includes
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Descriptors.h"
#include "usb/usb.h"

// Flags in GPIO register

typedef struct 
{ 
  uint8_t bit0:1; 
  uint8_t bit1:1; 
  uint8_t bit2:1; 
  uint8_t bit3:1; 
  uint8_t bit4:1; 
  uint8_t bit5:1; 
  uint8_t bit6:1; 
  uint8_t bit7:1; 
}io_reg; 

#define GPIOR_VAR(GPIOR, BIT)  ((volatile io_reg*)_SFR_MEM_ADDR(GPIOR))->bit##BIT

#define sampling GPIOR_VAR(GPIOR0, 0)
#define havePacket GPIOR_VAR(GPIOR0, 1)
#define dac_write_state GPIOR_VAR(GPIOR0, 2)
#define dac_done GPIOR_VAR(GPIOR0, 3)

// generic defines

typedef enum chan_mode{
	DISABLED = 0,
	SVMI = 1,
	SIMV = 2,
	CALIBRATE = 3,
} chan_mode;

#define A 0
#define B 1

#define FLAG_PACKET_DROPPED (1<<0)

void configChannelA(chan_mode state);
void configChannelB(chan_mode state);

// type definitions
typedef struct IN_sample{
	uint8_t avl, ail, aih_avh, bvl, bil, bih_bvh;
} IN_sample;

typedef struct IN_packet{
	uint8_t mode_a;
	uint8_t mode_b;
	uint8_t flags;
	uint8_t mode_seq;
	IN_sample data[10];	
} __attribute__((packed)) IN_packet;

typedef struct OUT_sample{
	uint8_t al, bl, bh_ah;
} __attribute__((packed)) OUT_sample;

typedef struct OUT_packet{
	uint8_t mode_a;
	uint8_t mode_b;
	OUT_sample data[10];
} __attribute__((packed)) OUT_packet;

// function prototypes
void init_hardware(void);
void initDAC(void);
void readADC(IN_sample* s);
inline static void pollSamplingEndpoints(void);

typedef struct CEE_version_descriptor{
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t flags;
	uint8_t per_ns;
	uint8_t min_per;
} __attribute__((packed)) CEE_version_descriptor;

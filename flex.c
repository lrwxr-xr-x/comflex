/*
 * Copyright 2016 Ross Snider
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <cc1110.h>
#include "ioCCxx10_bitdef.h"
#include "display.h"
#include "keys.h"
#include "stdio.h"
#include "flex.h"
#include "pm.h"

/* globals */
__bit sleepy;
static volatile u8 rxdone = 0;
__xdata DMA_DESC dmaConfig;

#define LEN 8
__xdata u8 rxbuf[LEN];

void setup_dma_rx()
{
	dmaConfig.PRIORITY       = 2;  // high priority
	dmaConfig.M8             = 0;  // not applicable
	dmaConfig.IRQMASK        = 0;  // disable interrupts
	dmaConfig.TRIG           = 19; // radio
	dmaConfig.TMODE          = 0;  // single byte mode
	dmaConfig.WORDSIZE       = 0;  // one byte words;
	dmaConfig.VLEN           = 0;  // use LEN
	SET_WORD(dmaConfig.LENH, dmaConfig.LENL, LEN);

	SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, &X_RFD);
	SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, rxbuf);
	dmaConfig.SRCINC         = 0;  // do not increment
	dmaConfig.DESTINC        = 1;  // increment by one

	SET_WORD(DMA0CFGH, DMA0CFGL, &dmaConfig);

	return;
}

void radio_setup() {
	/* IF of 457.031 kHz */
	FSCTRL1 = 0x12; // 18 * 26 MHz / 2^10
	FSCTRL0 = 0x00; // This gets added, twos complement, zero = ignore

	/* disable 3 highest DVGA settings */
	AGCCTRL2 |= AGCCTRL2_MAX_DVGA_GAIN;

	/* frequency synthesizer calibration */
	FSCAL3 = 0xEA;
	FSCAL2 = 0x2A;
	FSCAL1 = 0x00;
	FSCAL0 = 0x1F;

	/* "various test settings" */
	TEST2 = 0x88;
	TEST1 = 0x31;
	TEST0 = 0x09;

	/* radio control state machine */
	MCSM0 = 0x04; // Default, never automatically calibrate
	MCSM1 = 0x3C; // Stay in RX
	MCSM2 = 0x07; // Default, minimum RX timeout

	/* modem configuration */
//	MDMCFG0 = 0xF8; // Default, 200 kHz channel spacing
//	MDMCFG1 = 0x22; // Default, 4 preamble bytes for TX, FEC disabled
	MDMCFG2 = 0x02; // Default, 2-FSK, 16/16 bit sync word detection
//	MDMCFG3 = 0x22; // Default, data rate of 115.051 kbps
//	MDMCFG4 = 0x8C; // Default, 203 kHz channel filter bandwidth
//	MDMCFG4 = 0xFC; // ~58 kHz channel filter bandwidth

	// This is to get a data rate of 1.6 kbps
	MDMCFG3 = 0x02; // Data rate = ((256 + this)*2^that)/2^28 * XOSC
	MDMCFG4 = 0xF6; // "that"
	// ((256 + 2) * 2^6) / 2^28 * XOSC[26MHz] = 1.599 kHz
	// Otherwise this is the same default values for bandwidth, etc

	// This is to get a channel spacing of ~25 kHz, though I hear
	// this is only for POGSAC and new FCC mandates 12.5 which
	// the chip can't even be programmed for...
	MDMCFG0 = 0x00; // chspacing = (XOSC>>18)*(256+this)*2^that*n_cha
	MDMCFG1 = 0x00; // "that"
	// (XOSC[26MHz]>>18)*(256)*2^0*1
	// This also sets the preamble size to 2 (minimum)
	// It's undetermined how exactly the num preamble affects RX

	/* packet control configuraton */
//	PKTCTRL0 = 0x42; // Data whitening, CRC off, infinite packet sizes
	PKTCTRL0 = 0x02; // No data whitening, CRC off, infinite packet sizes
	PKTCTRL1 = 0x04; // Default, preamble quality off, no ADDR check

	/* sync word configuration */
	SYNC1 = ~(0xA6);
	SYNC0 = ~(0xC6);
	// Sync word for FLEX is 0xA6C6AAAA

	/* address configuration */
	ADDR = 0x00; // This is a test, assuming 0 = broadcast

	/* deviation settings */
	// The deviation for the FSK modulation
	DEVIATN = 0x14; // Aim at ~4800 Hz
	// ((XOSC[26MHz] / 2^17) * (8 + LO_BYTE) * 2^HI_BYTE)
	// ((XOSC[26MHz] / 2^17) * (8 + 4) * 2^1) == 4.76 kHz
}

/* set the channel bandwidth */
void set_filter() {
}

/* set the radio frequency in Hz */
void set_radio_freq(u32 freq) {
	/* the frequency setting is in units of 396.728515625 Hz */
	u32 setting = (u32) (freq * .0025206154);
	FREQ2 = (setting >> 16) & 0xff;
	FREQ1 = (setting >> 8) & 0xff;
	FREQ0 = setting & 0xff;

	/* select high VCO */
	FSCAL2 = 0x2A;
}

void setup_modulation() {
        RFST = RFST_SCAL;
        RFST = RFST_SRX;

        /* wait for calibration */
        sleepMillis(2);

	/* RX on */
}

/* freq in Hz */
void calibrate_freq(u32 freq, u8 ch) {
	set_radio_freq(freq);

	setup_modulation();
}


#define UPPER(a, b, c)  ((((a) - (b) + ((c) / 2)) / (c)) * (c))
#define LOWER(a, b, c)  ((((a) + (b)) / (c)) * (c))

void poll_keyboard() {
	switch (getkey()) {
	case ' ':
		/* pause */
		while (getkey() == ' ');
		while (getkey() != ' ')
			sleepMillis(200);
		break;
	default:
		break;
	}
}

int count = 0;

/* IRQ_DONE interrupt service routine */
void rf_isr() __interrupt (RF_VECTOR) {
	/* clear the interrupt flags */
	RFIF &= ~RFIF_IRQ_DONE;
	S1CON &= ~0x03;           // Clear the general RFIF interrupt registers

	rxdone = 1;
}

void main(void) {
	u8 counter;
	u8 i;
	u8 ci;
	u8 update = 1;

	xtalClock();
	setIOPorts();
	configureSPI();
	LCDReset();
	setup_dma_rx();
	radio_setup();
	set_filter();
	set_radio_freq(929612500); // 929.6125 MHz
	setup_modulation();

	clear();

	SSN = LOW;
	setCursor(0,0);
	printf("Waiting...?");
	SSN = HIGH;

	while(1) {
//		if (((update++)%2) == 0) {
//			clear();

//		}


//		sleepMillis(2);

		EA = 1; // enable interrupts globally
		IEN2 |= IEN2_RFIE; // enable RF interrupt
		RFIM = RFIM_IM_DONE; // mask IRQ_DONE only
		DMAARM |= DMAARM0;  // Arm DMA channel 0
//    		RFST = RFST_SRX;
//	        sleepMillis(1);

		setup_modulation();

		while (!rxdone);
		rxdone = 0;

		SSN = LOW;
		setCursor(0,0);
		printf("RX: ");
		for (ci = 0; ci < LEN; ci++) {
	        	printf("%2X ", rxbuf[ci]);
		}
//		setCursor(1,0);
//		printf("MCSM2: %2x, ", MCSM2);
        	SSN = HIGH;

/*		for (ci = 0; ci < (LEN - 1); ci++) {
			if (rxbuf[ci] == rxbuf[ci+1]) {
				if (rxbuf[ci] == 0xAA || rxbuf[ci] == 0x55) {
					clear();

					SSN = LOW;
					setCursor(0,0);
					for (ci = 0; ci < LEN; ci++) {
		        			printf("%02X", rxbuf[ci]);
					}
					SSN = HIGH;

					break;
				}
			}
		}*/

		RFST = RFST_SIDLE;


	}

	while(1) { }

}

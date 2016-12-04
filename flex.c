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

#define XTAL (26000000) // 26 MHz
#define UPPER(a, b, c)  ((((a) - (b) + ((c) / 2)) / (c)) * (c))
#define LOWER(a, b, c)  ((((a) + (b)) / (c)) * (c))

typedef enum { TWOFSK = 0, GFSK = 1, BLANK1 = 2, ONOFFKEY = 3, FOURFSK = 4, BLANK2 = 5, BLANK3 = 6, MSK = 7 } modulation;

/* globals */
__bit sleepy;
static volatile u8 rxdone = 0;
__xdata DMA_DESC dmaConfig;

#define LEN 12
__xdata u8 rxbuf[LEN];

void setup_dma_rx()
{
	dmaConfig.PRIORITY       = 2;  // high priority
	dmaConfig.M8             = 0;  // not applicable
	dmaConfig.IRQMASK        = 0;  // disable interrupts
	dmaConfig.TRIG           = 19; // radio
	dmaConfig.TMODE          = 0;  // single byte mode
	dmaConfig.WORDSIZE       = 0;  // one byte words
	dmaConfig.VLEN           = 0;  // use LEN
	SET_WORD(dmaConfig.LENH, dmaConfig.LENL, LEN);

	SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, &X_RFD);
	SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, rxbuf);
	dmaConfig.SRCINC         = 0;  // do not increment
	dmaConfig.DESTINC        = 1;  // increment by one

	SET_WORD(DMA0CFGH, DMA0CFGL, &dmaConfig);

	return;
}

void set_intermediate_frequency(u32 freq) {
	FSCTRL1 &= 0xE0;        // 0b11100000
	FSCTRL0 &= 0x00;        // 0b00000000

	// IF = XTAL / (2^10) * freq
	// TODO convert to double/float
	freq /= XTAL;
	freq *= (2 << 10);
	freq &= 0x0000001F;

	// 0b...xxxxx
	FSCTRL1 |= freq & 0x1F;
}

void set_modulation(modulation m) {
	MDMCFG2 &= 0x8F;       // 0b1...1111
	MDMCFG2 |= (m << 4);   // 0b.....xxx << 4
}

// dev is the desired deviation frequency in Hz
void set_deviation(u32 dev) {
	u8 M = 0, E = 0;

	// Mask out the bits to clear
	DEVIATN &= 0x88;       // 0b10001000

	// This may not be an appropriate
	// way to handle this division
	// TODO convert to double/float?
	// dev *= (2 << 17) / XTAL;
	// TODO: Allow settings to change with XTAL freq
	dev *= 0.00504123076923077;

	// Calculate the exponent
	while (dev >= (1 << 5)) {
		dev >>= 1;
		E++;
	}

	// Calculate the manissa
	M = dev - 8;

	// F_dev = XTAL / 2^17 * (8+M) * 2^E
	// 0b.EEE.MMM
	DEVIATN |= (E << 4) + (M);
}

void calibrate_freq_synth(void) {
	// These values were copied. The documentation
	// doesn't fully specify their behavior.
	FSCAL3 = 0xEA;
	FSCAL2 = 0x2A;
	FSCAL1 = 0x00;
	FSCAL0 = 0x1F;
}

void set_test_settings(void) {
	// These values were copied. The documentation
	// doesn't fully specify their behavior.
	TEST2 = 0x88;
	TEST1 = 0x31;
	TEST0 = 0x09;
}

void config_radio_control(void) {
	// Need to parameterize this one.

	MCSM0 = 0x04; // Default, never automatically calibrate
	MCSM1 = 0x3C; // Stay in RX
	MCSM2 = 0x07; // Default, minimum RX timeout
}

void set_channel_spacing(u32 chan) {
	u8 M = 0, E = 0;

	MDMCFG1 &= 0xFC; // 0b11111100

	// Channel spacing is (XTAL / 2^18) * (256 + M) * 2^E
	chan = chan / (XTAL >> 18);
	// TODO: divide this by the number of channels configured
	// TODO: right now assume this value is 1

	// Calculate the nessesary exponent
	while (chan >= (1 << 9)) {
		chan >>= 1;
		E++;
	}

	// Calculate the nessesary mantissa
	M = chan - 256;

	MDMCFG0 =  M;       // 0bMMMMMMMM
	MDMCFG1 |= E & 0x3; // 0b......EE
}

// Expected data rate in kbps
void set_data_rate(u32 rate) {
	u8 M = 0, E = 0;

	MDMCFG4 &= 0xF0; // 0b11110000

	// XTAL is assumed to be 26 MHz
	// These calculations are wrong if it changes
	// (10.32444... = 2^18 / 26000000)
	rate = rate * 10.3244406153846;

	// Calculate the correct exponent
	while (rate >= (1 << 9)) {
		rate >>= 1;
		E++;
	}

	// Calculate the correct mantissa
	M = rate - 256;

	MDMCFG3 =  M;       // 0bMMMMMMMM
	MDMCFG4 |= E & 0xF; // 0b....EEEE
}

void configure_packet_control(void) {
	// TODO: parameterize this function
	PKTCTRL0 = 0x02; // No data whitening, CRC off, infinite packet sizes
	PKTCTRL1 = 0x04; // Default, preamble quality off, no ADDR check
}

void set_sync_word(u16 word, u8 on) {
	// Split the word into two bytes
        SYNC1 = ~((word >> 8) & 0xff);
        SYNC0 = ~(word & 0xff);

	// Turn the sync word detection on and off
	// TODO: Give this the full granularity of the
	// TODO: features of the chip (partial matchs)
	MDMCFG2 &= 0xFD;               // 0b11111101
	MDMCFG2 |= (on ? 2 : 0) & 0x2; // 0b......?.
}

void set_address(u8 addr) {
	// Set the address, 0xFF and 0x00 are broadcast
	// TODO: Support flags to toggle address checks
	ADDR = addr;
}

void radio_setup() {
	/* IF of 457.031 kHz */
	set_intermediate_frequency(457031);

	/* disable 3 highest DVGA settings */
	AGCCTRL2 |= AGCCTRL2_MAX_DVGA_GAIN;

	/* frequency synthesizer calibration */
	calibrate_freq_synth();

	/* "various test settings" */
	set_test_settings();

	/* radio control state machine */
	config_radio_control();

	/* modem configuration */
	set_modulation(TWOFSK);
	set_data_rate(1600);
	set_channel_spacing(2500);

	/* packet control configuraton */
	configure_packet_control();

	/* sync word configuration */
	set_sync_word(0xDEA0, 1);
	// Look for 870C 5939 5555 DEA0
	// This is  DEA0 A6C6 AAAA 870C but with bits flipped

	/*
	0x870C, 1600 baud, 2-FSK
	0xB068, 1600 baud, 4-FSK
	0xDEA0, 3200 baud, 4-FSK
	0x4C7C, 3200 baud, 4-FSK
	*/

	/* address configuration */
	set_address(0x00);

	/* deviation settings */
	set_deviation(1600);
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
void calibrate_freq(u32 freq, u8 chan) {
	set_radio_freq(freq);

	setup_modulation();
}

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

u8 reverse(u8 b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

/*
	American Messaging
	------------------
	set_radio_freq(929937500); // 929.9375 MHz, 6400 baud, inverted, ABCD
	set_radio_freq(931062500); // 931.0625 MHz, 6400 baud, inverted + Pog

	Skytel
	------
	set_radio_freq(931937500); // 931.9375 MHz, 6400 baud, inverted, ABCD

	USA Mobility
	------------
	set_radio_freq(929662500); // 929.6625 MHz, 6400 baud, inverted, ABCD
	set_radio_freq(929587500); // 929.5875 MHz, 6400 baud, inverted, ABCD
	set_radio_freq(929612500); // 929.6125 MHz, 3200 baud, inverted, AB
	set_radio_freq(931212500); // 931.2125 MHz, 3200 baud, inverted, AB
*/

void main(void) {
	u8 ci;
	u8 update = 1;
	u8 state = 0;
	u8 cycleno = 0;
	u8 frameno = 0;
	u32 fiw = 0;
	u32 checksum = 0;
	u8 line = 0;
	u16 time = 0;

	xtalClock();
	setIOPorts();
	configureSPI();
	LCDReset();
	setup_dma_rx();
	radio_setup();

	set_radio_freq(929612500); // 929.6125 MHz, 3200 baud, inverted, AB

	setup_modulation();

	clear();

	SSN = LOW;
	setCursor(0,0);
	printf("Waiting...?");
	SSN = HIGH;

	while(1) {
		// Set DMA and radio interrupts
		EA = 1;			// enable interrupts globally
		IEN2 |= IEN2_RFIE;	// enable RF interrupt
		RFIM = RFIM_IM_DONE;	// mask IRQ_DONE only
		DMAARM |= DMAARM0;	// Arm DMA channel 0

		// Turn the radio on
		//setup_modulation();

		// Wait for the interrupt (which sets rxdone = 1)
		while (!rxdone);
		rxdone = 0;

//		if (state == 1) {
			// Print the incoming data

//			break;
//		}

		if (state == 0 && rxbuf[0] == 0x59 && rxbuf[1] == 0x39 && rxbuf[2] == 0x55 && rxbuf[3] == 0x55 && rxbuf[4] == 0xDE && rxbuf[5] == 0xA0) {
			SSN = LOW;
			setCursor(0,0);
			for (ci = 8; ci < LEN; ci++)
				printf("%2x", rxbuf[ci] ^ 0xff);
			setCursor(1,1);
			for (ci = 8; ci < LEN; ci++)
				printf("%2x", reverse(rxbuf[ci]));

			fiw = 0;
			//fiw |= reverse(rxbuf[11]) & 0x7F;
			//fiw <<= 8;
			//fiw |= reverse(rxbuf[10]);
			//fiw <<= 8;
			fiw |= reverse(rxbuf[ 9]);
			fiw <<= 8;
			fiw |= reverse(rxbuf[ 8]);

			setCursor(2,2);
			printf("%8x", fiw);
			SSN = HIGH;

/*			checksum = (fiw & 0xF);
			checksum += ((fiw >> 4) & 0xF);
			checksum += ((fiw >> 8) & 0xF);
			checksum += ((fiw >> 12) & 0xF);
			checksum += ((fiw >> 16) & 0xF);
			checksum += ((fiw >> 20) & 0x01);

			checksum &= 0xF;

			if (checksum != 0xF) {*/
				cycleno = (fiw >> 4) & 0xF;
				frameno = (fiw >> 8) & 0x7F;
				time = cycleno*4*60 + frameno*4*60/128;

				SSN = LOW;
				setCursor(3,3);
				printf("%02u:%02u", time / 60, time % 60);
		        	SSN = HIGH;
//			}

//			set_modulation(FOURFSK);
//			set_deviation(4800);
//			set_data_rate(3200);
//			set_sync_word(0x00, 0);
//			state = 1;
		}

		RFST = RFST_SIDLE;
		RFST = RFST_SCAL;
		RFST = RFST_SRX;

		// Set the radio control state to idle
		// TODO: this probably isn't needed
		//RFST = RFST_SIDLE;
	}

	while (1) {}
}

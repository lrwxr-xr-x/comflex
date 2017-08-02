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
static volatile u8 rxdone = 0, timer1done = 0;
__xdata DMA_DESC dmaConfig;

#define LEN 12
__xdata u8 rxbuf[LEN];

__xdata u8 page_buffer[4*11*8*4]; // 4 phases, 11 blocks, 8 codewords, 32 bits
__xdata u8 page[4][11*8*4]; // 2 phases, 11 blocks, 8 codewords, 32 bits

void setup_dma_rx(u8* dest, u16 len)
{
	dmaConfig.PRIORITY       = 2;  // high priority
	dmaConfig.M8             = 0;  // not applicable
	dmaConfig.IRQMASK        = 0;  // disable interrupts
	dmaConfig.TRIG           = 19; // radio
	dmaConfig.TMODE          = 0;  // single byte mode
	dmaConfig.WORDSIZE       = 0;  // one byte words
	dmaConfig.VLEN           = 0;  // use LEN

	SET_WORD(dmaConfig.LENH, dmaConfig.LENL, len);

	SET_WORD(dmaConfig.SRCADDRH, dmaConfig.SRCADDRL, &X_RFD);
	SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, (u16)dest);
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
	// (10.32444... = 2^28 / 26000000)
	rate = rate * 10.3244406153846;

	// rate = (256 + M) * 2^E

	// Calculate the correct exponent
	while (rate >= 512) { // Maximum possible value
		rate /= 2;
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

/* Global timer configuration */
void setup_global_timer_config(void) {
	/* System Clock Configuration */
	CLKCON &= 0xC7;	// 0b11000111
	// this means system clock prescale set to 0.25
	// one out of four ticks will register for timers
	CLKCON |= 0x10; // 0b..010...

	/* Timer 1 Configuration */
	T1CTL &= 0xf2;  // 0b11110011
	// Set the "div", prescale on sysclock
	// '11' = one out of 128 ticks will increment
	T1CTL |= 0x0C;
	// Don't set the "mode" - mode of operation
	// Setting the mode triggers the timer
	// '10' = modulo the value in T1CC0
	// T1CTL |= 0x02;

	/* Interrupt flags */
	IEN0 |= 0x80;
	IEN1 |= 0x02;

	T1CTL &= 0xEF;  // 0b...1....
			// Turn off the flag
	TIMIF |= 0x40;	// 0b.1......
			// Turn on the overflow interrupt
}

void set_timer1(u16 ms, u16 us) {
	/* Set the interrupt flags */
	IEN0 |= 0x80;
	IEN1 |= 0x2;

	T1CTL &= 0xEF;  // Turn off the interrupt flag
	TIMIF |= 0x40;  // Turn on the interrupt mask

	T1CNTL = 0;

//	SSN = LOW;
//	printf(" %x", T1CTL);
//	SSN = HIGH;

	// The following magic numbers come from...
	// Timer1 will trigger when ms total increments
	//  are reached. Each increment takes one unit
	//  of (sysclock / sysprescale / timer prescale)

	// We have configured sysprescale = 4 and
	//  timer prescale to 128. The sysclock runs at
	//  26 MHz. Therefore one unit of time for timer
	//  increment is (26 MHz / 4 / 128) = 50.78 kHz

	// Therefore 50.78 increments happen per ms

	// The prescale values have been calculated so
	// that the timer can trigger on a full second.
	// It does not appear possible to configure the
	// prescale values to wait minutes.

	ms = (ms * 50.78);
	us = (us * .05078);
	ms = ms + us;

	T1CC0L = ms & 0xFF;
	T1CC0H = (ms >> 8) & 0xFF;

	T1CTL &= 0xEF;  // Turn off the interrupt flag
	TIMIF |= 0x40;  // Turn on the interrupt mask

	// Set Timer 1 control mode to modulus mode
	T1CTL &= 0xFC; // 0b11111100
	T1CTL |= 0x02;
//	T1CTL |= 0x01; // 0b......10 - "10" = mod mode
	// Setting the mode triggers the timer start

//	SSN = LOW;
//	printf(" %x", T1CTL);
//	SSN = HIGH;

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

void t1_isr() __interrupt (T1_VECTOR) {
//	SSN = LOW;
//	printf("X");
//	SSN = HIGH;

	timer1done = 1;

//	T1CTL = 0;
//	TIMIF = 0; // 0b10111111 -> Turn mask off
	T1CTL &= 0xEF; // 0b11101111 -> Turn flag off
	TIMIF &= 0xBF; // 0b10111111 -> Turn mask off
}

void t2_isr() __interrupt (T2_VECTOR) {
	t1_isr();
}

void t3_isr() __interrupt (T3_VECTOR) {
	t1_isr();
}

u8 reverse(u8 b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

u8 fix(u8 val) {
	u8 a, b, c, d;
	a = (val >> 6) & 3;
	b = (val >> 4) & 3;
	c = (val >> 2) & 3;
	d = (val >> 0) & 3;

	if (a < 2) a ^= 1;
	if (b < 2) b ^= 1;
	if (c < 2) c ^= 1;
	if (d < 2) d ^= 1;

//	a = 3 - a;
//	b = 3 - b;
//	c = 3 - c;
//	d = 3 - d;

	val = (a << 6) + (b << 4) + (c << 2) + (d << 0);
	return val;
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
	u16 ci;
	u8 update = 1;
	u16 state = 0;
	u8 cycleno = 0;
	u8 frameno = 0;
	u32 fiw = 0;
	u32 checksum = 0;
	u8 line = 0;
	u16 time = 0;
	u8 countdown = 5;
	u16 temp = 0;
	u8 col = 0;
	u8 block = 0;
	u8 bit = 0;
	u8 sb = 0;
	u8 biw;
	u8 voffset;
	u8 aoffset;
	u16 capcode;
	u8 bita = 0;
	u8 bitb = 0;
	u8 toggle = 0;
	u8 index = 0;
	u16 count = 0;
	u8 lcount = 0;
	u8 hcount = 0;
	u8 timerdown = 0;

	xtalClock();
	setIOPorts();
	configureSPI();
	setup_global_timer_config();
	LCDReset();

	clear();

	SSN = LOW;
	setCursor(0,0);
	printf("Waiting...?");
	SSN = HIGH;

	setup_dma_rx(rxbuf, LEN);
	radio_setup();
	set_radio_freq(929612500); // 929.6125 MHz, 3200 baud, inverted, AB
	setup_modulation();

	while(1) {
		SSN = LOW;
		setCursor(3,3);
		printf("state: %u", 0);
		SSN = HIGH;

		switch (state) {
		case 0:
		// Sync1
			// Set DMA and radio interrupts
			EA = 1;			// enable interrupts globally
			IEN2 |= IEN2_RFIE;	// enable RF interrupt
			RFIM = RFIM_IM_DONE;	// mask IRQ_DONE only
			DMAARM |= DMAARM0;	// Arm DMA channel 0

			// Turn the radio on
			setup_modulation();

			// Wait for the interrupt (which sets rxdone = 1)
			while (!rxdone);
			rxdone = 0;
			RFST = RFST_SIDLE;
//			RFST = RFST_SCAL;
			RFST = RFST_SRX;

			if (rxbuf[0] == 0x59 && rxbuf[1] == 0x39 && rxbuf[2] == 0x55 && rxbuf[3] == 0x55 && rxbuf[4] == 0xDE && rxbuf[5] == 0xA0) {
//				fiw = 0;
//				fiw |= reverse(rxbuf[9]);
//				fiw <<= 8;
//				fiw |= reverse(rxbuf[8]);

/*				cycleno = (fiw >> 4) & 0xF;
				frameno = (fiw >> 8) & 0x7F;
				time = cycleno*4*60 + frameno*4*60/128;

				SSN = LOW;
				setCursor(0,0);
				printf("%02u:%02u", time / 60, time % 60);
	       		 	SSN = HIGH;*/

				state = 1;
// Look at 28 is real val
				set_timer1(5, 104+(312.5 * 20)); // 25 ms wait over Sync2
							// 25 ms / 80 symbols is one symbol
							// per 312.5 microseconds
			}
		break;

		case 1:
//		SSN = LOW;
//		setCursor(3,3);
//		printf("state: %u", 1);
//		SSN = HIGH;
		// Pause over Sync2
			while (!timer1done) { }
			timer1done = 0;
			state = 2;
		break;

		case 2:
//		SSN = LOW;
//		setCursor(3,3);
//		printf("state: %u", 2);
//		SSN = HIGH;
		// Get a data frame
			set_modulation(FOURFSK);
			set_deviation(4800);
			set_data_rate(3200);	// Data rate = bps or baud?
						// Assuming it is baud
			set_sync_word(0x0000, 0);

		        SET_WORD(dmaConfig.LENH, dmaConfig.LENL, 40);//4*11*8*4);
		        SET_WORD(dmaConfig.DESTADDRH, dmaConfig.DESTADDRL, (u16)page_buffer);
//			setup_dma_rx(page_buffer, 40);

			EA = 1;			// enable interrupts globally
			IEN2 |= IEN2_RFIE;	// enable RF interrupt
			RFIM = RFIM_IM_DONE;	// mask IRQ_DONE only
			DMAARM |= DMAARM0;	// Arm DMA channel 0

			while (!rxdone) { }
			rxdone = 0;
			RFST = RFST_SIDLE;

			state = 3;

			timerdown = 0;
		break;

		case 3:
		// Deinterleave and decode
			ci = 0;
			sb = 6;
			count = 0;
			lcount = 0;
			hcount = 0;
			toggle = 0;

			for (ci = 0; ci < (4*11*8*4); ci++) {
			page_buffer[ci] = fix(page_buffer[ci]);
			for (sb = 6; sb >= 0 && sb <= 6; sb-=2) {
				temp = page_buffer[ci];
				temp = temp >> sb;
				temp = temp & 0x3;
//				temp = temp ^ 0x3;

				if (temp > 1) bita = 1;
				else bita = 0;

				if (temp == 1 || temp == 2) bitb = 1;
				else bitb = 0;

				index = ((count>>5)&0xFFF8) | (count&0x0007);
				index = index * 4;

				if (toggle == 0) {
					page[0][index+3-lcount] = page[0][index+3-lcount] & ~(1 << hcount);
					page[0][index+3-lcount] = page[0][index+3-lcount] | ((bita) << hcount);
					page[1][index+3-lcount] = page[1][index+3-lcount] & ~(1 << hcount);
					page[1][index+3-lcount] = page[1][index+3-lcount] | ((bitb) << hcount);
				} else {
					page[2][index+3-lcount] = page[2][index+3-lcount] & ~(1 << hcount);
					page[2][index+3-lcount] = page[2][index+3-lcount] | ((bita) << hcount);
					page[3][index+3-lcount] = page[3][index+3-lcount] & ~(1 << hcount);
					page[3][index+3-lcount] = page[3][index+3-lcount] | ((bitb) << hcount);
				}

				toggle = (toggle == 1) ? 0 : 1;

				if (toggle == 0) {
					count++;

					if ((count % 8) == 0) {
						hcount++;
						if (hcount == 8) { hcount = 0; lcount++; }
						if (lcount == 4) { lcount = 0; }
					}
				}

			}
			}

/*			for (block = 0; block < 11; block++) {
				for (bit = 0; bit < 8; bit++) {
					page[0][block][bit][0] = page[0][block][bit][1] = page[0][block][bit][2] = page[0][block][bit][3] = 0;
					page[1][block][bit][0] = page[1][block][bit][1] = page[1][block][bit][2] = page[1][block][bit][3] = 0;
				}
				for (col = 0; col < 32; col++) {
					for (bit = 0; bit < 8; bit++) {
						temp = page_buffer[ci];
						temp >>= sb;
						temp &= 0x03;

						if (sb > 0) sb -= 2;
						else { sb = 6; ci++; }

						page[0][block][bit][(col-(col%8))/8] |= ((temp & 0x02) ? 1 : 0) << (7 - (col%8));
						page[1][block][bit][(col-(col%8))/8] |= ((temp & 0x01) ? 1 : 0) << (7 - (col%8));
					}
				}
			}*/

			SSN = LOW;
			setCursor(6,6);
			printf("Raw: %2x%2x%2x%2x", page_buffer[timerdown+0], page_buffer[timerdown+1], page_buffer[timerdown+2], page_buffer[timerdown+3]);
			setCursor(5,5);
			printf("Int: %2x%2x%2x%2x", page[0][timerdown+0], page[0][timerdown+1], page[0][timerdown+2], page[0][timerdown+3]);
			SSN = HIGH;

			set_timer1(1000, 0);

			while (!timer1done) { }
			timer1done = 0;

			set_timer1(1000, 0);

			while (!timer1done) { }
			timer1done = 0;

			set_timer1(1000, 0);

			while (!timer1done) { }
			timer1done = 0;

			timerdown+=4;

			if (timerdown == 20) {
/*
			biw = page[0][0][0][0];

			voffset = (biw >> 2) & 0x3f;
			aoffset = (biw & 0x03) + 1;

			// Iterate through pages and dispatch to appropriate handler
			for (ci = aoffset; ci < voffset; ci++) {
				capcode = page[0][(ci - (ci%8))/8][ci%8][2];
				capcode <<= 8;
				capcode |= page[0][(ci - (ci%8))/8][ci%8][3];
				SSN = LOW;
				setCursor(5,5);
				printf("B%2x A%2x V%2x C%2x", biw, aoffset, voffset, capcode);
				SSN = HIGH;
			}*/

			setup_dma_rx(rxbuf, LEN);
			radio_setup();

			RFST = RFST_SCAL;
			RFST = RFST_SRX;

			state = 0;
			}
		break;
		}
	}

	while (1) {}
}

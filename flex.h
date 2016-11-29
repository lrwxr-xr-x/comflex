/* Copyright 2016 Ross Snider
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

/* frequencies in MHz */
#define DEFAULT_FREQ     (929612500) // 929.6125 MHz
#define DEFAULT_WIDT     (5000)      // 5 KHz

/* band limits in MHz */
#define MIN_300  281
#define MAX_300  361
#define MIN_400  378
#define MAX_400  481
#define MIN_900  749
#define MAX_900  962

/* band transition points in MHz */
#define EDGE_400 369
#define EDGE_900 615

/* VCO transition points in Hz */
#define MID_300  318000000
#define MID_400  424000000
#define MID_900  848000000

/* power button debouncing for wake from sleep */
#define DEBOUNCE_COUNT  4
#define DEBOUNCE_PERIOD 50

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))


void clear();
void putchar(char c);
u8 getkey();
void radio_setup();
void set_filter();
void set_radio_freq(u32 freq);
void calibrate_freq(u32 freq, u8 ch);
u16 set_center_freq(u16 freq);
void tune(u8 ch);
void poll_keyboard();
void main(void);

typedef struct {
	u8 SRCADDRH;
	u8 SRCADDRL;
	u8 DESTADDRH;
	u8 DESTADDRL;
	u8 LENH      : 5;
	u8 VLEN      : 3;

	u8 LENL      : 8;

	u8 TRIG      : 5;
	u8 TMODE     : 2;
	u8 WORDSIZE  : 1;

	u8 PRIORITY  : 2;
	u8 M8        : 1;
	u8 IRQMASK   : 1;
	u8 DESTINC   : 2;
	u8 SRCINC    : 2;
} DMA_DESC;

#define HIBYTE(a)  (u8) ((u16)(a) >> 8 )
#define LOBYTE(a)  (u8)  (u16)(a)

#define SET_WORD(regH, regL, word) \
	do {                           \
		(regH) = HIBYTE( word );   \
		(regL) = LOBYTE( word );   \
	} while (0)

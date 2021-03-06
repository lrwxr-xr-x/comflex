# Copyright 2010 Travis Goodspeed, Michael Ossmann
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

libs = display.rel keys.rel pm.rel
CC = sdcc
CFLAGS = --no-pack-iram
LFLAGS = --xram-loc 0xF000

all: flex.hex

%.rel : %.c
	$(CC) $(CFLAGS) -c $<

flex.hex: flex.rel $(libs)
	sdcc $(LFLAGS) flex.rel $(libs)
	packihx <flex.ihx >flex.hex

install: flex.hex
	goodfet.cc erase
	goodfet.cc flash flex.hex
verify: flex.hex
	goodfet.cc verify flex.hex
clean:
	rm -f *.hex *.ihx *.rel *.asm *.lst *.rst *.sym *.lnk *.map *.mem

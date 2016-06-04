/*
 * This file is part of belnanosat project.
 *
 * Copyright (C) 2016 Uladzislau Paulovich <selatnick@gmail.com>
 *
 * belnanosat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * belnanosat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with belnanosat.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "eeprom_fileid.h"
#include "eeprom.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "i2c.h"
#include "utils.h"

// Maximum length of the message: 255 + 1 byte for length field
static uint8_t buffer[256];

static int min(int a, int b) {
	return a<b?a:b;
}

void EEPROM_setup(EEPROM *eeprom, uint32_t i2c) {
	eeprom->i2c = i2c;
#ifndef EEPROM_OVERWRITE
	// Search for an empty space
	uint16_t cur_address = 0;
	volatile uint8_t cur_len = i2c_wread_byte(eeprom->i2c, EEPROM_ADDRESS,
	                                          cur_address);
	// TODO: handle out of memory error
	while (cur_len) {
		/*
		 * cur_len = length of the message.
		 * We also need to consider 1 byte, which actually
		 * stores length of the message.
		 */
		cur_address += cur_len + 1;
		cur_len = i2c_wread_byte(eeprom->i2c, EEPROM_ADDRESS, cur_address);
	}
	eeprom->cur_page_id = (cur_address >> 6);
	eeprom->cur_page_shift = (cur_address & 0x3F);
#else
	EEPROM_erase(eeprom);
	eeprom->cur_page_id = 0;
	eeprom->cur_page_shift = 0;
#endif
}

// TODO: handle out of memory error
void EEPROM_write(EEPROM *eeprom, uint8_t *input_buffer, uint8_t n) {
	// Copying data to a temporary buffer and writing everything at once
	// is faster than writing length of a buffer separately
	buffer[0] = n;
	memmove(buffer + 1, input_buffer, n);
	int cur_length = n + 1;
	int buf_start = 0;
	while (cur_length > 0) {
		// Completely fill current page
		int bytes_to_write = min(cur_length, 64 - eeprom->cur_page_shift);
		i2c_write_sequence(eeprom->i2c, EEPROM_ADDRESS,
		                   eeprom->cur_page_id * 64 + eeprom->cur_page_shift,
		                   buffer + buf_start, bytes_to_write);
		msleep(10);
		buf_start += bytes_to_write;
		cur_length -= bytes_to_write;
		eeprom->cur_page_shift += bytes_to_write;
		if (eeprom->cur_page_shift == 64) {
			++eeprom->cur_page_id;
			eeprom->cur_page_shift = 0;
		}
	}
}

void EEPROM_read(EEPROM *eeprom, uint16_t start_address,
                 uint8_t *output_buffer, int n) {
	i2c_read_sequence(eeprom->i2c, EEPROM_ADDRESS, start_address,
	                  output_buffer, n);
}

void EEPROM_erase(EEPROM *eeprom) {
	int page_id;
	memset(buffer, 0, 64);
	for (page_id = 0; page_id < EEPROM_PAGES_NUM; ++page_id) {
		i2c_write_sequence(eeprom->i2c, EEPROM_ADDRESS,
		                   page_id * 64, buffer, 64);
		msleep(10);
	}
}

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

#include "log.h"

#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/gpio.h>

#include "sdcard.h"
#include "sdcard2.h"

static uint8_t buff[512];

static uint32_t cur_block_id = 0x7740;
static uint32_t cur_block_shift = 0;
static uint32_t cur_block_id2 = 16640;

// TODO: search for a place to append log. We don't want to override
// existing datax
void log_setup(void) {
}

void log_write(const uint8_t* data, uint32_t len) {
	if (cur_block_shift + len < 512) {
		memcpy(buff + cur_block_shift, data, len);
		cur_block_shift += len;
	} else {
		gpio_toggle(GPIOC, GPIO3);

		memcpy(buff + cur_block_shift, data, 512 - cur_block_shift);
		sdcard_single_block_write(cur_block_id, buff);
		sdcard2_single_block_write(cur_block_id2, buff);

		++cur_block_id;
		++cur_block_id2;
		memset(buff, 0, 512);
		cur_block_shift = len - (512 - cur_block_shift);
		memcpy(buff, data + len - cur_block_shift, cur_block_shift);
	}
}

void log_clear(void) {
	gpio_clear(GPIOC, GPIO3);
	uint32_t i;
	for (i = 0; i < 512; ++i) {
		buff[i] = 0;
	}
	for (i = 0; i < 1024 * 100; ++i) {
		sdcard_single_block_write(cur_block_id + i, buff);
//		sdcard2_single_block_write(cur_block_id2 + i, buff);
	}
	gpio_set(GPIOC, GPIO3);
}

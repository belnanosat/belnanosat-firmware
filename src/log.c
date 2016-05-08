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

static uint8_t buff[512];

static uint32_t cur_block_id = (0x3acc00 >> 9) + 1000 - 863;
static uint32_t cur_block_shift = 0;

// TODO: search for a place to append log. We don't want to override
// existing datax
void log_setup(void) {

}

void log_write(const uint8_t* data, uint32_t len) {
	if (cur_block_shift + len < 512) {
		memcpy(buff + cur_block_shift, data, len);
		cur_block_shift += len;
	} else {
		gpio_toggle(GPIOD, GPIO15);

		memcpy(buff + cur_block_shift, data, 512 - cur_block_shift);
		sdcard_single_block_write(cur_block_id << 9, buff);

		++cur_block_id;
		memset(buff, 0, 512);
		cur_block_shift = len - (512 - cur_block_shift);
		memcpy(buff, data, cur_block_shift);
	}
}

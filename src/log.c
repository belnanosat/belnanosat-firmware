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
static uint8_t buff2[512];

static uint32_t cur_block_id = 36864;//100000;
static uint32_t cur_block_shift = 0;
static uint32_t cur_block_id2 = 16640;

// TODO: search for a place to append log. We don't want to override
// existing datax
void log_setup(void) {
}

void log_write(const uint8_t* data, uint32_t len) {
	/* memset(buff, 0, 512); */
	/* memcpy(buff, data, len); */
	/* sdcard_single_block_write(cur_block_id, buff); */
	/* sdcard_single_block_read(cur_block_id, buff2); */
	/* if (!memcmp(buff, buff2, 512)) { */
	/* 	gpio_toggle(GPIOC, GPIO3); */
	/* } */
	/* ++cur_block_id; */
	/* sdcard_setup(); */
	/* return; */
	if (cur_block_shift + len < 512) {
		memcpy(buff + cur_block_shift, data, len);
		cur_block_shift += len;
	} else {
		memcpy(buff + cur_block_shift, data, 512 - cur_block_shift);
		/* sdcard_setup(); */
		/* msleep(100); */
//		sdcard_setup();
//		msleep(100);
		sdcard_single_block_write(cur_block_id, buff);
		memset(buff2, 0, 512);
		sdcard_single_block_read(cur_block_id, buff2);
		if (!memcmp(buff, buff2, 512)) {
			gpio_toggle(GPIOC, GPIO3);
		}
		sdcard_setup();

//		sdcard2_single_block_write(cur_block_id, buff);

		++cur_block_id;
		++cur_block_id2;
		memset(buff, 0, 512);
		cur_block_shift = len - (512 - cur_block_shift);
		memcpy(buff, data + len - cur_block_shift, cur_block_shift);
	}
}

void log_write2(uint8_t* data, uint32_t id) {
//	memset(data, id % 256, 512);
	sdcard_single_block_write(cur_block_id + id, data);
}

void log_clear(void) {
	gpio_clear(GPIOC, GPIO3);
	uint32_t i, j;
	for (j = 0; j < 512; ++j) {
		buff[j] = 0;
	}
	for (i = 0; i < 1024 * 100; ++i) {
		memset(buff, (i + 10) % 256, 512);
		sdcard_single_block_write(cur_block_id + i, buff);
 //		sdcard2_single_block_write(cur_block_id2 + i, buff);
	}
	gpio_set(GPIOC, GPIO3);
}

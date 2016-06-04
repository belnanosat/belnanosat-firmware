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

#include "utils_fileid.h"
#include "utils.h"

#include <limits.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

volatile uint64_t system_microseconds;

void sys_tick_handler(void) {
	system_microseconds++;
}

void usleep(uint64_t delay) {
	uint64_t wake = system_microseconds + delay;
	while (wake > system_microseconds);
}

void msleep(uint64_t delay) {
	uint64_t i;
	for (i = 0; i < delay; ++i) usleep(1000);
}

uint64_t get_time_ms(void) {
	return system_microseconds / 1000;
}

uint64_t get_time_us(void) {
	return system_microseconds;
}

uint64_t get_time_since_ms(uint64_t start_time) {
	return get_time_ms() - start_time;
}

uint64_t get_time_since_us(uint64_t start_time) {
	return system_microseconds - start_time;
}

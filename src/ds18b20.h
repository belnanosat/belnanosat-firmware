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

#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include <stdbool.h>

#include "onewire.h"

#define DS18B20_MAX_DEVICES_NUM 4

#define DS18B20_CMD_SEARCH_ROM        0xF0
#define DS18B20_CMD_READ_ROM          0x33
#define DS18B20_CMD_MATCH_ROM         0x55
#define DS18B20_CMD_SKIP_ROM          0xCC
#define DS18B20_CMD_ALARM_SEARCH      0xEC

#define DS18B20_CMD_CONVERT_T         0x44
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48
#define DS18B20_CMD_RECALL            0xB8
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4

#define DS18B20_RESOLUTION_9_BITS     0x00
#define DS18B20_RESOLUTION_10_BITS    0x01
#define DS18B20_RESOLUTION_11_BITS    0x02
#define DS18B20_RESOLUTION_12_BITS    0x03

typedef struct DS18B20 {
	uint8_t rom[8];
	bool is_present;
} DS18B20;

typedef struct DS18B20Bus
{
	OneWire_t one_wire;
	uint32_t devices_num;
	uint8_t resolution;

	DS18B20 devices[DS18B20_MAX_DEVICES_NUM];
} DS18B20Bus;

/* Returns number of found devices on the 1-Wire bus */
uint8_t ds18b20_setup(DS18B20Bus *bus, uint32_t gpio_port, uint16_t gpio_pin,
                      uint8_t resolution);
uint8_t ds18b20_start_all(DS18B20Bus *bus);
uint8_t ds18b20_start_one(DS18B20Bus *bus, uint8_t id);
uint8_t ds18b20_read(DS18B20Bus *bus, uint8_t id, float *res);
uint16_t ds18b20_read_raw(DS18B20Bus *bus, uint8_t id);
bool ds18b20_is_conversion_finished(DS18B20Bus *bus);

#endif

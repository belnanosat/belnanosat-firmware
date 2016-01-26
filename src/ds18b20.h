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

#include "onewire.h"

#define DS18B20_MAX_DEVICES_NUM 16

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

typedef struct DS18B20
{
	OneWire_t one_wire;
	uint32_t devices_num;
	uint32_t start_time;
	uint8_t conv_process;
	uint8_t rom[DS18B20_MAX_DEVICES_NUM][8];
} DS18B20;

/* Returns number of found devices on the 1-Wire bus */
uint8_t ds18b20_setup(DS18B20 *sensor, uint32_t gpio_port, uint16_t gpio_pin);
uint8_t ds18b20_start_all(DS18B20 *sensor);
uint8_t ds18b20_start_one(DS18B20 *sensor, uint8_t id);
uint8_t ds18b20_read(DS18B20 *sensor, uint8_t id, float *res);

#endif

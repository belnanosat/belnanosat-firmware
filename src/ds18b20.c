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

#include "ds18b20.h"

#include <string.h>
#include <stdio.h>

#include "utils.h"

/*
 * This lets us determine a short id (from 0 to 3) of each sensor.
 */
static uint8_t devices_map[4][8] = {
	{0x28, 0xFF, 0x5D, 0x18, 0x67, 0x14, 0x02, 0xA1},
	/* {0x28, 0xFF, 0x43, 0x1F, 0x67, 0x14, 0x02, 0xF9}, */
	{0x28, 0xFF, 0x36, 0x20, 0x67, 0x14, 0x02, 0x94},
	{0x28, 0xFF, 0x6E, 0x08, 0x67, 0x14, 0x02, 0x53},
	{0x28, 0xFF, 0x6F, 0x16, 0x67, 0x14, 0x02, 0xB3},
};

uint8_t ds18b20_setup(DS18B20Bus *bus, uint32_t gpio_port, uint16_t gpio_pin,
                      uint8_t resolution) {
	uint8_t devices;
	uint8_t tmp_rom[8];
	int i;

	for (i = 0; i < 4; ++i) {
		bus->devices[i].is_present = false;
	}

	OneWire_Init(&bus->one_wire, gpio_port, gpio_pin);
	devices = OneWire_First(&bus->one_wire);
	bus->devices_num = 0;
	bus->resolution = resolution;
	while (devices) {
		bus->devices_num++;
		OneWire_GetFullROM(&bus->one_wire, tmp_rom);
		// Is it a known device?
		bool is_found = false;
		for (i = 0; i < 4; ++i) {
			if (!memcmp(devices_map[i], tmp_rom, 8)) {
				bus->devices[i].is_present = true;
				memmove(bus->devices[i].rom, tmp_rom, 8);
				is_found = true;
				break;
			}
		}
		if (!is_found) {
			printf("Warning: unknown device serial code, ignoring it: ");
			for (i = 0; i < 8; ++i) {
				printf("0x%02X, ", tmp_rom[i]);
			}
			printf("\n\r");
		}
		devices = OneWire_Next(&bus->one_wire);
	}

	// Choose resolution level
	for (i = 0; i < 4; ++i) {
		if (!bus->devices[i].is_present) continue;
		OneWire_Reset(&bus->one_wire);
		OneWire_SelectWithPointer(&bus->one_wire, bus->devices[i].rom);
		OneWire_WriteByte(&bus->one_wire, DS18B20_CMD_WRITE_SCRATCHPAD);
		/*
		 * Write just zero values as T_h and T_l because we
		 * dont't use this alarm signaling feature
		 */
		OneWire_WriteByte(&bus->one_wire, 0);
		OneWire_WriteByte(&bus->one_wire, 0);
		uint8_t conf_register = 0x1F;
		conf_register |= (resolution << 5);
		OneWire_WriteByte(&bus->one_wire, conf_register);
	}
	return bus->devices_num;
}

uint8_t ds18b20_start_all(DS18B20Bus *bus) {
	OneWire_Reset(&bus->one_wire);
	OneWire_WriteByte(&bus->one_wire, DS18B20_CMD_SKIP_ROM);
	OneWire_WriteByte(&bus->one_wire, DS18B20_CMD_CONVERT_T);
	return 1;
}

uint8_t ds18b20_start_one(DS18B20Bus *bus, uint8_t id) {
	if (!bus->devices[id].is_present) return 0;
	OneWire_Reset(&bus->one_wire);
	OneWire_SelectWithPointer(&bus->one_wire, bus->devices[id].rom);
	OneWire_WriteByte(&bus->one_wire, DS18B20_CMD_CONVERT_T);
	return 1;
}

uint8_t ds18b20_read(DS18B20Bus *bus, uint8_t id, float *res) {
	if (!bus->devices[id].is_present) return 0;
	uint32_t temperature = ds18b20_read_raw(bus, id);
	*res = (temperature >> 4) | (((temperature >> 8) & 0x07) << 4);
	*res += ((temperature & 0x0F) * 0.0625f);
	return 1;
}

uint16_t ds18b20_read_raw(DS18B20Bus *bus, uint8_t id) {
	int i;
	uint8_t data[9];
	uint8_t crc;
	if (!bus->devices[id].is_present) return 0;
	/* Wait for the end of the conversion */
	while(!ds18b20_is_conversion_finished(bus));
	OneWire_Reset(&bus->one_wire);
	OneWire_SelectWithPointer(&bus->one_wire, bus->devices[id].rom);
	OneWire_WriteByte(&bus->one_wire, DS18B20_CMD_READ_SCRATCHPAD);
	for(i = 0; i < 9; ++i) {
		data[i] = OneWire_ReadByte(&bus->one_wire);
	}
	crc = OneWire_CRC8(data, 8);
	if(crc != data[8]) {
		return 0;
	}
	return data[0] | ((uint32_t)data[1] << 8);
}

bool ds18b20_is_conversion_finished(DS18B20Bus *bus) {
	return OneWire_ReadBit(&bus->one_wire);
}

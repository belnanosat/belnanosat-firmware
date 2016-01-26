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

uint8_t ds18b20_setup(DS18B20 *sensor, uint32_t gpio_port, uint16_t gpio_pin)
{
	uint8_t devices;
	OneWire_Init(&sensor->one_wire, gpio_port, gpio_pin);
	devices = OneWire_First(&sensor->one_wire);
	sensor->devices_num = 0;
	sensor->conv_process = 0;
	sensor->start_time = 0;
	while (devices) {
		sensor->devices_num++;
		OneWire_GetFullROM(&sensor->one_wire, sensor->rom[sensor->devices_num - 1]);
		devices = OneWire_Next(&sensor->one_wire);
	}
	return sensor->devices_num;
}

uint8_t ds18b20_start_all(DS18B20 *sensor)
{
	OneWire_Reset(&sensor->one_wire);
	OneWire_WriteByte(&sensor->one_wire, DS18B20_CMD_SKIP_ROM);
	OneWire_WriteByte(&sensor->one_wire, DS18B20_CMD_CONVERT_T);
	return 1;
}

uint8_t ds18b20_start_one(DS18B20 *sensor, uint8_t id)
{
	OneWire_Reset(&sensor->one_wire);
	OneWire_SelectWithPointer(&sensor->one_wire, sensor->rom[id]);
	OneWire_WriteByte(&sensor->one_wire, DS18B20_CMD_CONVERT_T);
	return 1;
}

uint8_t ds18b20_read(DS18B20 *sensor, uint8_t id, float *res)
{
	int i;
	uint8_t data[9];
	uint8_t crc;
	/* Wait for the end of the conversion */
	while(!OneWire_ReadBit(&sensor->one_wire));
	OneWire_Reset(&sensor->one_wire);
	OneWire_SelectWithPointer(&sensor->one_wire, sensor->rom[id]);
	OneWire_WriteByte(&sensor->one_wire, DS18B20_CMD_READ_SCRATCHPAD);
	for(i = 0; i < 9; ++i) {
		data[i] = OneWire_ReadByte(&sensor->one_wire);
	}
	crc = OneWire_CRC8(data, 8);
	if(crc != data[8]) {
		return 0;
	}
	uint32_t temperature = data[0] | ((uint32_t)data[1] << 8);
	*res = (temperature >> 4) | (((temperature >> 8) & 0x07) << 4);
	*res += ((temperature & 0x0F) * 0.0625f);
	return 1;
}

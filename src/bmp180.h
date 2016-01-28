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

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include <stdbool.h>

#define BMP180_ADDRESS        0x77
#define BMP180_CHIP_ID        0xD0

#define BMP180_AC1            0xAA
#define BMP180_AC2            0xAC
#define BMP180_AC3            0xAE
#define BMP180_AC4            0xB0
#define BMP180_AC5            0xB2
#define BMP180_AC6            0xB4
#define BMP180_B1             0xB6
#define BMP180_B2             0xB8
#define BMP180_MB             0xBA
#define BMP180_MC             0xBC
#define BMP180_MD             0xBE
#define BMP180_CONTROL        0xF4
#define BMP180_CONTROL_OUTPUT 0xF6

// BMP180 Modes
#define BMP180_MODE_ULTRA_LOW_POWER    0
#define BMP180_MODE_STANDARD           1
#define BMP180_MODE_HIGHRES            2
#define BMP180_MODE_ULTRA_HIGHRES      3

// Control register
#define BMP180_READ_TEMPERATURE        0x2E
#define BMP180_READ_PRESSURE           0x34
// Mean Sea Level Pressure = 1013.25 hPA
#define BMP180_MSLP                    101325.0f

typedef struct
{
	int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
	uint16_t AC4, AC5, AC6;
	int32_t B5, UT, UP, Pressure0;
	uint32_t i2c;
	uint32_t conv_start_time;
	uint8_t oss;

	int32_t pressure, temperature;
	float altitude;
} BMP180;

extern void bmp180_setup(BMP180 *sensor, uint32_t i2c, uint8_t oss);
extern void bmp180_start_conv(BMP180 *sensor);
extern void bmp180_finish_conv(BMP180 *sensor);
extern bool bmp180_is_conversion_finished(BMP180 *sensor);

#endif

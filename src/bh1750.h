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


#ifndef BH1750_H
#define BH1750_H

#include <stdint.h>
#include <stdbool.h>

#define BH1750_GPIO GPIOE
#define BH1750_GPIO_RCC RCC_GPIOE
#define BH1750_SENSORS_NUM 3
#define BH1750_SENSOR0 GPIO10
#define BH1750_SENSOR1 GPIO11
#define BH1750_SENSOR2 GPIO12
#define BH1750_SENSOR3 GPIO13
#define BH1750_ADDRESS        0x5C
#define BH1750_OP_POWER_ON    0x01
#define BH1750_CONT_H_MODE 0x10

typedef struct {
	uint32_t i2c;
	uint8_t mode;

	uint16_t illumination[4];
} BH1750;

extern void bh1750_setup(BH1750 *sensor, uint32_t i2c);
extern uint16_t bh1750_read(BH1750 *sensor, int id);

#endif

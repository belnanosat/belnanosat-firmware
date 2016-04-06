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

#include "bh1750.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "utils.h"
#include "i2c.h"

static uint16_t gpios_map[] = {
	BH1750_SENSOR0,
	BH1750_SENSOR1,
	BH1750_SENSOR2,
	BH1750_SENSOR3,
};

void bh1750_setup(BH1750 *sensor, uint32_t i2c) {
	int i;
	sensor->i2c = i2c;
	sensor->mode = BH1750_CONT_H_MODE;

	rcc_periph_clock_enable(BH1750_GPIO_RCC);
	gpio_mode_setup(BH1750_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
	                BH1750_SENSOR0 | BH1750_SENSOR1
	                | BH1750_SENSOR2 | BH1750_SENSOR3);
	gpio_clear(BH1750_GPIO, BH1750_SENSOR0);
	gpio_clear(BH1750_GPIO, BH1750_SENSOR1);
	gpio_clear(BH1750_GPIO, BH1750_SENSOR2);
	gpio_clear(BH1750_GPIO, BH1750_SENSOR3);

	// Turn on all devices at set them up for a continuous measurement
	for (i = 0; i < BH1750_SENSORS_NUM; ++i) {
		gpio_set(BH1750_GPIO, gpios_map[i]);
		i2c_write_byte_raw(sensor->i2c, BH1750_ADDRESS, BH1750_OP_POWER_ON);
		i2c_write_byte_raw(sensor->i2c, BH1750_ADDRESS, BH1750_CONT_H_MODE);
		gpio_clear(BH1750_GPIO, gpios_map[i]);
		msleep(10);
	}
	sensor->conv_start_time = get_time_ms();
}

uint16_t bh1750_read(BH1750 *sensor, int id) {
	gpio_set(BH1750_GPIO, gpios_map[id]);
	msleep(1);
	uint16_t res = i2c_read_word_raw(sensor->i2c, BH1750_ADDRESS);
	gpio_clear(BH1750_GPIO, gpios_map[id]);
	msleep(1);
	return res;
}

bool bh1750_is_conversion_finished(BH1750 *sensor) {
	return get_time_since(sensor->conv_start_time) >= 400;
}

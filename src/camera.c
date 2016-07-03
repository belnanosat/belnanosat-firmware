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

#include "camera.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "utils.h"

#define MODE_OFF 0
#define MODE_VIDEO_STANDBY 1
#define MODE_PHOTO_STANDBY 2
#define MODE_SOUND_STANDBY 3
#define MODE_VIDEO_RECORDING 4
#define MODE_SOUND_RECORDING 5

static uint8_t camera_mode = MODE_OFF;

void camera_setup() {
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(CAMERA_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CAMERA_GPIO_PIN | CAMERA_GPIO_PIN2);
}

void camera_power(uint8_t state) {
	if (state && camera_mode == MODE_OFF) {
		gpio_set(CAMERA_GPIO_PORT, CAMERA_GPIO_PIN);
		msleep(3000);
		gpio_clear(CAMERA_GPIO_PORT, CAMERA_GPIO_PIN);
		camera_mode = MODE_VIDEO_STANDBY;
	}
}

void camera_photo() {
}

void camera_start_video() {
	if (camera_mode == MODE_VIDEO_STANDBY) {
		gpio_set(CAMERA_GPIO_PORT, CAMERA_GPIO_PIN);
		msleep(1000);
		gpio_clear(CAMERA_GPIO_PORT, CAMERA_GPIO_PIN);
		camera_mode = MODE_VIDEO_RECORDING;
	}
}

void camera_stop_video() {
	if (camera_mode == MODE_VIDEO_RECORDING) {
		gpio_set(CAMERA_GPIO_PORT, CAMERA_GPIO_PIN);
//		msleep(500);
//		gpio_clear(CAMERA_GPIO_PORT, CAMERA_GPIO_PIN);
		camera_mode = MODE_VIDEO_RECORDING;
	}
}

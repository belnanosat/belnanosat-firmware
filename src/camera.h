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

#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

#define CAMERA_GPIO_RCC GPIOE_RCC
#define CAMERA_GPIO_PORT GPIOE
#define CAMERA_GPIO_PIN GPIO5
#define CAMERA_GPIO_PIN2 GPIO1

extern void camera_setup(void);
extern void camera_power(uint8_t state);
extern void camera_photo(void);
extern void camera_start_video(void);
extern void camera_stop_video(void);

#endif

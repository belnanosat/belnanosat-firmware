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

#ifndef MLX90614_H
#define MLX90614_H

#include <stdint.h>

extern void mlx90614_disable_pwm(void);
extern void mlx90614_setup(void);
//extern int mlx90614_TestConnection(void);
extern uint16_t mlx90614_read_ambient_temp(void);
extern uint16_t mlx90614_read_object_temp(void);
extern void mlx90614_set_address(uint8_t address);

#endif

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

#ifndef SMBUS_H
#define SMBUS_H

#include <inttypes.h>

#define SMBUS_NAME             I2C1

extern void smbus_setup(void);
extern void smbus_write_word(uint8_t device_address, uint8_t reg_address, uint16_t data);
extern uint16_t smbus_read_word(uint8_t device_address, uint8_t reg_address);
extern uint8_t smbus_crc8(uint32_t data);

#endif

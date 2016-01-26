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

#ifndef EEPROM_H
#define EEPROM_H

#define EEPROM_ADDRESS 0x50
#define EEPROM_PAGE_SIZE 64
#define EEPROM_PAGES_NUM 256
//#define EEPROM_OVERWRITE

#include <stdint.h>

typedef struct {
	uint32_t i2c;
	int cur_page_id;
	uint8_t cur_page_shift;
} EEPROM;

extern void EEPROM_setup(EEPROM *eeprom, uint32_t i2c);
/*
 * Maximum length of a message is 255 bytes
 */
extern void EEPROM_write(EEPROM *eeprom, uint8_t *buffer, uint8_t n);
extern void EEPROM_read(EEPROM *eeprom, uint16_t start_address,
                        uint8_t *buffer, int n);
/*
 * Complete erase takes about 2.8s
 */
extern void EEPROM_erase(EEPROM *eeprom);

#endif

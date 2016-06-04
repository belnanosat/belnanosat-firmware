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

#include "stuffer_fileid.h"
#include "stuffer.h"

int unstuff_data(uint8_t *input_data, int length, uint8_t *output_data) {
	while (*input_data) {
		uint8_t *cur_data = input_data + 1;
		int block_len = *input_data;
		input_data += block_len;
		while (cur_data < input_data) {
			*output_data++ = *cur_data++;
		}
		if (*input_data) {
			if (block_len < 0xFF) {
			    *output_data++ = 0;
			} else {
				--length;
			}
		}
	}
	return length - 2;
}

int stuff_data(uint8_t *input_data, int length, uint8_t *output_data) {
	uint8_t *input_data_end = input_data + length;
	uint8_t *block_start = output_data++;
	uint8_t cur_code = 0x01;
	while (input_data < input_data_end) {
		if (*input_data == 0) {
			*block_start = cur_code;
			block_start = output_data++;
			cur_code = 0x01;
		} else {
			*(output_data++) = *input_data;
			++cur_code;
			if (cur_code == 0xFF) {
				++length;
				*block_start = cur_code;
				block_start = output_data++;
				cur_code = 0x01;
			}
		}
		++input_data;
	}
	*block_start++ = cur_code;
	*output_data = 0;
	return length + 2;
}

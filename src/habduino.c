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

#include "habduino.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/*
 * Format of the Habduino packets (MSB):
 * int32_t (4 bytes) - latitude
 * int32_t (4 bytes) - longitude
 * int32_t (4 bytes) - altitude
 * 1 byte - number of satellites
 */
#define HABDUINO_PACKET_SIZE (4 * 3 + 1)

int32_t habduino_latitude, habduino_longitude, habduino_altitude;
uint8_t habduino_sats_num;
uint8_t habduino_has_new_data = 0;
uint8_t habduino_has_pending_packet_request = 0;

static uint16_t cur_id = 0;
static uint8_t buf[HABDUINO_PACKET_SIZE];

inline int32_t int32_from_binary(uint8_t *b) {
	// We use unsigned value to prevent problems with sign extension
	uint32_t res = 0;
	res |= ((uint32_t)b[0] << 24);
	res |= ((uint32_t)b[1] << 16);
	res |= ((uint32_t)b[2] << 8);
	res |= ((uint32_t)b[3] << 0);
	return res;
}

void usart2_isr(void) {
	if (usart_get_interrupt_source(USART2, USART_SR_RXNE)) {
		buf[cur_id++] = usart_recv(USART2);
		if (cur_id == HABDUINO_PACKET_SIZE) {
			gpio_toggle(GPIOD, GPIO12);
			habduino_latitude = int32_from_binary(buf);
			habduino_longitude = int32_from_binary(buf + 4);
			habduino_altitude = int32_from_binary(buf + 8);
			habduino_sats_num = buf[12];
			habduino_has_new_data = 1;
			habduino_has_pending_packet_request = 1;
			cur_id = 0;
		}
	}
}

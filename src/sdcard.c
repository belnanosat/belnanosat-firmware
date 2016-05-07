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

#include "sdcard.h"

#include <stdio.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "spi.h"

#define DUMMY_CLOCK_BYTES_NUM        10
#define DUMMY_BYTE                   0xFF
#define SDC_V2_CMD8_CORRECT_RESPONSE 0x01AA
#define CARD_INIT_TIMEOUT            0xFFF
#define ACMD41_ARGUMENT              (1 << 30)
#define CMD_RESPONSE_TIME            10      /* Also called N_{cr} */

#define CMD0_CRC                     0x95
#define CMD8_CRC                     0x87

#define R1_IDLE_STATE                (1 << 0)
#define R1_ERASE_RESET               (1 << 1)
#define R1_ILLEGAL_COMMAND           (1 << 2)
#define R1_COM_CRC_ERROR             (1 << 3)
#define R1_ERASE_SEQ_ERROR           (1 << 4)
#define R1_ADDRESS_ERROR             (1 << 5)
#define R1_PARAMETER_ERROR           (1 << 6)

#define card_enable()               gpio_clear(SPI_PORT, SPI_NSS);
#define card_disable()              gpio_set(SPI_PORT, SPI_NSS);

static uint8_t send_command(uint8_t cmd, uint32_t arg, uint8_t crc);
static uint8_t send_command_without_disabling(uint8_t cmd, uint32_t arg, uint8_t crc);

/*
 * For now supports SDC v2 only
 */
uint8_t sdcard_setup(void) {
	uint8_t r1, succesfull_init;
	uint8_t buff[4] = {0};
	uint16_t i;

	/* To initialize a card we need to send at least 74 dummy clock ticks */
	for (i = 0; i < DUMMY_CLOCK_BYTES_NUM; ++i) {
		/*
		 * Fore some reason microSD card doesn't initialize correctly
		 * if we use spi_write_and_read() instead. Perhaps that's
		 * caused by an increased delay between output data packets.
		 */
		spi_send(SPI_ID, DUMMY_BYTE);
	}

	/* Start send CMD0 till return 0x01 means in IDLE state */
	for (i = 0; i < 0xFF; ++i) {
		r1 = send_command(0, 0, CMD0_CRC);
		if (r1 == R1_IDLE_STATE) {
			break;
		}
	}
	/* Timeout return */
	if (r1 != R1_IDLE_STATE) {
		return 0;
	}

	r1 = send_command_without_disabling(8, 0x1AA, CMD8_CRC);
	if (r1 != R1_IDLE_STATE) {
		return 0;
	}
	buff[0] = spi_write_and_read(DUMMY_BYTE);
	buff[1] = spi_write_and_read(DUMMY_BYTE);
	buff[2] = spi_write_and_read(DUMMY_BYTE);
	buff[3] = spi_write_and_read(DUMMY_BYTE);
	card_disable();
	spi_write_and_read(DUMMY_BYTE);

	/*
	 * Last two bytes should contain 0x01AA - that means that that the card is
	 * SDC v2 and it can work at voltage range of 2.7 to 3.6 volts.
	 */
	if ((((uint16_t)buff[2] << 8) | buff[3]) != SDC_V2_CMD8_CORRECT_RESPONSE) {
		return 0;
	}

	succesfull_init = 0;
	for (i = 0; i < CARD_INIT_TIMEOUT; ++i) {
		r1 = send_command(55, 0, 0);
		if (r1 != 0x01) {
			return 0;
		}
		r1 = send_command(41, ACMD41_ARGUMENT, 0);
		if (r1 == 0x00) {
			succesfull_init = 1;
			break;
		}
	}

	return succesfull_init;
}

uint8_t send_command(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t r1 = send_command_without_disabling(cmd, arg, crc);

	/* Chip disable and dummy byte */
	card_disable();
	spi_write_and_read(DUMMY_BYTE);

	return r1;
}

uint8_t send_command_without_disabling(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t r1;
	uint8_t i;

	/* Dummy byte and chip enable */
	spi_write_and_read(DUMMY_BYTE);
	card_enable();

	/* Command, argument and crc */
	spi_write_and_read(cmd | 0x40);
	spi_write_and_read(arg >> 24);
	spi_write_and_read(arg >> 16);
	spi_write_and_read(arg >> 8);
	spi_write_and_read(arg);
	spi_write_and_read(crc);

	for (i = 0; i < 200; ++i) {
		r1 = spi_write_and_read(DUMMY_BYTE);
		if (r1 != 0xFF) {
			break;
		}
	}

	return r1;
}

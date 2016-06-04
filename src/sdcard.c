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

#include "sdcard_fileid.h"
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


#define STATUS_DATA_ACCEPTED        0b010

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
		} else if (r1 != 0x01) {
			break;
		}
	}

	if (r1 != 0x00) {
//		printf("CMD55 error occured: %d\r\n", (int)r1);
		return 0;
	}

	if (!succesfull_init) {
		for (i = 0; i < CARD_INIT_TIMEOUT; ++i) {
			r1 = send_command(1, 0, 0);
			if (r1 == 0x00) {
				succesfull_init = 1;
				break;
			}
		}
	}

	/* r1 = send_command_without_disabling(58, 0, 0); */
	/* printf("CMD58 perhaps works!\r\n"); */
	/* if (r1 != 0) { */
	/* 	printf("%d\r\n", (int)r1); */
	/* 	return 0; */
	/* } */
	/* printf("CMD58 works!\r\n"); */
	/* buff[0] = spi_write_and_read(DUMMY_BYTE); */
	/* buff[1] = spi_write_and_read(DUMMY_BYTE); */
	/* buff[2] = spi_write_and_read(DUMMY_BYTE); */
	/* buff[3] = spi_write_and_read(DUMMY_BYTE); */
	/* card_disable(); */
	/* spi_write_and_read(DUMMY_BYTE); */

	/* printf("Flag: %d\r\n", (int)buff[0]); */

	r1 = send_command(16, 0x200, 0);

//	printf("Response: %d\r\n", (int)r1);

	return succesfull_init;
}

void sdcard_single_block_read(uint32_t address, uint8_t *block) {
	uint8_t r1 = send_command_without_disabling(17, address, 0);
	uint32_t i;

	if (r1 != 0x00) {
//		printf("Read Error1: %d\r\n", (int)r1);
		return;
	}

	for (i = 0; i < 4000; ++i) {
		r1 = spi_write_and_read(DUMMY_BYTE);
		if (r1 == 0xFE) {
			break;
		}
	}

	if (r1 != 0xFE) {
//		printf("Read Error2: %d\r\n", (int)r1);
		return;
	}

	for (i = 0; i < 512; ++i) {
		block[i] = spi_write_and_read(DUMMY_BYTE);
	}

	// Read CRC
	uint16_t crc = 0;
	crc = spi_write_and_read(DUMMY_BYTE);
	crc <<= 8;
	crc |= spi_write_and_read(DUMMY_BYTE);

	card_disable();
	spi_write_and_read(DUMMY_BYTE);
}

void sdcard_single_block_write(uint32_t address, uint8_t *block) {
	uint8_t r1 = send_command_without_disabling(24, address, 0);
	uint32_t i;

	if (r1 != 0x00) {
//		printf("Write Error1: %d\r\n", (int)r1);
		return;
	}

	// Wait for 3 bytes before sending data packet
	spi_write_and_read(DUMMY_BYTE);
	spi_write_and_read(DUMMY_BYTE);
	spi_write_and_read(DUMMY_BYTE);

	// Start data packet transmission
	spi_write_and_read(0xFE);
	for (i = 0; i < 512; ++i) {
		spi_write_and_read(block[i]);
	}

	// Card doesn't check CRC, so we can send some constant value
	spi_write_and_read(DUMMY_BYTE);
	spi_write_and_read(DUMMY_BYTE);

	// We should wait for a Data Response byte
	i = 0;
	while ((r1 = spi_write_and_read(DUMMY_BYTE)) == 0xFF && i < 10000) ++i;

	if (i == 10000) {
		card_disable();
		spi_write_and_read(0xFF);
		return;
	}

	// Status "010" means that data is accepted
	if (((r1 & 0xE) >> 1) != STATUS_DATA_ACCEPTED) {
//		printf("Write error2: %x\r\n", r1);
		return;
	}

	// During the writing operation sd card holds data line low
	i = 0;
	while (spi_write_and_read(DUMMY_BYTE) != 0xFF) ++i;

	card_disable();
	spi_write_and_read(DUMMY_BYTE);

	r1 = send_command_without_disabling(13, 0, 0);
	uint8_t r2 = spi_write_and_read(DUMMY_BYTE);

	if (r1 || r2) {
//		printf("CMD13 Error!\r\n");
	}
	card_disable();
	spi_write_and_read(DUMMY_BYTE);
}

int write_started = 0;
int blockw_id = 0;

void sdcard_multiple_block_write(uint32_t address, uint8_t *block) {
	if (write_started) {
		uint8_t r1;
		uint32_t i;
		// Start data packet transmission
		spi_write_and_read(0xFE);
		for (i = 0; i < 512; ++i) {
			spi_write_and_read(block[i]);
		}

		// Card doesn't check CRC, so we can send some constant value
		spi_write_and_read(DUMMY_BYTE);
		spi_write_and_read(DUMMY_BYTE);

		// We should wait for a Data Response byte
		i = 0;
		while ((r1 = spi_write_and_read(DUMMY_BYTE)) == 0xFF && i < 100000);

		if (i == 100000) {
			card_disable();
			spi_write_and_read(0xFF);
			return;
		}

		// Status "010" means that data is accepted
		/* if (((r1 & 0xE) >> 1) != STATUS_DATA_ACCEPTED) { */
		/* 	printf("Write error3: %x\r\n", r1); */
		/* 	return; */
		/* } */

		// During the writing operation sd card holds data line low
		i = 0;
		while (spi_write_and_read(DUMMY_BYTE) != 0xFF && i < 1000) ++i;

//		printf("%d One more write took: %d\r\n", blockw_id++, (int)i);
	} else {
		// set earse block count
		uint8_t r1 = send_command(55, 0, 0);
		r1 = send_command(23, 0x10000, 0);

		write_started = 1;
		r1 = send_command_without_disabling(25, address, 0);
		uint32_t i;

		if (r1 != 0x00) {
//			printf("Write Error1: %d\r\n", (int)r1);
			return;
		}

		// Wait for 3 bytes before sending data packet
		spi_write_and_read(DUMMY_BYTE);
		spi_write_and_read(DUMMY_BYTE);
		spi_write_and_read(DUMMY_BYTE);


		// Start data packet transmission
		spi_write_and_read(0xFE);
		for (i = 0; i < 512; ++i) {
			spi_write_and_read(block[i]);
		}

		// Card doesn't check CRC, so we can send some constant value
		spi_write_and_read(DUMMY_BYTE);
		spi_write_and_read(DUMMY_BYTE);

		// We should wait for a Data Response byte
		i = 0;
		while ((r1 = spi_write_and_read(DUMMY_BYTE)) == 0xFF && i < 10000) ++i;

		if (i == 10000) {
			card_disable();
			spi_write_and_read(0xFF);
			return;
		}

		// During the writing operation sd card holds data line low
		i = 0;
		while (spi_write_and_read(DUMMY_BYTE) != 0xFF) ++i;
	}
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

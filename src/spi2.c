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

#include "spi2.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

void spi2_setup() {
	// NSS pins seems to be optional and conflicts with JTAG
	uint32_t cr_tmp;
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI3);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
	                GPIO5);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
	                GPIO3 | GPIO4);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
	                        GPIO3 | GPIO5);
	gpio_set_af(GPIOB, GPIO_AF6,
	            GPIO3 | GPIO4 | GPIO5);
	gpio_set_af(GPIOA, GPIO_AF6,
	            GPIO15);

	spi_reset(SPI2_ID);

	spi_init_master(SPI2_ID, SPI_CR1_BAUDRATE_FPCLK_DIV_64, 0,//SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
	                0,//SPI_CR1_CPHA_CLK_TRANSITION_1,
	                SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_set_full_duplex_mode(SPI2_ID);

	spi_enable_software_slave_management(SPI2_ID);
	spi_set_nss_high(SPI2_ID);
	SPI_CR2(SPI2_ID) |= SPI_CR2_FRF;
	gpio_set(GPIOA, GPIO15);
	spi_disable_crc(SPI2_ID);
	spi_enable(SPI2_ID);
}

uint8_t spi2_write_and_read(uint8_t data) {
	uint16_t i = 0;
	while (!(SPI_SR(SPI2_ID) & SPI_SR_TXE) && i < 10000) ++i;
	if (i == 10000) {
		return 0xFF;
	}

	SPI_DR(SPI2_ID) = data;

	i = 0;
	while (!(SPI_SR(SPI2_ID) & SPI_SR_RXNE) && i < 10000) ++i;
	if (i == 10000) {
		return 0xFF;
	} else {
		return SPI_DR(SPI2_ID);
	}
}

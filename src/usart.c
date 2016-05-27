/*
 * This file is part of belnanosat project.
 *
 * Copyright (C) 2015 Uladzislau Paulovich <selatnick@gmail.com>
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

#include "usart.h"

#include <errno.h>
#include <unistd.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

extern int _write(int file, char *ptr, int len);
extern int _read(int file, char *buf, int len);
/*
 * USART1 pins:
 * RX - PA10, TX - PA9 - works only if you remove PA9-VBUS jumper!
 * USART2 pins:
 * RX - PA3, TX - PA2 - works!
 * USART3 pins:
 * RX - PB11, TX - PB10 - works!
 */

void usart_setup(void) {
	/* Enable the USART2 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);

	/* Enable all required USART modules */
	rcc_periph_clock_enable(USART_RCC_PORT);
	rcc_periph_clock_enable(USART_RCC_ID);

	/* Setup GPIO pins for USART transmit. */
	gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_RX);
	gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX);
//	gpio_set_output_options(USART_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, USART_TX);

	/* Setup USART TX pin as alternate function. */
	gpio_set_af(USART_PORT, GPIO_AF7, USART_RX);
	gpio_set_af(USART_PORT, GPIO_AF7, USART_TX);

	/* Setup USART parameters. */
	usart_set_baudrate(USART_ID, USART_BAUD_RATE);
	usart_set_databits(USART_ID, 8);
	usart_set_stopbits(USART_ID, USART_STOPBITS_1);
	usart_set_mode(USART_ID, USART_MODE_TX_RX);
	usart_set_parity(USART_ID, USART_PARITY_NONE);
	usart_set_flow_control(USART_ID, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(USART_ID);

	/* Finally enable the USART. */
	usart_enable(USART_ID);
}

/**
 * Use USART_ID as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			usart_send_blocking(USART_ID, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

int _read(int file, char *buf, int len)
{
	if (STDIN_FILENO == file) {
		buf[0] = usart_recv_blocking(USART_ID);
#ifdef USART_AUTO_ECHO
		putchar(buf[0]);
		fflush(stdout);
#endif
		return 1;
	} else {
		errno = EBADF;
		return -1;
	}
}

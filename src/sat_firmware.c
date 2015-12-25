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

#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define USART_ID USART3
#define USART_RCC_ID RCC_USART3
#define USART_PORT GPIOB
#define USART_RCC_PORT RCC_GPIOB
#define USART_TX GPIO10
#define USART_RX GPIO9

volatile uint32_t system_millis;

int _write(int file, char *ptr, int len);

void sys_tick_handler(void)
{
	system_millis++;
}

static void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(168000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	rcc_periph_clock_enable(USART_RCC_PORT);
	rcc_periph_clock_enable(RCC_GPIOD);

	rcc_periph_clock_enable(USART_RCC_ID);
}


/*
 * USART1 pins:
 * RX - PA10, TX - PA9 - works only if you remove PA9-VBUS jumper!
 * USART2 pins:
 * RX - PA1, TX - PA2 - works!
 * USART3 pins:
 * RX - PB11, TX - PB10 - works!
 */
static void usart_setup(void)
{
	/* Setup USART parameters. */
	usart_set_baudrate(USART_ID, 38400);
	usart_set_databits(USART_ID, 8);
	usart_set_stopbits(USART_ID, USART_STOPBITS_1);
	usart_set_mode(USART_ID, USART_MODE_TX);
	usart_set_parity(USART_ID, USART_PARITY_NONE);
	usart_set_flow_control(USART_ID, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_ID);
}

static void gpio_setup(void)
{
	/* Set GPIO11-15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
	                GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);

	/* Setup GPIO pins for USART transmit. */
	gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX);

	/* Setup USART TX pin as alternate function. */
	gpio_set_af(USART_PORT, GPIO_AF7, USART_TX);
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
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_ID, '\r');
			}
			usart_send_blocking(USART_ID, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

int main(void)
{
	clock_setup();
	gpio_setup();
	systick_setup();
	usart_setup();

	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(GPIOD, GPIO12 | GPIO14);

	/* Blink the LEDs (PD12, PD13, PD14 and PD15) on the board. */
	while (1) {
		gpio_toggle(GPIOD, GPIO12);
		printf("Hello from stm32!\n");
		msleep(1000);
	}

	return 0;
}

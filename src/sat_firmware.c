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

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "usart.h"

volatile uint32_t system_millis;

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

	rcc_periph_clock_enable(RCC_GPIOD);
}

static void gpio_setup(void)
{
	/* Set GPIO11-15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
	                GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
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

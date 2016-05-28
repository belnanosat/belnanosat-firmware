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

#include "radiation_sensor.h"

#include <stdbool.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

#include "utils.h"

uint32_t radiation_sensor_cnt = 0;
uint64_t radiation_sensor_window_start_time;
uint32_t radiation_sensor_pulses[RADIATION_SENSOR_MAX_TIMESTAMPS];

void exti0_isr(void) {
	exti_reset_request(EXTI0);
	if (radiation_sensor_cnt < RADIATION_SENSOR_MAX_TIMESTAMPS) {
		radiation_sensor_pulses[radiation_sensor_cnt] =
			get_time_us() - radiation_sensor_window_start_time;
	}
	++radiation_sensor_cnt;
}

void exti1_isr(void) {
	exti_reset_request(EXTI1);
	if (radiation_sensor_cnt == 0) return;
	uint32_t i = radiation_sensor_cnt - 1;
	uint32_t cur_time = get_time_us() - radiation_sensor_window_start_time;
	while (i > 0 && (cur_time - radiation_sensor_pulses[i]) < RADIATION_SENSOR_NOISE_TIMEOUT) {
		--i;
	}
	radiation_sensor_cnt = i;
}

void radiation_sensor_setup(void) {
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	nvic_enable_irq(NVIC_EXTI1_IRQ);

	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);

	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);

	exti_select_source(EXTI1, GPIOA);
	exti_set_trigger(EXTI1, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI1);

	radiation_sensor_window_start_time = get_time_us();
}

/*
 * There's a really small possibility of getting an incorrect
 * pulse offset time (or missing one pulse) in case interrupt
 * occurs after setting number of pulses to 0. But it should
 * be really rare, so we don't handle this case.
 */
void radiation_sensor_clear_data(void) {
	radiation_sensor_cnt = 0;
	radiation_sensor_window_start_time = get_time_us();
}

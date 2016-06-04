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

#include "adc_fileid.h"
#include "adc.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

static void dma_setup(uint16_t *data, uint16_t len);

void adc_setup(adc_channel channels[], uint8_t channel_ids[],
               uint16_t *data, uint16_t len) {
	uint16_t i;
	for (i = 0; i < len; ++i) {
		if (channels[i].rcc_port == 0) {
			if (channels[i].gpio == 0) {
				adc_enable_temperature_sensor();
			}
		} else {
			rcc_periph_clock_enable(channels[i].rcc_port);
			gpio_mode_setup(channels[i].gpio_port, GPIO_MODE_ANALOG,
			                GPIO_PUPD_NONE, channels[i].gpio);
		}
	}

	adc_off(ADC1);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_enable_scan_mode(ADC1);
	adc_set_continuous_conversion_mode(ADC1); // May cause overrun
	adc_set_regular_sequence(ADC1, len, channel_ids);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_480CYC);
	adc_enable_dma(ADC1);
	adc_set_dma_continue(ADC1);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
	adc_set_right_aligned(ADC1);

	dma_setup(data, len);

	adc_power_on(ADC1);
	adc_start_conversion_regular(ADC1);
}

void dma_setup(uint16_t *data, uint16_t len) {
	rcc_periph_clock_enable(RCC_DMA2);
	dma_stream_reset(DMA2, DMA_STREAM4);
	dma_disable_stream(DMA2, DMA_STREAM4);
	dma_set_peripheral_address(DMA2, DMA_STREAM4, (uint32_t)&(ADC_DR(ADC1)));
	dma_set_memory_address(DMA2, DMA_STREAM4, (uint32_t)data);
	dma_set_number_of_data(DMA2, DMA_STREAM4, len);
	dma_enable_circular_mode(DMA2, DMA_STREAM4);
	dma_disable_double_buffer_mode(DMA2, DMA_STREAM4);
	dma_channel_select(DMA2, DMA_STREAM4, DMA_SxCR_CHSEL_0);
	dma_set_priority(DMA2, DMA_STREAM4, DMA_SxCR_PL_LOW);
	dma_enable_direct_mode(DMA2, DMA_STREAM4);
	dma_set_transfer_mode(DMA2, DMA_STREAM4, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	dma_enable_memory_increment_mode(DMA2, DMA_STREAM4);
	dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM4);
	dma_set_memory_size(DMA2, DMA_STREAM4, DMA_SxCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA2, DMA_STREAM4, DMA_SxCR_PSIZE_16BIT);
	dma_enable_transfer_error_interrupt(DMA2, DMA_STREAM4);
	dma_enable_stream(DMA2, DMA_STREAM4);
}

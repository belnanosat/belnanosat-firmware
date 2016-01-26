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

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <pb_encode.h>

#include "proto/telemetry.pb.h"

#include "usart.h"
#include "adc.h"
#include "i2c.h"
#include "bmp180.h"
#include "utils.h"
#include "eeprom.h"
#include "ds18b20.h"
#include "smbus.h"

//#define LOG_TO_EEPROM

/* Set up a timer to create 1mS ticks. */
static void systick_setup(void)
{
	/* clock rate / 1000_000 to get 1uS interrupt rate */
	systick_set_reload(168);
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

static void getline(char* buf, int maxlen) {
	int curlen = 0;
	while (curlen < maxlen) {
		buf[curlen] = getchar();
		if (buf[curlen] == '\r') {
			buf[curlen] = 0;
			break;
		}
		++curlen;
	}
}

int main(void)
{
	BMP180 bmp180_sensor;
	EEPROM eeprom;
	DS18B20 ds18b20;
	clock_setup();
	gpio_setup();
	systick_setup();
	usart_setup();
//	adc_setup();
	i2c_setup();
	smbus_setup();

	// Wait for initialization of all external sensors
	msleep(100);

	BMP180_setup(&bmp180_sensor, I2C2, BMP180_MODE_ULTRA_HIGHRES);
	EEPROM_setup(&eeprom, I2C2);
	msleep(1000);

	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(GPIOD, GPIO12 | GPIO14);

	/* Blink the LEDs (PD12, PD13, PD14 and PD15) on the board. */
	uint8_t buffer[128];
//	char *ptr = buffer;
	int packet_id = 0;

	volatile uint8_t devices_num = ds18b20_setup(&ds18b20, GPIOE, GPIO3);

//	volatile uint16_t temp = smbus_read_word(0, 0x06);

	TelemetryPacket packet = TelemetryPacket_init_zero;
	while (1) {
		int i;
		uint8_t checksum;
		gpio_toggle(GPIOD, GPIO12);
		BMP180_start_conv(&bmp180_sensor);
		BMP180_finish_conv(&bmp180_sensor);


		for (i = 0; i < TelemetryPacket_size + 2; ++i) {
			buffer[i] = 0;
		}
		packet.packet_id = packet_id;
		packet.status = 0xFFFFFFFF;
		packet.altitude = bmp180_sensor.altitude;
		packet.pressure = bmp180_sensor.pressure;
		packet.bmp180_temperature = bmp180_sensor.temperature;


		pb_ostream_t stream = pb_ostream_from_buffer(buffer + 1, sizeof(buffer) - 1);
		bool status = pb_encode(&stream, TelemetryPacket_fields, &packet);

#ifdef LOG_TO_EEPROM
		EEPROM_write(&eeprom, buffer, stream.bytes_written);
#endif

		buffer[0] = stream.bytes_written;
//		printf("\n\r%d", stream.bytes_written);
		checksum = 0;
		for (i = 0; i <= stream.bytes_written; ++i) {
			checksum ^= buffer[i];
		}
		buffer[stream.bytes_written + 1] = checksum;

		for (i = 0; i < TelemetryPacket_size + 2; ++i) {
			putc(buffer[i], stdout);
		}

		/* printf("\n\r%d: stm32-user@satellite $ %d %d %d %d %f", ++packet_id, */
		/*        (int)status, */
		/*        (int)stream.bytes_written, */
		/*        (int)bmp180_sensor.temperature, (int)bmp180_sensor.pressure, */
		/*        bmp180_sensor.altitude); */
		/* val = i2c_read(); */
		/* printf("ok! %d\n", (int)val); */
		fflush(stdout);
		msleep(1000);
	}

	return 0;
}

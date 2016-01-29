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
#include "mpu6050.h"
#include "bh1750.h"

//#define LOG_TO_EEPROM
#define PACKET_DELAY_MS 500

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
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

static void process_ds18b20(DS18B20Bus *bus, TelemetryPacket *packet) {
	if (!ds18b20_is_conversion_finished(bus)) return;

	if (bus->devices[0].is_present) {
		packet->has_ds18b20_temperature1 = true;
		packet->ds18b20_temperature1 = ds18b20_read_raw(bus, 0);
	}

	if (bus->devices[1].is_present) {
		packet->has_ds18b20_temperature2 = true;
		packet->ds18b20_temperature2 = ds18b20_read_raw(bus, 1);
	}

	if (bus->devices[2].is_present) {
		packet->has_ds18b20_temperature3 = true;
		packet->ds18b20_temperature3 = ds18b20_read_raw(bus, 2);
	}

	if (bus->devices[3].is_present) {
		packet->has_ds18b20_temperature4 = true;
		packet->ds18b20_temperature4 = ds18b20_read_raw(bus, 3);
	}

	ds18b20_start_all(bus);
}

static void process_bmp180(BMP180 *sensor, TelemetryPacket *packet) {
	if (!bmp180_is_conversion_finished(sensor)) return;
	bmp180_finish_conv(sensor);

	packet->altitude = sensor->altitude;
	packet->has_altitude = true;
	packet->pressure = sensor->pressure;
	packet->has_pressure = true;
	packet->bmp180_temperature = sensor->temperature;
	packet->has_bmp180_temperature = true;

	bmp180_start_conv(sensor);
}

uint32_t last_mpu6050_conversion;

static void process_mpu6050(TelemetryPacket *packet) {
	if (get_time_since(last_mpu6050_conversion) < 100) return;

	int16_t ax, ay, az, gx, gy, gz;
	MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	packet->has_acceleration_x = true;
	packet->has_acceleration_y = true;
	packet->has_acceleration_z = true;
	packet->has_gyroscope_x = true;
	packet->has_gyroscope_y = true;
	packet->has_gyroscope_z = true;
	packet->acceleration_x = ax;
	packet->acceleration_y = ay;
	packet->acceleration_z = az;
	packet->gyroscope_x = gx;
	packet->gyroscope_y = gy;
	packet->gyroscope_z = gz;

	last_mpu6050_conversion = get_time_ms();
}

static void process_bh1750(BH1750 *sensor, TelemetryPacket *packet) {
	if (!bh1750_is_conversion_finished(sensor)) return;

	packet->has_sun_sensor1 = true;
	packet->has_sun_sensor2 = true;
	packet->has_sun_sensor3 = true;
	packet->has_sun_sensor4 = true;
	packet->sun_sensor1 = bh1750_read(sensor, 0);
	packet->sun_sensor2 = bh1750_read(sensor, 1);
	packet->sun_sensor3 = bh1750_read(sensor, 2);
	packet->sun_sensor4 = bh1750_read(sensor, 3);

	sensor->conv_start_time = get_time_ms();
}

// NOTE: This function assumees that length <= 255
static void stuff_data(uint8_t *input_data, int length, uint8_t *output_data) {
	uint8_t *input_data_end = input_data + length;
	uint8_t *block_start = output_data++;
	uint8_t cur_code = 0x01;
	while (input_data < input_data_end) {
		if (*input_data == 0) {
			*block_start++ = cur_code;
			block_start = output_data++;
			cur_code = 0x01;
		} else {
			*(output_data++) = *input_data;
			++cur_code;
		}
		++input_data;
	}
	*block_start++ = cur_code;
	*output_data = 0;
}

int main(void)
{
	BMP180 bmp180_sensor;
	EEPROM eeprom;
	DS18B20Bus ds18b20_bus;
	BH1750 bh1750;
	clock_setup();
	gpio_setup();
	systick_setup();
	usart_setup();
//	adc_setup();
	i2c_setup();
//	smbus_setup();
	MPU6050_initialize();
	last_mpu6050_conversion = get_time_ms();

	// Wait for initialization of all external sensors
	msleep(100);

	bmp180_setup(&bmp180_sensor, I2C2, BMP180_MODE_ULTRA_HIGHRES);
	bh1750_setup(&bh1750, I2C2);
//	EEPROM_setup(&eeprom, I2C2);
	msleep(1000);

	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(GPIOD, GPIO12);

	static uint8_t buffer1[256];
	static uint8_t buffer2[256];

	rcc_periph_clock_enable(RCC_GPIOE);
	volatile uint8_t devices_num = ds18b20_setup(&ds18b20_bus, GPIOE, GPIO3,
	                                             DS18B20_RESOLUTION_12_BITS);

//	volatile uint16_t temp = smbus_read_word(0, 0x06);

	TelemetryPacket packet = TelemetryPacket_init_zero;
	uint32_t last_packet_time = get_time_ms();
	uint32_t packet_id = 0;
	while (1) {
		gpio_toggle(GPIOD, GPIO12);

		process_bmp180(&bmp180_sensor, &packet);
		process_ds18b20(&ds18b20_bus, &packet);
		process_mpu6050(&packet);
		process_bh1750(&bh1750, &packet);

		/* Is it time to send a packet? */
		if (get_time_since(last_packet_time) > PACKET_DELAY_MS) {
			int i;
			uint8_t checksum;

			packet.packet_id = ++packet_id;
			packet.timestamp = get_time_ms();
			packet.status = 0xFFFFFFFF;

			pb_ostream_t stream = pb_ostream_from_buffer(buffer1, sizeof(buffer1));
			bool status = pb_encode(&stream, TelemetryPacket_fields, &packet);

			checksum = 0;
			for (i = 0; i < stream.bytes_written; ++i) {
				checksum ^= buffer1[i];
			}
			buffer1[stream.bytes_written] = checksum;

			stuff_data(buffer1, stream.bytes_written + 1, buffer2);

#ifdef LOG_TO_EEPROM
			EEPROM_write(&eeprom, buffer2, stream.bytes_written);
#endif

			for (i = 0; i < stream.bytes_written + 3; ++i) {
				putc(buffer2[i], stdout);
			}
			fflush(stdout);

			last_packet_time = get_time_ms();

			/* Clear packet */
			packet = (TelemetryPacket)TelemetryPacket_init_zero;
		}
	}

	return 0;
}

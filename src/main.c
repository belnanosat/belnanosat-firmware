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
#include <math.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/adc.h>
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
#include "habduino.h"
#include "smbus.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "bh1750.h"
#include "madgwick_ahrs.h"
#include "spi.h"
#include "sdcard.h"
#include "log.h"
#include "stuffer.h"

#define CHECK_SETUP(interface)  do{\
		printf("Setting up " #interface ".......");\
		if (interface ## _setup()) { \
			printf("ok!\r\n");\
		} else {\
			printf("failed!\r\n");\
		}\
	}while(0)


//#define LOG_TO_EEPROM
#define PACKET_DELAY_MS 100

float fax, fay, faz, fgx, fgy, fgz;
float fmx, fmy, fmz;
bool has_read_mpu6050 = false;
bool has_read_hmc5883l = false;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

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
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO15);
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

	packet->bmp180_altitude = sensor->altitude;
	packet->has_bmp180_altitude = true;
	packet->pressure = sensor->pressure;
	packet->has_pressure = true;
	packet->bmp180_temperature = sensor->temperature;
	packet->has_bmp180_temperature = true;

	bmp180_start_conv(sensor);
}

uint32_t last_mpu6050_conversion;
uint32_t last_hmc5883l_conversion;
uint32_t last_madgwick;
uint32_t last_adc_read;

static void process_mpu6050(TelemetryPacket *packet) {
	if (get_time_since_ms(last_mpu6050_conversion) < 10) return;

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

	gx -= gx_offset;
	gy -= gy_offset;
	gz -= gz_offset;
	fax = ax / 16384.0f;
	fay = ay / 16384.0f;
	faz = az / 16384.0f;
	float ratio = (32767.0 / 250.0f) * 180.0f / M_PI;
	fgx = gx / ratio;
	fgy = gy / ratio;
	fgz = gz / ratio;

	last_mpu6050_conversion = get_time_ms();
	has_read_mpu6050 = true;
}

static void process_hmc5883l(TelemetryPacket *packet) {
	if (get_time_since_ms(last_hmc5883l_conversion) < 20) return;

	int16_t mx, my, mz;
	HMC5883L_GetHeading(&mx, &my, &mz);

	packet->has_magnetometer_x = true;
	packet->has_magnetometer_y = true;
	packet->has_magnetometer_z = true;

	packet->magnetometer_x = mx;
	packet->magnetometer_y = my;
	packet->magnetometer_z = mz;

	float mratio = 0.92f / 1000.0f;
	fmx = mx * mratio;
	fmy = my * mratio;
	fmz = mz * mratio;

	last_hmc5883l_conversion = get_time_ms();
	has_read_hmc5883l = true;
}

static void process_madgwick(TelemetryPacket *packet) {
	if (get_time_since_ms(last_madgwick) < 10) return;

	if (!has_read_mpu6050 || !has_read_hmc5883l) return;

	MadgwickAHRSupdate(fgx, fgy, fgz, fax, fay, faz, fmx, fmy, fmz);

	packet->has_quaternion0 = true;
	packet->has_quaternion1 = true;
	packet->has_quaternion2 = true;
	packet->has_quaternion3 = true;
	packet->quaternion0 = q0;
	packet->quaternion1 = q1;
	packet->quaternion2 = q2;
	packet->quaternion3 = q3;
}

static void process_bh1750(BH1750 *sensor, TelemetryPacket *packet) {
	if (!bh1750_is_conversion_finished(sensor)) return;

	packet->has_sun_sensor1 = true;
	packet->has_sun_sensor2 = true;
	packet->has_sun_sensor3 = true;
//	packet->has_sun_sensor4 = true;
	packet->sun_sensor1 = bh1750_read(sensor, 0);
	packet->sun_sensor2 = bh1750_read(sensor, 2);
	packet->sun_sensor3 = bh1750_read(sensor, 3);
//	packet->sun_sensor4 = bh1750_read(sensor, 3);

	sensor->conv_start_time = get_time_ms();
}

static uint16_t adc_data[3];
static uint32_t ozone_sensor = 0;
static uint32_t ozone_sensor_num = 0;
static uint32_t uv_light_sensor = 0;
static uint32_t uv_light_sensor_num = 0;
static uint32_t temperature_sensor = 0;
static uint32_t temperature_sensor_num = 0;
static uint8_t adc_channel_ids[] = {ADC_CHANNEL10, ADC_CHANNEL11, ADC_CHANNEL16};
static adc_channel adc_channels[] = {
	{.rcc_port = RCC_GPIOC, .gpio_port = GPIOC, .gpio = GPIO0},
	{.rcc_port = RCC_GPIOC, .gpio_port = GPIOC, .gpio = GPIO1},
	{.rcc_port = 0, .gpio_port = 0, .gpio = 0},
};

static void process_adc(TelemetryPacket *packet) {
	uv_light_sensor += adc_data[0];
	ozone_sensor += adc_data[1];
	temperature_sensor += adc_data[2];
	++ozone_sensor_num;
	++uv_light_sensor_num;
	++temperature_sensor_num;

	if (get_time_since_ms(last_adc_read) < 300) return;

	packet->has_ozone = true;
	packet->ozone = (float)ozone_sensor / ozone_sensor_num;

	packet->has_uv_light = true;
	packet->uv_light = (float)uv_light_sensor / uv_light_sensor_num;

	packet->has_cpu_temperature = true;
	packet->cpu_temperature = (float)temperature_sensor / temperature_sensor_num;

	ozone_sensor = 0;
	uv_light_sensor = 0;
	temperature_sensor = 0;
	ozone_sensor_num = 0;
	uv_light_sensor_num = 0;
	temperature_sensor_num = 0;

	last_adc_read = get_time_ms();
}

static void process_gps(TelemetryPacket *packet) {
	if (!habduino_has_new_data) return;

	packet->gps_altitude = habduino_altitude;
	packet->has_gps_altitude = true;

	packet->latitude = habduino_latitude;
	packet->has_latitude = true;

	packet->longitude = habduino_longitude;
	packet->has_longitude = true;

	packet->gps_sats_num = habduino_sats_num;
	packet->has_gps_sats_num = true;

	habduino_has_new_data = false;
}

static uint32_t packet_mask;

static void mask_packet_fields(TelemetryPacket *packet) {
	packet_mask = 0;
	if (packet->has_acceleration_x) packet_mask |= (1 << 0);
	if (packet->has_acceleration_y) packet_mask |= (1 << 1);
	if (packet->has_acceleration_z) packet_mask |= (1 << 2);
	if (packet->has_gyroscope_x) packet_mask |= (1 << 3);
	if (packet->has_gyroscope_y) packet_mask |= (1 << 4);
	if (packet->has_gyroscope_z) packet_mask |= (1 << 5);
	if (packet->has_sun_sensor1) packet_mask |= (1 << 6);
	if (packet->has_sun_sensor2) packet_mask |= (1 << 7);
	if (packet->has_sun_sensor3) packet_mask |= (1 << 8);
	if (packet->has_sun_sensor4) packet_mask |= (1 << 9);
	if (packet->has_magnetometer_x) packet_mask |= (1 << 10);
	if (packet->has_magnetometer_y) packet_mask |= (1 << 11);
	if (packet->has_magnetometer_z) packet_mask |= (1 << 12);
	if (packet->has_quaternion0) packet_mask |= (1 << 13);
	if (packet->has_quaternion1) packet_mask |= (1 << 14);
	if (packet->has_quaternion2) packet_mask |= (1 << 15);
	if (packet->has_quaternion3) packet_mask |= (1 << 16);
	if (packet->has_ozone) packet_mask |= (1 << 17);
	if (packet->has_uv_light) packet_mask |= (1 << 18);

	packet->has_acceleration_x = false;
	packet->has_acceleration_y = false;
	packet->has_acceleration_z = false;
	packet->has_gyroscope_x = false;
	packet->has_gyroscope_y = false;
	packet->has_gyroscope_z = false;
	packet->has_sun_sensor1 = false;
	packet->has_sun_sensor2 = false;
	packet->has_sun_sensor3 = false;
	packet->has_sun_sensor4 = false;
	packet->has_magnetometer_x = false;
	packet->has_magnetometer_y = false;
	packet->has_magnetometer_z = false;
	packet->has_quaternion0 = false;
	packet->has_quaternion1 = false;
	packet->has_quaternion2 = false;
	packet->has_quaternion3 = false;
	packet->has_ozone = false;
	packet->has_uv_light = false;
}

static void unmask_packet_fields(TelemetryPacket *packet) {
	if (packet_mask & (1 << 0)) packet->has_acceleration_x = true;
	if (packet_mask & (1 << 1)) packet->has_acceleration_y = true;
	if (packet_mask & (1 << 2)) packet->has_acceleration_z = true;
	if (packet_mask & (1 << 3)) packet->has_gyroscope_x = true;
	if (packet_mask & (1 << 4)) packet->has_gyroscope_y = true;
	if (packet_mask & (1 << 5)) packet->has_gyroscope_z = true;
	if (packet_mask & (1 << 6)) packet->has_sun_sensor1 = true;
	if (packet_mask & (1 << 7)) packet->has_sun_sensor2 = true;
	if (packet_mask & (1 << 8)) packet->has_sun_sensor3 = true;
	if (packet_mask & (1 << 9)) packet->has_sun_sensor4 = true;
	if (packet_mask & (1 << 10)) packet->has_magnetometer_x = true;
	if (packet_mask & (1 << 11)) packet->has_magnetometer_y = true;
	if (packet_mask & (1 << 12)) packet->has_magnetometer_z = true;
	if (packet_mask & (1 << 13)) packet->has_quaternion0 = true;
	if (packet_mask & (1 << 14)) packet->has_quaternion1 = true;
	if (packet_mask & (1 << 15)) packet->has_quaternion2 = true;
	if (packet_mask & (1 << 16)) packet->has_quaternion3 = true;
	if (packet_mask & (1 << 17)) packet->has_ozone = true;
	if (packet_mask & (1 << 18)) packet->has_uv_light = true;
}

int main(void) {
	BMP180 bmp180_sensor;
	EEPROM eeprom;
	DS18B20Bus ds18b20_bus;
	BH1750 bh1750;
	clock_setup();
	gpio_setup();
	systick_setup();
//	usart_setup();
	spi_setup();
	sdcard_setup();
//	CHECK_SETUP(sdcard);
	log_setup();
	adc_setup(adc_channels, adc_channel_ids, adc_data, 3);
	i2c_setup();
	msleep(1000);
//	smbus_setup();
	MPU6050_initialize();
	HMC5883L_Init();
	last_mpu6050_conversion = get_time_ms();
	last_hmc5883l_conversion = get_time_ms();
	last_madgwick = get_time_ms();
	// Init Gyroscope offsets
	int i;
	for(i = 0; i < 32; i ++) {
		gx_offset += MPU6050_getRotationX();
		gy_offset += MPU6050_getRotationY();
		gz_offset += MPU6050_getRotationZ();
		msleep(100);
	}
	gx_offset >>= 5;
	gy_offset >>= 5;
	gz_offset >>= 5;

	/* last_ozone_read = get_time_ms(); */

	bmp180_setup(&bmp180_sensor, I2C2, BMP180_MODE_ULTRA_HIGHRES);
/* 	bh1750_setup(&bh1750, I2C3); */
/* //	EEPROM_setup(&eeprom, I2C2); */

/* 	iwdg_set_period_ms(2000); */
/* 	iwdg_start(); */

	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(GPIOD, GPIO12);
	gpio_clear(GPIOD, GPIO15);

	static uint8_t buffer1[256];
	static uint8_t buffer2[256];

	rcc_periph_clock_enable(RCC_GPIOE);
	volatile uint8_t devices_num = ds18b20_setup(&ds18b20_bus, GPIOE, GPIO3,
	                                             DS18B20_RESOLUTION_12_BITS);
	ds18b20_start_all(&ds18b20_bus);

//	volatile uint16_t temp = smbus_read_word(0, 0x06);

	TelemetryPacket packet = TelemetryPacket_init_zero;
	/* packet.has_quaternion0 = true; */
	/* packet.has_quaternion1 = true; */
	/* packet.has_quaternion2 = true; */
	/* packet.has_quaternion3 = true; */
	/* packet.has_acceleration_x = true; */
	/* packet.has_acceleration_y = true; */
	/* packet.has_acceleration_z = true; */
	/* packet.has_gyroscope_x = true; */
	/* packet.has_gyroscope_y = true; */
	/* packet.has_gyroscope_z = true; */


	uint64_t last_packet_time = get_time_ms();
	uint32_t packet_id = 0;
	uint32_t habduino_packet_id = 0;
	uint8_t checksum;
	msleep(1000);
	while (1) {

		/* iwdg_reset(); */
		process_bmp180(&bmp180_sensor, &packet);
		process_ds18b20(&ds18b20_bus, &packet);
		process_mpu6050(&packet);
		process_hmc5883l(&packet);
		process_madgwick(&packet);
		process_adc(&packet);
/* 		process_bh1750(&bh1750, &packet); */
		process_gps(&packet);

		if (habduino_has_pending_packet_request) {
			packet.packet_id = habduino_packet_id;
			packet.timestamp = get_time_ms();
			packet.status = 0xFFFFFFFF;

			/* mask_packet_fields(&packet); */
			/* pb_ostream_t stream = pb_ostream_from_buffer(buffer1, sizeof(buffer1)); */
			/* bool status = pb_encode(&stream, TelemetryPacket_fields, &packet); */
			/* unmask_packet_fields(&packet); */

			/* int len = stuff_data(buffer1, stream.bytes_written, buffer2); */

			/* usart_send_blocking(USART_ID, len); */
			/* for (i = 0; i < len; ++i) { */
			/* 	usart_send_blocking(USART_ID, buffer2[i]); */
			/* } */

			usart_send_blocking(USART_ID, 10);
			for (i = 0; i < 10; ++i) {
				usart_send_blocking(USART_ID, 0xFF);
			}

			habduino_has_pending_packet_request = false;
		}

		/* Is it time to send a packet? */
		if (get_time_since_ms(last_packet_time) > PACKET_DELAY_MS) {
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

			log_write(buffer2, stream.bytes_written + 3);

			last_packet_time = get_time_ms();

			/* Clear packet */
			packet = (TelemetryPacket)TelemetryPacket_init_zero;
		}
	}

	return 0;
}

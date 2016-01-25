#include "bmp180.h"

#include <assert.h>
#include <math.h>

#include <libopencm3/stm32/i2c.h>

#include "i2c.h"
#include "utils.h"

/*
 * Based on an official Bosch API for BMP180:
 * https://github.com/BoschSensortec/BMP180_driver/blob/master/bmp180.c
 *
 * NOTE: This pressure sensor works only for altitudes below 9000m
 */

/*
 * Datasheet states that maximum conversion times are
 * 4.5ms, 7.5ms, 13.5ms, 25.5ms (Table 3)
 */
static uint32_t pressure_conversion_time[] = {5, 8, 14, 26};

static void read_calib_param(BMP180 *sensor);
static int32_t read_temperature(BMP180 *sensor);
static void start_pressure_conv(BMP180 *sensor);
static int32_t finish_pressure_conv(BMP180 *sensor);

void BMP180_setup(BMP180 *sensor, uint32_t i2c, uint8_t oss) {
	sensor->i2c = i2c;
	sensor->oss = oss;
	// TODO: don't crash in case of failure
	assert(i2c_read_byte(sensor->i2c, BMP180_ADDRESS, BMP180_CHIP_ID) == 0x55);

	read_calib_param(sensor);
}

void BMP180_start_conv(BMP180 *sensor) {
	sensor->temperature = read_temperature(sensor);

	start_pressure_conv(sensor);
	sensor->conv_start_time = get_time_ms();
}

void BMP180_finish_conv(BMP180 *sensor) {
	while (get_time_ms() < sensor->conv_start_time +
	       pressure_conversion_time[sensor->oss]);

	sensor->pressure = finish_pressure_conv(sensor);
	sensor->altitude = 44330 * (1 - powf(
	    (sensor->pressure / (float)101325), 0.1903f));
}

int32_t read_temperature(BMP180 *sensor) {
	// TODO: sometimes return INVALID_DATA (like in an official API)
	// Start temperature measurement
	i2c_write_byte(sensor->i2c, BMP180_ADDR, BMP180_CONTROL,
	               BMP180_READ_TEMPERATURE);

	// Datasheet states that maximum conversion time is 4.5ms (Table 1)
	msleep(5);

	int32_t UT = i2c_read_word(sensor->i2c, BMP180_ADDR, BMP180_CONTROL_OUTPUT);

	int32_t X1 = (UT - (int32_t)sensor->AC6) * (int32_t)sensor->AC5;
	X1 >>= 15;
	int32_t X2 = ((int32_t)sensor->MC << 11) / (X1 + sensor->MD);
	sensor->B5 = X1 + X2;
	int32_t T = (sensor->B5 + 8) / (1 << 4);
	return T;
}

void start_pressure_conv(BMP180 *sensor) {
	i2c_write_byte(sensor->i2c, BMP180_ADDR, BMP180_CONTROL,
	               BMP180_READ_PRESSURE + (sensor->oss << 6));
}

int32_t finish_pressure_conv(BMP180 *sensor) {
	int32_t UP = i2c_read_word(sensor->i2c, BMP180_ADDR, BMP180_CONTROL_OUTPUT);
	UP <<= 8;
	UP |= i2c_read_byte(sensor->i2c, BMP180_ADDR, BMP180_CONTROL_OUTPUT + 2);
	UP >>= 8 - sensor->oss;

	int32_t X1, X2, X3, B3, B6, P, Temp, pressure;
	uint32_t  B4, B7;

	B6 = sensor->B5 - 4000;
	X1 = (sensor->B2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (sensor->AC2 * B6) >> 11;
	X3 = X1 + X2;
	Temp = (((int32_t)sensor->AC1 << 2) + X3) << sensor->oss;
	B3 = (Temp + 2) >> 2;
	X1 = (sensor->AC3 * B6) >> 13;
	X2 = (sensor->B1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = (sensor->AC4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> sensor->oss);
	if(B7 < 0x80000000)	{
		P = (B7 << 1) / B4;
	} else {
		P = ((B7 / B4) << 1);
	}

	X1 = (P >> 8) * (P >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * P) >> 16;

	pressure = P + ((X1 + X2 + 3791) >> 4);
	return pressure;
}

void read_calib_param(BMP180 *sensor) {
	sensor->AC1 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_AC1);
	sensor->AC2 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_AC2);
	sensor->AC3 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_AC3);
	sensor->AC4 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_AC4);
	sensor->AC5 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_AC5);
	sensor->AC6 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_AC6);

	sensor->B1 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_B1);
	sensor->B2 = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_B2);

	sensor->MB = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_MB);
	sensor->MC = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_MC);
	sensor->MD = i2c_read_word(sensor->i2c, BMP180_ADDRESS, BMP180_MD);
}

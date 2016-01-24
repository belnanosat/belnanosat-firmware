#include "bmp180.h"

#include <assert.h>

#include <libopencm3/stm32/i2c.h>

#include "i2c.h"
#include "utils.h"

/*
 * Based on an official Bosch API for BMP180:
 * https://github.com/BoschSensortec/BMP180_driver/blob/master/bmp180.c
 */

static void read_calib_param(BMP180* sensor);

void BMP180_setup(BMP180 *sensor, uint32_t i2c) {
	sensor->i2c = i2c;
	// TODO: don't crash in case of failure
	assert(i2c_read_byte(sensor->i2c, BMP180_ADDRESS, BMP180_CHIP_ID) == 0x55);

	read_calib_param(sensor);
}

float BMP180_read_temperature(BMP180 *sensor) {
	// TODO: sometimes return INVALID_DATA (like in an official API)
	// Start temperature measurement
	i2c_write_byte(sensor->i2c, BMP180_ADDR, BMP180_CONTROL,
	               BMP180_READ_TEMPERATURE);
	msleep(30);
	int32_t UT = i2c_read_word(sensor->i2c, BMP180_ADDR, BMP180_CONTROL_OUTPUT);

	int32_t X1 = (UT - (int32_t)sensor->AC6) * (int32_t)sensor->AC5;
	X1 >>= 15;
	int32_t X2 = ((int32_t)sensor->MC << 11) / (X1 + sensor->MD);
	int32_t B5 = X1 + X2;
	int32_t T = (B5 + 8) / (1 << 4);
	return T / 10.0f;
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

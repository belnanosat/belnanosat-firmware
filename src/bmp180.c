#include "bmp180.h"

#include <assert.h>

#include <libopencm3/stm32/i2c.h>

#include "i2c.h"

void BMP180_setup(void) {
	// TODO: don't crash in case of failure
	assert(i2c_read_byte(BMP180_I2C_ID, BMP180_ADDRESS, BMP180_CHIP_ID) == 0x55);
}

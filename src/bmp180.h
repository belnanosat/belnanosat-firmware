#ifndef BMP180_H
#define BMP180_H

#define BMP180_ADDRESS        0x77
#define BMP180_CHIP_ID        0xD0

#define BMP180_AC1            0xAA
#define BMP180_AC2            0xAC
#define BMP180_AC3            0xAE
#define BMP180_AC4            0xB0
#define BMP180_AC5            0xB2
#define BMP180_AC6            0xB4
#define BMP180_B1             0xB6
#define BMP180_B2             0xB8
#define BMP180_MB             0xBA
#define BMP180_MC             0xBC
#define BMP180_MD             0xBE
#define BMP180_CONTROL        0xF4
#define BMP180_CONTROL_OUTPUT 0xF6

// BMP180 Modes
#define BMP180_MODE_ULTRA_LOW_POWER    0
#define BMP180_MODE_STANDARD           1
#define BMP180_MODE_HIGHRES            2
#define BMP180_MODE_ULTRA_HIGHRES      3

// Control register
#define BMP180_READ_TEMPERATURE        0x2E
#define BMP180_READ_PRESSURE           0x34
#define MSLP                    101325  // Mean Sea Level Pressure = 1013.25 hPA

#include <stdint.h>

typedef struct
{
	int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
	uint16_t AC4, AC5, AC6;
	int32_t B5, UT, UP, Pressure0;
	uint32_t i2c;
	uint32_t conv_start_time;
	uint8_t oss;

	int32_t pressure, temperature;
	float altitude;
} BMP180;

extern void BMP180_setup(BMP180 *sensor, uint32_t i2c, uint8_t oss);
extern void BMP180_start_conv(BMP180 *sensor);
extern void BMP180_finish_conv(BMP180 *sensor);

#endif

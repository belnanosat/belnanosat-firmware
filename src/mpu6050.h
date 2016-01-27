#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48


#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//Power Management. Typical values:0x00(run mode)
#define	WHO_AM_I		0x75	//identity of the device


#define	GYRO_ADDRESS   0xD0
#define MAG_ADDRESS    0x18
#define ACCEL_ADDRESS  0xD0

#define ADDRESS_AD0_LOW     0xD0 //address pin low (GND), default for InvenSense evaluation board
#define ADDRESS_AD0_HIGH    0xD1 //address pin high (VCC)
#define DEFAULT_ADDRESS     GYRO_ADDRESS
#define WHO_AM_I_VAL				0x73 //identity of MPU9250 is 0x71. identity of MPU9255 is 0x73.

#define MPU6050_I2C

typedef struct {
	uint32_t i2c;
} MPU6050;

extern void mpu6050_setup(MPU6050 *sensor, uint32_t i2c);

#endif

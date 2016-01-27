#include "mpu6050.h"

#include "i2c.h"

void mpu6050_setup(MPU6050 *sensor, uint32_t i2c) {
	sensor->i2c = i2c;
	i2c_write_byte(sensor->i2c, GYRO_ADDRESS, PWR_MGMT_1, 0x00);
}

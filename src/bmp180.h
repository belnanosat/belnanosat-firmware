#ifndef BMP180_H
#define BMP180_H

#define BMP180_I2C_ID  I2C2
#define BMP180_ADDRESS 0x77
#define BMP180_CHIP_ID 0xD0

extern void BMP180_setup(void);
extern float BMP180_read_temperature(void);

#endif

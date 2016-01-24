#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define BMP180_ADDR                 0x77//0xEE
#define READ_TEMPERATURE            0xAA//0x2E

void i2c_setup(void);
void i2c_write_byte(uint32_t i2c, uint8_t reg_address, uint8_t value);
uint8_t i2c_read_byte(uint32_t i2c, uint8_t reg_address);
uint16_t i2c_read_word(uint32_t i2c, uint8_t reg_address);
uint16_t i2c_read(uint32_t i2c, uint8_t device_address);

#endif

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/*
 * This I2C library is implemented for STM32F407
 */

void i2c_setup(void);

uint8_t i2c_read_bit(uint32_t i2c, uint8_t device_address,
                     uint8_t reg_address, uint8_t bit_id);
void i2c_write_bit(uint32_t i2c, uint8_t device_address,
                   uint8_t reg_address, uint8_t bit_id, uint8_t data);
uint8_t i2c_read_bits(uint32_t i2c, uint8_t devAddr, uint8_t regAddr,
                      uint8_t bitStart, uint8_t length);
void i2c_write_bits(uint32_t i2c, uint8_t devAddr, uint8_t regAddr,
                    uint8_t bitStart, uint8_t length, uint8_t data);
void i2c_write_byte(uint32_t i2c, uint8_t device_address,
                    uint8_t reg_address, uint8_t value);
void i2c_write_word(uint32_t i2c, uint8_t device_address,
                    uint8_t reg_address, uint16_t value);
uint8_t i2c_read_byte(uint32_t i2c, uint8_t device_address, uint8_t reg_address);
uint16_t i2c_read_word(uint32_t i2c, uint8_t device_address, uint8_t reg_address);

// These funcitons are used for EEPROM access and require 2-byte register address
// NOTE: we assume that buf_length > 2
void i2c_read_sequence(uint32_t i2c, uint8_t device_address,
                       uint16_t reg_address, uint8_t *buffer, int buf_length);
void i2c_write_sequence(uint32_t i2c, uint8_t device_address,
                        uint16_t reg_address, uint8_t *buffer, int buf_length);
// For reading from registers with 2 bytes addresses
uint8_t i2c_wread_byte(uint32_t i2c, uint8_t device_address,
                       uint16_t reg_address);

#endif

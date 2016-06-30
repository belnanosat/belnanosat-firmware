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

#include "smbus.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

/*
 * I2C1 pins:
 * SDA - PB7, SCL - PB6
 * I2C2 pins:
 * SDA - PB11, SCL - PB10
 * I2C3 pins:
 * SDA - PC9, SCL - PA8
 */
void smbus_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);

	rcc_periph_clock_enable(RCC_I2C1);
	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);

	// rcc_apb1_frequency = 42MHz
	i2c_set_clock_frequency(I2C1, rcc_apb1_frequency/1000000);
	i2c_set_standard_mode(I2C1);
	i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2); // Div2
	I2C_CR1(I2C1) |= I2C_CR1_SMBTYPE;

	/*
	 * For APB1 PCLK1 = 42MHz => I2C speed = 100kHz
	 * General equation is CCR * I2C_frequency = PCLK1_frequency
	 */
	i2c_set_ccr(I2C1, 420);

	i2c_peripheral_enable(I2C1);

	// ACK bit is cleared by hardware when I2C is disabled
	i2c_enable_ack(SMBUS_NAME);
}


/**
 * @brief  Writes one word to the peripheral device.
 * @param  device_address : slave address.
 * @param  reg_address : address of the register in which data will be written.
 * @param  data : data to be written
 * @retval None
 */
void smbus_write_word(uint8_t device_address, uint8_t reg_address,
                      uint16_t data) {
	uint32_t reg32 __attribute__((unused));
	volatile uint32_t pec = smbus_crc8(((uint32_t)reg_address << 16)
	                                   | ((uint32_t)(data & 0xFF) << 8)
	                                   | (data >> 8));
	i2c_send_start(SMBUS_NAME);
	while (!((I2C_SR1(SMBUS_NAME) & I2C_SR1_SB)
	         & (I2C_SR2(SMBUS_NAME) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(SMBUS_NAME, device_address, I2C_WRITE);
	while (!(I2C_SR1(SMBUS_NAME) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(SMBUS_NAME);
	i2c_send_data(SMBUS_NAME, reg_address);
	while (!(I2C_SR1(SMBUS_NAME) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_data(SMBUS_NAME, data & 0xFF);
	while (!(I2C_SR1(SMBUS_NAME) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_data(SMBUS_NAME, data >> 8);
	while (!(I2C_SR1(SMBUS_NAME) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_data(SMBUS_NAME, pec);
	while (!(I2C_SR1(SMBUS_NAME) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_stop(SMBUS_NAME);
}

/**
 * @brief  Reads a word from the peripheral device.
 * @param  device_address : slave address.
 * @param  reg_address : register address to read from.
 * @retval data read from the device
 */
uint16_t smbus_read_word(uint8_t device_address, uint8_t reg_address) {
	uint32_t reg32 __attribute__((unused));
	uint16_t res = 0;
	uint32_t i = 0;
	i2c_send_start(SMBUS_NAME);
	while (!((I2C_SR1(SMBUS_NAME) & I2C_SR1_SB)
	         & (I2C_SR2(SMBUS_NAME) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(SMBUS_NAME, device_address, I2C_WRITE);
	while (!(I2C_SR1(SMBUS_NAME) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(SMBUS_NAME);
	i2c_send_data(SMBUS_NAME, reg_address);
	while (!(I2C_SR1(SMBUS_NAME) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(SMBUS_NAME);
	while (!((I2C_SR1(SMBUS_NAME) & I2C_SR1_SB)
	         & (I2C_SR2(SMBUS_NAME) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(SMBUS_NAME, device_address, I2C_READ);
	while (!(I2C_SR1(SMBUS_NAME) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(SMBUS_NAME);

	while (!(I2C_SR1(SMBUS_NAME) & I2C_SR1_RxNE));
	res = I2C_DR(SMBUS_NAME);

	while (!(I2C_SR1(SMBUS_NAME) & I2C_SR1_RxNE));
	res |= ((uint16_t)I2C_DR(SMBUS_NAME) << 8);

	while (!(I2C_SR1(SMBUS_NAME) & I2C_SR1_RxNE));
	uint8_t pec = I2C_DR(SMBUS_NAME);

	i2c_send_stop(SMBUS_NAME);

	return res;
}


/**
 * Calculate CRC8 sum for a word
 * @param data input word
 * @return calculated CRC8 for an input word
 */
uint8_t smbus_crc8(uint32_t data)
{
	uint32_t msg = data << 8;
	uint32_t key = 0x107 << 23;
	uint32_t mask = 1 << 31;
	while(mask > 0x80) {
		if(mask & msg) {
			msg ^= key;
		}
		key >>= 1;
		mask >>= 1;
	}
	return msg;
}

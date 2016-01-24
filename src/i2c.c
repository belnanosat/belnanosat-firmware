#include "i2c.h"

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>


/*
 * I2C1 pins:
 * SDA - PB7, SCL - PB6
 * I2C2 pins:
 * SDA - PB11, SCL - PB10
 */
void i2c_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO10 | GPIO11);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO10 | GPIO11);

	rcc_periph_clock_enable(RCC_I2C2);
	i2c_reset(I2C2);
	i2c_peripheral_disable(I2C2);

	// rcc_apb1_frequency = 42MHz
	i2c_set_clock_frequency(I2C2, rcc_apb1_frequency/1000000);
	i2c_set_fast_mode(I2C2);
	i2c_set_dutycycle(I2C2, I2C_CCR_DUTY_16_DIV_9);

	// For APB1 PCLK1 = 42MHz => I2C speed = 400kHz
	i2c_set_ccr(I2C2, 3);

	i2c_peripheral_enable(I2C2);
}

void i2c_write_byte(uint32_t i2c, uint8_t device_address, uint8_t reg_address, uint8_t value) {
	uint32_t reg32 __attribute__((unused));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, reg_address);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_data(i2c, value);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_stop(i2c);
}

uint8_t i2c_read_byte(uint32_t i2c, uint8_t device_address, uint8_t reg_address) {
	uint32_t reg32 __attribute__((unused));
	volatile uint8_t res;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, reg_address);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_READ);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	// Why does it hang? According to the datasheet BTF is set iff (RxNE & DR isn't read)
//	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_stop(i2c);
	res = I2C_DR(i2c);

	return res;
}

uint16_t i2c_read_word(uint32_t i2c, uint8_t device_address, uint8_t reg_address) {
	uint32_t reg32 __attribute__((unused));
	uint16_t res;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));


	i2c_send_7bit_address(i2c, device_address, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);
	i2c_send_data(i2c, reg_address);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_READ);

	/* 2-byte receive is a special case. See datasheet POS bit. */
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	res = (uint16_t)(I2C_DR(i2c) << 8); /* MSB */

	I2C_CR1(i2c) |= I2C_CR1_STOP;

	res |= I2C_DR(i2c); /* LSB */

	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	return res;
}

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
 * I2C3 pins:
 * SDA - PC9, SCL - PA8
 */
void i2c_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
	                        GPIO10 | GPIO11);
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

void i2c_write_byte(uint32_t i2c, uint8_t device_address,
                    uint8_t reg_address, uint8_t value) {
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

void i2c_write_word(uint32_t i2c, uint8_t device_address,
                    uint8_t reg_address, uint16_t value) {
	uint8_t tbuf[2];
	tbuf[0] = (value >> 8);
	tbuf[1] = (value & 0xFF);
	i2c_write_sequence(i2c, device_address, reg_address, tbuf, 2);
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
	// Why does it hang? According to the datasheet BTF is set
	// iff (RxNE & DR isn't read)
//	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_stop(i2c);
	res = I2C_DR(i2c);

	return res;
}

uint8_t i2c_read_bit(uint32_t i2c, uint8_t device_address,
                     uint8_t reg_address, uint8_t bit_id) {
	uint8_t b =  i2c_read_byte(i2c, device_address, reg_address);
    return b & (1 << bit_id);
}

void i2c_write_bit(uint32_t i2c, uint8_t device_address,
                   uint8_t reg_address, uint8_t bit_id, uint8_t data) {
	uint8_t b = i2c_read_byte(i2c, device_address, reg_address);
    b = (data != 0) ? (b | (1 << bit_id)) : (b & ~(1 << bit_id));
    i2c_write_byte(i2c, device_address, reg_address, b);
}

uint8_t i2c_read_bits(uint32_t i2c, uint8_t devAddr, uint8_t regAddr,
                      uint8_t bitStart, uint8_t length) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b = i2c_read_byte(i2c, devAddr, regAddr);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    return b;
}

void i2c_write_bits(uint32_t i2c, uint8_t devAddr, uint8_t regAddr,
                    uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    b = i2c_read_byte(i2c, devAddr, regAddr);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    i2c_write_byte(i2c, devAddr, regAddr, b);
}

uint16_t i2c_read_word(uint32_t i2c, uint8_t device_address,
                       uint8_t reg_address) {
	uint32_t reg32 __attribute__((unused));
	uint16_t res;

	while (I2C_SR2(i2c) & I2C_SR2_BUSY);

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
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);

	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	res = (uint16_t)(I2C_DR(i2c) << 8); /* MSB */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	res |= I2C_DR(i2c); /* LSB */
	i2c_send_stop(i2c);


	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	return res;
}

void i2c_read_sequence(uint32_t i2c, uint8_t device_address,
                       uint16_t reg_address, uint8_t *buffer, int buf_length) {
	uint32_t reg32 __attribute__((unused));
	int i;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));


	i2c_send_7bit_address(i2c, device_address, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);
	// Send MSB
	i2c_send_data(i2c, (reg_address & 0xFF00) >> 8);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	// Send LSB
	i2c_send_data(i2c, reg_address & 0xFF);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_READ);
	I2C_CR1(i2c) |= I2C_CR1_ACK;
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));
	reg32 = I2C_SR2(i2c);

	for (i = 0; i + 3 < buf_length; ++i) {
		while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
		buffer[i] = I2C_DR(i2c);
	}

	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	buffer[buf_length - 3] = I2C_DR(i2c);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_stop(i2c);
	buffer[buf_length - 2] = I2C_DR(i2c);
	buffer[buf_length - 1] = I2C_DR(i2c);
}

void i2c_write_sequence(uint32_t i2c, uint8_t device_address,
                        uint16_t reg_address, uint8_t *buffer, int buf_length) {
	uint32_t reg32 __attribute__((unused));
	int i;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);

	// Send MSB of the register address
	i2c_send_data(i2c, (reg_address & 0xFF00) >> 8);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	// Send LSB of the register address
	i2c_send_data(i2c, reg_address & 0xFF);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	for (i = 0; i < buf_length; ++i) {
		i2c_send_data(i2c, buffer[i]);
		while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	}

	i2c_send_stop(i2c);
}

uint8_t i2c_wread_byte(uint32_t i2c, uint8_t device_address,
                       uint16_t reg_address) {
		uint32_t reg32 __attribute__((unused));
	volatile uint8_t res;

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_WRITE);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);

	// Send MSB of the register address
	i2c_send_data(i2c, (reg_address & 0xFF00) >> 8);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
	// Send LSB of the register address
	i2c_send_data(i2c, reg_address & 0xFF);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	i2c_send_start(i2c);
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	         & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, device_address, I2C_READ);
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	reg32 = I2C_SR2(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;
	i2c_send_stop(i2c);
	res = I2C_DR(i2c);

	return res;
}

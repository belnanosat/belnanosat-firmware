#include "spi.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

void spi_setup() {
	// NSS pins seems to be optional and conflicts with JTAG
	uint32_t cr_tmp;
	rcc_periph_clock_enable(SPI_RCC_PORT);
	rcc_periph_clock_enable(SPI_RCC);

	gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
	                SPI_MOSI);
	gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
	                SPI_MISO | SPI_SCK);
	gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_NSS);
	gpio_set_output_options(SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
	                        SPI_SCK | SPI_MOSI);
	gpio_set_af(SPI_PORT, SPI_GPIO_AF,
	            SPI_SCK | SPI_MISO | SPI_MOSI | SPI_NSS);

	spi_reset(SPI_ID);

	spi_init_master(SPI_ID, SPI_CR1_BAUDRATE_FPCLK_DIV_256, 0,//SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
	                0,//SPI_CR1_CPHA_CLK_TRANSITION_1,
	                SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_set_full_duplex_mode(SPI_ID);

	spi_enable_software_slave_management(SPI_ID);
	spi_set_nss_high(SPI_ID);
	SPI_CR2(SPI_ID) |= SPI_CR2_FRF;
	gpio_set(SPI_PORT, SPI_NSS);
	spi_disable_crc(SPI_ID);
	spi_enable(SPI_ID);
}


uint8_t spi_write_and_read(uint8_t data) {
	while (!(SPI_SR(SPI_ID) & SPI_SR_TXE));
	SPI_DR(SPI_ID) = data;

	while (!(SPI_SR(SPI_ID) & SPI_SR_RXNE));
	return SPI_DR(SPI_ID);
}

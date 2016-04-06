#include "adc.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	adc_off(ADC1);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
//	adc_enable_temperature_sensor();
//	adc_set_clk_prescale(ADC_CCR_TSVREFE);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL10, ADC_SMPR_SMP_15CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL11, ADC_SMPR_SMP_15CYC);
	uint8_t channels[] = {ADC_CHANNEL10, ADC_CHANNEL11};
	adc_set_regular_sequence(ADC1, 2, channels);
	adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
	adc_power_on(ADC1);
}

void adc_read(uint16_t *res1, uint16_t *res2)
{
	uint8_t channels1[] = {ADC_CHANNEL10};
	adc_set_regular_sequence(ADC1, 1, channels1);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	*res1 = adc_read_regular(ADC1);

	uint8_t channels2[] = {ADC_CHANNEL11};
	adc_set_regular_sequence(ADC1, 1, channels2);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	*res2 = adc_read_regular(ADC1);
}

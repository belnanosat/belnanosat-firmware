#include "adc.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

void adc_setup(void)
{
	/* rcc_periph_clock_enable(RCC_GPIOA); */
	/* gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2); */
	adc_off(ADC1);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
	adc_enable_temperature_sensor();
//	adc_set_clk_prescale(ADC_CCR_TSVREFE);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL16, ADC_SMPR_SMP_15CYC);
	uint8_t channels[] = {ADC_CHANNEL16};
	adc_set_regular_sequence(ADC1, 1, channels);
	adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
	adc_power_on(ADC1);
}

uint16_t adc_read(void)
{
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	return adc_read_regular(ADC1);
}

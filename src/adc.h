#ifndef ADC_H
#define ADC_H

#include <stdint.h>

extern void adc_setup(void);
extern void adc_read(uint16_t *res1, uint16_t *res2);

#endif

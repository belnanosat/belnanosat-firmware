#include "utils.h"

#include <limits.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

volatile uint64_t system_microseconds;

void sys_tick_handler(void) {
	system_microseconds++;
}

void usleep(uint64_t delay) {
	uint64_t wake = system_microseconds + delay;
	while (wake > system_microseconds);
}

void msleep(uint64_t delay) {
	uint64_t i;
	for (i = 0; i < delay; ++i) usleep(1000);
}

uint64_t get_time_ms(void) {
	return system_microseconds / 1000;
}

uint64_t get_time_us(void) {
	return system_microseconds;
}

uint64_t get_time_since_ms(uint64_t start_time) {
	return get_time_ms() - start_time;
}

uint64_t get_time_since_us(uint64_t start_time) {
	return system_microseconds - start_time;
}

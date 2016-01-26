#include "utils.h"

volatile uint32_t system_microseconds;

void sys_tick_handler(void) {
	system_microseconds++;
}

void usleep(uint32_t delay) {
	uint32_t wake = system_microseconds + delay;
	while (wake > system_microseconds);
}

void msleep(uint32_t delay) {
	uint32_t i;
	for (i = 0; i < delay; ++i) usleep(1000);
}

uint32_t get_time_ms(void) {
	return system_microseconds / 1000;
}

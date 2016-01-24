#include "utils.h"

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
	system_millis++;
}

void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}

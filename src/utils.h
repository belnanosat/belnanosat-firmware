#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

extern void usleep(uint32_t delay);
extern void msleep(uint32_t delay);
extern uint32_t get_time_ms(void);

#endif

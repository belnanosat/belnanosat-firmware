#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

extern void usleep(uint32_t delay);
extern void msleep(uint32_t delay);
extern uint32_t get_time_ms(void);
extern uint32_t get_time_since(uint32_t start_time);

#endif

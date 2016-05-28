#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

extern void usleep(uint64_t delay);
extern void msleep(uint64_t delay);
extern uint64_t get_time_ms(void);
extern uint64_t get_time_us(void);
extern uint64_t get_time_since_ms(uint64_t start_time);
extern uint64_t get_time_since_us(uint64_t start_time);

#endif

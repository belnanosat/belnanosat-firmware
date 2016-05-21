#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

extern void usleep(uint64_t delay);
extern void msleep(uint64_t delay);
extern uint64_t get_time_ms(void);
extern uint64_t get_time_since(uint64_t start_time);

#endif

#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>

#define STATUS_BYTES_PER_DEVICE 3

#define STATUS_I2C_BUSY          0x01
#define STATUS_I2C_ADDRESS_NACK  0x02
#define STATUS_I2C_TX_TIMEOUT    0x03
#define STATUS_I2C_RX_TIMEOUT    0x04
#define STATUS_I2C_REPS_TIMEOUT  0x05

#define STATUS_MPU6050_SHIFT
#define STATUS_HMC5883L_SHIFT
#define STATUS_BMP180_SHIFT
#define STATUS_MLX90614_SHIFT
#define STATUS_BH1750_SHIFT

#define EC_I2C_BUS_BUSY          0x01
#define EC_I2C_ADDRESS_NACK      0x02
#define EC_I2C_TX_TIMEOUT        0x03
#define EC_I2C_RX_TIMEOUT        0x04

#define STATUS_MAX_STACK_FRAMES  10

extern int32_t status_stack_frame_id;
extern uint32_t status_stack_frames[STATUS_MAX_STACK_FRAMES];

#define PUSH_STACK_FRAME(error_code) do{	  \
		if (status_stack_frame_id + 1 < (STATUS_MAX_STACK_FRAMES)) { \
			status_stack_frames[status_stack_frame_id++] =\
				__LINE__ | (FILE_ID << 16) | (error_code << 24); \
		}\
	} while (0)

#define POP_STACK_FRAME do{	\
		if (status_stack_frame_id) --status_stack_frame_id;\
	} while (0)

#define SAFE_SPIN_LOCK(expr, n, error_code, error_status) do{	  \
		int i = 0; \
		while (i < (n) && (expr)) ++i; \
		if (i == (n)) { \
			PUSH_STACK_FRAME(error_code); \
			return (uint32_t)error_status; \
		} \
	} while (0)

#endif

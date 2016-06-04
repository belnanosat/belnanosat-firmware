/*
 * This file is part of belnanosat project.
 *
 * Copyright (C) 2016 Uladzislau Paulovich <selatnick@gmail.com>
 *
 * belnanosat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * belnanosat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with belnanosat.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "mlx90614.h"

#include <stdio.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "smbus.h"
#include "utils.h"
#include "i2c.h"

static uint16_t to_max, to_min;
static uint8_t mlx90614_address = 0x0;

/**
 * @brief  Enable SMBus communication mode for MLX90614.
 * To select SMBus mode instead of PWM for MLX90614 it's just required
 * to reset SCL line for more than 2ms.
 * @param  None
 * @retval None
 */

void mlx90614_disable_pwm(void) {
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
        gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO6);

        gpio_clear(GPIOB, GPIO6);
        msleep(10);
        gpio_set(GPIOB, GPIO6);
}

/**
 * @brief  Initialize mlx90614
 * @param  None
 * @retval None
 */
void mlx90614_setup(void) {
        to_min = smbus_read_word(mlx90614_address, 0x20);
        to_max = smbus_read_word(mlx90614_address, 0x21);
}

/**
 * @brief  Test connection with MLX90614
 * @param  None
 * @retval 1 - sensor works properly, 0 - error
 */
/* int mlx90614_TestConnection(void) */
/* { */
/*      uint16_t tmp; */
/*      SMBus_ReadWord(mlx90614_address, &tmp, 0x06); */
/*      return 1; */
/* } */

uint16_t mlx90614_read_ambient_temp(void) {
        return smbus_read_word(mlx90614_address, 0x06);// ^ (1 << 15);
}

uint16_t mlx90614_read_object_temp(void) {
        return smbus_read_word(mlx90614_address, 0x07);// ^ (1 << 15);
}

void mlx90614_set_address(uint8_t address)
{
        uint16_t res;
        /* SMBus_WriteWord(mlx90614_address, 0x00, 0x2E); */
        /* DelayMs(10); */
        /* SMBus_WriteWord(mlx90614_address, address | ((uint32_t)address << 8), 0x2E); */
        /* DelayMs(10); */
        res = smbus_read_word(mlx90614_address, 0x2E);
//      printf("Address write result: %"PRIu16"\r\n", res);
}

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

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

#define USE_SPI2

#ifdef USE_SPI2

#define SPI_ID SPI2
#define SPI_GPIO_AF GPIO_AF5
#define SPI_RCC RCC_SPI2
#define SPI_NSS GPIO12
#define SPI_SCK GPIO13
#define SPI_MISO GPIO14
#define SPI_MOSI GPIO15
#define SPI_RCC_PORT RCC_GPIOB
#define SPI_PORT GPIOB

#elif defined(USE_SPI3)

#define SPI_ID SPI3
#define SPI_GPIO_AF GPIO_AF6
#define SPI_RCC RCC_SPI3
#define SPI_NSS GPIO4
#define SPI_SCK GPIO10
#define SPI_MISO GPIO11
#define SPI_MOSI GPIO12
#define SPI_RCC_PORT RCC_GPIOC
#define SPI_PORT GPIOC

#else
#error "No SPI specified"
#endif
// Adds random MSB for some reason
// #define SPI_ID SPI1
// #define SPI_RCC RCC_SPI1
// #define SPI_NSS GPIO4
// #define SPI_SCK GPIO5
// #define SPI_MISO GPIO6
// #define SPI_MOSI GPIO7
// #define SPI_RCC_PORT RCC_GPIOA
// #define SPI_PORT GPIOA

extern void spi_setup(void);
extern uint8_t spi_write_and_read(uint8_t data);

#endif
